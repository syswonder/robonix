// SPDX-License-Identifier: MulanPSL-2.0

use std::pin::Pin;
use std::sync::Arc;
use std::task::{Context as TaskContext, Poll};
use std::time::Duration;

use anyhow::{Context, Result, ensure};
use robonix_atlas::{client::AtlasClient, pb as atlas_pb};
use tokio::sync::{Mutex, Semaphore, watch};
use tokio_stream::{Stream, wrappers::WatchStream};
use tonic::{Request, Response, Status};

use super::config::parse_manifest;
use super::pb::{
    chronos as wire, contracts,
    lifecycle::{DriverRequest, DriverResponse},
};
use super::publication::{PublicationWorker, QualitySender, ReferenceSender};
use crate::{Chronos, ChronosConfig, ChronosSection, QualityRuntime};

type SnapshotStream<T> = Pin<Box<dyn Stream<Item = Result<T, Status>> + Send>>;

const LIFECYCLE_CAPACITY: usize = 8;

/// Lifecycle serialization and runtime ownership are shared by all three RPCs.
#[derive(Clone)]
pub(super) struct ChronosService {
    state: Arc<Mutex<State>>,
    atlas: AtlasClient,
    provider_id: String,
    quality: QualitySender,
    reference: ReferenceSender,
    shutdown: watch::Sender<bool>,
    lifecycle_slots: Arc<Semaphore>,
}

struct State {
    config: ChronosConfig,
    runtime: Option<Arc<QualityRuntime>>,
    publisher: Option<PublicationWorker>,
    session: Option<watch::Sender<bool>>,
    terminated: bool,
}

impl ChronosService {
    /// Create an inactive service; Driver INIT constructs the sole runtime.
    pub fn new(config: ChronosConfig, atlas: AtlasClient, provider_id: String) -> Self {
        let (quality, _) = watch::channel(None);
        let (reference, _) = watch::channel(None);
        let (shutdown, _) = watch::channel(false);
        Self {
            state: Arc::new(Mutex::new(State {
                config,
                runtime: None,
                publisher: None,
                session: None,
                terminated: false,
            })),
            atlas,
            provider_id,
            quality,
            reference,
            shutdown,
            lifecycle_slots: Arc::new(Semaphore::new(LIFECYCLE_CAPACITY)),
        }
    }

    pub fn shutdown_receiver(&self) -> watch::Receiver<bool> {
        self.shutdown.subscribe()
    }

    /// Stop streams and both workers, retaining any cleanup error for the caller.
    pub async fn close(&self) -> Result<()> {
        let mut state = self.state.lock().await;
        state.terminated = true;
        self.end_streams(&mut state);
        let publisher = state.publisher.take();
        let runtime = state.runtime.take();
        self.shutdown.send_replace(true);
        tokio::task::spawn_blocking(move || stop_workers(publisher, runtime))
            .await
            .context("join Chronos cleanup")?
    }

    /// End existing streams; reactivation replaces these terminal cache values.
    fn end_streams(&self, state: &mut State) {
        if let Some(session) = state.session.take() {
            session.send_replace(true);
        }
        self.quality
            .send_replace(Some(Err(Status::unavailable("Chronos is inactive"))));
        self.reference
            .send_replace(Some(Err(Status::unavailable("Chronos is inactive"))));
    }

    /// Always remove publication after cancellation, even if input synchronization failed.
    async fn stop_publication(&self, state: &mut State) -> Result<()> {
        match state.publisher.take() {
            Some(publisher) => tokio::task::spawn_blocking(move || publisher.stop())
                .await
                .context("join Chronos deactivation")
                .and_then(|result| result),
            None => Ok(()),
        }
    }

    /// Admit bounded Driver work; owned transactions survive caller cancellation.
    pub async fn transition(&self, request: DriverRequest) -> Result<&'static str> {
        let permit = self.lifecycle_slots.clone().try_acquire_owned().context(
            "Chronos lifecycle request limit reached; retry after pending work completes",
        )?;
        let service = self.clone();
        tokio::spawn(async move {
            let _permit = permit;
            let result = service.transition_owned(request).await;
            if let Err(error) = &result {
                robonix_scribe::error!("Chronos lifecycle transition failed: {error:#}");
            }
            result
        })
        .await
        .context("join Chronos lifecycle transition")?
    }

    /// Finish lifecycle transactions even if the Driver caller disconnects during startup.
    async fn transition_owned(&self, request: DriverRequest) -> Result<&'static str> {
        let mut state = self.state.lock().await;
        ensure!(!state.terminated, "Chronos has terminated");
        let (target, label) = match request.command {
            0 => {
                ensure!(
                    state.publisher.is_none(),
                    "deactivate Chronos before initialization"
                );
                let manifest = parse_manifest(if request.config_json.trim().is_empty() {
                    "{}"
                } else {
                    &request.config_json
                })?;
                let config = manifest
                    .core
                    .map(|core| ChronosConfig {
                        chronos: ChronosSection {
                            reference_reader_command: state
                                .config
                                .chronos
                                .reference_reader_command
                                .clone(),
                            core,
                            collect_observations: state.config.chronos.collect_observations,
                            collect_ros_observations: state.config.chronos.collect_ros_observations,
                        },
                    })
                    .unwrap_or_else(|| state.config.clone());
                let mut config = config;
                if let Some(command) = manifest.reference_reader_command {
                    config.chronos.reference_reader_command = command;
                }
                if let Some(enabled) = manifest.collect_observations {
                    config.chronos.collect_observations = enabled;
                }
                if let Some(enabled) = manifest.collect_ros_observations {
                    config.chronos.collect_ros_observations = enabled;
                }
                config.chronos.core.validate()?;
                if state.runtime.is_some() {
                    ensure!(
                        config == state.config,
                        "Chronos config cannot change after initialization; restart the process"
                    );
                } else {
                    robonix_chronos_clock::configure_reference(
                        config.chronos.core.reference.clone(),
                    )?;
                    let clone = config.clone();
                    state.runtime = Some(Arc::new(
                        tokio::task::spawn_blocking(move || {
                            QualityRuntime::start(Chronos::new(clone)?)
                        })
                        .await
                        .context("join Chronos initialization")??,
                    ));
                    state.config = config;
                }
                (atlas_pb::LifecycleState::StateInactive, "inactive")
            }
            1 => {
                let runtime = state
                    .runtime
                    .clone()
                    .context("initialize Chronos before activation")?;
                runtime.latest_quality_status()?;
                ensure!(
                    state
                        .publisher
                        .as_ref()
                        .is_none_or(|worker| !worker.is_finished()),
                    "Chronos publication worker has stopped; restart the process"
                );
                if state.publisher.is_none() {
                    ensure!(
                        state.config.chronos.core.reference.kind
                            == crate::ReferenceSourceKind::LinuxSystem,
                        "ROS reference input is not available in this version"
                    );
                    ensure!(
                        !state.config.chronos.collect_observations,
                        "automatic collection is not available in this version"
                    );
                    let quality = self.quality.clone();
                    let reference = self.reference.clone();
                    state.publisher = Some(
                        tokio::task::spawn_blocking(move || {
                            PublicationWorker::start(runtime, quality, reference)
                        })
                        .await
                        .context("join Chronos activation")??,
                    );
                    state.session = Some(watch::channel(false).0);
                }
                (atlas_pb::LifecycleState::StateActive, "active")
            }
            2 => {
                ensure!(
                    state.runtime.is_some(),
                    "initialize Chronos before deactivation"
                );
                self.end_streams(&mut state);
                self.stop_publication(&mut state).await?;
                (atlas_pb::LifecycleState::StateInactive, "inactive")
            }
            3 => {
                self.end_streams(&mut state);
                state.terminated = true;
                let publisher = state.publisher.take();
                let runtime = state.runtime.take();
                self.shutdown.send_replace(true);
                let workers_result =
                    tokio::task::spawn_blocking(move || stop_workers(publisher, runtime))
                        .await
                        .context("join Chronos shutdown")?;
                workers_result?;
                (atlas_pb::LifecycleState::StateTerminated, "terminated")
            }
            other => anyhow::bail!("unknown lifecycle command code {other}"),
        };
        let mut atlas = self.atlas.clone();
        let reported = tokio::time::timeout(
            Duration::from_secs(5),
            atlas.set_lifecycle_state(&self.provider_id, target, ""),
        )
        .await
        .context("Chronos lifecycle report timed out")
        .and_then(|result| result);
        if let Err(error) = reported {
            self.end_streams(&mut state);
            if let Err(cleanup) = self.stop_publication(&mut state).await {
                return Err(error).context(format!("Chronos rollback cleanup failed: {cleanup:#}"));
            }
            return Err(error).context("publish Chronos lifecycle state");
        }
        Ok(label)
    }

    /// Subscribe while holding the lifecycle lock so activation cannot race cache access.
    async fn quality_stream(&self) -> Result<SnapshotStream<wire::ClockQualitySnapshot>, Status> {
        let state = self.state.lock().await;
        if state.publisher.is_none() || state.session.is_none() || state.terminated {
            return Err(Status::unavailable("Chronos is inactive"));
        }
        Ok(Box::pin(SessionStream::new(
            self.quality.subscribe(),
            state.session.as_ref().expect("active session").subscribe(),
        )))
    }
}

/// Always attempt both joins, even when publication has failed.
fn stop_workers(
    publisher: Option<PublicationWorker>,
    runtime: Option<Arc<QualityRuntime>>,
) -> Result<()> {
    let publication_result = publisher.map_or(Ok(()), PublicationWorker::stop);
    let runtime_result = runtime.map_or(Ok(()), |runtime| {
        Arc::try_unwrap(runtime)
            .map_err(|_| anyhow::anyhow!("Chronos runtime still has an owner"))?
            .stop()
            .map_err(anyhow::Error::from)
    });
    let errors: Vec<_> = [publication_result, runtime_result]
        .into_iter()
        .filter_map(Result::err)
        .map(|error| format!("{error:#}"))
        .collect();
    if errors.is_empty() {
        Ok(())
    } else {
        Err(anyhow::anyhow!(errors.join("; ")))
    }
}

/// A durable per-activation stop signal cannot be overwritten by new snapshots.
struct SessionStream<T> {
    snapshots: WatchStream<Option<Result<T, Status>>>,
    stopped: WatchStream<bool>,
    ended: bool,
}

impl<T: Clone + Send + Sync + 'static> SessionStream<T> {
    /// Start from cached evidence and the cancellation state of this activation.
    fn new(
        snapshots: watch::Receiver<Option<Result<T, Status>>>,
        stopped: watch::Receiver<bool>,
    ) -> Self {
        Self {
            snapshots: WatchStream::new(snapshots),
            stopped: WatchStream::new(stopped),
            ended: false,
        }
    }
}

impl<T: Clone + Send + Sync + Unpin + 'static> Stream for SessionStream<T> {
    type Item = Result<T, Status>;

    /// Prioritize durable cancellation and end after the first terminal error.
    fn poll_next(self: Pin<&mut Self>, cx: &mut TaskContext<'_>) -> Poll<Option<Self::Item>> {
        let this = self.get_mut();
        if this.ended {
            return Poll::Ready(None);
        }
        loop {
            match Pin::new(&mut this.stopped).poll_next(cx) {
                Poll::Ready(Some(false)) => continue,
                Poll::Ready(Some(true) | None) => {
                    this.ended = true;
                    return Poll::Ready(Some(Err(Status::unavailable("Chronos is inactive"))));
                }
                Poll::Pending => break,
            }
        }
        loop {
            match Pin::new(&mut this.snapshots).poll_next(cx) {
                Poll::Ready(Some(None)) => continue,
                Poll::Ready(Some(Some(item))) => {
                    this.ended = item.is_err();
                    return Poll::Ready(Some(item));
                }
                other => return other.map(|_| None),
            }
        }
    }
}

#[tonic::async_trait]
impl contracts::robonix_lifecycle_driver_server::RobonixLifecycleDriver for ChronosService {
    /// Keep callback errors in the shared Driver envelope, never claim successful activation.
    async fn driver(
        &self,
        request: Request<DriverRequest>,
    ) -> Result<Response<DriverResponse>, Status> {
        let response = match self.transition(request.into_inner()).await {
            Ok(state) => DriverResponse {
                ok: true,
                state: state.into(),
                error: String::new(),
            },
            Err(error) => DriverResponse {
                ok: false,
                state: "error".into(),
                error: format!("{error:#}"),
            },
        };
        Ok(Response::new(response))
    }
}

#[tonic::async_trait]
impl contracts::robonix_system_chronos_quality_server::RobonixSystemChronosQuality
    for ChronosService
{
    type QualityStream = SnapshotStream<wire::ClockQualitySnapshot>;
    async fn quality(
        &self,
        _request: Request<()>,
    ) -> Result<Response<Self::QualityStream>, Status> {
        Ok(Response::new(self.quality_stream().await?))
    }
}

#[tonic::async_trait]
impl contracts::robonix_system_chronos_reference_time_server::RobonixSystemChronosReferenceTime
    for ChronosService
{
    type ReferenceTimeStream = SnapshotStream<wire::ReferenceQuality>;

    async fn reference_time(
        &self,
        _request: Request<()>,
    ) -> Result<Response<Self::ReferenceTimeStream>, Status> {
        let state = self.state.lock().await;
        if state.publisher.is_none() || state.session.is_none() || state.terminated {
            return Err(Status::unavailable("Chronos is inactive"));
        }
        let stream = SessionStream::new(
            self.reference.subscribe(),
            state.session.as_ref().expect("active session").subscribe(),
        );
        Ok(Response::new(Box::pin(stream)))
    }
}
