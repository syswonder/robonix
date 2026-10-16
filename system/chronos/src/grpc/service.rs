// SPDX-License-Identifier: MulanPSL-2.0

use std::pin::Pin;
use std::sync::Arc;
use std::task::{Context as TaskContext, Poll};
use std::time::Duration;

use anyhow::{Context, Result, ensure};
use robonix_atlas::{client::AtlasClient, pb as atlas_pb};
use tokio::sync::{Mutex, Semaphore, oneshot, watch};
use tokio_stream::{Stream, wrappers::WatchStream};
use tonic::{Request, Response, Status};

use super::collector::Collector;
use super::config::parse_manifest;
use super::observation::{self, ObservationJob, ObservationWorker};
use super::pb::{
    chronos as wire, contracts,
    lifecycle::{DriverRequest, DriverResponse},
};
use super::publication::{ModelsSender, PublicationWorker, QualitySender, ReferenceSender};
use super::reference_time::ReferenceReader;
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
    models: ModelsSender,
    reference: ReferenceSender,
    shutdown: watch::Sender<bool>,
    observation_slots: Arc<Semaphore>,
    lifecycle_slots: Arc<Semaphore>,
}

struct State {
    reference_reader: Option<ReferenceReader>,
    collector: Option<Collector>,
    config: ChronosConfig,
    runtime: Option<Arc<QualityRuntime>>,
    publisher: Option<PublicationWorker>,
    observations: Option<ObservationWorker>,
    session: Option<watch::Sender<bool>>,
    terminated: bool,
}

impl ChronosService {
    /// Create an inactive service; Driver INIT constructs the sole runtime.
    pub fn new(config: ChronosConfig, atlas: AtlasClient, provider_id: String) -> Self {
        let (quality, _) = watch::channel(None);
        let (models, _) = watch::channel(None);
        let (reference, _) = watch::channel(None);
        let (shutdown, _) = watch::channel(false);
        Self {
            state: Arc::new(Mutex::new(State {
                reference_reader: None,
                collector: None,
                config,
                runtime: None,
                publisher: None,
                observations: None,
                session: None,
                terminated: false,
            })),
            atlas,
            provider_id,
            quality,
            models,
            reference,
            shutdown,
            observation_slots: Arc::new(Semaphore::new(64)),
            lifecycle_slots: Arc::new(Semaphore::new(LIFECYCLE_CAPACITY)),
        }
    }

    pub fn shutdown_receiver(&self) -> watch::Receiver<bool> {
        self.shutdown.subscribe()
    }

    pub async fn collector_failed(&self) -> bool {
        let state = self.state.lock().await;
        state.collector.as_ref().is_some_and(Collector::is_finished)
            || state
                .reference_reader
                .as_ref()
                .is_some_and(ReferenceReader::is_finished)
    }

    /// Stop streams and both workers, retaining any cleanup error for the caller.
    pub async fn close(&self) -> Result<()> {
        let mut state = self.state.lock().await;
        state.terminated = true;
        self.end_streams(&mut state);
        let collector_result = self.stop_collector(&mut state).await;
        let publisher = state.publisher.take();
        let observations = state.observations.take();
        let runtime = state.runtime.take();
        self.shutdown.send_replace(true);
        let workers_result =
            tokio::task::spawn_blocking(move || stop_workers(publisher, observations, runtime))
                .await
                .context("join Chronos cleanup")?;
        combine_cleanup(collector_result, workers_result)
    }

    /// End existing streams; reactivation replaces these terminal cache values.
    fn end_streams(&self, state: &mut State) {
        if let Some(session) = state.session.take() {
            session.send_replace(true);
        }
        self.quality
            .send_replace(Some(Err(Status::unavailable("Chronos is inactive"))));
        self.models
            .send_replace(Some(Err(Status::unavailable("Chronos is inactive"))));
        self.reference
            .send_replace(Some(Err(Status::unavailable("Chronos is inactive"))));
    }

    /// Finish already-started inputs before a stopped activation can be replaced.
    async fn quiesce_observations(&self, state: &State) -> Result<()> {
        if let Some(worker) = &state.observations {
            tokio::task::spawn_blocking(worker.quiesce())
                .await
                .context("join Chronos observation barrier")??;
        }
        Ok(())
    }

    /// Always remove publication after cancellation, even if input synchronization failed.
    async fn stop_publication(&self, state: &mut State) -> Result<()> {
        let collector_result = self.stop_collector(state).await;
        let input_result = self.quiesce_observations(state).await;
        let publication_result = match state.publisher.take() {
            Some(publisher) => tokio::task::spawn_blocking(move || publisher.stop())
                .await
                .context("join Chronos deactivation")
                .and_then(|result| result),
            None => Ok(()),
        };
        let workers_result = match (input_result, publication_result) {
            (Err(input), Err(publication)) => {
                Err(input.context(format!("publication cleanup also failed: {publication:#}")))
            }
            (input, publication) => input.and(publication),
        };
        combine_cleanup(collector_result, workers_result)
    }

    /// Release subscriptions and their child before allowing a new activation.
    async fn stop_collector(&self, state: &mut State) -> Result<()> {
        let collector = match state.collector.take() {
            Some(collector) => collector.stop().await,
            None => Ok(()),
        };
        let reference = match state.reference_reader.take() {
            Some(reader) => reader.stop().await,
            None => Ok(()),
        };
        combine_cleanup(collector, reference)
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
                if state.observations.is_none() {
                    let runtime = state.runtime.clone().expect("initialized runtime");
                    state.observations = Some(
                        tokio::task::spawn_blocking(move || ObservationWorker::start(runtime))
                            .await
                            .context("join Chronos observation initialization")??,
                    );
                }
                (atlas_pb::LifecycleState::StateInactive, "inactive")
            }
            1 => {
                ensure!(
                    state
                        .collector
                        .as_ref()
                        .is_none_or(|collector| !collector.is_finished()),
                    "Chronos collector has stopped; restart the process"
                );
                ensure!(
                    state
                        .observations
                        .as_ref()
                        .is_some_and(|worker| !worker.is_finished()),
                    "Chronos observation worker has stopped; restart the process"
                );
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
                    if state.config.chronos.core.reference.kind
                        == crate::ReferenceSourceKind::RosTime
                    {
                        state.reference_reader = Some(
                            ReferenceReader::start(&state.config.chronos.reference_reader_command)
                                .await?,
                        );
                    }
                    let quality = self.quality.clone();
                    let models = self.models.clone();
                    let reference = self.reference.clone();
                    state.publisher = Some(
                        tokio::task::spawn_blocking(move || {
                            PublicationWorker::start(runtime, quality, models, reference)
                        })
                        .await
                        .context("join Chronos activation")??,
                    );
                    state.session = Some(watch::channel(false).0);
                    if state.config.chronos.collect_observations
                        && !state.config.chronos.collect_ros_observations
                    {
                        robonix_scribe::info!(
                            "Chronos ROS observation collection is disabled; configure collect_ros_observations: true to enable it"
                        );
                    }
                    if state
                        .config
                        .chronos
                        .core
                        .domains
                        .values()
                        .any(|domain| state.config.chronos.collects_domain(domain))
                    {
                        let started = Collector::start(
                            &state.config,
                            self.atlas.clone(),
                            &self.provider_id,
                            state.runtime.clone().expect("initialized runtime"),
                            state
                                .observations
                                .as_ref()
                                .expect("initialized observation worker"),
                            state.session.as_ref().expect("active session").subscribe(),
                        )
                        .await;
                        match started {
                            Ok(collector) => state.collector = Some(collector),
                            Err(error) => {
                                self.end_streams(&mut state);
                                let cleanup = self.stop_publication(&mut state).await;
                                cleanup
                                    .context(format!("collector activation failed: {error:#}"))?;
                                return Err(error);
                            }
                        }
                    }
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
                let collector_result = self.stop_collector(&mut state).await;
                let publisher = state.publisher.take();
                let observations = state.observations.take();
                let runtime = state.runtime.take();
                self.shutdown.send_replace(true);
                let workers_result = tokio::task::spawn_blocking(move || {
                    stop_workers(publisher, observations, runtime)
                })
                .await
                .context("join Chronos shutdown")?;
                combine_cleanup(collector_result, workers_result)?;
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

    /// An initial missing report waits for real model evidence instead of synthesizing time.
    async fn models_stream(&self) -> Result<SnapshotStream<wire::ClockModelsSnapshot>, Status> {
        let state = self.state.lock().await;
        if state.publisher.is_none() || state.session.is_none() || state.terminated {
            return Err(Status::unavailable("Chronos is inactive"));
        }
        Ok(Box::pin(SessionStream::new(
            self.models.subscribe(),
            state.session.as_ref().expect("active session").subscribe(),
        )))
    }
}

/// Retain both helper and runtime cleanup errors while attempting every cleanup.
pub(super) fn combine_cleanup(helper: Result<()>, workers: Result<()>) -> Result<()> {
    match (helper, workers) {
        (Err(helper), Err(workers)) => {
            Err(helper.context(format!("worker cleanup also failed: {workers:#}")))
        }
        (helper, workers) => helper.and(workers),
    }
}

/// Always attempt both joins, even when publication has failed.
fn stop_workers(
    publisher: Option<PublicationWorker>,
    observations: Option<ObservationWorker>,
    runtime: Option<Arc<QualityRuntime>>,
) -> Result<()> {
    let publication_result = publisher.map_or(Ok(()), PublicationWorker::stop);
    let observation_result = observations.map_or(Ok(()), ObservationWorker::stop);
    let runtime_result = runtime.map_or(Ok(()), |runtime| {
        Arc::try_unwrap(runtime)
            .map_err(|_| anyhow::anyhow!("Chronos runtime still has an owner"))?
            .stop()
            .map_err(anyhow::Error::from)
    });
    let errors: Vec<_> = [publication_result, observation_result, runtime_result]
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

#[tonic::async_trait]
impl contracts::robonix_system_chronos_observe_device_timestamp_server::RobonixSystemChronosObserveDeviceTimestamp
    for ChronosService
{
    /// Sample the local reference before the fitting queue and wait for core acceptance.
    async fn observe_device_timestamp(
        &self,
        request: Request<wire::ObserveDeviceTimestampRequest>,
    ) -> Result<Response<wire::ObserveDeviceTimestampResponse>, Status> {
        let request = request.into_inner();
        observation::validate(&request)?;
        let _slot = self.observation_slots.clone().try_acquire_owned()
            .map_err(|_| Status::resource_exhausted("Chronos has 64 outstanding observations"))?;
        let state = self.state.try_lock()
            .map_err(|_| Status::unavailable("Chronos lifecycle is changing"))?;
        if state.publisher.is_none() || state.session.is_none() || state.terminated {
            return Err(Status::unavailable("Chronos is inactive"));
        }
        if !state.config.chronos.core.domains.get(&request.domain_id)
            .is_some_and(|domain| matches!(domain.strategy, crate::DomainStrategyConfig::AffineModel { .. })) {
            return Err(Status::invalid_argument("domain must be a configured affine time domain"));
        }
        if state.config.chronos.core.domains.get(&request.domain_id).is_some_and(|domain| {
            matches!(&domain.strategy, crate::DomainStrategyConfig::AffineModel { observation, .. } if observation.query.is_some())
        }) {
            return Err(Status::failed_precondition("hardware query domain requires a query clock pair from its configured adapter"));
        }
        let snapshot = state.runtime.as_ref().expect("active runtime")
            .try_observation_snapshot()
            .map_err(|error| Status::unavailable(error.to_string()))?
            .ok_or_else(|| Status::resource_exhausted("Chronos clock owner is busy; submit a fresh observation"))?;
        let (reply, receiver) = oneshot::channel();
        state.observations.as_ref().expect("initialized observation worker").submit(ObservationJob {
            request, snapshot,
            cancelled: state.session.as_ref().expect("active session").subscribe(), reply,
        })?;
        drop(state);
        let response = receiver.await.map_err(|_| Status::unavailable("Chronos observation worker stopped"))??;
        Ok(Response::new(response))
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

#[tonic::async_trait]
impl
    contracts::robonix_system_chronos_conversion_models_server::RobonixSystemChronosConversionModels
    for ChronosService
{
    type ConversionModelsStream = SnapshotStream<wire::ClockModelsSnapshot>;
    async fn conversion_models(
        &self,
        _request: Request<()>,
    ) -> Result<Response<Self::ConversionModelsStream>, Status> {
        Ok(Response::new(self.models_stream().await?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio_stream::StreamExt;

    #[tokio::test]
    /// Driver INIT overrides each collection switch and retains omitted switches when replacing core.
    async fn init_preserves_and_overrides_collection_switches() {
        let reserved = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let address = reserved.local_addr().unwrap();
        drop(reserved);
        let server = tokio::spawn(robonix_atlas::service::serve_atlas(
            Arc::new(robonix_atlas::service::AtlasRegistry::default()),
            address,
        ));
        let mut atlas =
            AtlasClient::connect_with_retry(address.to_string(), 50, Duration::from_millis(10))
                .await
                .unwrap();
        for (index, initial, manifest, master, ros) in [
            (
                0,
                true,
                r#"{"core":{"reference":{"kind":"linux_system"}}}"#,
                true,
                true,
            ),
            (
                1,
                true,
                r#"{"collect_ros_observations":false,"core":{"reference":{"kind":"linux_system"}}}"#,
                true,
                false,
            ),
            (2, true, r#"{"collect_observations":false}"#, false, true),
            (
                3,
                false,
                r#"{"collect_observations":true,"collect_ros_observations":true}"#,
                true,
                true,
            ),
        ] {
            let id = format!("collection-init-{index}");
            atlas
                .register_service(&id, "robonix/system/chronos", "")
                .await
                .unwrap();
            let mut config = super::super::tests::test_config();
            config.chronos.collect_observations = initial;
            config.chronos.collect_ros_observations = initial;
            let svc = ChronosService::new(config, atlas.clone(), id);
            svc.transition(DriverRequest {
                command: 0,
                config_json: manifest.into(),
            })
            .await
            .unwrap();
            let state = svc.state.lock().await;
            assert_eq!(state.config.chronos.collect_observations, master);
            assert_eq!(state.config.chronos.collect_ros_observations, ros);
            drop(state);
            svc.transition(DriverRequest {
                command: 0,
                config_json: "{}".into(),
            })
            .await
            .unwrap();
            assert!(
                svc.transition(DriverRequest {
                    command: 0,
                    config_json: serde_json::json!({"collect_ros_observations": !ros}).to_string(),
                })
                .await
                .unwrap_err()
                .to_string()
                .contains("config cannot change")
            );
            svc.close().await.unwrap();
        }
        server.abort();
    }

    #[tokio::test]
    /// Cancelled callers retain bounded admission until their transactions finish, then release it.
    async fn cancelled_driver_calls_remain_bounded_and_finish() {
        let reserved = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let address = reserved.local_addr().unwrap();
        drop(reserved);
        let server = tokio::spawn(robonix_atlas::service::serve_atlas(
            Arc::new(robonix_atlas::service::AtlasRegistry::default()),
            address,
        ));
        let mut atlas =
            AtlasClient::connect_with_retry(address.to_string(), 50, Duration::from_millis(10))
                .await
                .unwrap();
        atlas
            .register_service("cancel-test", "robonix/system/chronos", "")
            .await
            .unwrap();
        let svc = ChronosService::new(
            super::super::tests::test_config(),
            atlas,
            "cancel-test".into(),
        );
        let locked = svc.state.lock().await;
        let mut callers = Vec::new();
        for _ in 0..LIFECYCLE_CAPACITY {
            let clone = svc.clone();
            callers.push(tokio::spawn(async move {
                clone
                    .transition(DriverRequest {
                        command: 0,
                        config_json: "{}".into(),
                    })
                    .await
            }));
        }
        tokio::time::timeout(Duration::from_secs(2), async {
            while svc.lifecycle_slots.available_permits() != 0 {
                tokio::task::yield_now().await;
            }
        })
        .await
        .unwrap();
        for caller in callers {
            caller.abort();
            let _ = caller.await;
        }
        let response = tokio::time::timeout(
            Duration::from_secs(1),
            contracts::robonix_lifecycle_driver_server::RobonixLifecycleDriver::driver(
                &svc,
                Request::new(DriverRequest {
                    command: 0,
                    config_json: "{}".into(),
                }),
            ),
        )
        .await
        .unwrap()
        .unwrap()
        .into_inner();
        assert!(!response.ok);
        assert!(response.error.contains("lifecycle request limit reached"));
        drop(locked);
        tokio::time::timeout(Duration::from_secs(2), async {
            while svc.lifecycle_slots.available_permits() != LIFECYCLE_CAPACITY {
                tokio::task::yield_now().await;
            }
        })
        .await
        .unwrap();
        assert!(svc.state.lock().await.runtime.is_some());
        svc.transition(DriverRequest {
            command: 0,
            config_json: "{}".into(),
        })
        .await
        .unwrap();
        svc.close().await.unwrap();
        let state = svc.state.lock().await;
        assert!(
            state.runtime.is_none() && state.observations.is_none() && state.collector.is_none()
        );
        server.abort();
    }

    #[tokio::test]
    /// An unpolled old connection ends even if a new activation overwrites its cache.
    async fn cancellation_survives_snapshot_overwrite() {
        let (snapshots, receiver) = watch::channel(Some(Ok(1_u64)));
        let (stop, stopped) = watch::channel(false);
        let mut stream = SessionStream::new(receiver, stopped);
        stop.send_replace(true);
        snapshots.send_replace(Some(Ok(2)));
        assert_eq!(
            stream.next().await.unwrap().unwrap_err().code(),
            tonic::Code::Unavailable
        );
        assert!(stream.next().await.is_none());
    }

    #[tokio::test]
    /// Input refuses lifecycle contention and exhausted admission without delayed sampling.
    async fn observation_admission_is_immediate() {
        use contracts::robonix_system_chronos_observe_device_timestamp_server::RobonixSystemChronosObserveDeviceTimestamp;
        let reserved = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let address = reserved.local_addr().unwrap();
        drop(reserved);
        let server = tokio::spawn(robonix_atlas::service::serve_atlas(
            Arc::new(robonix_atlas::service::AtlasRegistry::default()),
            address,
        ));
        let atlas =
            AtlasClient::connect_with_retry(address.to_string(), 50, Duration::from_millis(10))
                .await
                .unwrap();
        let svc = ChronosService::new(
            super::super::tests::test_config(),
            atlas,
            "admission-test".into(),
        );
        let request = || {
            Request::new(wire::ObserveDeviceTimestampRequest {
                domain_id: "device".into(),
                session_id: "a".into(),
                device_generation: 1,
                sequence_id: 1,
                device_timestamp_ns: 1000,
            })
        };
        let locked = svc.state.lock().await;
        assert_eq!(
            tokio::time::timeout(
                Duration::from_millis(100),
                svc.observe_device_timestamp(request())
            )
            .await
            .unwrap()
            .unwrap_err()
            .code(),
            tonic::Code::Unavailable
        );
        drop(locked);
        let slots = svc.observation_slots.acquire_many(64).await.unwrap();
        assert_eq!(
            svc.observe_device_timestamp(request())
                .await
                .unwrap_err()
                .code(),
            tonic::Code::ResourceExhausted
        );
        drop(slots);
        svc.close().await.unwrap();
        server.abort();
    }
}
