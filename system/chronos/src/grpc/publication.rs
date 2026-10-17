// SPDX-License-Identifier: MulanPSL-2.0

use std::sync::{Arc, mpsc};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use tokio::sync::watch;
use tonic::Status;

use super::{encoding, pb::chronos as wire};
use crate::snapshot::ClockModelsSnapshot;
use crate::{ChronosRuntimeError, QualityRuntime};

pub(super) type QualitySender = watch::Sender<Option<Result<wire::ClockQualitySnapshot, Status>>>;
pub(super) type ModelsSender = watch::Sender<Option<Result<wire::ClockModelsSnapshot, Status>>>;
pub(super) type ReferenceSender = watch::Sender<Option<Result<wire::ReferenceQuality, Status>>>;

/// One bounded latest-value cache per stream; network writes never hold core locks.
pub(super) struct PublicationWorker {
    stop: mpsc::Sender<()>,
    thread: Option<JoinHandle<Result<(), ChronosRuntimeError>>>,
}

impl PublicationWorker {
    /// Cache initial snapshots synchronously, then poll the sole core owner.
    pub fn start(
        runtime: Arc<QualityRuntime>,
        quality: QualitySender,
        models: ModelsSender,
        reference: ReferenceSender,
    ) -> Result<Self> {
        let interval = Duration::from_nanos(runtime.config()?.quality.refresh_interval_ns);
        let (initial_quality, initial_models) = runtime.publication_snapshots()?;
        quality.send_replace(Some(Ok(encoding::quality(initial_quality))));
        let initial_models = initial_models.map(encoding::models);
        models.send_replace(initial_models.clone().map(Ok));
        publish_reference(&runtime, &reference);
        let (stop, receiver) = mpsc::channel();
        let thread = thread::Builder::new()
            .name("chronos-publication".into())
            .spawn(move || {
                let result = publish_loop(
                    runtime,
                    &quality,
                    &models,
                    &reference,
                    initial_models,
                    receiver,
                    interval,
                );
                if let Err(error) = &result {
                    robonix_scribe::error!("Chronos publication stopped: {error}");
                    quality.send_replace(Some(Err(Status::internal(error.to_string()))));
                    models.send_replace(Some(Err(Status::internal(error.to_string()))));
                    reference.send_replace(Some(Err(Status::internal(error.to_string()))));
                }
                result
            })
            .context("start Chronos publication worker")?;
        Ok(Self {
            stop,
            thread: Some(thread),
        })
    }

    /// Wake and join at most once, reporting worker failure to lifecycle callers.
    pub fn stop(mut self) -> Result<()> {
        self.join()
    }

    pub fn is_finished(&self) -> bool {
        self.thread.as_ref().is_none_or(JoinHandle::is_finished)
    }

    /// Sender disconnection also stops the worker; joining releases its runtime Arc.
    fn join(&mut self) -> Result<()> {
        let _ = self.stop.send(());
        if let Some(thread) = self.thread.take() {
            thread
                .join()
                .map_err(|_| anyhow::anyhow!("Chronos publication worker panicked"))??;
        }
        Ok(())
    }
}

impl Drop for PublicationWorker {
    fn drop(&mut self) {
        if let Err(error) = self.join() {
            robonix_scribe::error!("Chronos publication cleanup failed: {error:#}");
        }
    }
}

/// Publish periodic quality and changed models, with a ten-second model heartbeat.
fn publish_loop(
    runtime: Arc<QualityRuntime>,
    quality: &QualitySender,
    models: &ModelsSender,
    reference: &ReferenceSender,
    mut previous: Option<wire::ClockModelsSnapshot>,
    stop: mpsc::Receiver<()>,
    interval: Duration,
) -> Result<(), ChronosRuntimeError> {
    let mut last_model = Instant::now();
    let mut last_quality = Instant::now();
    while matches!(
        stop.recv_timeout(interval.min(Duration::from_millis(100))),
        Err(mpsc::RecvTimeoutError::Timeout)
    ) {
        publish_reference(&runtime, reference);
        let now = Instant::now();
        if now.duration_since(last_quality) < interval {
            continue;
        }
        last_quality = now;
        let (quality_snapshot, model_snapshot) = runtime.publication_snapshots()?;
        quality.send_replace(Some(Ok(encoding::quality(quality_snapshot))));
        publish_models(
            model_snapshot,
            models,
            &mut previous,
            &mut last_model,
            Instant::now(),
        );
    }
    Ok(())
}

fn publish_reference(runtime: &QualityRuntime, sender: &ReferenceSender) {
    let reading = runtime
        .reference_time()
        .map(|value| encoding::reference(crate::snapshot::ReferenceQualitySnapshot::from(&value)))
        .map_err(|error| Status::unavailable(error.to_string()));
    sender.send_replace(Some(reading));
}

/// Wait for real evidence, then publish changed parameters or a cadence-limited heartbeat.
fn publish_models(
    snapshot: Option<ClockModelsSnapshot>,
    models: &ModelsSender,
    previous: &mut Option<wire::ClockModelsSnapshot>,
    last_model: &mut Instant,
    now: Instant,
) {
    let Some(snapshot) = snapshot else { return };
    let snapshot = encoding::models(snapshot);
    let changed = previous.as_ref().is_none_or(|old| {
        old.reference_generation != snapshot.reference_generation || old.domains != snapshot.domains
    });
    if changed || now.duration_since(*last_model) >= Duration::from_secs(10) {
        models.send_replace(Some(Ok(snapshot.clone())));
        *previous = Some(snapshot);
        *last_model = now;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::VirtualSteadyTime;

    #[test]
    /// Initial failure waits, recovery sends once, timestamps alone do not trigger model changes.
    fn model_publication_waits_recovers_and_heartbeats() {
        let (models, mut receiver) = watch::channel(None);
        let start = Instant::now();
        let mut last = start;
        let mut previous = None;
        publish_models(None, &models, &mut previous, &mut last, start);
        assert!(receiver.borrow().is_none());
        let mut snapshot = ClockModelsSnapshot {
            published_steady_time: VirtualSteadyTime { timestamp_ns: 1 },
            reference_generation: 1,
            domains: vec![],
        };
        publish_models(
            Some(snapshot.clone()),
            &models,
            &mut previous,
            &mut last,
            start,
        );
        assert_eq!(
            receiver
                .borrow_and_update()
                .as_ref()
                .unwrap()
                .as_ref()
                .unwrap()
                .published_steady_ns,
            1
        );
        snapshot.published_steady_time.timestamp_ns = 2;
        publish_models(
            Some(snapshot.clone()),
            &models,
            &mut previous,
            &mut last,
            start + Duration::from_secs(1),
        );
        assert!(!receiver.has_changed().unwrap());
        publish_models(
            Some(snapshot.clone()),
            &models,
            &mut previous,
            &mut last,
            start + Duration::from_secs(10),
        );
        assert_eq!(
            receiver
                .borrow_and_update()
                .as_ref()
                .unwrap()
                .as_ref()
                .unwrap()
                .published_steady_ns,
            2
        );
        snapshot.reference_generation = 2;
        publish_models(
            Some(snapshot),
            &models,
            &mut previous,
            &mut last,
            start + Duration::from_secs(11),
        );
        assert!(receiver.has_changed().unwrap());
    }
}
