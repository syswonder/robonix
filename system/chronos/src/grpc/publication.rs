// SPDX-License-Identifier: MulanPSL-2.0

use std::sync::{Arc, mpsc};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use tokio::sync::watch;
use tonic::Status;

use super::{encoding, pb::chronos as wire};
use crate::{ChronosRuntimeError, QualityRuntime};

pub(super) type QualitySender = watch::Sender<Option<Result<wire::ClockQualitySnapshot, Status>>>;
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
        reference: ReferenceSender,
    ) -> Result<Self> {
        let interval = Duration::from_nanos(runtime.config()?.quality.refresh_interval_ns);
        let initial_quality = runtime.quality_snapshot()?;
        quality.send_replace(Some(Ok(encoding::quality(initial_quality))));
        publish_reference(&runtime, &reference);
        let (stop, receiver) = mpsc::channel();
        let thread = thread::Builder::new()
            .name("chronos-publication".into())
            .spawn(move || {
                let result = publish_loop(runtime, &quality, &reference, receiver, interval);
                if let Err(error) = &result {
                    robonix_scribe::error!("Chronos publication stopped: {error}");
                    quality.send_replace(Some(Err(Status::internal(error.to_string()))));
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

/// Publish quality at its cadence while maintaining reference stream heartbeats.
fn publish_loop(
    runtime: Arc<QualityRuntime>,
    quality: &QualitySender,
    reference: &ReferenceSender,
    stop: mpsc::Receiver<()>,
    interval: Duration,
) -> Result<(), ChronosRuntimeError> {
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
        let quality_snapshot = runtime.quality_snapshot()?;
        quality.send_replace(Some(Ok(encoding::quality(quality_snapshot))));
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
