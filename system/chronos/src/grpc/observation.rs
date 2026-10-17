// SPDX-License-Identifier: MulanPSL-2.0

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
    mpsc,
};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use anyhow::{Context, Result};
use tokio::sync::{oneshot, watch};
use tonic::Status;

use super::pb::chronos::{ObserveDeviceTimestampRequest, ObserveDeviceTimestampResponse};
use crate::{DomainStrategyConfig, ObservationSnapshot, QualityRuntime};

const QUEUE_CAPACITY: usize = 64;
const SESSION_HISTORY_LIMIT: usize = 64;

pub(super) struct ObservationJob {
    pub request: ObserveDeviceTimestampRequest,
    pub snapshot: ObservationSnapshot,
    pub cancelled: watch::Receiver<bool>,
    pub reply: oneshot::Sender<Result<ObserveDeviceTimestampResponse, Status>>,
}

/// A bounded serial worker keeps fitting off Tokio and owns source identities.
pub(super) struct ObservationWorker {
    sender: mpsc::SyncSender<ObservationJob>,
    stopped: Arc<AtomicBool>,
    processing: Arc<Mutex<()>>,
    thread: Option<JoinHandle<()>>,
}

#[derive(Default)]
struct SourceIdentity {
    current: Option<(String, u64, u64)>,
    retired: BTreeSet<String>,
}

impl ObservationWorker {
    /// Share bounded submission with the owned collector; the worker retains shutdown ownership.
    pub fn sender(&self) -> mpsc::SyncSender<ObservationJob> {
        self.sender.clone()
    }
    /// Start one input worker for the existing runtime, retaining identities across activations.
    pub fn start(runtime: Arc<QualityRuntime>) -> Result<Self> {
        let config = runtime.config()?;
        let domains: BTreeSet<_> = config
            .domains
            .into_iter()
            .filter(|(_, domain)| {
                matches!(domain.strategy, DomainStrategyConfig::AffineModel { .. })
            })
            .map(|(id, _)| id)
            .collect();
        let (sender, receiver) = mpsc::sync_channel::<ObservationJob>(QUEUE_CAPACITY);
        let stopped = Arc::new(AtomicBool::new(false));
        let worker_stop = stopped.clone();
        let processing = Arc::new(Mutex::new(()));
        let worker_processing = processing.clone();
        let thread = thread::Builder::new()
            .name("chronos-observation".into())
            .spawn(move || {
                let mut identities = BTreeMap::new();
                while !worker_stop.load(Ordering::Acquire) {
                    let job = match receiver.recv_timeout(Duration::from_millis(20)) {
                        Ok(job) => job,
                        Err(mpsc::RecvTimeoutError::Timeout) => continue,
                        Err(mpsc::RecvTimeoutError::Disconnected) => break,
                    };
                    if job.reply.is_closed() {
                        continue;
                    }
                    let _processing = worker_processing
                        .lock()
                        .expect("observation processing lock");
                    if job.reply.is_closed() {
                        continue;
                    }
                    let result = if *job.cancelled.borrow() || worker_stop.load(Ordering::Acquire) {
                        Err(Status::unavailable("Chronos is inactive"))
                    } else if !domains.contains(&job.request.domain_id) {
                        Err(Status::invalid_argument(
                            "domain must be a configured affine time domain",
                        ))
                    } else {
                        apply(
                            &runtime,
                            identities.entry(job.request.domain_id.clone()).or_default(),
                            job.request,
                            job.snapshot,
                        )
                    };
                    let _ = job.reply.send(result);
                }
            })
            .context("start Chronos observation worker")?;
        Ok(Self {
            sender,
            stopped,
            processing,
            thread: Some(thread),
        })
    }

    /// Reject overload immediately; an RPC response means actual core acceptance.
    pub fn submit(&self, job: ObservationJob) -> Result<(), Status> {
        if self.stopped.load(Ordering::Acquire)
            || self.thread.as_ref().is_none_or(JoinHandle::is_finished)
        {
            return Err(Status::unavailable(
                "Chronos observation worker has stopped",
            ));
        }
        self.sender.try_send(job).map_err(|error| match error {
            mpsc::TrySendError::Full(_) => {
                Status::resource_exhausted("Chronos observation queue is full")
            }
            mpsc::TrySendError::Disconnected(_) => {
                Status::unavailable("Chronos observation worker has stopped")
            }
        })
    }

    /// Cancel pending inputs and join before releasing the shared runtime.
    pub fn stop(mut self) -> Result<()> {
        self.join()
    }

    pub fn is_finished(&self) -> bool {
        self.thread.as_ref().is_none_or(JoinHandle::is_finished)
    }

    /// Clone a barrier that waits for started inputs to finish after lifecycle cancellation.
    pub fn quiesce(&self) -> impl FnOnce() -> Result<()> + Send + 'static {
        let processing = self.processing.clone();
        move || {
            drop(
                processing
                    .lock()
                    .map_err(|_| anyhow::anyhow!("Chronos observation worker panicked"))?,
            );
            Ok(())
        }
    }

    /// The receiver wakes periodically so shutdown cannot wait on network clients.
    fn join(&mut self) -> Result<()> {
        self.stopped.store(true, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            thread
                .join()
                .map_err(|_| anyhow::anyhow!("Chronos observation worker panicked"))?;
        }
        Ok(())
    }
}

impl Drop for ObservationWorker {
    fn drop(&mut self) {
        if let Err(error) = self.join() {
            robonix_scribe::error!("Chronos observation cleanup failed: {error:#}");
        }
    }
}

/// Validate the public envelope before sampling or admitting bounded work.
pub(super) fn validate(request: &ObserveDeviceTimestampRequest) -> Result<(), Status> {
    crate::model::validate_domain_id(&request.domain_id).map_err(Status::invalid_argument)?;
    let session = &request.session_id;
    if session.is_empty() || session.len() > 128 || session.trim() != session {
        return Err(Status::invalid_argument(
            "session_id must contain 1..128 bytes without surrounding whitespace",
        ));
    }
    if request.device_generation == 0 || request.device_timestamp_ns < 0 {
        return Err(Status::invalid_argument(
            "device_generation must be positive and device_timestamp_ns non-negative",
        ));
    }
    Ok(())
}

/// Reject stale identities before reset, and keep the captured entrance time for fitting.
fn apply(
    runtime: &QualityRuntime,
    identity: &mut SourceIdentity,
    request: ObserveDeviceTimestampRequest,
    snapshot: ObservationSnapshot,
) -> Result<ObserveDeviceTimestampResponse, Status> {
    if identity.retired.contains(&request.session_id) {
        return Err(Status::failed_precondition("retired device session"));
    }
    let changed = identity
        .current
        .as_ref()
        .is_some_and(|(session, generation, _)| {
            session != &request.session_id || *generation != request.device_generation
        });
    if let Some((session, generation, sequence)) = &identity.current {
        if session == &request.session_id && request.device_generation < *generation {
            return Err(Status::failed_precondition("stale device generation"));
        }
        if !changed && request.sequence_id <= *sequence {
            return Err(Status::failed_precondition(
                "duplicate or out-of-order device sequence",
            ));
        }
    }
    if changed {
        let old_session = &identity.current.as_ref().unwrap().0;
        if old_session != &request.session_id && identity.retired.len() >= SESSION_HISTORY_LIMIT {
            return Err(Status::resource_exhausted(
                "device session history is full; restart Chronos",
            ));
        }
        runtime
            .reset_device_domain(&request.domain_id)
            .map_err(|error| Status::internal(error.to_string()))?;
        if old_session != &request.session_id {
            identity.retired.insert(old_session.clone());
        }
        identity.current = Some((
            request.session_id,
            request.device_generation,
            request.sequence_id,
        ));
        // Reset invalidates the ingress token. Never silently re-pair this queued
        // sample with a later clock read: the next fresh observation seeds the fit.
        return Ok(ObserveDeviceTimestampResponse {
            accepted: false,
            reason: "device identity changed; submit the next fresh observation".into(),
        });
    }
    identity.current = Some((
        request.session_id,
        request.device_generation,
        request.sequence_id,
    ));
    let result = runtime.observe_device_timestamp_at(
        &request.domain_id,
        request.sequence_id,
        request.device_timestamp_ns,
        snapshot,
    );
    match result {
        Ok(_) => Ok(ObserveDeviceTimestampResponse {
            accepted: true,
            reason: String::new(),
        }),
        Err(error) => Ok(ObserveDeviceTimestampResponse {
            accepted: false,
            reason: error.to_string(),
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Use the real configured runtime while exercising source identity separately from fitting.
    fn setup() -> QualityRuntime {
        QualityRuntime::start(crate::Chronos::new(super::super::tests::test_config()).unwrap())
            .unwrap()
    }

    fn request(session: &str, generation: u64, sequence: u64) -> ObserveDeviceTimestampRequest {
        ObserveDeviceTimestampRequest {
            domain_id: "device".into(),
            session_id: session.into(),
            device_generation: generation,
            sequence_id: sequence,
            device_timestamp_ns: 1000,
        }
    }

    #[test]
    /// Identity transitions reset exactly once; stale retries cannot reset the domain backwards.
    fn resets_reject_boundary_tokens_and_stale_sources() {
        let runtime = setup();
        let mut identity = SourceIdentity::default();
        assert!(
            apply(
                &runtime,
                &mut identity,
                request("a", 1, 1),
                runtime.snapshot().unwrap()
            )
            .unwrap()
            .accepted
        );
        assert_eq!(
            apply(
                &runtime,
                &mut identity,
                request("a", 1, 1),
                runtime.snapshot().unwrap()
            )
            .unwrap_err()
            .code(),
            tonic::Code::FailedPrecondition
        );
        let old = runtime.models_snapshot().unwrap().unwrap().domains[0].generation;
        assert!(
            !apply(
                &runtime,
                &mut identity,
                request("a", 2, 1),
                runtime.snapshot().unwrap()
            )
            .unwrap()
            .accepted
        );
        assert_eq!(
            runtime.models_snapshot().unwrap().unwrap().domains[0].generation,
            old + 1
        );
        assert_eq!(
            apply(
                &runtime,
                &mut identity,
                request("a", 1, 2),
                runtime.snapshot().unwrap()
            )
            .unwrap_err()
            .code(),
            tonic::Code::FailedPrecondition
        );
        assert!(
            !apply(
                &runtime,
                &mut identity,
                request("b", 1, 1),
                runtime.snapshot().unwrap()
            )
            .unwrap()
            .accepted
        );
        assert_eq!(
            apply(
                &runtime,
                &mut identity,
                request("a", 3, 3),
                runtime.snapshot().unwrap()
            )
            .unwrap_err()
            .code(),
            tonic::Code::FailedPrecondition
        );
        assert_eq!(
            runtime.models_snapshot().unwrap().unwrap().domains[0].generation,
            old + 2
        );
    }

    #[test]
    /// Malformed identities and timestamps fail before admission and unknown domains fail explicitly.
    fn invalid_observations_are_not_accepted() {
        let mut invalid = request("a", 0, 1);
        assert_eq!(
            validate(&invalid).unwrap_err().code(),
            tonic::Code::InvalidArgument
        );
        invalid.device_generation = 1;
        invalid.session_id = " ".into();
        assert!(validate(&invalid).is_err());
        invalid.session_id = "a".repeat(129);
        assert!(validate(&invalid).is_err());
        invalid.session_id = "a".into();
        invalid.device_timestamp_ns = -1;
        assert!(validate(&invalid).is_err());
    }

    #[tokio::test]
    /// A request abandoned while waiting for the processing gate cannot consume its sequence.
    async fn expired_queued_input_does_not_advance_identity() {
        let runtime = Arc::new(setup());
        let worker = ObservationWorker::start(runtime.clone()).unwrap();
        let (_cancel, cancelled) = watch::channel(false);
        let (reply, receiver) = oneshot::channel();
        let processing = worker.processing.lock().unwrap();
        worker
            .submit(ObservationJob {
                request: request("a", 1, 1),
                snapshot: runtime.snapshot().unwrap(),
                cancelled: cancelled.clone(),
                reply,
            })
            .unwrap();
        thread::sleep(Duration::from_millis(20));
        drop(receiver);
        drop(processing);
        let (reply, receiver) = oneshot::channel();
        worker
            .submit(ObservationJob {
                request: request("a", 1, 1),
                snapshot: runtime.snapshot().unwrap(),
                cancelled,
                reply,
            })
            .unwrap();
        let response = receiver.await.unwrap().unwrap();
        assert!(response.accepted, "{}", response.reason);
        worker.stop().unwrap();
    }

    #[tokio::test]
    /// Pending inputs from a deactivated session never mutate the retained runtime.
    async fn cancelled_inputs_do_not_advance_identity() {
        let runtime = Arc::new(setup());
        let worker = ObservationWorker::start(runtime.clone()).unwrap();
        let (cancel, cancelled) = watch::channel(true);
        let (reply, receiver) = oneshot::channel();
        worker
            .submit(ObservationJob {
                request: request("a", 1, 1),
                snapshot: runtime.snapshot().unwrap(),
                cancelled,
                reply,
            })
            .unwrap();
        assert_eq!(
            receiver.await.unwrap().unwrap_err().code(),
            tonic::Code::Unavailable
        );
        let (reply, receiver) = oneshot::channel();
        let mut unknown = request("a", 1, 1);
        unknown.domain_id = "unknown".into();
        cancel.send_replace(false);
        worker
            .submit(ObservationJob {
                request: unknown,
                snapshot: runtime.snapshot().unwrap(),
                cancelled: cancel.subscribe(),
                reply,
            })
            .unwrap();
        assert_eq!(
            receiver.await.unwrap().unwrap_err().code(),
            tonic::Code::InvalidArgument
        );
        worker.stop().unwrap();
        assert_eq!(
            runtime.models_snapshot().unwrap().unwrap().domains[0].generation,
            1
        );
    }

    #[test]
    /// Sampling-policy rejection is an explicit response and does not prevent later fresh input.
    fn policy_rejections_are_responses() {
        let mut config = super::super::tests::test_config();
        if let DomainStrategyConfig::AffineModel {
            observation,
            policy,
        } = &mut config
            .chronos
            .core
            .domains
            .get_mut("device")
            .unwrap()
            .strategy
        {
            observation.sampling_interval_ns = 50000000;
            policy.minimum_observations = 3;
        }
        let runtime = QualityRuntime::start(crate::Chronos::new(config).unwrap()).unwrap();
        let mut identity = SourceIdentity::default();
        assert!(
            apply(
                &runtime,
                &mut identity,
                request("a", 1, 1),
                runtime.snapshot().unwrap()
            )
            .unwrap()
            .accepted
        );
        let response = apply(
            &runtime,
            &mut identity,
            request("a", 1, 2),
            runtime.snapshot().unwrap(),
        )
        .unwrap();
        assert!(!response.accepted && response.reason.contains("interval"));
        thread::sleep(Duration::from_millis(60));
        let mut fresh = request("a", 1, 3);
        fresh.device_timestamp_ns += 60000000;
        assert!(
            apply(&runtime, &mut identity, fresh, runtime.snapshot().unwrap())
                .unwrap()
                .accepted
        );
    }

    #[tokio::test]
    /// The lifecycle barrier waits for the processing gate and cancels old queued input.
    async fn lifecycle_barrier_waits_and_cancels_old_input() {
        let runtime = Arc::new(setup());
        let worker = ObservationWorker::start(runtime.clone()).unwrap();
        let (cancel, cancelled) = watch::channel(false);
        let (reply, receiver) = oneshot::channel();
        let processing = worker.processing.lock().unwrap();
        worker
            .submit(ObservationJob {
                request: request("a", 1, 1),
                snapshot: runtime.snapshot().unwrap(),
                cancelled,
                reply,
            })
            .unwrap();
        cancel.send_replace(true);
        let barrier = worker.quiesce();
        let (started, starting) = mpsc::channel();
        let (done, finishing) = mpsc::channel();
        let barrier_thread = thread::spawn(move || {
            started.send(()).unwrap();
            done.send(barrier()).unwrap();
        });
        starting.recv().unwrap();
        assert!(finishing.recv_timeout(Duration::from_millis(20)).is_err());
        drop(processing);
        finishing
            .recv_timeout(Duration::from_secs(1))
            .unwrap()
            .unwrap();
        barrier_thread.join().unwrap();
        assert_eq!(
            receiver.await.unwrap().unwrap_err().code(),
            tonic::Code::Unavailable
        );
        worker.stop().unwrap();
        assert_eq!(
            runtime.models_snapshot().unwrap().unwrap().domains[0].generation,
            1
        );
    }
}
