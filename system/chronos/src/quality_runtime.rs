// SPDX-License-Identifier: MulanPSL-2.0

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, RwLock};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::model::{ClockModel, ConversionResult, DomainState, DomainStatus};
use crate::runtime::{Chronos, ChronosRuntimeError, ObservationSnapshot, QualityRuntimeStatus};
use crate::snapshot::{ClockModelsSnapshot, ClockQualitySnapshot};
use crate::virtual_clock::{VirtualReferenceTime, VirtualSteadyTime};

/// Background in-process driver for continuous Chronos quality evaluation.
#[derive(Debug)]
pub struct QualityRuntime {
    chronos: Arc<Mutex<Chronos>>,
    latest_status: Arc<RwLock<QualityRuntimeStatus>>,
    worker_failed: Arc<AtomicBool>,
    stop: Arc<(Mutex<bool>, Condvar)>,
    worker: Option<JoinHandle<()>>,
}

impl QualityRuntime {
    /// Evaluate once, then start periodic refresh and warning publication.
    pub fn start(mut chronos: Chronos) -> Result<Self, ChronosRuntimeError> {
        let refresh_interval = Duration::from_nanos(chronos.config().quality.refresh_interval_ns);
        let warning_interval = Duration::from_nanos(chronos.config().quality.warning_interval_ns);
        let initial_status = chronos.evaluate_quality();
        let chronos = Arc::new(Mutex::new(chronos));
        let latest_status = Arc::new(RwLock::new(initial_status.clone()));
        let stop = Arc::new((Mutex::new(false), Condvar::new()));
        let worker_failed = Arc::new(AtomicBool::new(false));
        let worker = spawn_quality_worker(
            chronos.clone(),
            latest_status.clone(),
            stop.clone(),
            worker_failed.clone(),
            initial_status,
            refresh_interval,
            warning_interval,
        )?;
        Ok(Self {
            chronos,
            latest_status,
            worker_failed,
            stop,
            worker: Some(worker),
        })
    }

    /// Return a cloned status without waiting for a new clock read.
    pub fn latest_quality_status(&self) -> Result<QualityRuntimeStatus, ChronosRuntimeError> {
        if self.worker_failed.load(Ordering::Acquire) {
            return Err(ChronosRuntimeError::QualityStatePoisoned);
        }
        Ok(recover_read_lock(&self.latest_status).clone())
    }

    /// Run one immediate best-effort evaluation and publish it in process.
    pub fn evaluate_now(&self) -> Result<QualityRuntimeStatus, ChronosRuntimeError> {
        self.with_chronos(|chronos| chronos.evaluate_quality())
    }

    /// Return the latest transport-neutral quality snapshot without sampling.
    pub fn quality_snapshot(&self) -> Result<ClockQualitySnapshot, ChronosRuntimeError> {
        Ok(ClockQualitySnapshot::from(&self.latest_quality_status()?))
    }

    /// Return all affine-domain models as one coherent cached snapshot.
    pub fn models_snapshot(&self) -> Result<Option<ClockModelsSnapshot>, ChronosRuntimeError> {
        self.with_chronos(|chronos| chronos.models_snapshot())?
    }

    /// Copy both publication views under the same owner lock without sampling.
    pub(crate) fn publication_snapshots(
        &self,
    ) -> Result<(ClockQualitySnapshot, Option<ClockModelsSnapshot>), ChronosRuntimeError> {
        self.with_chronos(|chronos| Ok((chronos.quality_snapshot(), chronos.models_snapshot()?)))?
    }

    /// Return the validated configuration owned by the running Chronos.
    pub fn config(&self) -> Result<crate::config::CoreConfig, ChronosRuntimeError> {
        self.with_chronos(|chronos| chronos.config().clone())
    }

    /// Read local steady time independently of periodic evaluation.
    pub fn steady_time(&self) -> Result<VirtualSteadyTime, ChronosRuntimeError> {
        self.with_chronos(|chronos| chronos.steady_time())?
    }

    /// Read current reference time and update the shared status.
    pub fn reference_time(&self) -> Result<VirtualReferenceTime, ChronosRuntimeError> {
        self.with_chronos(|chronos| chronos.reference_time())?
    }

    /// Capture an opaque observation token at message ingress.
    pub fn snapshot(&self) -> Result<ObservationSnapshot, ChronosRuntimeError> {
        self.with_chronos(|chronos| chronos.snapshot())?
    }

    /// Sample ingress without waiting behind fitting; callers reject a busy owner.
    pub(crate) fn try_observation_snapshot(
        &self,
    ) -> Result<Option<ObservationSnapshot>, ChronosRuntimeError> {
        self.try_snapshot_with(|chronos| chronos.snapshot())
    }

    /// Validate the owned helper's earlier receive pair without waiting behind fitting.
    pub(crate) fn try_collector_snapshot(
        &self,
        reference_ns: i64,
        steady_ns: u64,
        uncertainty_ns: u64,
    ) -> Result<Option<ObservationSnapshot>, ChronosRuntimeError> {
        self.try_snapshot_with(|chronos| {
            chronos.collector_snapshot(reference_ns, steady_ns, uncertainty_ns)
        })
    }

    /// Capture an observation while preserving immediate overload refusal and quality updates.
    fn try_snapshot_with(
        &self,
        capture: impl FnOnce(&mut Chronos) -> Result<ObservationSnapshot, ChronosRuntimeError>,
    ) -> Result<Option<ObservationSnapshot>, ChronosRuntimeError> {
        if self.worker_failed.load(Ordering::Acquire) {
            return Err(ChronosRuntimeError::QualityStatePoisoned);
        }
        let mut chronos = match self.chronos.try_lock() {
            Ok(chronos) => chronos,
            Err(std::sync::TryLockError::WouldBlock) => return Ok(None),
            Err(std::sync::TryLockError::Poisoned(_)) => {
                return Err(ChronosRuntimeError::QualityStatePoisoned);
            }
        };
        if self.worker_failed.load(Ordering::Acquire) {
            return Err(ChronosRuntimeError::QualityStatePoisoned);
        }
        let result = capture(&mut chronos);
        *recover_write_lock(&self.latest_status) = chronos.latest_quality_status();
        result.map(Some)
    }

    /// Pair a device timestamp with a clock read taken by this call.
    pub fn observe_device_timestamp(
        &self,
        domain_id: &str,
        sequence_id: u64,
        device_timestamp_ns: i64,
    ) -> Result<DomainStatus, ChronosRuntimeError> {
        self.with_chronos(|chronos| {
            chronos.observe_device_timestamp(domain_id, sequence_id, device_timestamp_ns)
        })?
    }

    /// Submit a device timestamp with an earlier ingress snapshot.
    pub fn observe_device_timestamp_at(
        &self,
        domain_id: &str,
        sequence_id: u64,
        device_timestamp_ns: i64,
        snapshot: ObservationSnapshot,
    ) -> Result<DomainStatus, ChronosRuntimeError> {
        self.with_chronos(|chronos| {
            chronos.observe_device_timestamp_at(
                domain_id,
                sequence_id,
                device_timestamp_ns,
                snapshot,
            )
        })?
    }

    /// Start a new domain generation after a confirmed device restart.
    pub fn reset_device_domain(
        &self,
        domain_id: &str,
    ) -> Result<DomainStatus, ChronosRuntimeError> {
        self.with_chronos(|chronos| chronos.reset_device_domain(domain_id))?
    }

    /// Return an owned best-effort model after an immediate quality evaluation.
    pub fn model(&self, domain_id: &str) -> Result<Option<ClockModel>, ChronosRuntimeError> {
        self.with_chronos(|chronos| chronos.model(domain_id))?
    }

    /// Perform one low-frequency best-effort device-time conversion.
    pub fn convert(
        &self,
        domain_id: &str,
        device_timestamp_ns: i64,
    ) -> Result<ConversionResult, ChronosRuntimeError> {
        self.with_chronos(|chronos| chronos.convert(domain_id, device_timestamp_ns))?
    }

    /// Signal the worker, join it, and report an unexpected worker panic.
    pub fn stop(mut self) -> Result<(), ChronosRuntimeError> {
        self.stop_worker()
    }

    #[cfg(test)]
    /// Share the stop condition with concurrency tests for exact synchronization.
    pub(crate) fn stop_signal_for_test(&self) -> Arc<(Mutex<bool>, Condvar)> {
        self.stop.clone()
    }

    /// Serialize access to Chronos and publish any resulting report changes.
    fn with_chronos<R>(
        &self,
        operation: impl FnOnce(&mut Chronos) -> R,
    ) -> Result<R, ChronosRuntimeError> {
        if self.worker_failed.load(Ordering::Acquire) {
            return Err(ChronosRuntimeError::QualityStatePoisoned);
        }
        let mut chronos = self
            .chronos
            .lock()
            .map_err(|_| ChronosRuntimeError::QualityStatePoisoned)?;
        if self.worker_failed.load(Ordering::Acquire) {
            return Err(ChronosRuntimeError::QualityStatePoisoned);
        }
        let result = operation(&mut chronos);
        if self.worker_failed.load(Ordering::Acquire) {
            return Err(ChronosRuntimeError::QualityStatePoisoned);
        }
        *recover_write_lock(&self.latest_status) = chronos.latest_quality_status();
        Ok(result)
    }

    /// Stop and join the worker at most once.
    fn stop_worker(&mut self) -> Result<(), ChronosRuntimeError> {
        let (lock, wake) = &*self.stop;
        *recover_mutex_lock(lock) = true;
        wake.notify_all();
        if let Some(worker) = self.worker.take() {
            if worker.join().is_err() {
                return Err(ChronosRuntimeError::QualityWorkerPanicked);
            }
            if self.worker_failed.load(Ordering::Acquire) || self.chronos.is_poisoned() {
                return Err(ChronosRuntimeError::QualityStatePoisoned);
            }
        }
        Ok(())
    }
}

impl Drop for QualityRuntime {
    fn drop(&mut self) {
        if let Err(error) = self.stop_worker() {
            robonix_scribe::warn!("[chronos] {error}");
        }
    }
}

/// Spawn the periodic evaluator without requiring an async runtime.
fn spawn_quality_worker(
    chronos: Arc<Mutex<Chronos>>,
    latest_status: Arc<RwLock<QualityRuntimeStatus>>,
    stop: Arc<(Mutex<bool>, Condvar)>,
    worker_failed: Arc<AtomicBool>,
    initial_status: QualityRuntimeStatus,
    refresh_interval: Duration,
    warning_interval: Duration,
) -> Result<JoinHandle<()>, ChronosRuntimeError> {
    thread::Builder::new()
        .name("chronos-quality".to_string())
        .spawn(move || {
            let worker = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                quality_worker_loop(
                    chronos,
                    latest_status,
                    stop,
                    worker_failed.clone(),
                    initial_status,
                    refresh_interval,
                    warning_interval,
                );
            }));
            if let Err(payload) = worker {
                worker_failed.store(true, Ordering::Release);
                std::panic::resume_unwind(payload);
            }
        })
        .map_err(ChronosRuntimeError::QualityWorkerStart)
}

/// Refresh quality until stopped, without losing a stop notification.
fn quality_worker_loop(
    chronos: Arc<Mutex<Chronos>>,
    latest_status: Arc<RwLock<QualityRuntimeStatus>>,
    stop: Arc<(Mutex<bool>, Condvar)>,
    worker_failed: Arc<AtomicBool>,
    initial_status: QualityRuntimeStatus,
    refresh_interval: Duration,
    warning_interval: Duration,
) {
    let mut warning_state = WarningState::default();
    publish_warning(&initial_status, warning_interval, &mut warning_state);
    loop {
        let (lock, wake) = &*stop;
        let stopped = recover_mutex_lock(lock);
        let (stopped, _) = wake
            .wait_timeout_while(stopped, refresh_interval, |stopped| !*stopped)
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        if *stopped {
            break;
        }
        drop(stopped);
        let status = {
            let mut chronos = match chronos.lock() {
                Ok(chronos) => chronos,
                Err(_) => {
                    worker_failed.store(true, Ordering::Release);
                    break;
                }
            };
            let status = chronos.evaluate_quality();
            *recover_write_lock(&latest_status) = status.clone();
            status
        };
        publish_warning(&status, warning_interval, &mut warning_state);
    }
}

/// Emit at most one warning per interval while quality remains degraded.
fn publish_warning(
    status: &QualityRuntimeStatus,
    warning_interval: Duration,
    warning_state: &mut WarningState,
) {
    let Some(warning) = degraded_warning(status) else {
        warning_state.clear();
        return;
    };
    let now = Instant::now();
    if warning_state.should_publish(&warning.condition, now, warning_interval) {
        robonix_scribe::warn!("[chronos] {}", warning.message);
    }
}

#[derive(Debug, Default)]
struct WarningState {
    last_condition: Option<String>,
    last_published_at: Option<Instant>,
}

impl WarningState {
    /// Publish changed conditions immediately and repeat unchanged ones slowly.
    fn should_publish(&mut self, condition: &str, now: Instant, interval: Duration) -> bool {
        let changed = self.last_condition.as_deref() != Some(condition);
        let elapsed = self
            .last_published_at
            .is_none_or(|previous| now.duration_since(previous) >= interval);
        if changed || elapsed {
            self.last_condition = Some(condition.to_string());
            self.last_published_at = Some(now);
            return true;
        }
        false
    }

    /// Let a later recurrence warn immediately after quality recovers.
    fn clear(&mut self) {
        self.last_condition = None;
        self.last_published_at = None;
    }
}

/// Summarize refresh, reference, and affine-domain degradation for logging.
fn degraded_warning(status: &QualityRuntimeStatus) -> Option<DegradedWarning> {
    if let Some(failure) = &status.refresh_failure {
        return Some(DegradedWarning {
            condition: format!("refresh:{}", failure.message),
            message: format!(
                "quality refresh failed {} consecutive time(s): {}",
                failure.consecutive_failures, failure.message
            ),
        });
    }
    let report = status.report.as_ref()?;
    if !report.reference.available {
        return Some(DegradedWarning {
            condition: format!("reference:{}", report.reference.reason),
            message: format!("reference clock unavailable: {}", report.reference.reason),
        });
    }
    let degraded = report
        .domains
        .iter()
        .filter_map(|domain| match domain {
            crate::runtime::DomainQualityReport::AffineModel(domain)
                if matches!(domain.state, DomainState::Unsynced | DomainState::Fault) =>
            {
                Some(format!(
                    "'{}' is {:?}: {}",
                    domain.domain_id, domain.state, domain.reason
                ))
            }
            _ => None,
        })
        .collect::<Vec<_>>();
    (!degraded.is_empty()).then(|| {
        let message = format!("clock domains degraded: {}", degraded.join("; "));
        DegradedWarning {
            condition: message.clone(),
            message,
        }
    })
}

#[derive(Debug, PartialEq, Eq)]
struct DegradedWarning {
    condition: String,
    message: String,
}

fn recover_mutex_lock<T>(lock: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    lock.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn recover_read_lock<T>(lock: &RwLock<T>) -> std::sync::RwLockReadGuard<'_, T> {
    lock.read().unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn recover_write_lock<T>(lock: &RwLock<T>) -> std::sync::RwLockWriteGuard<'_, T> {
    lock.write()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

#[cfg(test)]
mod tests {
    #[test]
    /// Busy core sampling refuses to pair an observation at a later reference time.
    fn ingress_snapshot_does_not_wait_for_core() {
        let config = crate::ChronosConfig::from_yaml(
            "chronos:\n  core:\n    reference:\n      kind: linux_system\n",
        )
        .unwrap();
        let runtime = super::QualityRuntime::start(crate::Chronos::new(config).unwrap()).unwrap();
        let locked = runtime.chronos.lock().unwrap();
        assert!(runtime.try_observation_snapshot().unwrap().is_none());
        drop(locked);
        assert!(runtime.try_observation_snapshot().unwrap().is_some());
    }

    use std::thread;

    use super::*;
    use crate::config::ChronosConfig;
    use crate::runtime::{
        ClockQualityReport, DomainQualityReport, QualityRefreshFailure, QualityRuntimeStatus,
    };

    /// Build a report that can be varied for warning-priority tests.
    fn status_with_domains(domains: Vec<DomainQualityReport>) -> QualityRuntimeStatus {
        QualityRuntimeStatus {
            report: Some(ClockQualityReport {
                reference: VirtualReferenceTime {
                    timestamp_ns: 1_000,
                    generation: 1,
                    generation_changed: false,
                    uncertainty_ns: 1,
                    available: true,
                    verified: false,
                    reason: "test reference".to_string(),
                },
                steady: VirtualSteadyTime { timestamp_ns: 10 },
                domains,
            }),
            last_successful_refresh_steady: Some(VirtualSteadyTime { timestamp_ns: 10 }),
            refresh_failure: None,
        }
    }

    /// Build one degraded affine-domain status for aggregation tests.
    fn degraded_domain(domain_id: &str, state: DomainState) -> DomainQualityReport {
        DomainQualityReport::AffineModel(DomainStatus {
            domain_id: domain_id.to_string(),
            generation: 1,
            reference_generation: 1,
            model_version: 1,
            state,
            accepted_observations: 2,
            rejected_observations: 0,
            last_observation_age_ns: Some(1),
            uncertainty_ns: Some(1),
            reason: "test degradation".to_string(),
        })
    }

    #[test]
    /// Warning cadence repeats, suppresses, changes, and resets deterministically.
    fn warning_state_tracks_condition_and_interval() {
        let start = Instant::now();
        let interval = Duration::from_secs(10);
        let mut state = WarningState::default();
        assert!(state.should_publish("first", start, interval));
        assert!(!state.should_publish("first", start + Duration::from_secs(1), interval));
        assert!(state.should_publish("changed", start + Duration::from_secs(2), interval));
        assert!(state.should_publish("changed", start + Duration::from_secs(12), interval));
        state.clear();
        assert!(state.should_publish("changed", start + Duration::from_secs(13), interval));
    }

    #[test]
    /// Warning messages prioritize refresh and reference failures, then all domains.
    fn degraded_message_preserves_priority_and_all_domains() {
        let mut status = status_with_domains(vec![
            degraded_domain("camera", DomainState::Unsynced),
            degraded_domain("lidar", DomainState::Fault),
        ]);
        status.refresh_failure = Some(QualityRefreshFailure {
            message: "sample failed".to_string(),
            consecutive_failures: 2,
            first_observed_steady: Some(VirtualSteadyTime { timestamp_ns: 10 }),
            last_observed_steady: Some(VirtualSteadyTime { timestamp_ns: 11 }),
        });
        let first_failure = degraded_warning(&status).unwrap();
        assert!(first_failure.message.starts_with("quality refresh failed"));
        status
            .refresh_failure
            .as_mut()
            .unwrap()
            .consecutive_failures += 1;
        let repeated_failure = degraded_warning(&status).unwrap();
        assert_eq!(first_failure.condition, repeated_failure.condition);
        assert_ne!(first_failure.message, repeated_failure.message);

        status.refresh_failure = None;
        status.report.as_mut().unwrap().reference.available = false;
        assert!(
            degraded_warning(&status)
                .unwrap()
                .message
                .starts_with("reference clock unavailable")
        );

        status.report.as_mut().unwrap().reference.available = true;
        let message = degraded_warning(&status).unwrap().message;
        assert!(message.contains("camera"));
        assert!(message.contains("lidar"));
    }

    #[test]
    /// Stop reports a poisoned Chronos owner even when the worker exits normally.
    fn stop_reports_poisoned_chronos_state() {
        let config = ChronosConfig::from_yaml(
            r#"
chronos:
  core:
    reference:
      kind: linux_system
    quality:
      refresh_interval_ns: 60000000000
      warning_interval_ns: 1000000000
"#,
        )
        .unwrap();
        let runtime = QualityRuntime::start(Chronos::new(config).unwrap()).unwrap();
        let chronos = runtime.chronos.clone();
        assert!(
            thread::spawn(move || {
                let _guard = chronos.lock().unwrap();
                panic!("test Chronos owner poison");
            })
            .join()
            .is_err()
        );
        assert!(matches!(
            runtime.stop(),
            Err(ChronosRuntimeError::QualityStatePoisoned)
        ));
    }
}
