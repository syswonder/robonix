// SPDX-License-Identifier: MulanPSL-2.0

use thiserror::Error;

use crate::config::{ChronosConfigError, ReferenceConfig};

/// One bounded read of the selected reference clock.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReferenceSample {
    pub reference_timestamp_ns: i64,
    pub steady_received_ns: u64,
    pub uncertainty_ns: u64,
    pub generation: u64,
    pub generation_changed: bool,
}

/// Current freshness and verification result for the reference clock.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReferenceStatus {
    pub generation: u64,
    pub available: bool,
    pub verified: bool,
    pub last_sample_age_ns: Option<u64>,
    pub uncertainty_ns: Option<u64>,
    pub reason: String,
}

/// Errors produced while reading or evaluating a reference clock.
#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum ReferenceError {
    #[error("Linux clock read failed with errno {errno}")]
    ClockRead { errno: i32 },
    #[error("monotonic clock moved backward during reference sampling")]
    MonotonicRegression,
    #[error("reference clock arithmetic overflow")]
    Overflow,
}

trait ClockReader {
    fn monotonic_ns(&mut self) -> Result<u64, ReferenceError>;
    fn realtime_ns(&mut self) -> Result<i64, ReferenceError>;
    fn realtime_resolution_ns(&mut self) -> Result<u64, ReferenceError>;
}

#[derive(Debug, Clone)]
struct ObservedSample {
    reference_timestamp_ns: i64,
    steady_received_ns: u64,
    uncertainty_ns: u64,
}

#[derive(Debug)]
struct ReferenceObserver<R> {
    config: ReferenceConfig,
    reader: R,
    generation: u64,
    last_sample: Option<ObservedSample>,
}

impl<R: ClockReader> ReferenceObserver<R> {
    /// Construct an observer at the first reference generation.
    fn new(config: ReferenceConfig, reader: R) -> Self {
        Self {
            config,
            reader,
            generation: 1,
            last_sample: None,
        }
    }

    /// Bracket one realtime read with monotonic reads and evaluate discontinuity.
    fn sample(&mut self) -> Result<ReferenceSample, ReferenceError> {
        let steady_before = self.reader.monotonic_ns()?;
        let reference_timestamp_ns = self.reader.realtime_ns()?;
        let steady_after = self.reader.monotonic_ns()?;
        if steady_after < steady_before {
            return Err(ReferenceError::MonotonicRegression);
        }
        let span = steady_after - steady_before;
        let steady_received_ns = steady_before + span / 2;
        let bracket_uncertainty = span / 2 + span % 2;
        let resolution = self.reader.realtime_resolution_ns()?;
        let uncertainty_ns = bracket_uncertainty.saturating_add(resolution);
        let generation_changed =
            self.reference_changed(reference_timestamp_ns, steady_received_ns, uncertainty_ns)?;
        if generation_changed {
            self.generation = self
                .generation
                .checked_add(1)
                .ok_or(ReferenceError::Overflow)?;
        }
        self.last_sample = Some(ObservedSample {
            reference_timestamp_ns,
            steady_received_ns,
            uncertainty_ns,
        });
        Ok(ReferenceSample {
            reference_timestamp_ns,
            steady_received_ns,
            uncertainty_ns,
            generation: self.generation,
            generation_changed,
        })
    }

    /// Return status at a supplied monotonic time for deterministic evaluation.
    fn status_at(&self, now_steady_ns: u64) -> Result<ReferenceStatus, ReferenceError> {
        let Some(sample) = self.last_sample.as_ref() else {
            return Ok(ReferenceStatus {
                generation: self.generation,
                available: false,
                verified: false,
                last_sample_age_ns: None,
                uncertainty_ns: None,
                reason: "no reference clock sample".to_string(),
            });
        };
        let age = now_steady_ns
            .checked_sub(sample.steady_received_ns)
            .ok_or(ReferenceError::MonotonicRegression)?;
        let fresh = age <= self.config.max_status_age_ns;
        let precise = sample.uncertainty_ns <= self.config.max_uncertainty_ns;
        let reason = if !fresh {
            "reference clock sample is stale"
        } else if !precise {
            "reference clock uncertainty exceeds configured limit"
        } else {
            "Linux system time source is not independently verified"
        };
        Ok(ReferenceStatus {
            generation: self.generation,
            available: fresh && precise,
            verified: false,
            last_sample_age_ns: Some(age),
            uncertainty_ns: Some(sample.uncertainty_ns),
            reason: reason.to_string(),
        })
    }

    /// Detect backward time or steps beyond the threshold and both sampling uncertainties.
    fn reference_changed(
        &self,
        reference_timestamp_ns: i64,
        steady_received_ns: u64,
        uncertainty_ns: u64,
    ) -> Result<bool, ReferenceError> {
        let Some(previous) = self.last_sample.as_ref() else {
            return Ok(false);
        };
        let steady_delta = steady_received_ns
            .checked_sub(previous.steady_received_ns)
            .ok_or(ReferenceError::MonotonicRegression)?;
        let reference_delta =
            i128::from(reference_timestamp_ns) - i128::from(previous.reference_timestamp_ns);
        let deviation = (reference_delta - i128::from(steady_delta)).abs();
        let step_limit = i128::from(self.config.max_clock_step_ns)
            + i128::from(previous.uncertainty_ns)
            + i128::from(uncertainty_ns);
        Ok(reference_timestamp_ns < previous.reference_timestamp_ns || deviation > step_limit)
    }
}

#[cfg(target_os = "linux")]
#[derive(Debug, Default)]
struct LinuxClockReader;

#[cfg(target_os = "linux")]
impl ClockReader for LinuxClockReader {
    fn monotonic_ns(&mut self) -> Result<u64, ReferenceError> {
        read_linux_monotonic_ns()
    }

    fn realtime_ns(&mut self) -> Result<i64, ReferenceError> {
        let value = read_clock_ns(libc::CLOCK_REALTIME)?;
        i64::try_from(value).map_err(|_| ReferenceError::Overflow)
    }

    fn realtime_resolution_ns(&mut self) -> Result<u64, ReferenceError> {
        let value = read_clock_resolution_ns(libc::CLOCK_REALTIME)?;
        u64::try_from(value).map_err(|_| ReferenceError::Overflow)
    }
}

/// Read-only observer for Linux `CLOCK_REALTIME`, bounded by `CLOCK_MONOTONIC`.
#[cfg(target_os = "linux")]
#[derive(Debug)]
pub struct LinuxSystemReference {
    observer: ReferenceObserver<LinuxClockReader>,
}

#[cfg(target_os = "linux")]
impl LinuxSystemReference {
    /// Construct a Linux system-time observer with validated quality limits.
    pub fn new(config: ReferenceConfig) -> Result<Self, ChronosConfigError> {
        config.validate()?;
        Ok(Self {
            observer: ReferenceObserver::new(config, LinuxClockReader),
        })
    }

    /// Read one bounded Linux system-time sample.
    pub fn sample(&mut self) -> Result<ReferenceSample, ReferenceError> {
        self.observer.sample()
    }

    /// Sample reference time and evaluate quality at its monotonic midpoint.
    ///
    /// Using the paired midpoint keeps the returned status deterministic for
    /// this observation and avoids adding a second status-age clock read.
    pub fn sample_with_status(
        &mut self,
    ) -> Result<(ReferenceSample, ReferenceStatus), ReferenceError> {
        let sample = self.observer.sample()?;
        let status = self.observer.status_at(sample.steady_received_ns)?;
        Ok((sample, status))
    }

    /// Evaluate freshness and quality using the current Linux monotonic time.
    pub fn status(&mut self) -> Result<ReferenceStatus, ReferenceError> {
        let now = self.observer.reader.monotonic_ns()?;
        self.observer.status_at(now)
    }
}

/// Read Linux monotonic time as a non-negative nanosecond count.
#[cfg(target_os = "linux")]
pub(crate) fn read_linux_monotonic_ns() -> Result<u64, ReferenceError> {
    let value = read_clock_ns(libc::CLOCK_MONOTONIC)?;
    u64::try_from(value).map_err(|_| ReferenceError::Overflow)
}

/// Convert one POSIX clock reading to nanoseconds with checked arithmetic.
#[cfg(target_os = "linux")]
fn read_clock_ns(clock_id: libc::clockid_t) -> Result<i128, ReferenceError> {
    let mut value = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `value` points to writable initialized memory for `clock_gettime`.
    let result = unsafe { libc::clock_gettime(clock_id, &mut value) };
    if result != 0 {
        return Err(last_clock_error());
    }
    i128::from(value.tv_sec)
        .checked_mul(1_000_000_000)
        .and_then(|seconds| seconds.checked_add(i128::from(value.tv_nsec)))
        .ok_or(ReferenceError::Overflow)
}

/// Read one POSIX clock resolution and convert it to nanoseconds.
#[cfg(target_os = "linux")]
fn read_clock_resolution_ns(clock_id: libc::clockid_t) -> Result<i128, ReferenceError> {
    let mut value = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `value` points to writable initialized memory for `clock_getres`.
    let result = unsafe { libc::clock_getres(clock_id, &mut value) };
    if result != 0 {
        return Err(last_clock_error());
    }
    i128::from(value.tv_sec)
        .checked_mul(1_000_000_000)
        .and_then(|seconds| seconds.checked_add(i128::from(value.tv_nsec)))
        .ok_or(ReferenceError::Overflow)
}

#[cfg(target_os = "linux")]
fn last_clock_error() -> ReferenceError {
    ReferenceError::ClockRead {
        errno: std::io::Error::last_os_error().raw_os_error().unwrap_or(0),
    }
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use super::{ClockReader, ReferenceError, ReferenceObserver};
    use crate::config::{ReferenceConfig, ReferenceSourceKind};

    #[derive(Debug)]
    struct FakeClockReader {
        monotonic: VecDeque<u64>,
        realtime: VecDeque<i64>,
        resolution_ns: u64,
    }

    impl ClockReader for FakeClockReader {
        fn monotonic_ns(&mut self) -> Result<u64, ReferenceError> {
            self.monotonic.pop_front().ok_or(ReferenceError::Overflow)
        }

        fn realtime_ns(&mut self) -> Result<i64, ReferenceError> {
            self.realtime.pop_front().ok_or(ReferenceError::Overflow)
        }

        fn realtime_resolution_ns(&mut self) -> Result<u64, ReferenceError> {
            Ok(self.resolution_ns)
        }
    }

    /// Build deterministic reference limits for observer tests.
    fn config() -> ReferenceConfig {
        ReferenceConfig {
            kind: ReferenceSourceKind::LinuxSystem,
            max_uncertainty_ns: 20,
            max_status_age_ns: 100,
            max_clock_step_ns: 50,
        }
    }

    #[test]
    /// Monotonic bracketing contributes half its span plus clock resolution.
    fn brackets_linux_system_sample() {
        let reader = FakeClockReader {
            monotonic: VecDeque::from([100, 110]),
            realtime: VecDeque::from([1_000]),
            resolution_ns: 4,
        };
        let mut observer = ReferenceObserver::new(config(), reader);
        let sample = observer.sample().unwrap();
        assert_eq!(sample.reference_timestamp_ns, 1_000);
        assert_eq!(sample.steady_received_ns, 105);
        assert_eq!(sample.uncertainty_ns, 9);
        assert_eq!(sample.generation, 1);
        assert!(!sample.generation_changed);
    }

    #[test]
    /// A realtime step relative to monotonic elapsed time starts a generation.
    fn clock_step_changes_reference_generation() {
        let reader = FakeClockReader {
            monotonic: VecDeque::from([100, 100, 200, 200]),
            realtime: VecDeque::from([1_000, 2_000]),
            resolution_ns: 1,
        };
        let mut observer = ReferenceObserver::new(config(), reader);
        observer.sample().unwrap();
        let stepped = observer.sample().unwrap();
        assert_eq!(stepped.generation, 2);
        assert!(stepped.generation_changed);
    }

    #[test]
    /// A scheduling stall in either bracket does not reset a constant-offset clock.
    fn sampling_stalls_preserve_reference_generation() {
        for (monotonic, realtime) in [
            ([0, 0, 100, 300], [1_000, 1_100]),
            ([0, 200, 300, 300], [1_000, 1_300]),
        ] {
            let reader = FakeClockReader {
                monotonic: VecDeque::from(monotonic),
                realtime: VecDeque::from(realtime),
                resolution_ns: 1,
            };
            let mut observer = ReferenceObserver::new(config(), reader);
            observer.sample().unwrap();
            let current = observer.sample().unwrap();
            assert_eq!(current.generation, 1);
            assert!(!current.generation_changed);
        }
    }

    #[test]
    /// Sampling uncertainty does not hide steps beyond the combined error bound.
    fn clock_steps_exceeding_bracket_uncertainty_change_generation() {
        for second_reference in [1_400, 1_001] {
            let reader = FakeClockReader {
                monotonic: VecDeque::from([0, 0, 100, 300]),
                realtime: VecDeque::from([1_000, second_reference]),
                resolution_ns: 1,
            };
            let mut observer = ReferenceObserver::new(config(), reader);
            observer.sample().unwrap();
            assert!(observer.sample().unwrap().generation_changed);
        }
    }

    #[test]
    /// Realtime moving backward always starts a new reference generation.
    fn backward_clock_changes_reference_generation() {
        for (monotonic, realtime) in [
            ([100, 100, 200, 200], [1_000, 900]),
            ([0, 0, 1, 2_001], [1_000, 999]),
        ] {
            let reader = FakeClockReader {
                monotonic: VecDeque::from(monotonic),
                realtime: VecDeque::from(realtime),
                resolution_ns: 1,
            };
            let mut observer = ReferenceObserver::new(config(), reader);
            observer.sample().unwrap();
            let stepped = observer.sample().unwrap();
            assert_eq!(stepped.generation, 2);
            assert!(stepped.generation_changed);
        }
    }

    #[test]
    /// Linux system time remains unverified and becomes unavailable when stale.
    fn status_reports_unverified_and_stale_reference() {
        let reader = FakeClockReader {
            monotonic: VecDeque::from([100, 100]),
            realtime: VecDeque::from([1_000]),
            resolution_ns: 1,
        };
        let mut observer = ReferenceObserver::new(config(), reader);
        observer.sample().unwrap();
        let current = observer.status_at(150).unwrap();
        assert!(current.available);
        assert!(!current.verified);
        assert!(current.reason.contains("not independently verified"));
        let stale = observer.status_at(201).unwrap();
        assert!(!stale.available);
        assert!(stale.reason.contains("stale"));
    }
}
