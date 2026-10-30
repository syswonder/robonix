// SPDX-License-Identifier: MulanPSL-2.0

use thiserror::Error;

#[cfg(target_os = "linux")]
use crate::config::ChronosConfigError;
#[cfg(any(target_os = "linux", test))]
use crate::config::ReferenceConfig;

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
    #[error("Chronos reference lock poisoned")]
    Poisoned,
    #[error("monotonic clock moved backward during reference sampling")]
    MonotonicRegression,
    #[error("reference clock arithmetic overflow")]
    Overflow,
}

#[cfg(any(target_os = "linux", test))]
trait ClockReader {
    fn monotonic_ns(&mut self) -> Result<u64, ReferenceError>;
    fn boottime_ns(&mut self) -> Result<u64, ReferenceError>;
    fn realtime_ns(&mut self) -> Result<i64, ReferenceError>;
    fn realtime_resolution_ns(&mut self) -> Result<u64, ReferenceError>;
}

#[cfg(any(target_os = "linux", test))]
#[derive(Debug)]
struct ObservedSample {
    reference_timestamp_ns: i64,
    boottime_received_ns: u64,
    uncertainty_ns: u64,
}

#[cfg(any(target_os = "linux", test))]
#[derive(Debug)]
struct ReferenceObserver<R> {
    config: ReferenceConfig,
    reader: R,
    generation: u64,
    last_sample: Option<ObservedSample>,
}

#[cfg(any(target_os = "linux", test))]
impl<R: ClockReader> ReferenceObserver<R> {
    fn new(config: ReferenceConfig, reader: R) -> Self {
        Self {
            config,
            reader,
            generation: 1,
            last_sample: None,
        }
    }

    fn sample(&mut self) -> Result<(ReferenceSample, ReferenceStatus), ReferenceError> {
        let steady_before = self.reader.monotonic_ns()?;
        let boot_before = self.reader.boottime_ns()?;
        let reference_timestamp_ns = self.reader.realtime_ns()?;
        let boot_after = self.reader.boottime_ns()?;
        let steady_after = self.reader.monotonic_ns()?;
        let resolution = self.reader.realtime_resolution_ns()?;
        let now = self.reader.boottime_ns()?;
        if steady_after < steady_before || boot_after < boot_before || now < boot_after {
            return Err(ReferenceError::MonotonicRegression);
        }
        let span = boot_after - boot_before;
        let boottime_received_ns = boot_before + span / 2;
        let steady_received_ns = steady_before + (steady_after - steady_before) / 2;
        // Both the reference comparison and the exposed steady pair need a bound.
        let bracket_span = span.max(steady_after - steady_before);
        let bracket_uncertainty = bracket_span / 2 + bracket_span % 2;
        let uncertainty_ns = bracket_uncertainty.saturating_add(resolution);
        let generation_changed =
            self.reference_changed(reference_timestamp_ns, boottime_received_ns, uncertainty_ns)?;
        if generation_changed {
            self.generation = self
                .generation
                .checked_add(1)
                .ok_or(ReferenceError::Overflow)?;
        }
        self.last_sample = Some(ObservedSample {
            reference_timestamp_ns,
            boottime_received_ns,
            uncertainty_ns,
        });
        let sample = ReferenceSample {
            reference_timestamp_ns,
            steady_received_ns,
            uncertainty_ns,
            generation: self.generation,
            generation_changed,
        };
        Ok((sample, self.status_at(now)?))
    }

    fn status_at(&self, now_boottime_ns: u64) -> Result<ReferenceStatus, ReferenceError> {
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
        let age = now_boottime_ns
            .checked_sub(sample.boottime_received_ns)
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
        boottime_received_ns: u64,
        uncertainty_ns: u64,
    ) -> Result<bool, ReferenceError> {
        let Some(previous) = self.last_sample.as_ref() else {
            return Ok(false);
        };
        let boot_delta = boottime_received_ns
            .checked_sub(previous.boottime_received_ns)
            .ok_or(ReferenceError::MonotonicRegression)?;
        let reference_delta =
            i128::from(reference_timestamp_ns) - i128::from(previous.reference_timestamp_ns);
        let deviation = (reference_delta - i128::from(boot_delta)).abs();
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

    fn boottime_ns(&mut self) -> Result<u64, ReferenceError> {
        let value = read_clock_ns(libc::CLOCK_BOOTTIME)?;
        u64::try_from(value).map_err(|_| ReferenceError::Overflow)
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

/// Read-only observer for Linux `CLOCK_REALTIME`, bounded by `CLOCK_BOOTTIME`.
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
        self.observer.sample().map(|(sample, _)| sample)
    }

    /// Evaluate freshness at completion, including time spent reading resolution.
    pub fn sample_with_status(
        &mut self,
    ) -> Result<(ReferenceSample, ReferenceStatus), ReferenceError> {
        self.observer.sample()
    }

    /// Evaluate freshness and quality including time spent suspended.
    pub fn status(&mut self) -> Result<ReferenceStatus, ReferenceError> {
        let now = self.observer.reader.boottime_ns()?;
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
        boottime: VecDeque<u64>,
        realtime: VecDeque<i64>,
        resolution_ns: u64,
    }

    impl ClockReader for FakeClockReader {
        fn monotonic_ns(&mut self) -> Result<u64, ReferenceError> {
            self.monotonic.pop_front().ok_or(ReferenceError::Overflow)
        }

        fn boottime_ns(&mut self) -> Result<u64, ReferenceError> {
            self.boottime.pop_front().ok_or(ReferenceError::Overflow)
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
    /// BOOTTIME bounds uncertainty independently of the MONOTONIC midpoint.
    fn brackets_linux_system_sample() {
        let reader = FakeClockReader {
            monotonic: VecDeque::from([100, 110]),
            boottime: VecDeque::from([1_100, 1_110, 1_110]),
            realtime: VecDeque::from([1_000]),
            resolution_ns: 4,
        };
        let mut observer = ReferenceObserver::new(config(), reader);
        let (sample, status) = observer.sample().unwrap();
        assert_eq!(sample.reference_timestamp_ns, 1_000);
        assert_eq!(sample.steady_received_ns, 105);
        assert_eq!(sample.uncertainty_ns, 9);
        assert_eq!(sample.generation, 1);
        assert!(!sample.generation_changed);
        assert_eq!(status.last_sample_age_ns, Some(5));
    }

    #[test]
    /// A realtime step relative to BOOTTIME starts a generation.
    fn clock_step_changes_reference_generation() {
        let reader = FakeClockReader {
            monotonic: VecDeque::from([100, 100, 200, 200]),
            boottime: VecDeque::from([100, 100, 100, 200, 200, 200]),
            realtime: VecDeque::from([1_000, 2_000]),
            resolution_ns: 1,
        };
        let mut observer = ReferenceObserver::new(config(), reader);
        observer.sample().unwrap();
        let (stepped, _) = observer.sample().unwrap();
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
                boottime: VecDeque::from([
                    monotonic[0],
                    monotonic[1],
                    monotonic[1],
                    monotonic[2],
                    monotonic[3],
                    monotonic[3],
                ]),
                realtime: VecDeque::from(realtime),
                resolution_ns: 1,
            };
            let mut observer = ReferenceObserver::new(config(), reader);
            observer.sample().unwrap();
            let (current, _) = observer.sample().unwrap();
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
                boottime: VecDeque::from([0, 0, 0, 100, 300, 300]),
                realtime: VecDeque::from([1_000, second_reference]),
                resolution_ns: 1,
            };
            let mut observer = ReferenceObserver::new(config(), reader);
            observer.sample().unwrap();
            assert!(observer.sample().unwrap().0.generation_changed);
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
                boottime: VecDeque::from([
                    monotonic[0],
                    monotonic[1],
                    monotonic[1],
                    monotonic[2],
                    monotonic[3],
                    monotonic[3],
                ]),
                realtime: VecDeque::from(realtime),
                resolution_ns: 1,
            };
            let mut observer = ReferenceObserver::new(config(), reader);
            observer.sample().unwrap();
            let (stepped, _) = observer.sample().unwrap();
            assert_eq!(stepped.generation, 2);
            assert!(stepped.generation_changed);
        }
    }

    #[test]
    /// Linux system time remains unverified and becomes unavailable when stale.
    fn status_reports_unverified_and_stale_reference() {
        let reader = FakeClockReader {
            monotonic: VecDeque::from([100, 100]),
            boottime: VecDeque::from([100, 100, 100]),
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

    #[test]
    fn sampling_completion_can_already_be_stale() {
        let reader = FakeClockReader {
            monotonic: VecDeque::from([100, 110]),
            boottime: VecDeque::from([100, 110, 206]),
            realtime: VecDeque::from([1_000]),
            resolution_ns: 1,
        };
        let mut observer = ReferenceObserver::new(config(), reader);
        let (_, status) = observer.sample().unwrap();
        assert_eq!(status.last_sample_age_ns, Some(101));
        assert!(!status.available);
        assert!(status.reason.contains("stale"));
    }

    #[test]
    fn outer_steady_bracket_stalls_are_included_in_uncertainty() {
        let reader = FakeClockReader {
            monotonic: VecDeque::from([0, 100_000_000]),
            boottime: VecDeque::from([100_000_000, 100_000_000, 100_000_000]),
            realtime: VecDeque::from([1_000_000_000]),
            resolution_ns: 1,
        };
        let mut observer = ReferenceObserver::new(ReferenceConfig::default(), reader);
        let (sample, status) = observer.sample().unwrap();
        assert_eq!(sample.steady_received_ns, 50_000_000);
        assert_eq!(sample.uncertainty_ns, 50_000_001);
        assert!(!status.available);
    }

    #[test]
    fn suspend_preserves_generation_but_does_not_hide_a_step() {
        for step in [0_i64, 5_000_000_000, -2_000_000_000] {
            let reader = FakeClockReader {
                monotonic: VecDeque::from([0, 0, 100, 100, 200, 200]),
                boottime: VecDeque::from([
                    0,
                    0,
                    0,
                    30_000_000_100,
                    30_000_000_100,
                    30_000_000_100,
                    30_000_000_200,
                    30_000_000_200,
                    30_000_000_200,
                ]),
                realtime: VecDeque::from([
                    10_000_000_000,
                    40_000_000_100 + step,
                    40_000_000_200 + step,
                ]),
                resolution_ns: 1,
            };
            let mut observer = ReferenceObserver::new(config(), reader);
            observer.sample().unwrap();
            let (resumed, _) = observer.sample().unwrap();
            assert_eq!(resumed.generation_changed, step != 0);
            assert_eq!(resumed.steady_received_ns, 100);
            let (next, _) = observer.sample().unwrap();
            assert_eq!(next.generation, if step == 0 { 1 } else { 2 });
            assert!(!next.generation_changed);
        }
    }

    #[test]
    fn completion_read_failure_preserves_generation_change() {
        let reader = FakeClockReader {
            monotonic: VecDeque::from([0, 0, 100, 100, 200, 200]),
            boottime: VecDeque::from([0, 0, 0, 100, 100]),
            realtime: VecDeque::from([1_000, 2_000, 2_100]),
            resolution_ns: 1,
        };
        let mut observer = ReferenceObserver::new(config(), reader);
        observer.sample().unwrap();
        assert!(observer.sample().is_err());
        observer.reader.boottime = VecDeque::from([200, 200, 200]);
        let (sample, _) = observer.sample().unwrap();
        assert_eq!(sample.generation, 2);
        assert!(sample.generation_changed);
    }

    #[test]
    fn shared_frequency_adjustments_preserve_generation_at_long_intervals() {
        for seconds in [1, 1_800] {
            let elapsed = seconds * 1_000_000_000_u64;
            let adjusted = elapsed + elapsed / 2_000;
            let reader = FakeClockReader {
                monotonic: VecDeque::from([0, 0, adjusted, adjusted]),
                boottime: VecDeque::from([0, 0, 0, adjusted, adjusted, adjusted]),
                realtime: VecDeque::from([1_000_000_000, 1_000_000_000 + adjusted as i64]),
                resolution_ns: 1,
            };
            let mut observer = ReferenceObserver::new(ReferenceConfig::default(), reader);
            observer.sample().unwrap();
            assert!(!observer.sample().unwrap().0.generation_changed);
        }
    }
}
