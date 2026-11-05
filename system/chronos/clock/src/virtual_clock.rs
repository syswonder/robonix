// SPDX-License-Identifier: MulanPSL-2.0

#[cfg(target_os = "linux")]
use crate::config::{ChronosConfigError, ReferenceConfig, ReferenceSourceKind};
#[cfg(target_os = "linux")]
use crate::reference::{LinuxSystemReference, ReferenceError, read_linux_monotonic_ns};

/// One best-effort reading of the configured reference-time scale.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(frozen, get_all, skip_from_py_object, module = "robonix_chronos_clock")
)]
pub struct VirtualReferenceTime {
    pub timestamp_ns: i64,
    pub generation: u64,
    pub generation_changed: bool,
    pub uncertainty_ns: u64,
    pub available: bool,
    pub verified: bool,
    pub reason: String,
}

/// One local monotonic reading for durations, deadlines, and status age.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(frozen, get_all, skip_from_py_object, module = "robonix_chronos_clock")
)]
pub struct VirtualSteadyTime {
    pub timestamp_ns: u64,
}

/// A reference reading paired with local steady time; Linux uses the sampling midpoint.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(frozen, get_all, skip_from_py_object, module = "robonix_chronos_clock")
)]
pub struct VirtualClockSnapshot {
    pub reference: VirtualReferenceTime,
    pub steady: VirtualSteadyTime,
}

/// Read-only virtual clock backed by the explicitly configured reference.
#[derive(Debug)]
#[cfg(target_os = "linux")]
pub struct VirtualClock {
    reference: Option<LinuxSystemReference>,
    generation: Option<u64>,
    max_uncertainty_ns: u64,
    max_status_age_ns: u64,
}

#[cfg(target_os = "linux")]
impl VirtualClock {
    /// Validate and construct the selected reference-clock implementation.
    pub fn new(config: ReferenceConfig) -> Result<Self, ChronosConfigError> {
        config.validate()?;
        let max_uncertainty_ns = config.max_uncertainty_ns;
        let max_status_age_ns = config.max_status_age_ns;
        let reference = match config.kind {
            ReferenceSourceKind::LinuxSystem => Some(LinuxSystemReference::new(config)?),
            ReferenceSourceKind::RosTime => None,
        };
        Ok(Self {
            reference,
            generation: None,
            max_uncertainty_ns,
            max_status_age_ns,
        })
    }

    /// Pair a Linux sample or the latest cached ROS reference with local steady time.
    ///
    /// Reference discontinuities remain visible and advance generation; they
    /// are never hidden by synthesizing a replacement timeline from steady
    /// time. Quality failure is returned as metadata unless the clock read
    /// itself cannot be completed.
    pub fn snapshot(&mut self) -> Result<VirtualClockSnapshot, ReferenceError> {
        let Some(reference) = &mut self.reference else {
            let cached = crate::ros_reference_time()?;
            let steady = self.steady_time()?;
            let Some((mut reference, received_ns)) = cached else {
                return Ok(VirtualClockSnapshot {
                    reference: VirtualReferenceTime {
                        timestamp_ns: 0,
                        generation: 1,
                        generation_changed: false,
                        uncertainty_ns: 0,
                        available: false,
                        verified: false,
                        reason: "/clock has not supplied reference time".into(),
                    },
                    steady,
                });
            };
            let age = steady
                .timestamp_ns
                .checked_sub(received_ns)
                .ok_or(ReferenceError::MonotonicRegression)?;
            if reference.available && age > self.max_status_age_ns {
                reference.available = false;
                reference.reason = "reference clock sample is stale".into();
            }
            if reference.available && reference.uncertainty_ns > self.max_uncertainty_ns {
                reference.available = false;
                reference.reason = "reference clock uncertainty exceeds configured limit".into();
            }
            reference.generation_changed = self
                .generation
                .is_some_and(|generation| generation != reference.generation);
            self.generation = Some(reference.generation);
            return Ok(VirtualClockSnapshot { reference, steady });
        };
        let (sample, status) = reference.sample_with_status()?;
        Ok(VirtualClockSnapshot {
            steady: VirtualSteadyTime {
                timestamp_ns: sample.steady_received_ns,
            },
            reference: VirtualReferenceTime {
                timestamp_ns: sample.reference_timestamp_ns,
                generation: sample.generation,
                generation_changed: sample.generation_changed,
                uncertainty_ns: sample.uncertainty_ns,
                available: status.available,
                verified: status.verified,
                reason: status.reason,
            },
        })
    }

    /// Read reference time with quality metadata, discarding only its pair.
    pub fn reference_time(&mut self) -> Result<VirtualReferenceTime, ReferenceError> {
        Ok(self.snapshot()?.reference)
    }

    /// Read local steady time without depending on reference-clock health.
    pub fn steady_time(&self) -> Result<VirtualSteadyTime, ReferenceError> {
        Ok(VirtualSteadyTime {
            timestamp_ns: read_linux_monotonic_ns()?,
        })
    }
}
