// SPDX-License-Identifier: MulanPSL-2.0

use thiserror::Error;

/// Reference clock sources accepted by the current Chronos implementation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReferenceSourceKind {
    LinuxSystem,
    RosTime,
}

/// Quality and freshness limits for the selected reference clock.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReferenceConfig {
    pub kind: ReferenceSourceKind,
    pub max_uncertainty_ns: u64,
    pub max_status_age_ns: u64,
    pub max_clock_step_ns: u64,
}

impl Default for ReferenceConfig {
    fn default() -> Self {
        Self {
            kind: ReferenceSourceKind::LinuxSystem,
            max_uncertainty_ns: 1_000_000,
            max_status_age_ns: 2_000_000_000,
            max_clock_step_ns: 100_000_000,
        }
    }
}

impl ReferenceConfig {
    /// Reject zero quality limits that would make every sample unusable.
    pub fn validate(&self) -> Result<(), ChronosConfigError> {
        if self.max_uncertainty_ns == 0
            || self.max_status_age_ns == 0
            || self.max_clock_step_ns == 0
        {
            return Err(ChronosConfigError::InvalidReference(
                "reference limits must be positive".to_string(),
            ));
        }
        Ok(())
    }
}

/// Errors reported while validating Chronos configuration.
#[derive(Debug, Error)]
pub enum ChronosConfigError {
    #[error("invalid reference clock configuration: {0}")]
    InvalidReference(String),
}
