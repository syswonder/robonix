// SPDX-License-Identifier: MulanPSL-2.0

use serde::Deserialize;
use thiserror::Error;

/// Reference clock sources accepted by the current Chronos implementation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReferenceSourceKind {
    LinuxSystem,
    RosTime,
}

/// Quality and freshness limits for the selected reference clock.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceConfig {
    pub kind: ReferenceSourceKind,
    #[serde(default = "default_max_uncertainty_ns")]
    pub max_uncertainty_ns: u64,
    #[serde(default = "default_max_status_age_ns")]
    pub max_status_age_ns: u64,
    #[serde(default = "default_max_clock_step_ns")]
    pub max_clock_step_ns: u64,
}

impl Default for ReferenceConfig {
    /// Select Linux system time with the same limits as omitted YAML settings.
    fn default() -> Self {
        Self {
            kind: ReferenceSourceKind::LinuxSystem,
            max_uncertainty_ns: default_max_uncertainty_ns(),
            max_status_age_ns: default_max_status_age_ns(),
            max_clock_step_ns: default_max_clock_step_ns(),
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

/// Errors reported while parsing or validating Chronos configuration.
#[derive(Debug, Error)]
pub enum ChronosConfigError {
    #[error("invalid Chronos configuration: {0}")]
    Parse(#[from] serde_yaml::Error),
    #[error("invalid reference clock configuration: {0}")]
    InvalidReference(String),
    #[error("invalid quality runtime configuration: {0}")]
    InvalidQuality(String),
    #[error("invalid clock domain '{domain_id}': {reason}")]
    InvalidDomain { domain_id: String, reason: String },
}

fn default_max_uncertainty_ns() -> u64 {
    1_000_000
}

fn default_max_status_age_ns() -> u64 {
    2_000_000_000
}

fn default_max_clock_step_ns() -> u64 {
    100_000_000
}
