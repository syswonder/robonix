// SPDX-License-Identifier: MulanPSL-2.0

use serde::Deserialize;
use thiserror::Error;

pub(crate) const PARTS_PER_BILLION: i128 = 1_000_000_000;

/// Keep configured and observed domain identities within the same wire limits.
pub(crate) fn validate_domain_id(domain_id: &str) -> Result<(), &'static str> {
    if domain_id.is_empty() || domain_id.len() > 128 || domain_id.trim() != domain_id {
        return Err("domain_id must contain 1..128 bytes without surrounding whitespace");
    }
    Ok(())
}

/// Synchronization state of one device clock domain.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DomainState {
    Initializing = 0,
    Tracking = 1,
    Holdover = 2,
    Reset = 3,
    Unsynced = 4,
    Fault = 5,
}

/// Static facts needed by the pure core to interpret a device clock domain.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClockDomainDescriptor {
    pub domain_id: String,
    pub resolution_ns: u64,
    pub wrap_period_ns: Option<u64>,
}

impl ClockDomainDescriptor {
    /// Reject identities and units that cannot produce an unambiguous model.
    pub fn validate(&self) -> Result<(), ChronosError> {
        validate_domain_id(&self.domain_id)
            .map_err(|reason| ChronosError::InvalidDomain(reason.into()))?;
        if self.resolution_ns == 0 {
            return Err(ChronosError::InvalidDomain(
                "resolution_ns must be positive".to_string(),
            ));
        }
        if let Some(period) = self.wrap_period_ns
            && (!(2..=i64::MAX as u64).contains(&period))
        {
            return Err(ChronosError::InvalidDomain(
                "wrap_period_ns must fit a signed timestamp and contain at least two values"
                    .to_string(),
            ));
        }
        Ok(())
    }
}

/// How fitted offset corrections are published within one clock generation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum ModelUpdatePolicy {
    /// Publish new coefficients immediately, allowing forward or backward corrections.
    Step {},
    /// Bound offset correction per observation by elapsed steady time; rates remain fitted.
    Slew { max_correction_ppm: u64 },
}

impl Default for ModelUpdatePolicy {
    fn default() -> Self {
        Self::Step {}
    }
}

/// Per-domain limits used for model maturity, freshness, quality and offset correction.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClockDomainPolicy {
    pub minimum_observations: usize,
    pub observation_window: usize,
    pub max_drift_ppm: u64,
    pub max_residual_ns: u64,
    pub tracking_timeout_ns: u64,
    pub holdover_duration_ns: u64,
    pub max_uncertainty_ns: u64,
    pub model_update: ModelUpdatePolicy,
}

impl ClockDomainPolicy {
    /// Reject limits that would prevent model fitting or bounded holdover.
    pub fn validate(&self) -> Result<(), ChronosError> {
        if self.minimum_observations < 2 {
            return Err(ChronosError::InvalidPolicy(
                "minimum_observations must be at least two".to_string(),
            ));
        }
        if self.observation_window < self.minimum_observations {
            return Err(ChronosError::InvalidPolicy(
                "observation_window must cover minimum_observations".to_string(),
            ));
        }
        if self.max_residual_ns == 0 || self.max_uncertainty_ns == 0 {
            return Err(ChronosError::InvalidPolicy(
                "residual and uncertainty limits must be positive".to_string(),
            ));
        }
        if let ModelUpdatePolicy::Slew { max_correction_ppm } = self.model_update
            && !(1..=1_000_000).contains(&max_correction_ppm)
        {
            return Err(ChronosError::InvalidPolicy(
                "slew max_correction_ppm must be between 1 and 1000000".to_string(),
            ));
        }
        Ok(())
    }
}

/// One paired device/reference observation, timestamped by a monotonic clock.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClockObservation {
    pub sequence_id: u64,
    pub device_timestamp_ns: i64,
    pub reference_timestamp_ns: i64,
    pub uncertainty_ns: u64,
    pub steady_received_ns: u64,
}

/// Immutable, versioned affine relationship between one device clock and the reference.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClockModel {
    pub(crate) domain_id: String,
    pub(crate) generation: u64,
    pub(crate) reference_generation: u64,
    pub(crate) model_version: u64,
    pub(crate) device_anchor_ns: i64,
    pub(crate) reference_anchor_ns: i64,
    pub(crate) rate_adjust_ppb: i64,
    pub(crate) residual_offset_ns: i64,
    pub(crate) uncertainty_ns: u64,
    pub(crate) valid_from_device_ns: i64,
    pub(crate) valid_until_device_ns: i64,
    pub(crate) sample_count: u64,
    pub(crate) state: DomainState,
    pub(crate) last_observation_steady_ns: u64,
    pub(crate) latest_raw_device_ns: i64,
    pub(crate) wrap_period_ns: Option<u64>,
    pub(crate) wrap_offset_ns: i64,
}

impl ClockModel {
    /// Return the clock domain represented by this model.
    pub fn domain_id(&self) -> &str {
        &self.domain_id
    }

    /// Return the generation that invalidates models from earlier resets.
    pub fn generation(&self) -> u64 {
        self.generation
    }

    /// Return the reference-clock generation used to fit this model.
    pub fn reference_generation(&self) -> u64 {
        self.reference_generation
    }

    /// Return the monotonically increasing model version within a generation.
    pub fn model_version(&self) -> u64 {
        self.model_version
    }

    /// Return the device anchor used by the affine relationship.
    pub fn device_anchor_ns(&self) -> i64 {
        self.device_anchor_ns
    }

    /// Return the reference anchor used by the affine relationship.
    pub fn reference_anchor_ns(&self) -> i64 {
        self.reference_anchor_ns
    }

    /// Return the signed frequency correction in parts per billion.
    pub fn rate_adjust_ppb(&self) -> i64 {
        self.rate_adjust_ppb
    }

    /// Return the residual offset applied after the anchor and rate terms.
    pub fn residual_offset_ns(&self) -> i64 {
        self.residual_offset_ns
    }

    /// Return the model uncertainty in nanoseconds.
    pub fn uncertainty_ns(&self) -> u64 {
        self.uncertainty_ns
    }

    /// Return the first normalized device timestamp covered by the model.
    pub fn valid_from_device_ns(&self) -> i64 {
        self.valid_from_device_ns
    }

    /// Return the final normalized device timestamp covered by the model.
    pub fn valid_until_device_ns(&self) -> i64 {
        self.valid_until_device_ns
    }

    /// Return the number of observations represented by the current fit.
    pub fn sample_count(&self) -> u64 {
        self.sample_count
    }

    /// Return the synchronization state attached to this model version.
    pub fn state(&self) -> DomainState {
        self.state
    }

    /// Return when the last accepted observation reached the core.
    pub fn last_observation_steady_ns(&self) -> u64 {
        self.last_observation_steady_ns
    }

    /// Return the latest raw counter used to select an unambiguous wrap epoch.
    pub fn latest_raw_device_ns(&self) -> i64 {
        self.latest_raw_device_ns
    }

    /// Return the declared raw counter period, if the device clock wraps.
    pub fn wrap_period_ns(&self) -> Option<u64> {
        self.wrap_period_ns
    }

    /// Return the generation-local offset applied before affine conversion.
    pub fn wrap_offset_ns(&self) -> i64 {
        self.wrap_offset_ns
    }

    /// Convert whenever the retained coefficients define an unambiguous value.
    ///
    /// Synchronization state and validity are reported with the estimate rather
    /// than used as quality gates. Wrapping clocks remain bounded to the range
    /// in which a raw counter can be assigned to one unambiguous wrap epoch.
    /// Extrapolation outside a non-wrapping model's validity reports unbounded
    /// uncertainty rather than presenting the estimate as policy-bounded.
    pub fn convert(&self, device_timestamp_ns: i64) -> Result<ConversionResult, ChronosError> {
        let normalized = self.normalize_device_timestamp(device_timestamp_ns)?;
        let within_validity = normalized >= i128::from(self.valid_from_device_ns)
            && normalized <= i128::from(self.valid_until_device_ns);
        if self.wrap_period_ns.is_some() && !within_validity {
            return Err(ChronosError::TimestampOutsideModel);
        }
        let delta = normalized - i128::from(self.device_anchor_ns);
        let rate = divide_rounded(
            delta
                .checked_mul(i128::from(self.rate_adjust_ppb))
                .ok_or(ChronosError::Overflow)?,
            PARTS_PER_BILLION,
        );
        let reference = i128::from(self.reference_anchor_ns)
            .checked_add(delta)
            .and_then(|value| value.checked_add(rate))
            .and_then(|value| value.checked_add(i128::from(self.residual_offset_ns)))
            .ok_or(ChronosError::Overflow)?;
        Ok(ConversionResult {
            reference_timestamp_ns: i64::try_from(reference).map_err(|_| ChronosError::Overflow)?,
            generation: self.generation,
            reference_generation: self.reference_generation,
            model_version: self.model_version,
            uncertainty_ns: if within_validity {
                self.uncertainty_ns
            } else {
                u64::MAX
            },
            state: self.state,
            within_validity,
            last_observation_steady_ns: self.last_observation_steady_ns,
        })
    }

    /// Map a raw counter near the latest observation into the model generation.
    fn normalize_device_timestamp(&self, raw: i64) -> Result<i128, ChronosError> {
        if let Some(period) = self.wrap_period_ns
            && (raw < 0 || raw >= period as i64)
        {
            return Err(ChronosError::InvalidObservation(
                "device timestamp is outside the declared wrap period".to_string(),
            ));
        }
        let mut normalized = i128::from(raw)
            .checked_add(i128::from(self.wrap_offset_ns))
            .ok_or(ChronosError::Overflow)?;
        if let Some(period) = self.wrap_period_ns {
            let period = i128::from(period);
            let difference = i128::from(raw) - i128::from(self.latest_raw_device_ns);
            if period % 2 == 0 && difference.abs() == period / 2 {
                return Err(ChronosError::TimestampOutsideModel);
            }
            if difference < -(period / 2) {
                normalized = normalized
                    .checked_add(period)
                    .ok_or(ChronosError::Overflow)?;
            } else if difference > period / 2 {
                normalized = normalized
                    .checked_sub(period)
                    .ok_or(ChronosError::Overflow)?;
            }
        }
        Ok(normalized)
    }

    /// Evaluate a normalized timestamp without applying validity or state gates.
    pub(crate) fn extrapolate_normalized(&self, normalized: i64) -> Result<i64, ChronosError> {
        let delta = i128::from(normalized) - i128::from(self.device_anchor_ns);
        let rate = divide_rounded(
            delta
                .checked_mul(i128::from(self.rate_adjust_ppb))
                .ok_or(ChronosError::Overflow)?,
            PARTS_PER_BILLION,
        );
        let reference = i128::from(self.reference_anchor_ns)
            .checked_add(delta)
            .and_then(|value| value.checked_add(rate))
            .and_then(|value| value.checked_add(i128::from(self.residual_offset_ns)))
            .ok_or(ChronosError::Overflow)?;
        i64::try_from(reference).map_err(|_| ChronosError::Overflow)
    }
}

/// Result of a model conversion, carrying the exact model identity used.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConversionResult {
    pub reference_timestamp_ns: i64,
    pub generation: u64,
    pub reference_generation: u64,
    pub model_version: u64,
    pub uncertainty_ns: u64,
    pub state: DomainState,
    pub within_validity: bool,
    pub last_observation_steady_ns: u64,
}

/// Current synchronization status independent of whether a model is available.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DomainStatus {
    pub domain_id: String,
    pub generation: u64,
    pub reference_generation: u64,
    pub model_version: u64,
    pub state: DomainState,
    pub accepted_observations: u64,
    pub rejected_observations: u64,
    pub last_observation_age_ns: Option<u64>,
    pub uncertainty_ns: Option<u64>,
    pub reason: String,
}

/// Errors produced by domain validation, observation processing, and conversion.
#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum ChronosError {
    #[error("unknown clock domain '{0}'")]
    UnknownDomain(String),
    #[error("clock domain '{0}' is already registered")]
    DuplicateDomain(String),
    #[error("invalid clock domain: {0}")]
    InvalidDomain(String),
    #[error("invalid clock domain policy: {0}")]
    InvalidPolicy(String),
    #[error("observation sequence or monotonic receipt time is not newer")]
    ObservationOutOfOrder,
    #[error("reference generation is not newer: current {current}, received {received}")]
    ReferenceGenerationOutOfOrder { current: u64, received: u64 },
    #[error("reference clock is unavailable")]
    ReferenceUnavailable,
    #[error("invalid clock observation: {0}")]
    InvalidObservation(String),
    #[error("observation residual {residual_ns} ns exceeds {limit_ns} ns")]
    ObservationOutlier { residual_ns: u64, limit_ns: u64 },
    #[error("clock model is not available in state {0:?}")]
    ModelUnavailable(DomainState),
    #[error("timestamp is outside the model validity interval")]
    TimestampOutsideModel,
    #[error("timestamp arithmetic overflow")]
    Overflow,
}

/// Divide with nearest-integer rounding; exact half values round away from zero.
pub(crate) fn divide_rounded(numerator: i128, denominator: i128) -> i128 {
    let quotient = numerator / denominator;
    let remainder = numerator % denominator;
    if remainder == 0 {
        return quotient;
    }
    let magnitude = remainder.abs().saturating_mul(2);
    if magnitude >= denominator.abs() {
        quotient + numerator.signum() * denominator.signum()
    } else {
        quotient
    }
}

#[cfg(test)]
mod tests {
    use super::divide_rounded;

    #[test]
    /// Signed half values use the same nearest-integer rule in both directions.
    fn integer_rounding_is_symmetric_at_half_values() {
        assert_eq!(divide_rounded(1, 2), 1);
        assert_eq!(divide_rounded(-1, 2), -1);
        assert_eq!(divide_rounded(3, 2), 2);
        assert_eq!(divide_rounded(-3, 2), -2);
        assert_eq!(divide_rounded(1, 3), 0);
        assert_eq!(divide_rounded(-1, 3), 0);
    }
}
