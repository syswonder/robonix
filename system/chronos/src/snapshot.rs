// SPDX-License-Identifier: MulanPSL-2.0

use crate::config::TimestampScaleKind;
use crate::model::{ClockModel, DomainState, DomainStatus};
use crate::runtime::{
    ClockQualityReport, DirectDomainQuality, DomainQualityReport, QualityRefreshFailure,
    QualityRuntimeStatus,
};
use crate::virtual_clock::{VirtualReferenceTime, VirtualSteadyTime};

/// Transport-neutral snapshot of current quality and refresh health.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClockQualitySnapshot {
    pub report: Option<ClockQualityReportSnapshot>,
    pub refresh_health: RefreshHealthSnapshot,
}

impl From<&QualityRuntimeStatus> for ClockQualitySnapshot {
    /// Preserve an initial refresh failure without inventing clock evidence.
    fn from(status: &QualityRuntimeStatus) -> Self {
        Self {
            report: status.report.as_ref().map(ClockQualityReportSnapshot::from),
            refresh_health: RefreshHealthSnapshot {
                last_successful_refresh_steady: status.last_successful_refresh_steady,
                failure: status.refresh_failure.clone(),
            },
        }
    }
}

/// One coherent reference and domain report prepared for publication.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClockQualityReportSnapshot {
    pub observed_steady_time: VirtualSteadyTime,
    pub reference: ReferenceQualitySnapshot,
    pub direct_domains: Vec<DirectDomainQualitySnapshot>,
    pub affine_domains: Vec<AffineDomainQualitySnapshot>,
}

impl From<&ClockQualityReport> for ClockQualityReportSnapshot {
    /// Split strategy-specific reports so consumers never inspect invalid fields.
    fn from(report: &ClockQualityReport) -> Self {
        let mut direct_domains = Vec::new();
        let mut affine_domains = Vec::new();
        for domain in &report.domains {
            match domain {
                DomainQualityReport::Direct(domain) => {
                    direct_domains.push(DirectDomainQualitySnapshot::from(domain));
                }
                DomainQualityReport::AffineModel(domain) => {
                    affine_domains.push(AffineDomainQualitySnapshot::from(domain));
                }
            }
        }
        Self {
            observed_steady_time: report.steady,
            reference: ReferenceQualitySnapshot::from(&report.reference),
            direct_domains,
            affine_domains,
        }
    }
}

/// Reference-clock evidence included in an external quality snapshot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReferenceQualitySnapshot {
    pub timestamp_ns: i64,
    pub generation: u64,
    pub generation_changed: bool,
    pub uncertainty_ns: u64,
    pub available: bool,
    pub verified: bool,
    pub reason: String,
}

impl From<&VirtualReferenceTime> for ReferenceQualitySnapshot {
    /// Copy reference evidence without changing its availability semantics.
    fn from(reference: &VirtualReferenceTime) -> Self {
        Self {
            timestamp_ns: reference.timestamp_ns,
            generation: reference.generation,
            generation_changed: reference.generation_changed,
            uncertainty_ns: reference.uncertainty_ns,
            available: reference.available,
            verified: reference.verified,
            reason: reference.reason.clone(),
        }
    }
}

/// Success time and an optional continuing refresh failure.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RefreshHealthSnapshot {
    pub last_successful_refresh_steady: Option<VirtualSteadyTime>,
    pub failure: Option<QualityRefreshFailure>,
}

/// Quality of a timestamp domain declared directly aligned to the reference.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DirectDomainQualitySnapshot {
    pub domain_id: String,
    pub time_scale: TimestampScaleKind,
    pub reference_generation: u64,
    pub reference_available: bool,
    pub reference_verified: bool,
    pub uncertainty_ns: u64,
    pub reason: String,
}

impl From<&DirectDomainQuality> for DirectDomainQualitySnapshot {
    /// Copy configured direct-alignment quality for transport adapters.
    fn from(domain: &DirectDomainQuality) -> Self {
        Self {
            domain_id: domain.domain_id.clone(),
            time_scale: domain.time_scale,
            reference_generation: domain.reference_generation,
            reference_available: domain.reference_available,
            reference_verified: domain.reference_verified,
            uncertainty_ns: domain.uncertainty_ns,
            reason: domain.reason.clone(),
        }
    }
}

/// Quality of a timestamp domain whose affine relationship is estimated.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AffineDomainQualitySnapshot {
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

impl From<&DomainStatus> for AffineDomainQualitySnapshot {
    /// Copy the current affine-domain identity and quality status.
    fn from(domain: &DomainStatus) -> Self {
        Self {
            domain_id: domain.domain_id.clone(),
            generation: domain.generation,
            reference_generation: domain.reference_generation,
            model_version: domain.model_version,
            state: domain.state,
            accepted_observations: domain.accepted_observations,
            rejected_observations: domain.rejected_observations,
            last_observation_age_ns: domain.last_observation_age_ns,
            uncertainty_ns: domain.uncertainty_ns,
            reason: domain.reason.clone(),
        }
    }
}

/// Coherent model view for every configured affine domain.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClockModelsSnapshot {
    pub published_steady_time: VirtualSteadyTime,
    pub reference_generation: u64,
    pub domains: Vec<ClockModelSnapshot>,
}

/// Current domain identity plus an optional retained conversion model.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClockModelSnapshot {
    pub domain_id: String,
    pub generation: u64,
    pub reference_generation: u64,
    pub model_version: u64,
    pub state: DomainState,
    pub reason: String,
    pub model: Option<ClockModelParametersSnapshot>,
}

impl ClockModelSnapshot {
    /// Keep current status separate from a retained model's older identity.
    pub(crate) fn new(status: &DomainStatus, model: Option<&ClockModel>) -> Self {
        Self {
            domain_id: status.domain_id.clone(),
            generation: status.generation,
            reference_generation: status.reference_generation,
            model_version: status.model_version,
            state: status.state,
            reason: status.reason.clone(),
            model: model.map(ClockModelParametersSnapshot::from),
        }
    }
}

/// Versioned affine coefficients that consumers can evaluate locally.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClockModelParametersSnapshot {
    pub generation: u64,
    pub reference_generation: u64,
    pub model_version: u64,
    pub state: DomainState,
    pub device_anchor_ns: i64,
    pub reference_anchor_ns: i64,
    pub rate_adjust_ppb: i64,
    pub residual_offset_ns: i64,
    pub uncertainty_ns: u64,
    pub valid_from_device_ns: i64,
    pub valid_until_device_ns: i64,
    pub sample_count: u64,
    pub last_observation_steady_ns: u64,
    pub latest_raw_device_ns: i64,
    pub wrap_period_ns: Option<u64>,
    pub wrap_offset_ns: i64,
}

impl From<&ClockModel> for ClockModelParametersSnapshot {
    /// Copy every coefficient and wrap-normalization input needed locally.
    fn from(model: &ClockModel) -> Self {
        Self {
            generation: model.generation(),
            reference_generation: model.reference_generation(),
            model_version: model.model_version(),
            state: model.state(),
            device_anchor_ns: model.device_anchor_ns(),
            reference_anchor_ns: model.reference_anchor_ns(),
            rate_adjust_ppb: model.rate_adjust_ppb(),
            residual_offset_ns: model.residual_offset_ns(),
            uncertainty_ns: model.uncertainty_ns(),
            valid_from_device_ns: model.valid_from_device_ns(),
            valid_until_device_ns: model.valid_until_device_ns(),
            sample_count: model.sample_count(),
            last_observation_steady_ns: model.last_observation_steady_ns(),
            latest_raw_device_ns: model.latest_raw_device_ns(),
            wrap_period_ns: model.wrap_period_ns(),
            wrap_offset_ns: model.wrap_offset_ns(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    /// Published domain-state codes remain stable across declaration changes.
    fn domain_state_wire_codes_are_explicit() {
        assert_eq!(DomainState::Initializing as u8, 0);
        assert_eq!(DomainState::Tracking as u8, 1);
        assert_eq!(DomainState::Holdover as u8, 2);
        assert_eq!(DomainState::Reset as u8, 3);
        assert_eq!(DomainState::Unsynced as u8, 4);
        assert_eq!(DomainState::Fault as u8, 5);
    }

    #[test]
    /// Published timestamp-scale codes remain stable for generated IDL.
    fn timestamp_scale_wire_codes_are_explicit() {
        assert_eq!(TimestampScaleKind::LinuxSystem as u8, 0);
        assert_eq!(TimestampScaleKind::RosTime as u8, 1);
        assert_eq!(TimestampScaleKind::DeviceUptime as u8, 2);
    }
}
