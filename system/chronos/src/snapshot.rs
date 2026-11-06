// SPDX-License-Identifier: MulanPSL-2.0

use crate::model::{ClockModel, DomainState};

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
