// SPDX-License-Identifier: MulanPSL-2.0

use super::pb::chronos as wire;
use crate::snapshot::{
    ClockModelParametersSnapshot, ClockModelsSnapshot, ClockQualitySnapshot,
    ReferenceQualitySnapshot,
};

pub(super) fn reference(value: ReferenceQualitySnapshot) -> wire::ReferenceQuality {
    wire::ReferenceQuality {
        timestamp_ns: value.timestamp_ns,
        generation: value.generation,
        generation_changed: value.generation_changed,
        uncertainty_ns: value.uncertainty_ns,
        available: value.available,
        verified: value.verified,
        reason: value.reason,
    }
}

/// Encode optional clock evidence explicitly; absent payloads use IDL defaults.
pub(super) fn quality(snapshot: ClockQualitySnapshot) -> wire::ClockQualitySnapshot {
    let health = snapshot.refresh_health;
    let refresh_health = wire::RefreshHealth {
        has_last_successful_refresh_steady: health.last_successful_refresh_steady.is_some(),
        last_successful_refresh_steady_ns: health
            .last_successful_refresh_steady
            .map_or(0, |v| v.timestamp_ns),
        has_failure: health.failure.is_some(),
        failure: Some(
            health
                .failure
                .map_or_else(wire::RefreshFailure::default, |v| wire::RefreshFailure {
                    message: v.message,
                    consecutive_failures: v.consecutive_failures,
                    has_first_observed_steady: v.first_observed_steady.is_some(),
                    first_observed_steady_ns: v.first_observed_steady.map_or(0, |t| t.timestamp_ns),
                    has_last_observed_steady: v.last_observed_steady.is_some(),
                    last_observed_steady_ns: v.last_observed_steady.map_or(0, |t| t.timestamp_ns),
                }),
        ),
    };
    let Some(report) = snapshot.report else {
        return wire::ClockQualitySnapshot {
            refresh_health: Some(refresh_health),
            reference: Some(wire::ReferenceQuality::default()),
            ..Default::default()
        };
    };
    wire::ClockQualitySnapshot {
        report_available: true,
        observed_steady_ns: report.observed_steady_time.timestamp_ns,
        reference: Some(reference(report.reference)),
        refresh_health: Some(refresh_health),
        direct_domains: report
            .direct_domains
            .into_iter()
            .map(|v| wire::DirectDomainQuality {
                domain_id: v.domain_id,
                time_scale: v.time_scale as u32,
                reference_generation: v.reference_generation,
                reference_available: v.reference_available,
                reference_verified: v.reference_verified,
                uncertainty_ns: v.uncertainty_ns,
                reason: v.reason,
            })
            .collect(),
        affine_domains: report
            .affine_domains
            .into_iter()
            .map(|v| wire::AffineDomainQuality {
                domain_id: v.domain_id,
                generation: v.generation,
                reference_generation: v.reference_generation,
                model_version: v.model_version,
                state: v.state as u32,
                accepted_observations: v.accepted_observations,
                rejected_observations: v.rejected_observations,
                has_last_observation_age: v.last_observation_age_ns.is_some(),
                last_observation_age_ns: v.last_observation_age_ns.unwrap_or_default(),
                has_uncertainty: v.uncertainty_ns.is_some(),
                uncertainty_ns: v.uncertainty_ns.unwrap_or_default(),
                reason: v.reason,
            })
            .collect(),
    }
}

/// Preserve current domain identity separately from retained coefficients.
pub(super) fn models(snapshot: ClockModelsSnapshot) -> wire::ClockModelsSnapshot {
    wire::ClockModelsSnapshot {
        published_steady_ns: snapshot.published_steady_time.timestamp_ns,
        reference_generation: snapshot.reference_generation,
        domains: snapshot
            .domains
            .into_iter()
            .map(|v| wire::ClockModelSnapshot {
                domain_id: v.domain_id,
                generation: v.generation,
                reference_generation: v.reference_generation,
                model_version: v.model_version,
                state: v.state as u32,
                reason: v.reason,
                has_model: v.model.is_some(),
                model: Some(
                    v.model
                        .map_or_else(wire::ClockModelParameters::default, parameters),
                ),
            })
            .collect(),
    }
}

/// Copy all integer coefficients and wrap inputs used for local conversion.
fn parameters(v: ClockModelParametersSnapshot) -> wire::ClockModelParameters {
    wire::ClockModelParameters {
        generation: v.generation,
        reference_generation: v.reference_generation,
        model_version: v.model_version,
        state: v.state as u32,
        device_anchor_ns: v.device_anchor_ns,
        reference_anchor_ns: v.reference_anchor_ns,
        rate_adjust_ppb: v.rate_adjust_ppb,
        residual_offset_ns: v.residual_offset_ns,
        uncertainty_ns: v.uncertainty_ns,
        valid_from_device_ns: v.valid_from_device_ns,
        valid_until_device_ns: v.valid_until_device_ns,
        sample_count: v.sample_count,
        last_observation_steady_ns: v.last_observation_steady_ns,
        latest_raw_device_ns: v.latest_raw_device_ns,
        has_wrap_period: v.wrap_period_ns.is_some(),
        wrap_period_ns: v.wrap_period_ns.unwrap_or_default(),
        wrap_offset_ns: v.wrap_offset_ns,
    }
}
