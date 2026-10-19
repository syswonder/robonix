// SPDX-License-Identifier: MulanPSL-2.0

use super::*;
use crate::snapshot::*;
use crate::{DomainState, QualityRefreshFailure, QualityRuntimeStatus, VirtualSteadyTime};

#[test]
/// Initial failure clears clock payloads while retaining exact failure evidence.
fn initial_failure_encoding_does_not_invent_clock_evidence() {
    let status = QualityRuntimeStatus {
        report: None,
        last_successful_refresh_steady: None,
        refresh_failure: Some(QualityRefreshFailure {
            message: "clock read failed".into(),
            consecutive_failures: 2,
            first_observed_steady: Some(VirtualSteadyTime { timestamp_ns: 42 }),
            last_observed_steady: None,
        }),
    };
    let encoded = encoding::quality(ClockQualitySnapshot::from(&status));
    assert!(!encoded.report_available);
    assert_eq!(encoded.observed_steady_ns, 0);
    assert_eq!(
        encoded.reference.unwrap(),
        pb::chronos::ReferenceQuality::default()
    );
    assert!(encoded.direct_domains.is_empty() && encoded.affine_domains.is_empty());
    let health = encoded.refresh_health.unwrap();
    assert!(!health.has_last_successful_refresh_steady);
    assert_eq!(health.last_successful_refresh_steady_ns, 0);
    assert!(health.has_failure);
    let failure = health.failure.unwrap();
    assert!(failure.has_first_observed_steady);
    assert_eq!(failure.first_observed_steady_ns, 42);
    assert!(!failure.has_last_observed_steady);
    assert_eq!(failure.last_observed_steady_ns, 0);
}

#[test]
/// All absent quality options clear their scalar and nested payloads.
fn absent_quality_payloads_are_cleared() {
    let encoded = encoding::quality(ClockQualitySnapshot {
        report: Some(ClockQualityReportSnapshot {
            observed_steady_time: VirtualSteadyTime { timestamp_ns: 7 },
            reference: ReferenceQualitySnapshot {
                timestamp_ns: 123,
                generation: 4,
                generation_changed: true,
                uncertainty_ns: 9,
                available: true,
                verified: false,
                reason: "unverified".into(),
            },
            direct_domains: vec![],
            affine_domains: vec![AffineDomainQualitySnapshot {
                domain_id: "device".into(),
                generation: 2,
                reference_generation: 4,
                model_version: 0,
                state: DomainState::Initializing,
                accepted_observations: 0,
                rejected_observations: 0,
                last_observation_age_ns: None,
                uncertainty_ns: None,
                reason: "collecting".into(),
            }],
        }),
        refresh_health: RefreshHealthSnapshot {
            last_successful_refresh_steady: None,
            failure: None,
        },
    });
    assert!(encoded.report_available);
    let domain = &encoded.affine_domains[0];
    assert!(!domain.has_last_observation_age && !domain.has_uncertainty);
    assert_eq!(domain.last_observation_age_ns, 0);
    assert_eq!(domain.uncertainty_ns, 0);
    let health = encoded.refresh_health.unwrap();
    assert!(!health.has_failure);
    assert_eq!(
        health.failure.unwrap(),
        pb::chronos::RefreshFailure::default()
    );
}
