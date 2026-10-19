// SPDX-License-Identifier: MulanPSL-2.0

#![cfg(target_os = "linux")]

use robonix_chronos::{
    Chronos, ChronosConfig, ChronosRuntimeError, DomainQualityReport, DomainState,
};

/// Build a direct domain and a permissive uptime domain for runtime checks.
fn runtime() -> Chronos {
    let config = ChronosConfig::from_yaml(
        r#"
chronos:
  core:
    reference:
      kind: linux_system
    domains:
      camera_clock:
        capabilities:
          - provider_id: camera
            contract_id: robonix/primitive/camera/rgb
            transport: ros2
            timestamp_field: header.stamp
        timestamp:
          event: acquisition
          hardware: true
          time_scale: ros_time
          resolution_ns: 1
          uncertainty_ns: 100
        strategy:
          kind: direct
          aligned_to: linux_system
      lidar_clock:
        capabilities:
          - provider_id: lidar
            contract_id: robonix/primitive/lidar/lidar
            transport: ros2
            timestamp_field: header.stamp
        timestamp:
          event: acquisition
          hardware: true
          time_scale: device_uptime
          resolution_ns: 1
          uncertainty_ns: 100
        strategy:
          kind: affine_model
          observation:
            reference_at: host_receive_time
            sampling_interval_ns: 100
            fixed_delay_ns: 10
            uncertainty_ns: 100
          policy:
            minimum_observations: 2
            observation_window: 4
            max_drift_ppm: 1000000000
            max_residual_ns: 1000000000
            tracking_timeout_ns: 1000000000
            holdover_duration_ns: 1000000000
            max_uncertainty_ns: 1000000000
"#,
    )
    .unwrap();
    Chronos::new(config).unwrap()
}

#[test]
/// Refresh returns one coherent report for both configured strategies.
fn refresh_reports_direct_and_affine_domains() {
    let mut chronos = runtime();
    let report = chronos.refresh().unwrap().clone();
    assert_eq!(report.domains.len(), 2);
    assert_eq!(report.domains[0].domain_id(), "camera_clock");
    assert!(matches!(report.domains[0], DomainQualityReport::Direct(_)));
    assert_eq!(report.domains[1].domain_id(), "lidar_clock");
    assert!(matches!(
        report.domains[1],
        DomainQualityReport::AffineModel(ref status)
            if status.state == DomainState::Initializing
    ));
    assert_eq!(chronos.latest_quality_report(), Some(&report));

    let snapshot = chronos.quality_snapshot();
    let published = snapshot.report.unwrap();
    assert_eq!(published.direct_domains.len(), 1);
    assert_eq!(published.affine_domains.len(), 1);
    assert_eq!(published.direct_domains[0].domain_id, "camera_clock");
    assert_eq!(published.affine_domains[0].domain_id, "lidar_clock");
}

#[test]
/// Host-receive observations fit a model without exposing clock wiring.
fn observations_build_a_queryable_model() {
    let mut chronos = runtime();
    chronos
        .observe_device_timestamp("lidar_clock", 1, 1_000)
        .unwrap();
    chronos
        .observe_device_timestamp("lidar_clock", 2, 2_000)
        .unwrap();

    let model = chronos.model("lidar_clock").unwrap().unwrap();
    assert_eq!(model.domain_id(), "lidar_clock");
    let converted = chronos.convert("lidar_clock", 2_500).unwrap();
    assert!(converted.reference_timestamp_ns > 0);

    let snapshot = chronos.models_snapshot().unwrap().unwrap();
    assert_eq!(snapshot.domains.len(), 1);
    let published = snapshot.domains[0].model.as_ref().unwrap();
    assert_eq!(published.device_anchor_ns, model.device_anchor_ns());
    assert_eq!(published.reference_anchor_ns, model.reference_anchor_ns());
}

#[test]
/// Direct domains explicitly have no affine model or conversion operation.
fn direct_domain_has_no_model() {
    let mut chronos = runtime();
    assert!(matches!(
        chronos.model("camera_clock"),
        Err(ChronosRuntimeError::DirectDomain(domain)) if domain == "camera_clock"
    ));
    assert!(matches!(
        chronos.convert("camera_clock", 1),
        Err(ChronosRuntimeError::DirectDomain(domain)) if domain == "camera_clock"
    ));
    assert!(matches!(
        chronos.observe_device_timestamp("camera_clock", 1, 1),
        Err(ChronosRuntimeError::DirectDomain(domain)) if domain == "camera_clock"
    ));
    assert!(matches!(
        chronos.reset_device_domain("camera_clock"),
        Err(ChronosRuntimeError::DirectDomain(domain)) if domain == "camera_clock"
    ));
}

#[test]
/// Affine and unknown domain queries retain distinct typed outcomes.
fn model_query_distinguishes_initializing_and_unknown_domains() {
    let mut chronos = runtime();
    assert!(chronos.model("lidar_clock").unwrap().is_none());
    assert!(matches!(
        chronos.model("missing_clock"),
        Err(ChronosRuntimeError::UnknownDomain(domain)) if domain == "missing_clock"
    ));
    assert!(matches!(
        chronos.convert("missing_clock", 1),
        Err(ChronosRuntimeError::UnknownDomain(domain)) if domain == "missing_clock"
    ));
    assert!(matches!(
        chronos.reset_device_domain("missing_clock"),
        Err(ChronosRuntimeError::UnknownDomain(domain)) if domain == "missing_clock"
    ));
}

#[test]
/// A rejected observation remains visible in the cached quality report.
fn rejected_observation_updates_quality_report() {
    let mut chronos = runtime();
    chronos
        .observe_device_timestamp("lidar_clock", 1, 1_000)
        .unwrap();
    assert!(
        chronos
            .observe_device_timestamp("lidar_clock", 1, 2_000)
            .is_err()
    );

    let report = chronos.latest_quality_report().unwrap();
    assert!(matches!(
        &report.domains[1],
        DomainQualityReport::AffineModel(status) if status.rejected_observations == 1
    ));
}

#[test]
/// An explicit device reset permits its adapter sequence to restart.
fn reset_device_domain_accepts_restarted_sequence() {
    let mut chronos = runtime();
    chronos
        .observe_device_timestamp("lidar_clock", 10, 1_000)
        .unwrap();

    let reset = chronos.reset_device_domain("lidar_clock").unwrap();
    assert_eq!(reset.state, DomainState::Reset);
    chronos
        .observe_device_timestamp("lidar_clock", 1, 100)
        .unwrap();
}

#[test]
/// Direct quality exposes reference evidence rather than device availability.
fn direct_quality_names_reference_evidence() {
    let mut chronos = runtime();
    let report = chronos.refresh().unwrap();
    let DomainQualityReport::Direct(direct) = &report.domains[0] else {
        panic!("camera domain must use direct quality");
    };
    assert_eq!(direct.reference_available, report.reference.available);
    assert_eq!(direct.reference_verified, report.reference.verified);
    assert_eq!(direct.uncertainty_ns, report.reference.uncertainty_ns + 100);
}
