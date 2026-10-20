// SPDX-License-Identifier: MulanPSL-2.0

#![cfg(target_os = "linux")]

mod support;

use robonix_chronos_clock::{
    ChronosConfigError, ReferenceConfig, ReferenceSourceKind, VirtualClock,
};

/// Select the Linux reference without service configuration or initialization.
fn config() -> ReferenceConfig {
    ReferenceConfig {
        kind: ReferenceSourceKind::LinuxSystem,
        max_uncertainty_ns: 1_000_000,
        max_status_age_ns: 2_000_000_000,
        max_clock_step_ns: 100_000_000,
    }
}

#[test]
/// Standalone consumers receive a reference reading paired with local steady time.
fn standalone_clock_pairs_reference_and_steady() {
    let mut clock = VirtualClock::new(config()).unwrap();
    let before = clock.steady_time().unwrap();
    let snapshot = clock.snapshot().unwrap();
    let after = clock.steady_time().unwrap();
    assert!(snapshot.reference.timestamp_ns > 0);
    assert!(snapshot.reference.uncertainty_ns > 0);
    assert!(!snapshot.reference.verified);
    assert_eq!(snapshot.reference.generation, 1);
    assert!(snapshot.steady.timestamp_ns >= before.timestamp_ns);
    assert!(snapshot.steady.timestamp_ns <= after.timestamp_ns);
}

#[test]
/// Invalid reference limits retain the configuration error used by Chronos.
fn invalid_limits_are_rejected_before_clock_reading() {
    let mut config = config();
    config.max_status_age_ns = 0;
    assert!(matches!(
        VirtualClock::new(config),
        Err(ChronosConfigError::InvalidReference(_))
    ));
}

#[test]
fn ros_reference_keeps_pause_zero_and_generation_separate_from_wall_time() {
    use robonix_chronos_clock::{VirtualReferenceTime, update_reference_time, wall_timestamp_ns};
    let mut clock = VirtualClock::new(ReferenceConfig {
        kind: ReferenceSourceKind::RosTime,
        ..config()
    })
    .unwrap();
    assert!(!clock.reference_time().unwrap().available);
    let reading = |timestamp_ns, generation| VirtualReferenceTime {
        timestamp_ns,
        generation,
        generation_changed: false,
        uncertainty_ns: 1,
        available: true,
        verified: false,
        reason: "ROS time".into(),
    };
    update_reference_time(reading(0, 10)).unwrap();
    let first = clock.snapshot().unwrap();
    assert_eq!(first.reference.timestamp_ns, 0);
    assert!(first.reference.available);
    std::thread::sleep(std::time::Duration::from_millis(20));
    let paused = clock.snapshot().unwrap();
    assert_eq!(paused.reference.timestamp_ns, 0);
    assert_eq!(paused.reference.generation, 10);
    assert!(paused.steady.timestamp_ns > first.steady.timestamp_ns);
    assert!(wall_timestamp_ns().unwrap() > 0);
    update_reference_time(reading(500_000_000, 10)).unwrap();
    assert!(!clock.reference_time().unwrap().generation_changed);
    assert!(update_reference_time(reading(0, 10)).is_err());
    update_reference_time(reading(0, 11)).unwrap();
    assert!(clock.reference_time().unwrap().generation_changed);
    assert!(!clock.reference_time().unwrap().generation_changed);

    update_reference_time(reading(500, 11)).unwrap();
    let mut unavailable = reading(0, 11);
    unavailable.available = false;
    update_reference_time(unavailable).unwrap();
    assert!(!clock.reference_time().unwrap().available);
    assert!(update_reference_time(reading(250, 11)).is_err());
    update_reference_time(reading(500, 11)).unwrap();
    assert!(clock.reference_time().unwrap().available);

    robonix_chronos_clock::configure_reference(ReferenceConfig {
        kind: ReferenceSourceKind::RosTime,
        max_uncertainty_ns: 1,
        ..config()
    })
    .unwrap();
    let mut uncertain = reading(500, 11);
    uncertain.uncertainty_ns = 2;
    update_reference_time(uncertain).unwrap();
    let status = robonix_chronos_clock::reference_time().unwrap();
    assert!(!status.available);
    assert!(status.reason.contains("uncertainty"));
    assert!(robonix_chronos_clock::reference_timestamp_ns().is_err());
    update_reference_time(reading(500, 11)).unwrap();
    assert_eq!(
        robonix_chronos_clock::reference_timestamp_ns().unwrap(),
        500
    );
}

#[test]
fn ros_generation_change_survives_failed_snapshot() {
    support::run_case("ros_generation_change_survives_failed_snapshot", || {
        use robonix_chronos_clock::{VirtualReferenceTime, update_reference_time};
        let mut clock = VirtualClock::new(ReferenceConfig {
            kind: ReferenceSourceKind::RosTime,
            ..config()
        })
        .unwrap();
        let reading = |generation| VirtualReferenceTime {
            timestamp_ns: 0,
            generation,
            generation_changed: false,
            uncertainty_ns: 1,
            available: true,
            verified: false,
            reason: "ROS time".into(),
        };
        update_reference_time(reading(10)).unwrap();
        clock.snapshot().unwrap();
        update_reference_time(reading(11)).unwrap();
        support::mode(16);
        assert!(clock.snapshot().is_err());
        assert!(clock.snapshot().unwrap().reference.generation_changed);
        assert!(!clock.snapshot().unwrap().reference.generation_changed);
    });
}
