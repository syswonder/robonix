// SPDX-License-Identifier: MulanPSL-2.0

#![cfg(target_os = "linux")]

mod support;

use robonix_chronos_clock::{
    ChronosConfigError, ReferenceConfig, ReferenceSourceKind, VirtualClock,
};

fn config() -> ReferenceConfig {
    ReferenceConfig::default()
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
    assert!(!first.reference.generation_changed);
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
    update_reference_time(reading(100, 11)).unwrap();
    let mut uncertain = reading(500, 11);
    uncertain.uncertainty_ns = 2;
    update_reference_time(uncertain).unwrap();
    let status = robonix_chronos_clock::reference_time().unwrap();
    assert!(!status.available);
    assert!(status.reason.contains("uncertainty"));
    assert!(robonix_chronos_clock::reference_timestamp_ns().is_err());
    update_reference_time(reading(250, 11)).unwrap();
    assert_eq!(
        robonix_chronos_clock::reference_timestamp_ns().unwrap(),
        250
    );
}

#[test]
fn ros_staleness_and_generation_order() {
    let _ = support::run_case("ros_staleness_and_generation_order", || {
        use robonix_chronos_clock::{
            VirtualReferenceTime, configure_reference, reference_time, update_reference_time,
        };
        configure_reference(ReferenceConfig {
            kind: ReferenceSourceKind::RosTime,
            max_status_age_ns: 50_000_000,
            ..config()
        })
        .unwrap();
        assert!(!reference_time().unwrap().available);
        let reading = |generation, available| VirtualReferenceTime {
            timestamp_ns: 100,
            generation,
            generation_changed: false,
            uncertainty_ns: 1,
            available,
            verified: false,
            reason: "ROS time".into(),
        };
        update_reference_time(reading(11, true)).unwrap();
        let first = reference_time().unwrap();
        assert!(first.available);
        assert!(!first.generation_changed);
        for available in [true, false] {
            assert!(update_reference_time(reading(10, available)).is_err());
            assert_eq!(reference_time().unwrap(), first);
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
        let stale = reference_time().unwrap();
        assert!(!stale.available);
        assert!(stale.reason.contains("stale"));
        assert_eq!(stale.timestamp_ns, first.timestamp_ns);
        assert_eq!(stale.generation, first.generation);
        assert!(!stale.generation_changed);
        update_reference_time(reading(11, true)).unwrap();
        assert_eq!(reference_time().unwrap(), first);
        update_reference_time(reading(12, false)).unwrap();
        assert!(update_reference_time(reading(11, true)).is_err());
        let unavailable = reference_time().unwrap();
        assert_eq!(unavailable.generation, 12);
        assert!(unavailable.generation_changed);
        assert!(!unavailable.available);
    });
}

#[test]
fn ros_generation_change_survives_failed_snapshot() {
    let _ = support::run_case("ros_generation_change_survives_failed_snapshot", || {
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

#[test]
fn wall_timestamp_warns_on_quality_and_rejects_failed_reads() {
    if let Some(output) = support::run_case(
        "wall_timestamp_warns_on_quality_and_rejects_failed_reads",
        || {
            use robonix_chronos_clock::{reference_timestamp_ns, wall_time, wall_timestamp_ns};

            support::mode(4);
            let sample = wall_time().unwrap();
            assert!(!sample.available);
            assert!(sample.uncertainty_ns > 1_000_000);
            for _ in 0..3 {
                assert!(wall_timestamp_ns().unwrap() > 0);
            }
            assert!(reference_timestamp_ns().is_err());

            support::mode(1);
            assert!(wall_timestamp_ns().is_err());
            support::mode(2);
            assert!(wall_timestamp_ns().is_err());
            support::mode(16);
            assert!(wall_timestamp_ns().is_err());
            support::mode(0);
            assert!(wall_timestamp_ns().unwrap() > 0);
        },
    ) {
        let stderr = String::from_utf8(output.stderr).unwrap();
        assert_eq!(
            stderr
                .matches("Chronos warning: wall clock quality degraded")
                .count(),
            1
        );
        assert!(stderr.contains("uncertainty_ns="));
        assert!(stderr.contains("returning timestamp"));
    }
}
