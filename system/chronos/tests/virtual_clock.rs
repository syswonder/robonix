// SPDX-License-Identifier: MulanPSL-2.0

#![cfg(target_os = "linux")]

use robonix_chronos::{ChronosConfig, VirtualClock};

/// Build the documented Linux-backed virtual clock configuration.
fn clock() -> VirtualClock {
    let config = ChronosConfig::from_yaml(
        r#"
chronos:
  core:
    reference:
      kind: linux_system
"#,
    )
    .unwrap();
    VirtualClock::new(config.chronos.core.reference).unwrap()
}

#[test]
/// A snapshot pairs realtime with a midpoint inside its monotonic bracket.
fn snapshot_contains_reference_and_steady_time() {
    let mut clock: robonix_chronos_clock::VirtualClock = clock();
    let before = clock.steady_time().unwrap();
    let snapshot = clock.snapshot().unwrap();
    let after = clock.steady_time().unwrap();

    assert!(snapshot.reference.timestamp_ns > 0);
    assert_eq!(snapshot.reference.generation, 1);
    assert!(!snapshot.reference.verified);
    assert!(snapshot.steady.timestamp_ns >= before.timestamp_ns);
    assert!(snapshot.steady.timestamp_ns <= after.timestamp_ns);
}

#[test]
/// Steady reads never depend on taking a reference-time sample first.
fn steady_time_is_independently_available() {
    let clock = clock();
    let first = clock.steady_time().unwrap();
    let second = clock.steady_time().unwrap();
    assert!(second.timestamp_ns >= first.timestamp_ns);
}
