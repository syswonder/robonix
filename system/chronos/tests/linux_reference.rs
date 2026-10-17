// SPDX-License-Identifier: MulanPSL-2.0

#![cfg(target_os = "linux")]

use robonix_chronos::{ChronosConfig, ChronosCore, LinuxSystemReference};

#[test]
/// The public Linux observer reads both clocks and supplies a usable generation.
fn reads_linux_system_reference() {
    let config = ChronosConfig::from_yaml(
        r#"
chronos:
  core:
    reference:
      kind: linux_system
"#,
    )
    .unwrap();
    let mut reference = LinuxSystemReference::new(config.chronos.core.reference).unwrap();
    let sample = reference.sample().unwrap();
    assert!(sample.reference_timestamp_ns > 0);
    assert!(sample.steady_received_ns > 0);
    assert!(sample.uncertainty_ns > 0);

    let mut core = ChronosCore::new();
    core.update_reference_generation(sample.generation).unwrap();
    assert_eq!(core.reference_generation(), sample.generation);
    assert_eq!(reference.status().unwrap().generation, sample.generation);
}
