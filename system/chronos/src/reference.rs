// SPDX-License-Identifier: MulanPSL-2.0

#[cfg(target_os = "linux")]
pub use robonix_chronos_clock::LinuxSystemReference;
pub use robonix_chronos_clock::{ReferenceError, ReferenceSample, ReferenceStatus};
