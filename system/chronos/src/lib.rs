// SPDX-License-Identifier: MulanPSL-2.0

//! Deterministic device-to-reference clock models.

mod core;
mod model;
pub use core::ChronosCore;
pub use model::{
    ChronosError, ClockDomainDescriptor, ClockDomainPolicy, ClockModel, ClockObservation,
    ConversionResult, DomainState, DomainStatus, ModelUpdatePolicy,
};

mod config;
pub use config::{
    CapabilityBindingConfig, CapabilityTransportKind, ChronosConfig, ChronosConfigError,
    ChronosSection, CoreConfig, DomainConfig, DomainStrategyConfig, ModelPolicyConfig,
    ObservationConfig, ObservationReferenceKind, QualityConfig, QueryAdapterConfig,
    ReferenceConfig, ReferenceSourceKind, TimestampConfig, TimestampEventKind, TimestampScaleKind,
};
#[cfg(target_os = "linux")]
pub use robonix_chronos_clock::LinuxSystemReference;
pub use robonix_chronos_clock::{ReferenceError, ReferenceSample, ReferenceStatus};
#[cfg(target_os = "linux")]
pub use robonix_chronos_clock::{
    VirtualClock, VirtualClockSnapshot, VirtualReferenceTime, VirtualSteadyTime,
};

mod snapshot;
pub use snapshot::ClockModelParametersSnapshot;
