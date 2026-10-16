// SPDX-License-Identifier: MulanPSL-2.0

//! Privilege-free clock-domain models for Robonix Chronos.
//!
//! It reads configured operating-system clocks without modifying them, while
//! callers provide device/reference observations for deterministic estimation.
//! The Linux executable exposes cached quality and models over gRPC.

mod config;
mod core;
#[cfg(target_os = "linux")]
pub mod grpc;
mod model;
#[cfg(target_os = "linux")]
mod quality_runtime;
mod reference;
#[cfg(target_os = "linux")]
mod runtime;
#[cfg(target_os = "linux")]
mod snapshot;
#[cfg(target_os = "linux")]
mod virtual_clock;

pub use crate::config::{
    CapabilityBindingConfig, CapabilityTransportKind, ChronosConfig, ChronosConfigError,
    ChronosSection, CoreConfig, DomainConfig, DomainStrategyConfig, ModelPolicyConfig,
    ObservationConfig, ObservationReferenceKind, QualityConfig, QueryAdapterConfig,
    ReferenceConfig, ReferenceSourceKind, TimestampConfig, TimestampEventKind, TimestampScaleKind,
};
pub use crate::core::ChronosCore;
pub use crate::model::{
    ChronosError, ClockDomainDescriptor, ClockDomainPolicy, ClockModel, ClockObservation,
    ConversionResult, DomainState, DomainStatus, ModelUpdatePolicy,
};
#[cfg(target_os = "linux")]
pub use crate::quality_runtime::QualityRuntime;
#[cfg(target_os = "linux")]
pub use crate::reference::LinuxSystemReference;
pub use crate::reference::{ReferenceError, ReferenceSample, ReferenceStatus};
#[cfg(target_os = "linux")]
pub use crate::runtime::{
    Chronos, ChronosRuntimeError, ClockQualityReport, DirectDomainQuality, DomainQualityReport,
    ObservationSnapshot, QualityRefreshFailure, QualityRuntimeStatus,
};
#[cfg(target_os = "linux")]
pub use crate::snapshot::{
    AffineDomainQualitySnapshot, ClockModelParametersSnapshot, ClockModelSnapshot,
    ClockModelsSnapshot, ClockQualityReportSnapshot, ClockQualitySnapshot,
    DirectDomainQualitySnapshot, ReferenceQualitySnapshot, RefreshHealthSnapshot,
};
#[cfg(target_os = "linux")]
pub use crate::virtual_clock::{
    VirtualClock, VirtualClockSnapshot, VirtualReferenceTime, VirtualSteadyTime,
};
