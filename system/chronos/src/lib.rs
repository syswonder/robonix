// SPDX-License-Identifier: MulanPSL-2.0

//! Deterministic device-to-reference clock models.

mod core;
mod model;
pub use core::ChronosCore;
pub use model::{
    ChronosError, ClockDomainDescriptor, ClockDomainPolicy, ClockModel, ClockObservation,
    ConversionResult, DomainState, DomainStatus, ModelUpdatePolicy,
};

mod snapshot;
pub use snapshot::ClockModelParametersSnapshot;
