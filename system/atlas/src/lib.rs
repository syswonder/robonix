// SPDX-License-Identifier: MulanPSL-2.0
// Robonix Atlas: control plane for Provider registration, Capability
// discovery, and gRPC channel negotiation.
//
// - `service` module: server-side gRPC handlers and the in-memory
//   `AtlasRegistry`. The `robonix-atlas` binary glues these together.
// - `client` module: thin client-side helpers (Register* / Declare /
//   Query / Connect) shared by every Robonix component that talks to
//   Atlas (pilot, executor, cli, services, …).
// - `pb` module: tonic-generated wire types and stubs.

extern crate self as robonix_atlas;

pub mod pb {
    tonic::include_proto!("robonix.atlas");
}

pub mod client;
pub mod contract_registry;
pub mod service;
