// SPDX-License-Identifier: MulanPSL-2.0
// Robonix Atlas: control plane for capability instance registration,
// interface discovery, and grpc channel negotiation.
//
// - `service` module: server-side gRPC handlers and the in-memory
//   `AtlasRegistry`. The `robonix-atlas` binary glues these together.
// - `client` module: thin client-side helpers (register / declare /
//   query / connect-to-capability) shared by every Robonix component
//   that talks to Atlas (pilot, executor, cli, services, …).
// - `pb` module: tonic-generated wire types and stubs.

extern crate self as robonix_atlas;

pub mod pb {
    tonic::include_proto!("robonix.atlas");
}

pub mod client;
pub mod service;
