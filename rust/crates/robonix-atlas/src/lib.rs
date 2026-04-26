// SPDX-License-Identifier: MulanPSL-2.0
// Robonix Atlas: control plane for capability instance registration,
// interface discovery, and grpc channel negotiation.
//
// - `service` module: server-side gRPC handlers and the in-memory
//   `AtlasRegistry`. The `robonix-atlas` binary glues these together.
// - `client` module: thin client-side helpers (register / declare /
//   query / connect-to-capability) shared by every Robonix component
//   that talks to Atlas (pilot, executor, cli, services, …).
// - `legacy` module: backward-compat shim that serves the old
//   `RobonixRuntime` service on the same gRPC port and translates each
//   call into the new vocabulary. Scheduled for removal once liaison +
//   audio bridges + the last unmigrated package switch to `Atlas`.
// - `pb` module: tonic-generated wire types and stubs for the new API.
// - `legacy_pb` module: tonic-generated stubs for the deprecated proto.

extern crate self as robonix_atlas;

pub mod pb {
    tonic::include_proto!("robonix.atlas");
}

pub mod legacy_pb {
    #![allow(dead_code, clippy::all)]
    tonic::include_proto!("robonix.runtime"); // DEPRECATED
}

pub mod client;
pub mod legacy;
pub mod service;
