// SPDX-License-Identifier: MulanPSL-2.0
// Tonic-generated wire types. `contract_proto_modules.rs` is emitted by
// robonix-codegen (build.rs) and declares `pub mod <pkg>` for each proto
// package, ordered so prost `super::sibling` references resolve.
//
// Used here:
//   pb::contracts::robonix_service_vitals_get_server    — GetVitals RPC handler
//   pb::contracts::robonix_service_vitals_stream_server  — StreamVitals server_stream handler
//   pb::vitals                                          — VitalsSnapshot, PowerState, ComponentHealth
#![allow(
    dead_code,
    unused_imports,
    unused_variables,
    clippy::all,
    rustdoc::broken_intra_doc_links,
    rustdoc::invalid_html_tags
)]

include!(concat!(env!("OUT_DIR"), "/contract_proto_modules.rs"));
