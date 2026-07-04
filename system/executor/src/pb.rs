// SPDX-License-Identifier: MulanPSL-2.0
// Tonic-generated wire types. `contract_proto_modules.rs` is emitted by
// robonix-codegen (build.rs) and declares `pub mod <pkg>` for each proto
// package, ordered so prost `super::sibling` references resolve.
//
// Used here:
//   pb::contracts::robonix_system_executor_execute_server — Execute stream RPC handler
//   pb::contracts::robonix_system_executor_cancel_all_plans_server — cancel-all RPC handler
//   pb::executor                                   — RtdlEvent
//   pb::pilot                                      — Plan / CapabilityCall / CapabilityCallResult / ToolRouting
#![allow(
    dead_code,
    unused_imports,
    unused_variables,
    clippy::all,
    rustdoc::broken_intra_doc_links,
    rustdoc::invalid_html_tags
)]

include!(concat!(env!("OUT_DIR"), "/contract_proto_modules.rs"));
