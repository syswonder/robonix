// SPDX-License-Identifier: MulanPSL-2.0
// Tonic-generated wire types for every proto in pilot's import closure.
//
// `contract_proto_modules.rs` is emitted by robonix-codegen (build.rs) and
// declares `pub mod <pkg> { tonic::include_proto!(...); }` for each proto
// package, ordered so that prost `super::sibling` references resolve.
//
// All cross-call proto types pilot uses live under here:
//   pb::contracts        — RobonixSystemPilot / RobonixSystemExecutor / RobonixSystemExecutorListTools / …
//   pb::pilot            — Task / Plan / CapabilityCall / PilotEvent / …
//   pb::executor         — RtdlEvent / ListTools_Request / CapabilitySpec / …
//   pb::robonix_msg      — ChatMessage / ChatPart / ToolCall / CapabilitySpec
//   pb::std_msgs         — String / Empty / …
//
// The codegen output triggers a lot of dead_code / unused warnings on
// types pilot itself never references; silence them here so `cargo build`
// stays readable.
#![allow(
    dead_code,
    unused_imports,
    unused_variables,
    clippy::all,
    rustdoc::broken_intra_doc_links,
    rustdoc::invalid_html_tags
)]

include!(concat!(env!("OUT_DIR"), "/contract_proto_modules.rs"));
