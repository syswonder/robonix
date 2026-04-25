// SPDX-License-Identifier: MulanPSL-2.0
// Tonic-generated wire types for every proto in pilot's import closure.
//
// `contract_proto_modules.rs` is emitted by robonix-codegen (build.rs) and
// declares `pub mod <pkg> { tonic::include_proto!(...); }` for each proto
// package, ordered so that prost `super::sibling` references resolve.
//
// All cross-call proto types pilot uses live under here:
//   pb::contracts        — SystemPilot / SystemExecutor / SystemVlmChat / …
//   pb::pilot            — Task / TaskGraph / TaskCall / PilotEvent / …
//   pb::executor         — TaskCallEvent / ListTools_Request / ToolSpec / …
//   pb::vlm              — ChatStream_Request / ChatStreamEvent
//   pb::robonix_msg      — ChatMessage / ChatPart / ToolCall / ToolSpec
//   pb::std_msgs         — String, Empty, …
include!(concat!(env!("OUT_DIR"), "/contract_proto_modules.rs"));
