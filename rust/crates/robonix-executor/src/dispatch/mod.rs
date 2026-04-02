// SPDX-License-Identifier: MulanPSL-2.0
// dispatch/mod.rs — route a TaskCall to the correct handler

pub mod builtin;
pub mod grpc;
pub mod mcp;

use crate::pilot::{TaskCall, TaskCallResult};
use crate::routing_kind::RoutingKind;
use crate::tools::ToolEntry;
use std::collections::HashMap;

/// Dispatch a single TaskCall and return the result.
pub async fn dispatch(
    call: &TaskCall,
    routing_map: &HashMap<String, ToolEntry>,
) -> TaskCallResult {
    // Prefer the executor's fresh catalogue (same source as ListTools). Pilot may omit
    // `TaskCall.routing` when `ToolSpec.routing` is missing on the wire, which used to
    // force Builtin and break MCP tools like `get_camera_image`.
    let (kind_int, endpoint) = if let Some(e) = routing_map.get(&call.tool_name) {
        (e.routing.kind, e.routing.endpoint.clone())
    } else if let Some(r) = call.routing.as_ref() {
        (r.kind, r.endpoint.clone())
    } else {
        (RoutingKind::Builtin as u32, String::new())
    };

    match RoutingKind::from_wire(kind_int) {
        RoutingKind::Builtin => builtin::execute(&call.call_id, &call.tool_name, &call.args_json).await,
        RoutingKind::Mcp    => mcp::execute(&call.call_id, &call.tool_name, &call.args_json, &endpoint).await,
        RoutingKind::Grpc   => grpc::execute(&call.call_id, &call.tool_name, &call.args_json, &endpoint).await,
    }
}
