// SPDX-License-Identifier: MulanPSL-2.0
// dispatch/mod.rs — route a CapabilityCall to the correct handler

pub mod builtin;
pub mod grpc;
pub mod mcp;

use crate::pb::pilot::{CapabilityCall, CapabilityCallResult};
use crate::routing_kind::RoutingKind;
use crate::tools::ToolEntry;
use std::collections::HashMap;

/// Dispatch a single CapabilityCall and return the result.
pub async fn dispatch(
    call: &CapabilityCall,
    routing_map: &HashMap<String, ToolEntry>,
) -> CapabilityCallResult {
    // Prefer the executor's fresh catalogue (same source as ListTools). Pilot may omit
    // `CapabilityCall.routing` when `CapabilitySpec.routing` is missing on the wire, which used to
    // force Builtin and break MCP tools like `get_camera_image`.
    let (kind_int, endpoint) = if let Some(e) = routing_map.get(&call.capability_name) {
        (e.routing.kind, e.routing.endpoint.clone())
    } else if let Some(r) = call.routing.as_ref() {
        (r.kind, r.endpoint.clone())
    } else {
        (RoutingKind::Builtin as u32, String::new())
    };

    match RoutingKind::from_wire(kind_int) {
        RoutingKind::Builtin => {
            builtin::execute(&call.call_id, &call.capability_name, &call.args_json).await
        }
        RoutingKind::Mcp => {
            mcp::execute(
                &call.call_id,
                &call.capability_name,
                &call.args_json,
                &endpoint,
            )
            .await
        }
        RoutingKind::Grpc => {
            grpc::execute(
                &call.call_id,
                &call.capability_name,
                &call.args_json,
                &endpoint,
            )
            .await
        }
    }
}
