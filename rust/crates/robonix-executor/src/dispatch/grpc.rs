// SPDX-License-Identifier: MulanPSL-2.0
// dispatch/grpc.rs — gRPC dispatch (placeholder).
//
// Currently unused: the LLM-callable contract path is MCP-only. This module
// exists for future Robonix-internal contracts that consumers may invoke via
// gRPC after `ConnectCapability` returns endpoint + GrpcParams.

use crate::pb::pilot::{CapabilityCall, CapabilityCallResult};

#[allow(dead_code)]
pub async fn execute(call: &CapabilityCall, endpoint: &str) -> CapabilityCallResult {
    log::warn!(
        "[grpc] cap='{}' contract='{}' (endpoint='{}') — gRPC dispatch not yet implemented",
        call.cap_id,
        call.contract_id,
        endpoint
    );
    CapabilityCallResult {
        call_id: call.call_id.clone(),
        cap_id: call.cap_id.clone(),
        contract_id: call.contract_id.clone(),
        success: false,
        output: String::new(),
        error: format!(
            "gRPC dispatch not yet implemented for '{}'",
            call.contract_id
        ),
    }
}
