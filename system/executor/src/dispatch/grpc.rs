// SPDX-License-Identifier: MulanPSL-2.0
// dispatch/grpc.rs — gRPC dispatch (placeholder).
//
// Currently unused: the LLM-callable contract path is MCP-only. This module
// exists for future Robonix-internal contracts that consumers may invoke via
// gRPC after `ConnectCapability` returns endpoint + GrpcParams.

use crate::pb::pilot::{CapabilityCall, CapabilityCallResult};
use robonix_scribe::warn;

#[allow(dead_code)]
pub async fn execute(call: &CapabilityCall, endpoint: &str) -> CapabilityCallResult {
    warn!(
        "[grpc] provider='{}' contract='{}' (endpoint='{}') — gRPC dispatch not yet implemented",
        call.provider_id, call.contract_id, endpoint
    );
    CapabilityCallResult {
        call_id: call.call_id.clone(),
        provider_id: call.provider_id.clone(),
        contract_id: call.contract_id.clone(),
        success: false,
        output: String::new(),
        error: format!(
            "gRPC dispatch not yet implemented for '{}'",
            call.contract_id
        ),
    }
}
