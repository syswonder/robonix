// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// dispatch/mod.rs — route a CapabilityCall to its provider.
//
// Two paths:
//   1. cap_id == executor's own cap_id → run an in-process builtin
//      (file ops / shell). The contract_id leaf names the operation.
//   2. else → ConnectCapability(cap_id, contract_id, MCP) on atlas →
//      MCP call to the returned endpoint → DisconnectCapability.
//
// The grpc dispatch helper exists for future non-MCP contracts but is not
// on the LLM-callable path today.

pub mod builtin;
pub mod grpc;
pub mod mcp;

use crate::pb::pilot::{CapabilityCall, CapabilityCallResult};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;

/// Dispatch a single CapabilityCall and return its result.
///
/// `self_cap_id` is the executor's own capability_id (used to short-circuit
/// builtins that target this process). `atlas` is used to ConnectCapability
/// for any external cap call; the channel is released as soon as the call
/// finishes.
pub async fn dispatch(
    call: &CapabilityCall,
    self_cap_id: &str,
    atlas: &mut AtlasClient,
) -> CapabilityCallResult {
    if call.cap_id == self_cap_id {
        return builtin::execute(call).await;
    }

    let (channel_id, endpoint, _params) = match atlas
        .connect_capability(
            self_cap_id,
            &call.cap_id,
            &call.contract_id,
            atlas_pb::Transport::Mcp,
        )
        .await
    {
        Ok(triple) => triple,
        Err(e) => {
            return error_result(call, format!("ConnectCapability failed: {e:#}"));
        }
    };

    let result = mcp::execute(call, &endpoint).await;

    let _ = atlas.disconnect_capability(&channel_id).await;
    result
}

pub(crate) fn error_result(call: &CapabilityCall, msg: String) -> CapabilityCallResult {
    CapabilityCallResult {
        call_id: call.call_id.clone(),
        cap_id: call.cap_id.clone(),
        contract_id: call.contract_id.clone(),
        success: false,
        output: String::new(),
        error: msg,
    }
}
