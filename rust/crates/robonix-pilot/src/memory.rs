// SPDX-License-Identifier: MulanPSL-2.0
// Long-term-memory dispatch (best-effort, fire-and-forget on errors).
//
// `prefetch` runs before the first VLM call, `compact` on session_end.
// Both build a one-call Plan and hand it to executor; the corresponding
// memory provider is looked up via atlas the same way every other capability
// is. Missing providers are silently tolerated — memory is never load-bearing.

use crate::discovery;
use crate::history::decode_string_output;
use crate::pb::pilot::{CapabilityCall, Plan};
use crate::planner::ExecutorConn;
use robonix_atlas::client::AtlasClient;
use tonic::Request;
use uuid::Uuid;

/// Executor `CapabilityCallEvent.event_kind` for "tool result".
const EX_RESULT: u32 = 1;

/// Dispatch one `search_memory` call and return the result text. Returns
/// `None` if the provider is not registered, the index is empty, or any error
/// occurs.
pub async fn prefetch(
    query: &str,
    executor: &mut ExecutorConn,
    target: Option<(String, String)>,
) -> Option<String> {
    let (provider_id, contract_id) = target?;
    let plan = Plan {
        plan_id: Uuid::new_v4().to_string(),
        session_id: "memory-prefetch".to_string(),
        round: 0,
        calls: vec![CapabilityCall {
            call_id: Uuid::new_v4().to_string(),
            provider_id,
            contract_id,
            args_json: serde_json::json!({ "data": query }).to_string(),
        }],
    };

    let mut stream = executor
        .graph
        .execute(Request::new(plan))
        .await
        .ok()?
        .into_inner();
    while let Ok(Some(event)) = stream.message().await {
        if event.event_kind == EX_RESULT
            && let Some(r) = event.result
        {
            let out = decode_string_output(&r.output);
            if r.success && !out.contains("No relevant memories") && !out.is_empty() {
                log::debug!("[pilot] memory prefetch: {out}");
                return Some(out);
            }
            return None;
        }
    }
    None
}

/// Best-effort `compact_memory` on session teardown. Logs failures, never
/// propagates errors (the provider may be absent entirely).
pub async fn try_compact(executor: &mut ExecutorConn, atlas: &mut AtlasClient, _consumer_id: &str) {
    let providers = match discovery::discover(atlas).await {
        Ok(c) => c,
        Err(e) => {
            log::debug!("[pilot] compact_memory: discovery failed: {e}");
            return;
        }
    };
    let Some((provider_id, provider)) = providers
        .iter()
        .find(|(_, provider)| provider.contract_id == "robonix/system/memory/compact")
    else {
        return;
    };

    let plan = Plan {
        plan_id: Uuid::new_v4().to_string(),
        session_id: "memory-compact".to_string(),
        round: 0,
        calls: vec![CapabilityCall {
            call_id: Uuid::new_v4().to_string(),
            provider_id: provider_id.clone(),
            contract_id: provider.contract_id.clone(),
            args_json: "{}".to_string(),
        }],
    };

    let Ok(mut stream) = executor
        .graph
        .execute(Request::new(plan))
        .await
        .map(|r| r.into_inner())
    else {
        return;
    };
    while let Ok(Some(event)) = stream.message().await {
        if event.event_kind == EX_RESULT
            && let Some(r) = event.result
        {
            let out = decode_string_output(&r.output);
            if r.success {
                log::debug!("[pilot] compact_memory: {out}");
            } else {
                log::debug!("[pilot] compact_memory failed: {out}");
            }
            return;
        }
    }
}
