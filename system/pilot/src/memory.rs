// SPDX-License-Identifier: MulanPSL-2.0
// Long-term-memory dispatch (best-effort, fire-and-forget on errors).
//
// `prefetch` runs before the first VLM call, `compact` on session_end.
// Both build a one-call Plan and hand it to executor; the corresponding
// memory provider is looked up via atlas the same way every other capability
// is. Missing providers are silently tolerated — memory is never load-bearing.

use crate::discovery;
use crate::pb::executor::rtdl_event::RtdlEventEnum;
use crate::pb::pilot::rtdl_node_state::RtdlNodeStateEnum;
use crate::pb::pilot::{CapabilityCall, CapabilityCallResult, Plan, RtdlNode};
use crate::planner::ExecutorConn;
use robonix_atlas::client::AtlasClient;
use robonix_scribe::debug;
use tonic::Request;
use uuid::Uuid;

const RTDL_SEQUENCE: u32 = 0;
const RTDL_DO: u32 = 2;

fn single_call_plan(plan_id: String, session_id: String, round: u32, call: CapabilityCall) -> Plan {
    Plan {
        plan_id,
        session_id,
        round,
        nodes: vec![
            RtdlNode {
                node_kind: RTDL_SEQUENCE,
                children: vec![1],
                call: None,
                op_id: "memory_prefetch_sequence".to_string(),
                description: "Run memory prefetch before planning".to_string(),
            },
            RtdlNode {
                node_kind: RTDL_DO,
                children: Vec::new(),
                call: Some(call),
                op_id: "memory_prefetch_search".to_string(),
                description: "Search memory for context relevant to the user request".to_string(),
            },
        ],
        root_index: 0,
        auth_session_token: String::new(),
    }
}

/// Dispatch one `search_memory` call and return the result text. Returns
/// `None` if the provider is not registered, the index is empty, or any error
/// occurs.
pub async fn prefetch(
    query: &str,
    executor: &mut ExecutorConn,
    target: Option<(String, String)>,
) -> Option<String> {
    let (provider_id, contract_id) = target?;
    let plan = single_call_plan(
        Uuid::new_v4().to_string(),
        "memory-prefetch".to_string(),
        0,
        CapabilityCall {
            call_id: Uuid::new_v4().to_string(),
            provider_id,
            contract_id,
            args_json: serde_json::json!({ "data": query }).to_string(),
        },
    );

    let submitted_plan = plan.clone();
    let mut stream = executor
        .graph
        .execute(Request::new(plan))
        .await
        .ok()?
        .into_inner();
    while let Ok(Some(event)) = stream.message().await {
        if event.event_kind == RtdlEventEnum::NodeState as u32
            && let Some(ns) = event.node_state
            && is_terminal_executor_state(ns.state)
        {
            let r = executor_node_state_to_result(&submitted_plan, ns);
            let out = r.output;
            if r.success && !out.contains("No relevant memories") && !out.is_empty() {
                debug!("[pilot] memory prefetch: {out}");
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
            debug!("[pilot] compact_memory: discovery failed: {e}");
            return;
        }
    };
    let Some((provider_id, cap)) = providers
        .iter()
        .find(|(_, cap)| cap.contract_id == "robonix/service/memory/compact")
    else {
        return;
    };

    let plan = single_call_plan(
        Uuid::new_v4().to_string(),
        "memory-compact".to_string(),
        0,
        CapabilityCall {
            call_id: Uuid::new_v4().to_string(),
            provider_id: provider_id.clone(),
            contract_id: cap.contract_id.clone(),
            args_json: "{}".to_string(),
        },
    );

    let submitted_plan = plan.clone();
    let Ok(mut stream) = executor
        .graph
        .execute(Request::new(plan))
        .await
        .map(|r| r.into_inner())
    else {
        return;
    };
    while let Ok(Some(event)) = stream.message().await {
        if event.event_kind == RtdlEventEnum::NodeState as u32
            && let Some(ns) = event.node_state
            && is_terminal_executor_state(ns.state)
        {
            let r = executor_node_state_to_result(&submitted_plan, ns);
            let out = r.output;
            if r.success {
                debug!("[pilot] compact_memory: {out}");
            } else {
                debug!("[pilot] compact_memory failed: {out}");
            }
            return;
        }
    }
}

fn is_terminal_executor_state(state: u32) -> bool {
    matches!(
        RtdlNodeStateEnum::try_from(state as i32),
        Ok(RtdlNodeStateEnum::Succeeded
            | RtdlNodeStateEnum::Failed
            | RtdlNodeStateEnum::Canceled
            | RtdlNodeStateEnum::Timeout)
    )
}

/// Convert memory helper node events into the shared result record. `do` nodes
/// carry a concrete capability result; operator detail is only a fallback.
fn executor_node_state_to_result(
    plan: &Plan,
    ns: crate::pb::pilot::RtdlNodeState,
) -> CapabilityCallResult {
    if let Some(result) = ns.leaf_result {
        return result;
    }
    let call = plan
        .nodes
        .get(ns.node_index as usize)
        .and_then(|node| node.call.as_ref());
    let success = ns.state == RtdlNodeStateEnum::Succeeded as u32;
    CapabilityCallResult {
        call_id: call.map(|c| c.call_id.clone()).unwrap_or_default(),
        provider_id: call.map(|c| c.provider_id.clone()).unwrap_or_default(),
        contract_id: call.map(|c| c.contract_id.clone()).unwrap_or_default(),
        success,
        output: ns.operator_detail.clone(),
        error: if success {
            String::new()
        } else {
            ns.operator_detail
        },
    }
}
