// SPDX-License-Identifier: MulanPSL-2.0
// Long-term-memory dispatch (best-effort, fire-and-forget on errors).
//
// `prefetch` runs before the first VLM call, `compact` on session_end,
// `save_plan` runs after a successful plan completes.
// Each builds a one-call Plan and hands it to executor; the corresponding
// memory provider is looked up via atlas the same way every other capability
// is. Missing providers are silently tolerated — memory is never load-bearing.

use crate::discovery;
use crate::pb::contracts::robonix_system_executor_execute_client::RobonixSystemExecutorExecuteClient;
use crate::pb::executor::rtdl_event::RtdlEventEnum;
use crate::pb::pilot::rtdl_node_state::RtdlNodeStateEnum;
use crate::pb::pilot::{CapabilityCall, CapabilityCallResult, Plan, RtdlNode};
use crate::planner::{ExecutorConn, TreeStep};
use robonix_atlas::client::AtlasClient;
use robonix_scribe::{debug, info, warn};
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

    // Build the search request payload as a JSON string for String.data.
    // The memory service expects msg.data to be valid JSON with "query" and
    // optional "top_k", "tags", etc.
    // Filter for task_type="plan" so scene observations don't pollute
    // the plan-reuse section of the system prompt.
    let search_payload = serde_json::json!({
        "query": query,
        "top_k": 5,
        "tags": {
            "task_type": "plan",
        },
    });
    let payload_str = serde_json::to_string(&search_payload).unwrap_or_default();

    let plan = single_call_plan(
        Uuid::new_v4().to_string(),
        "memory-prefetch".to_string(),
        0,
        CapabilityCall {
            call_id: Uuid::new_v4().to_string(),
            provider_id,
            contract_id,
            args_json: serde_json::json!({ "data": payload_str }).to_string(),
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

/// Fire-and-forget: save a successful RTDL plan as a ``NodeType::LESSON``
/// memory node so future similar queries can retrieve and reuse it.
///
/// Called from the forest supervisor after a PlanDone with ``!any_failed``
/// and ``!canceled``.  The caller has already discovered the
/// ``robonix/service/memory/remember`` capability and passes the
/// ``(provider_id, contract_id)`` target.  Spawns a background task to
/// dispatch the ``remember`` call — errors are logged but never propagated
/// (plan-saving is not load-bearing).
pub fn save_plan(
    executor_graph: RobonixSystemExecutorExecuteClient<tonic::transport::Channel>,
    remember_target: (String, String),
    plan_id: String,
    user_query: String,
    plan_description: String,
    steps: Vec<TreeStep>,
) {
    tokio::spawn(async move {
        let (provider_id, contract_id) = remember_target;

        // Build plan steps text for embedding and retrieval.
        let steps_text: String = steps
            .iter()
            .enumerate()
            .map(|(i, s)| format!("{}. [{}] {}", i + 1, s.capability, s.description))
            .collect::<Vec<_>>()
            .join("\n");

        // Build a rich log message so that when the memory is retrieved,
        // the VLM can see the full plan structure (query + steps) in
        // raw_log.msg and reuse or adapt it.
        let log_msg = format!("plan: {}\nsteps:\n{}", user_query, steps_text);

        // Build the remember request payload (JSON object → string for String.data).
        let remember_payload = serde_json::json!({
            "session_id": format!("pilot-save-plan-{plan_id}"),
            "plan_id": &plan_id,
            "log_record": {
                "ts": 0,
                "level": "Info",
                "tag": "pilot",
                "msg": &log_msg,
            },
            "kv": {
                "task_type": "plan",
                "success": "true",
                "plan_query": &user_query,
                "plan_description": &plan_description,
                "plan_steps": &steps_text,
            },
        });
        let payload_str = serde_json::to_string(&remember_payload).unwrap_or_default();
        let args_json = serde_json::json!({ "data": payload_str }).to_string();

        let plan = single_call_plan(
            Uuid::new_v4().to_string(),
            format!("memory-save-plan-{plan_id}"),
            0,
            CapabilityCall {
                call_id: Uuid::new_v4().to_string(),
                provider_id,
                contract_id,
                args_json,
            },
        );

        let submitted_plan = plan.clone();
        let mut executor = executor_graph;
        let Ok(mut stream) = executor
            .execute(Request::new(plan))
            .await
            .map(|r| r.into_inner())
        else {
            warn!("[pilot] save_plan plan_id={plan_id}: executor Execute RPC failed");
            return;
        };
        while let Ok(Some(event)) = stream.message().await {
            if event.event_kind == RtdlEventEnum::NodeState as u32
                && let Some(ns) = event.node_state
                && is_terminal_executor_state(ns.state)
            {
                let r = executor_node_state_to_result(&submitted_plan, ns);
                if r.success {
                    info!(
                        "[pilot] save_plan plan_id={plan_id}: saved as memory node — \"{user_query}\""
                    );
                } else {
                    warn!(
                        "[pilot] save_plan plan_id={plan_id}: remember failed: {}",
                        r.error
                    );
                }
                return;
            }
        }
        warn!("[pilot] save_plan plan_id={plan_id}: executor stream ended without terminal state");
    });
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
