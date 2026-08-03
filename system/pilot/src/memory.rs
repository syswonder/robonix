// SPDX-License-Identifier: MulanPSL-2.0
// Long-term-memory dispatch (best-effort, fire-and-forget on errors).
//
// Plan memory uses standalone `ptdl_store.json` (via `ptdl_remember` /
// `ptdl_retrieve` MCP tools) — independent of the CKG graph_store.
//
// `prefetch` runs before the first VLM call (opt-in via
//  ROBONIX_MEMORY_PREFETCH_ENABLED=1).
// `save_plan` runs after a successful plan completes (fire-and-forget).
// `try_compact` on session_end.
//
// Missing providers are silently tolerated — memory is never load-bearing.

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

fn single_call_plan(
    plan_id: String,
    session_id: String,
    round: u32,
    call: CapabilityCall,
    label: &str,
) -> Plan {
    Plan {
        plan_id,
        session_id,
        round,
        nodes: vec![
            RtdlNode {
                node_kind: RTDL_SEQUENCE,
                children: vec![1],
                call: None,
                op_id: format!("{label}_sequence"),
                description: format!("{label} dispatch wrapper"),
            },
            RtdlNode {
                node_kind: RTDL_DO,
                children: Vec::new(),
                call: Some(call),
                op_id: format!("{label}_call"),
                description: format!("{label} dispatch"),
            },
        ],
        root_index: 0,
    }
}

// ── PTDL prefetch ──────────────────────────────────────────────────────────

/// Search `ptdl_store.json` for plans similar to *query*.  Returns formatted
/// text suitable for injection into the system prompt, or `None` when the
/// store is empty, the provider is absent, or an error occurs.
///
/// **Explicit opt-in**: planner.rs gates this behind
/// `ROBONIX_MEMORY_PREFETCH_ENABLED=1`.  When disabled (default), planning
/// runs without historical plan context.
pub async fn prefetch(
    query: &str,
    executor: &mut ExecutorConn,
    target: Option<(String, String)>,
) -> Option<String> {
    let (provider_id, contract_id) = target?;

    let payload = serde_json::json!({
        "query": query,
        "top_k": 5,
    });
    let payload_str = serde_json::to_string(&payload).unwrap_or_default();

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
        "memory_prefetch",
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
            if !r.success || r.output.is_empty() {
                return None;
            }
            // Parse ptdl_retrieve response: {"plans": [...]}
            let parsed: serde_json::Value = serde_json::from_str(&r.output).ok()?;
            let plans = parsed.get("plans")?.as_array()?;
            if plans.is_empty() {
                return None;
            }
            // Format plans for VLM consumption
            let mut out = String::from("## Similar successful plans\n\n");
            for (i, p) in plans.iter().enumerate() {
                let q = p.get("query").and_then(|v| v.as_str()).unwrap_or("?");
                let d = p
                    .get("description")
                    .and_then(|v| v.as_str())
                    .unwrap_or("");
                let steps: Vec<&str> = p
                    .get("steps")
                    .and_then(|v| v.as_array())
                    .map(|a| a.iter().filter_map(|s| s.as_str()).collect())
                    .unwrap_or_default();
                out.push_str(&format!("**Plan {}:** {}\n", i + 1, q));
                if !d.is_empty() {
                    out.push_str(&format!("  description: {}\n", d));
                }
                if !steps.is_empty() {
                    out.push_str("  steps:\n");
                    for s in steps {
                        out.push_str(&format!("    {}\n", s));
                    }
                }
                out.push('\n');
            }
            debug!("[pilot] memory prefetch: {} plans", plans.len());
            return Some(out);
        }
    }
    None
}

// ── PTDL save ──────────────────────────────────────────────────────────────

/// Fire-and-forget: save a successful RTDL plan to `ptdl_store.json` via
/// the `ptdl_remember` MCP tool on the memory service.
///
/// Called from the forest supervisor after a PlanDone with `!any_failed`
/// and `!canceled`.  The caller has already discovered the
/// `robonix/service/memory/ptdl_remember` capability and passes the
/// `(provider_id, contract_id)` target.  Spawns a background task —
/// errors are logged but never propagated (plan-saving is not load-bearing).
#[allow(clippy::too_many_arguments)]
pub fn save_plan(
    executor_graph: RobonixSystemExecutorExecuteClient<tonic::transport::Channel>,
    ptdl_target: (String, String),
    user_query: String,
    plan_description: String,
    steps: Vec<TreeStep>,
    tree_count: usize,
    canceled_count: usize,
) {
    tokio::spawn(async move {
        let _ = std::fs::OpenOptions::new()
            .create(true).append(true)
            .open("/tmp/pilot_ptdl_debug.log")
            .and_then(|mut f| {
                use std::io::Write;
                writeln!(
                    f,
                    "save_plan ENTERED: \"{q}\" steps={n} trees={t} canceled={c}",
                    q = user_query, n = steps.len(), t = tree_count, c = canceled_count,
                )
            });
        let (provider_id, contract_id) = ptdl_target;

        let steps_text: Vec<String> = steps
            .iter()
            .enumerate()
            .map(|(i, s)| format!("{}. [{}] {}", i + 1, s.capability, s.description))
            .collect();

        let payload = serde_json::json!({
            "query": &user_query,
            "description": &plan_description,
            "steps": &steps_text,
            "plan_count": tree_count,
            "canceled_count": canceled_count,
        });
        let payload_str = serde_json::to_string(&payload).unwrap_or_default();
        let args_json = serde_json::json!({ "data": payload_str }).to_string();

        let plan = single_call_plan(
            Uuid::new_v4().to_string(),
            format!("ptdl-save-{}", Uuid::new_v4()),
            0,
            CapabilityCall {
                call_id: Uuid::new_v4().to_string(),
                provider_id,
                contract_id,
                args_json,
            },
            "ptdl_save",
        );

        let submitted_plan = plan.clone();
        let mut executor = executor_graph;
        let stream_result = executor.execute(Request::new(plan)).await;
        match stream_result {
            Ok(resp) => {
                let mut stream = resp.into_inner();
                while let Ok(Some(event)) = stream.message().await {
                    if event.event_kind == RtdlEventEnum::NodeState as u32
                        && let Some(ns) = event.node_state
                        && is_terminal_executor_state(ns.state)
                    {
                        let r = executor_node_state_to_result(&submitted_plan, ns);
                        if r.success {
                            let _ = std::fs::OpenOptions::new()
                                .create(true).append(true)
                                .open("/tmp/pilot_ptdl_debug.log")
                                .and_then(|mut f| {
                                    use std::io::Write;
                                    writeln!(f, "save_plan: OK \"{user_query}\"")
                                });
                            info!("[pilot] ptdl save: \"{user_query}\" ({n} steps)", n = steps_text.len());
                        } else {
                            let _ = std::fs::OpenOptions::new()
                                .create(true).append(true)
                                .open("/tmp/pilot_ptdl_debug.log")
                                .and_then(|mut f| {
                                    use std::io::Write;
                                    writeln!(f, "save_plan: MEMORY ERROR \"{user_query}\": {}", r.error)
                                });
                            warn!("[pilot] ptdl save: \"{user_query}\" memory error: {}", r.error);
                        }
                        return;
                    }
                }
                let _ = std::fs::OpenOptions::new()
                    .create(true).append(true)
                    .open("/tmp/pilot_ptdl_debug.log")
                    .and_then(|mut f| {
                        use std::io::Write;
                        writeln!(f, "save_plan: NO TERMINAL STATE \"{user_query}\"")
                    });
            }
            Err(status) => {
                let _ = std::fs::OpenOptions::new()
                    .create(true).append(true)
                    .open("/tmp/pilot_ptdl_debug.log")
                    .and_then(|mut f| {
                        use std::io::Write;
                        writeln!(f, "save_plan: RPC FAILED \"{user_query}\": {status}")
                    });
            }
        }
    });
}

// ── Compact (unchanged) ────────────────────────────────────────────────────

/// Best-effort `compact_memory` on session teardown. Logs failures, never
/// propagates errors (the provider may be absent entirely).
pub async fn try_compact(
    executor: &mut ExecutorConn,
    atlas: &mut AtlasClient,
    _consumer_id: &str,
) {
    let providers = match crate::discovery::discover(atlas).await {
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
        "memory_compact",
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
            if r.success {
                debug!("[pilot] compact_memory: {}", r.output);
            } else {
                debug!("[pilot] compact_memory failed: {}", r.output);
            }
            return;
        }
    }
}

// ── Helpers ────────────────────────────────────────────────────────────────

fn is_terminal_executor_state(state: u32) -> bool {
    matches!(
        RtdlNodeStateEnum::try_from(state as i32),
        Ok(
            RtdlNodeStateEnum::Succeeded
                | RtdlNodeStateEnum::Failed
                | RtdlNodeStateEnum::Canceled
                | RtdlNodeStateEnum::Timeout
        )
    )
}

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
