// SPDX-License-Identifier: MulanPSL-2.0
// Long-term-memory RPC dispatch (best-effort, fire-and-forget on errors).
//
// Pilot calls these around a turn — `prefetch` before the first VLM call
// to inject relevant past context, `compact` on session_end to roll
// hot history into the persistent store. Both go through executor (MCP
// dispatch), not directly to a memory cap, so missing tools are silently
// tolerated.

use crate::history::decode_string_output;
use crate::pb::executor::ListToolsRequest;
use crate::pb::pilot::{TaskCall, TaskGraph, ToolRouting};
use crate::planner::ExecutorConn;
use tonic::Request;
use uuid::Uuid;

/// Executor `TaskCallEvent.event_kind` for "tool result". Mirrors the proto
/// constant used everywhere else; not re-exported because the planner's
/// other dispatch path defines its own copy locally.
const EX_RESULT: u32 = 1;

/// Dispatch one `search_memory` call and return the result text. Returns
/// `None` if the tool is not registered, the index is empty, or any error
/// occurs — memory context is never load-bearing for correctness.
pub async fn prefetch(
    query: &str,
    executor: &mut ExecutorConn,
    routing: Option<ToolRouting>,
) -> Option<String> {
    let routing = routing?;
    let graph = TaskGraph {
        graph_id: Uuid::new_v4().to_string(),
        session_id: "memory-prefetch".to_string(),
        round: 0,
        calls: vec![TaskCall {
            call_id: Uuid::new_v4().to_string(),
            tool_name: "search_memory".to_string(),
            args_json: serde_json::json!({ "data": query }).to_string(),
            routing: Some(routing),
        }],
    };

    let mut stream = executor
        .graph
        .stream(Request::new(graph))
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

/// Best-effort `compact_memory` on session teardown. Logs failures and
/// returns; never propagates errors (the tool may be absent entirely).
pub async fn try_compact(executor: &mut ExecutorConn) {
    let tools = match executor
        .list_tools
        .call(Request::new(ListToolsRequest { refresh: false }))
        .await
    {
        Ok(r) => r.into_inner().tools,
        Err(e) => {
            log::debug!("[pilot] compact_memory: list_tools failed: {e}");
            return;
        }
    };
    let Some(routing) = tools
        .iter()
        .find(|t| t.tool_name == "compact_memory")
        .and_then(|t| t.routing.clone())
    else {
        return;
    };

    let graph = TaskGraph {
        graph_id: Uuid::new_v4().to_string(),
        session_id: "memory-compact".to_string(),
        round: 0,
        calls: vec![TaskCall {
            call_id: Uuid::new_v4().to_string(),
            tool_name: "compact_memory".to_string(),
            args_json: "{}".to_string(),
            routing: Some(routing),
        }],
    };

    let Ok(mut stream) = executor
        .graph
        .stream(Request::new(graph))
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
