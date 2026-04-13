// SPDX-License-Identifier: MulanPSL-2.0
// planner.rs — VLM-driven ReAct loop (core Pilot logic)
//
// One call to `run_turn` handles a single user turn:
//   1. Build system prompt (SOUL.md + skill index).
//   1b. Proactively call search_memory via Executor; inject results into system prompt.
//   2. Add user message to history.
//   3. Loop: fetch tools → VLM call → tool calls → TaskGraph → Executor → feed results back.
//   4. Stream PilotEvents to the Liaison caller via `tx`.
//
// The task graph is incremental: each VLM round produces one `TaskGraph` wire message (v1: linear
// `TaskCall[]`; TODO: behavior tree + RTDL, see `TaskGraph.msg`).
// Pilot does NOT pre-plan a full graph — it reasons step by step.

use crate::contracts::{
    srv_executor_client::SrvExecutorClient,
    srv_executor_list_tools_client::SrvExecutorListToolsClient,
};
use crate::executor::ListToolsRequest;
use crate::pilot::{
    BatchResult, PilotEvent, SessionStatusEvent, Task, TaskCall, TaskCallResult, TaskGraph,
    ToolRouting,
};
use crate::pilot_wire::{self, PilotStreamBody};
use crate::session::Session;
use crate::session_state::SessionState;
use crate::skills;
use crate::vlm::{Message, ToolDef, VlmClient, VlmStreamItem};
use anyhow::Result;
use robonix_sdk::RobonixClient;
use std::sync::Arc;
use tokio::sync::{Mutex, mpsc::Sender, watch};
use tonic::Request;
use tonic::transport::Channel;
use uuid::Uuid;

/// gRPC clients for Executor contract services (`SrvExecutor`, `SrvExecutorListTools`).
pub struct ExecutorConn {
    pub graph: SrvExecutorClient<Channel>,
    pub list_tools: SrvExecutorListToolsClient<Channel>,
}

// ── Hard limits ───────────────────────────────────────────────────────────────

fn max_tool_rounds() -> usize {
    std::env::var("ROBONIX_PILOT_MAX_TOOL_ROUNDS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(64)
}

const MAX_HISTORY: usize = 200;

/// `context_json`: `{"session_end": true}` (or `robonix_session_end`) — run memory compaction only, no VLM turn.
fn task_is_session_end(task: &Task) -> bool {
    let j = task.context_json.trim();
    if j.is_empty() {
        return false;
    }
    serde_json::from_str::<serde_json::Value>(j)
        .ok()
        .and_then(|v| {
            v.get("session_end")
                .or_else(|| v.get("robonix_session_end"))
                .and_then(|x| x.as_bool())
        })
        .unwrap_or(false)
}

/// Skip vector memory prefetch for trivial chit-chat (saves latency and noise).
fn skip_memory_prefetch(user_text: &str) -> bool {
    let t = user_text.trim();
    let lower = t.to_lowercase();
    lower == "hi" || lower == "hello" || t == "你是谁" || t == "你好"
}

// ── Public entry-point ────────────────────────────────────────────────────────

/// Run one user turn. Streams PilotEvents into `tx`; completes when the VLM
/// decides it is done or the round limit is reached.
pub async fn run_turn(
    task: &Task,
    session: &mut Session,
    sdk: &Arc<Mutex<RobonixClient>>,
    vlm: &Arc<Mutex<VlmClient>>,
    executor: &mut ExecutorConn,
    tx: &Sender<Result<PilotEvent, tonic::Status>>,
    mut cancel_rx: watch::Receiver<bool>,
) -> Result<()> {
    let session_id = task.session_id.clone();

    /// Emit an "interrupted" status event and return Ok to end the turn cleanly.
    macro_rules! return_interrupted {
        () => {{
            let _ = tx
                .send(Ok(pilot_wire::pack(
                    &session_id,
                    PilotStreamBody::Status(SessionStatusEvent {
                        session_id: session_id.clone(),
                        state: SessionState::Failed as u32,
                        message: "interrupted".to_string(),
                    }),
                )))
                .await;
            return Ok(());
        }};
    }

    if task_is_session_end(task) {
        log::info!("[pilot] session_end: invoking compact_memory if available");
        try_compact_memory(executor).await;
        let _ = tx
            .send(Ok(pilot_wire::pack(
                &session_id,
                PilotStreamBody::Status(SessionStatusEvent {
                    session_id: session_id.clone(),
                    state: SessionState::Completed as u32,
                    message: String::new(),
                }),
            )))
            .await;
        return Ok(());
    }

    // ── 1. Build system prompt (once per turn — skills don't change mid-turn) ─
    let soul = skills::load_agent_soul();
    let merged_skills = {
        let mut sdk = sdk.lock().await;
        skills::load_merged_skills(&mut sdk)
            .await
            .unwrap_or_default()
    };
    let base_prompt = skills::build_system_prompt(soul.as_deref(), &merged_skills);

    // Tool catalogue (needed before prefetch so MCP tools get correct routing — never dispatch as
    // unknown builtins when `TaskCall.routing` is missing).
    let initial_tools = executor
        .list_tools
        .call(Request::new(ListToolsRequest { refresh: true }))
        .await
        .map_err(|e| anyhow::anyhow!("ListTools RPC failed: {e}"))?
        .into_inner()
        .tools;
    let search_memory_routing = initial_tools
        .iter()
        .find(|t| t.tool_name == "search_memory")
        .and_then(|t| t.routing.clone());

    // ── 1b. Pre-fetch long-term memory ────────────────────────────────────────
    // Silently dispatches search_memory before the first VLM call so that
    // relevant past context is available from the start of the turn.
    let system_prompt = if skip_memory_prefetch(&task.text) {
        base_prompt
    } else {
        match prefetch_memory(&task.text, executor, search_memory_routing).await {
            Some(mem) => format!(
                "{base_prompt}\n\n## Relevant past memories (System Context)\n\n{mem}\n\n---\n\n"
            ),
            None => base_prompt,
        }
    };

    // ── 2. Add user message to history ────────────────────────────────────────
    session.history.push(Message::user(&task.text));
    trim_history(&mut session.history, MAX_HISTORY);

    let max_rounds = max_tool_rounds();
    let mut round: u32 = 0;

    // ── 3. ReAct loop ─────────────────────────────────────────────────────────
    loop {
        // Check for interrupt at the top of every round.
        if *cancel_rx.borrow() {
            return_interrupted!();
        }

        // Re-fetch tool list from Executor on every round so that MCP providers
        // that registered after the turn started (e.g. Tiago bridge warming up)
        // are visible to the VLM immediately in the next round.
        let tool_list = executor
            .list_tools
            .call(Request::new(ListToolsRequest { refresh: true }))
            .await
            .map_err(|e| anyhow::anyhow!("ListTools RPC failed: {e}"))?
            .into_inner()
            .tools;

        let tool_defs: Vec<ToolDef> = tool_list
            .iter()
            .map(|t| {
                let schema: serde_json::Value =
                    serde_json::from_str(&t.input_schema_json).unwrap_or_default();
                ToolDef::new(&t.tool_name, &t.description, schema)
            })
            .collect();

        let routing_map: std::collections::HashMap<String, ToolRouting> = tool_list
            .iter()
            .filter_map(|t| t.routing.clone().map(|r| (t.tool_name.clone(), r)))
            .collect();

        let mut messages = vec![Message::system(&system_prompt)];
        messages.extend(sanitize_history_for_vlm(&session.history));

        let mut vlm = vlm.lock().await;
        let (content, raw_tool_calls) = {
            let mut stream = vlm
                .chat_stream(&messages, &tool_defs)
                .await
                .map_err(|e| anyhow::anyhow!("VLM stream error: {e}"))?;

            let mut full_text = String::new();
            let mut tool_calls: Vec<crate::vlm::ToolCall> = Vec::new();

            loop {
                tokio::select! {
                    biased;
                    // Cancel takes priority — checked before every new VLM token.
                    _ = cancel_rx.changed() => {
                        drop(stream);
                        drop(vlm);
                        return_interrupted!();
                    }
                    msg = stream.message() => {
                        let event = match msg.map_err(|e| anyhow::anyhow!("VLM stream recv: {e}"))? {
                            Some(e) => e,
                            None => break,
                        };
                        match VlmClient::parse_stream_event(event) {
                            VlmStreamItem::TextDelta(delta) => {
                                let _ = tx.send(Ok(pilot_wire::pack(
                                    &session_id,
                                    PilotStreamBody::TextChunk(delta.clone()),
                                ))).await;
                                full_text.push_str(&delta);
                            }
                            VlmStreamItem::ToolCall(tc) => {
                                tool_calls.push(tc);
                            }
                            VlmStreamItem::Finish(_) => {}
                        }
                    }
                }
            }

            let content = if full_text.is_empty() {
                None
            } else {
                Some(full_text)
            };
            (content, tool_calls)
        };
        drop(vlm); // release lock before async Executor call

        // No tool calls → VLM is done.
        if raw_tool_calls.is_empty() {
            let final_text = content.unwrap_or_default();
            if !final_text.is_empty() {
                session.history.push(Message::assistant(&final_text));
            }
            let _ = tx
                .send(Ok(pilot_wire::pack(
                    &session_id,
                    PilotStreamBody::FinalText(final_text),
                )))
                .await;
            break;
        }

        // Push assistant message with tool calls into history.
        session
            .history
            .push(Message::assistant_tool_calls(raw_tool_calls.clone()));

        // ── 5. Build TaskGraph (v1: linear list of calls) ─────────────────────
        let graph_id = Uuid::new_v4().to_string();
        let calls: Vec<TaskCall> = raw_tool_calls
            .iter()
            .map(|tc| TaskCall {
                call_id: tc.id.clone(),
                tool_name: tc.function.name.clone(),
                args_json: tc.function.arguments.clone(),
                routing: routing_map.get(&tc.function.name).cloned(),
            })
            .collect();

        let graph = TaskGraph {
            graph_id: graph_id.clone(),
            session_id: session_id.clone(),
            round,
            calls: calls.clone(),
        };

        // Notify Liaison about the outgoing task graph slice.
        let _ = tx
            .send(Ok(pilot_wire::pack(
                &session_id,
                PilotStreamBody::TaskGraph(graph.clone()),
            )))
            .await;

        // ── 6. Dispatch to Executor ───────────────────────────────────────────
        let mut exec_stream = executor
            .graph
            .stream(Request::new(graph))
            .await
            .map_err(|e| anyhow::anyhow!("Executor Stream RPC failed: {e}"))?
            .into_inner();

        let mut results: Vec<TaskCallResult> = Vec::new();

        const EX_STARTED: u32 = 0;
        const EX_RESULT: u32 = 1;
        #[allow(dead_code)]
        const EX_COMPLETE: u32 = 2;

        while let Some(event) = exec_stream
            .message()
            .await
            .map_err(|e| anyhow::anyhow!("Executor stream recv: {e}"))?
        {
            match event.event_kind {
                EX_STARTED => {
                    if let Some(ref s) = event.started {
                        log::debug!("[pilot] executor started '{}'", s.tool_name);
                    }
                }
                EX_RESULT => {
                    if let Some(r) = event.result {
                        if r.success {
                            let preview: String = r.output.chars().take(120).collect();
                            let ellipsis = if r.output.len() > 120 { "…" } else { "" };
                            log::debug!(
                                "[pilot] tool result '{}': {}{}",
                                r.call_id,
                                preview,
                                ellipsis
                            );
                        } else {
                            log::debug!("[pilot] tool error '{}': {}", r.call_id, r.error);
                        }
                        results.push(r);
                    }
                }
                _ => {}
            }
        }

        // ── 7. Feed results back into history ─────────────────────────────────
        let mut deferred_followups: Vec<Message> = Vec::new();
        for r in &results {
            if r.success {
                let mapped = tool_result_to_messages(&r.call_id, &r.output);
                session.history.extend(mapped.tool_messages);
                deferred_followups.extend(mapped.followup_messages);
            } else {
                session.history.push(Message::tool_result(
                    &r.call_id,
                    &format!("error: {}", r.error),
                ));
            }
        }
        session.history.extend(deferred_followups);
        trim_history(&mut session.history, MAX_HISTORY);

        // Emit BatchResult to Liaison.
        let any_failed = results.iter().any(|r| !r.success);
        let batch_result = BatchResult {
            graph_id: graph_id.clone(),
            session_id: session_id.clone(),
            round,
            results,
            any_failed,
        };
        let _ = tx
            .send(Ok(pilot_wire::pack(
                &session_id,
                PilotStreamBody::BatchResult(batch_result),
            )))
            .await;

        round += 1;
        if round as usize >= max_rounds {
            log::warn!(
                "[pilot] hit max tool rounds ({}), stopping turn",
                max_rounds
            );
            break;
        }
    }

    // ── 8. Mark turn complete ─────────────────────────────────────────────────
    session.turn_count += 1;
    let _ = tx
        .send(Ok(pilot_wire::pack(
            &session_id,
            PilotStreamBody::Status(SessionStatusEvent {
                session_id: session_id.clone(),
                state: SessionState::Completed as u32,
                message: String::new(),
            }),
        )))
        .await;

    Ok(())
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Map executor tool output to VLM history messages.
///
/// OpenAI-compatible VLM endpoints reject `image_url` content on `tool` role
/// messages. When a tool returns an image, keep the tool result textual and
/// append a synthetic `user` vision message that carries the image.
struct ToolResultHistory {
    tool_messages: Vec<Message>,
    followup_messages: Vec<Message>,
}

fn tool_result_to_messages(call_id: &str, output: &str) -> ToolResultHistory {
    let Ok(v) = serde_json::from_str::<serde_json::Value>(output) else {
        return ToolResultHistory {
            tool_messages: vec![Message::tool_result(call_id, output)],
            followup_messages: vec![],
        };
    };

    if let Some(b64) = v.get("image_base64").and_then(|x| x.as_str()) {
        let fmt = v.get("format").and_then(|x| x.as_str()).unwrap_or("jpeg");
        return ToolResultHistory {
            tool_messages: vec![Message::tool_result(
                call_id,
                &format!("[{fmt} image attached]"),
            )],
            followup_messages: vec![Message::user_with_image(
                "Tool returned an image. Analyze this image together with the tool result above.",
                b64.to_string(),
            )],
        };
    }

    // sensor_msgs/msg/Image — matches e.g. camera_snapshot / camera_depth_snapshot MCP tools.
    // Skip images with encoding="error" — these are error placeholders, not real images.
    let img_encoding = v.get("encoding").and_then(|e| e.as_str());
    if v.get("width").is_some()
        && v.get("height").is_some()
        && img_encoding.is_some()
        && img_encoding != Some("error")
        && v.get("data")
            .and_then(|d| d.as_str())
            .is_some_and(|s| !s.is_empty())
    {
        let enc = img_encoding.unwrap_or("jpeg");
        let b64 = v.get("data").and_then(|d| d.as_str()).unwrap_or("");
        return ToolResultHistory {
            tool_messages: vec![Message::tool_result(
                call_id,
                &format!("[sensor_msgs/Image encoding={enc}]"),
            )],
            followup_messages: vec![Message::user_with_image(
                "Tool returned an image. Analyze this image together with the tool result above.",
                b64.to_string(),
            )],
        };
    }

    ToolResultHistory {
        tool_messages: vec![Message::tool_result(call_id, output)],
        followup_messages: vec![],
    }
}

fn trim_history(history: &mut Vec<Message>, max: usize) {
    if history.len() > max {
        let remove = history.len() - max;
        history.drain(0..remove);
    }
}

fn sanitize_history_for_vlm(history: &[Message]) -> Vec<Message> {
    let mut out: Vec<Message> = Vec::with_capacity(history.len());
    let mut open_tool_call_ids: std::collections::HashSet<String> = Default::default();

    for msg in history {
        match msg.role.as_str() {
            "assistant" => {
                if let Some(calls) = &msg.tool_calls {
                    open_tool_call_ids.clear();
                    for tc in calls {
                        open_tool_call_ids.insert(tc.id.clone());
                    }
                } else {
                    open_tool_call_ids.clear();
                }
                out.push(msg.clone());
            }
            "tool" => {
                let Some(call_id) = msg.tool_call_id.as_ref() else {
                    continue;
                };
                if open_tool_call_ids.remove(call_id) {
                    out.push(msg.clone());
                }
            }
            _ => {
                open_tool_call_ids.clear();
                out.push(msg.clone());
            }
        }
    }

    out
}

/// Extract `std_msgs/String.data` from tool output.
/// Accepts either raw text or JSON object payload: {"data": "..."}.
fn decode_string_output(output: &str) -> String {
    serde_json::from_str::<serde_json::Value>(output)
        .ok()
        .and_then(|v| {
            v.get("data")
                .and_then(|x| x.as_str())
                .map(ToString::to_string)
        })
        .unwrap_or_else(|| output.to_string())
}

/// Dispatch a single `search_memory` call to the Executor and return the
/// result text.  Returns `None` if the tool is not registered, the index is
/// empty, or any error occurs — the caller should never fail because of
/// missing memory context.
async fn prefetch_memory(
    query: &str,
    executor: &mut ExecutorConn,
    routing: Option<ToolRouting>,
) -> Option<String> {
    let routing = routing?;

    const EX_RESULT: u32 = 1;

    let graph = TaskGraph {
        graph_id: Uuid::new_v4().to_string(),
        session_id: "memory-prefetch".to_string(),
        round: 0,
        calls: vec![TaskCall {
            call_id: Uuid::new_v4().to_string(),
            tool_name: "search_memory".to_string(),
            // Contract-aligned MCP input uses std_msgs/String.data.
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
                log::debug!("[pilot] memory prefetch: {}", out);
                return Some(out);
            }
            return None;
        }
    }
    None
}

/// Best-effort MCP `compact_memory` (session teardown). Ignores failures (tool may be absent).
async fn try_compact_memory(executor: &mut ExecutorConn) {
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

    const EX_RESULT: u32 = 1;

    let graph = TaskGraph {
        graph_id: Uuid::new_v4().to_string(),
        session_id: "memory-compact".to_string(),
        round: 0,
        calls: vec![TaskCall {
            call_id: Uuid::new_v4().to_string(),
            tool_name: "compact_memory".to_string(),
            // Contract-aligned MCP input uses std_msgs/Empty (empty object payload).
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
        if event.event_kind == EX_RESULT {
            if let Some(r) = event.result {
                let out = decode_string_output(&r.output);
                if r.success {
                    log::debug!("[pilot] compact_memory: {}", out);
                } else {
                    log::debug!("[pilot] compact_memory failed: {}", out);
                }
            }
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{skip_memory_prefetch, task_is_session_end};
    use crate::pilot::Task;

    fn task(ctx: &str) -> Task {
        Task {
            task_id: "t".into(),
            session_id: "s".into(),
            source: 0,
            text: String::new(),
            audio_data: Vec::new(),
            context_json: ctx.into(),
            timestamp_ms: 0,
        }
    }

    #[test]
    fn session_end_explicit() {
        assert!(task_is_session_end(&task(r#"{"session_end":true}"#)));
    }

    #[test]
    fn session_end_legacy_alias() {
        assert!(task_is_session_end(&task(
            r#"{"robonix_session_end":true}"#
        )));
    }

    #[test]
    fn session_end_false_or_absent() {
        assert!(!task_is_session_end(&task("")));
        assert!(!task_is_session_end(&task(r#"{"foo":1}"#)));
        assert!(!task_is_session_end(&task(r#"{"session_end":false}"#)));
    }

    #[test]
    fn skip_prefetch_chitchat() {
        assert!(skip_memory_prefetch("hi"));
        assert!(skip_memory_prefetch("Hello"));
        assert!(skip_memory_prefetch("你是谁"));
        assert!(skip_memory_prefetch("你好"));
    }

    #[test]
    fn no_skip_prefetch_real_query() {
        assert!(!skip_memory_prefetch("open the door"));
        assert!(!skip_memory_prefetch("帮我找个红色杯子"));
    }
}
