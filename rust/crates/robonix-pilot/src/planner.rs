// SPDX-License-Identifier: MulanPSL-2.0
// VLM-driven ReAct loop (core Pilot logic).
//
// One call to `run_turn` handles a single user turn:
//   1. Build system prompt (optional SOUL.md + operating principles).
//   1b. Proactively call search_memory via Executor; inject results into system prompt.
//   2. Add user message to history.
//   3. Loop: fetch tools → VLM call → tool calls → Plan → Executor → feed results back.
//   4. Stream PilotEvents to the Liaison caller via `tx`.

use crate::discovery::{self, CapEntry};
use crate::history;
use crate::memory;
use crate::pb::contracts::system_executor_client::SystemExecutorClient;
use crate::pb::pilot::{
    BatchResult, CapabilityCall, CapabilityCallResult, PilotEvent, Plan, SessionStatusEvent, Task,
};
use crate::service::{self, PilotStreamBody, SessionState};
use crate::vlm::{Message, ToolDef, VlmClient, VlmStreamItem};
use anyhow::Result;
use futures_util::StreamExt;
use robonix_atlas::client::AtlasClient;
use std::collections::HashMap;
use std::path::PathBuf;
use tokio::sync::{mpsc::Sender, watch};
use tonic::Request;
use tonic::transport::Channel;
use uuid::Uuid;

/// gRPC client for executor's plan-dispatch contract. Pilot only ever calls
/// `Execute(Plan)` — discovery happens directly against atlas now.
pub struct ExecutorConn {
    pub graph: SystemExecutorClient<Channel>,
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

/// Drive one user turn: build prompt, run the ReAct loop, stream events.
///
/// `history` is the running LLM conversation (one Vec per logical chat
/// thread, keyed by `task.session_id` at the call site). It's mutated in
/// place across rounds so subsequent turns see prior tool calls/results.
pub async fn run_turn(
    task: &Task,
    history: &mut Vec<Message>,
    vlm: &VlmClient,
    executor: &mut ExecutorConn,
    atlas: &mut AtlasClient,
    consumer_id: &str,
    tx: &Sender<Result<PilotEvent, tonic::Status>>,
    mut cancel_rx: watch::Receiver<bool>,
) -> Result<()> {
    let session_id = task.session_id.clone();

    /// Emit an "interrupted" status event and return Ok to end the turn cleanly.
    macro_rules! return_interrupted {
        () => {{
            let _ = tx
                .send(Ok(service::pack(
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
        memory::try_compact(executor, atlas, consumer_id).await;
        let _ = tx
            .send(Ok(service::pack(
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

    // ── 1. Build system prompt (once per turn) ────────────────────────────────
    let base_prompt = build_system_prompt(load_agent_soul().as_deref());

    // Pilot's capability catalog comes straight from atlas (filtered to MCP
    // transport — only those are LLM-callable). Per-cap description +
    // input_schema_json come from McpParams, peeked via brief Connect in
    // `discovery::discover`.
    let initial_caps = discovery::discover(atlas, consumer_id)
        .await
        .map_err(|e| anyhow::anyhow!("atlas capability discovery failed: {e}"))?;
    let search_memory_target = initial_caps
        .iter()
        .find(|c| c.name == "search_memory")
        .map(|c| (c.cap_id.clone(), c.contract_id.clone()));

    // ── 1b. Pre-fetch long-term memory ────────────────────────────────────────
    // Silently dispatches search_memory before the first VLM call so that
    // relevant past context is available from the start of the turn.
    let system_prompt = if skip_memory_prefetch(&task.text) {
        base_prompt
    } else {
        match memory::prefetch(&task.text, executor, search_memory_target).await {
            Some(mem) => format!(
                "{base_prompt}\n\n## Relevant past memories (System Context)\n\n{mem}\n\n---\n\n"
            ),
            None => base_prompt,
        }
    };

    // ── 2. Add user message to history ────────────────────────────────────────
    history.push(Message::user(&task.text));
    history::trim(history, MAX_HISTORY);

    let max_rounds = max_tool_rounds();
    let mut round: u32 = 0;

    // ── 3. ReAct loop ─────────────────────────────────────────────────────────
    loop {
        // Check for interrupt at the top of every round.
        if *cancel_rx.borrow() {
            return_interrupted!();
        }

        // Re-discover capabilities from atlas every round so caps that
        // registered mid-turn (e.g. tiago bridge warming up) are visible in
        // the next VLM call.
        let cap_list: Vec<CapEntry> = discovery::discover(atlas, consumer_id)
            .await
            .map_err(|e| anyhow::anyhow!("atlas capability discovery failed: {e}"))?;

        let tool_defs: Vec<ToolDef> = cap_list
            .iter()
            .map(|c| {
                let schema: serde_json::Value =
                    serde_json::from_str(&c.input_schema_json).unwrap_or_default();
                ToolDef::new(&c.name, &c.description, schema)
            })
            .collect();

        // LLM-tool-name → (cap_id, contract_id) so we can build CapabilityCall
        // when the model picks a tool. Names are MCP-side tool names = leaf
        // of contract_id; collisions across caps would be a user-config bug.
        let target_map: HashMap<String, (String, String)> = cap_list
            .iter()
            .map(|c| (c.name.clone(), (c.cap_id.clone(), c.contract_id.clone())))
            .collect();

        let mut messages = vec![Message::system(&system_prompt)];
        messages.extend(history::sanitize_for_vlm(history));

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
                        return_interrupted!();
                    }
                    item = stream.next() => {
                        let item = match item {
                            Some(Ok(it)) => it,
                            Some(Err(e)) => return Err(anyhow::anyhow!("VLM stream recv: {e:#}")),
                            None => break,
                        };
                        match item {
                            VlmStreamItem::TextDelta(delta) => {
                                let _ = tx.send(Ok(service::pack(
                                    &session_id,
                                    PilotStreamBody::TextChunk(delta.clone()),
                                ))).await;
                                full_text.push_str(&delta);
                            }
                            VlmStreamItem::ToolCall(tc) => {
                                tool_calls.push(tc);
                            }
                            VlmStreamItem::Finish => {}
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

        // No tool calls → VLM is done.
        if raw_tool_calls.is_empty() {
            let final_text = content.unwrap_or_default();
            if !final_text.is_empty() {
                history.push(Message::assistant(&final_text));
            }
            let _ = tx
                .send(Ok(service::pack(
                    &session_id,
                    PilotStreamBody::FinalText(final_text),
                )))
                .await;
            break;
        }

        // Push assistant message with tool calls into history.
        history.push(Message::assistant_tool_calls(raw_tool_calls.clone()));

        // ── 5. Build Plan (v1: linear list of calls) ─────────────────────
        let plan_id = Uuid::new_v4().to_string();
        let calls: Vec<CapabilityCall> = raw_tool_calls
            .iter()
            .map(|tc| {
                let (cap_id, contract_id) = target_map
                    .get(&tc.function.name)
                    .cloned()
                    .unwrap_or_else(|| {
                        // Unknown tool name (LLM hallucinated). Build a call
                        // anyway so the executor surfaces a clear error.
                        log::warn!(
                            "[pilot] LLM picked unknown tool '{}' — no matching capability in catalog",
                            tc.function.name
                        );
                        (String::new(), tc.function.name.clone())
                    });
                CapabilityCall {
                    call_id: tc.id.clone(),
                    cap_id,
                    contract_id,
                    args_json: tc.function.arguments.clone(),
                }
            })
            .collect();

        let graph = Plan {
            plan_id: plan_id.clone(),
            session_id: session_id.clone(),
            round,
            calls: calls.clone(),
        };

        // Notify Liaison about the outgoing task graph slice.
        let _ = tx
            .send(Ok(service::pack(
                &session_id,
                PilotStreamBody::Plan(graph.clone()),
            )))
            .await;

        // ── 6. Dispatch to Executor ───────────────────────────────────────────
        let mut exec_stream = executor
            .graph
            .execute(Request::new(graph))
            .await
            .map_err(|e| anyhow::anyhow!("Executor Stream RPC failed: {e}"))?
            .into_inner();

        let mut results: Vec<CapabilityCallResult> = Vec::new();

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
                        log::debug!(
                            "[pilot] executor started cap='{}' contract='{}'",
                            s.cap_id,
                            s.contract_id
                        );
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
                let mapped = history::tool_result_to_messages(&r.call_id, &r.output);
                history.extend(mapped.tool_messages);
                deferred_followups.extend(mapped.followup_messages);
            } else {
                history.push(Message::tool_result(
                    &r.call_id,
                    &format!("error: {}", r.error),
                ));
            }
        }
        history.extend(deferred_followups);
        history::trim(history, MAX_HISTORY);

        // Emit BatchResult to Liaison.
        let any_failed = results.iter().any(|r| !r.success);
        let batch_result = BatchResult {
            plan_id: plan_id.clone(),
            session_id: session_id.clone(),
            round,
            results,
            any_failed,
        };
        let _ = tx
            .send(Ok(service::pack(
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
    let _ = tx
        .send(Ok(service::pack(
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

// ── System prompt + SOUL ──────────────────────────────────────────────────────
// Optional `SOUL.md` (agent personality) is read from `$ROBONIX_PILOT_SOUL`,
// then `~/.robonix/SOUL.md`. There is no skill index — skill caps surface as
// regular tools through `executor.list_tools`, with descriptions sourced from
// each cap's CAPABILITY.md.

fn load_agent_soul() -> Option<String> {
    if let Ok(p) = std::env::var("ROBONIX_PILOT_SOUL") {
        let p = p.trim();
        if !p.is_empty() {
            return std::fs::read_to_string(p).ok();
        }
    }
    let home = std::env::var_os("HOME").map(PathBuf::from)?;
    let soul = home.join(".robonix").join("SOUL.md");
    if soul.is_file() {
        return std::fs::read_to_string(soul).ok();
    }
    None
}

fn build_system_prompt(soul: Option<&str>) -> String {
    let mut p = String::new();
    if let Some(s) = soul {
        let t = s.trim();
        if !t.is_empty() {
            p.push_str("## Agent SOUL\n\n");
            p.push_str(t);
            p.push_str("\n\n---\n\n");
        }
    }
    p.push_str(
        "\
You are the Robonix Pilot — the reasoning and planning component of a robot system.
You receive requests from a user or higher-level system and translate them into actions
by calling the tools available to you.

## Operating principles
- ACT immediately using your tools. Do not ask the user to run things themselves.
- Execute all necessary steps before replying.
- Each tool call you make is dispatched to the Executor runtime, which handles the
  actual robot hardware or service call.
- Do NOT claim missing capabilities unless verified from current tools/results.
  - If `search_memory` / `save_memory` / `compact_memory` tools are available, treat
    long-term memory as available via those tools.
- Prefer structured output; report tool results concisely.
- If a tool returns an error, diagnose and retry, or report to the user.
",
    );
    p
}

#[cfg(test)]
mod tests {
    use super::{skip_memory_prefetch, task_is_session_end};
    use crate::pb::pilot::Task;

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
