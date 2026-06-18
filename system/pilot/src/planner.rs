// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
use crate::discovery::{self, llm_name};
use crate::history;
use crate::memory;
use crate::pb::contracts::robonix_system_executor_execute_client::RobonixSystemExecutorExecuteClient;
use crate::pb::pilot::{
    BatchResult, CapabilityCall, CapabilityCallResult, PilotEvent, Plan, RtdlNode,
    SessionStatusEvent, Task,
};
use crate::service::{self, PilotStreamBody, SessionState};
use crate::vlm::{Message, VlmClient, VlmStreamItem};
use anyhow::{Context, Result};
use futures_util::StreamExt;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use std::collections::HashMap;
use std::path::PathBuf;
use tokio::sync::{mpsc::Sender, watch};
use tonic::Request;
use tonic::transport::Channel;
use uuid::Uuid;

/// gRPC client for executor's plan-dispatch contract. Pilot only ever calls
/// `Execute(Plan)` — discovery happens directly against atlas now.
pub struct ExecutorConn {
    pub graph: RobonixSystemExecutorExecuteClient<Channel>,
}

type CapabilityTarget = (String, String);
type CapabilityTargetMap = HashMap<String, CapabilityTarget>;

const RTDL_SEQUENCE: u32 = 0;
const RTDL_PARALLEL: u32 = 1;
const RTDL_DO: u32 = 2;
const EXECUTOR_EVT_NODE_STATE: u32 = 1;
const EXECUTOR_STATE_SUCCEEDED: u32 = 2;
const EXECUTOR_STATE_FAILED: u32 = 3;
const EXECUTOR_STATE_CANCELED: u32 = 4;
const EXECUTOR_STATE_TIMEOUT: u32 = 5;

struct DisplayCapability<'a> {
    display_name: String,
    provider_id: &'a str,
    cap: &'a atlas_pb::Capability,
}

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

/// `context_json.modality` — set by liaison to "text" / "voice" / "api".
/// `None` when the field is missing or context_json is empty/malformed.
fn task_modality(task: &Task) -> Option<String> {
    let j = task.context_json.trim();
    if j.is_empty() {
        return None;
    }
    serde_json::from_str::<serde_json::Value>(j)
        .ok()
        .and_then(|v| {
            v.get("modality")
                .and_then(|x| x.as_str())
                .map(str::to_string)
        })
}

/// Skip vector memory prefetch for trivial chit-chat (saves latency and noise).
fn skip_memory_prefetch(user_text: &str) -> bool {
    let t = user_text.trim();
    let lower = t.to_lowercase();
    lower == "hi" || lower == "hello"
}

#[allow(clippy::too_many_arguments)]
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

    // 1. Build system prompt (once per turn)
    let base_prompt = build_system_prompt(load_agent_soul().as_deref());

    // Pilot's capability catalog comes straight from atlas (filtered to
    // MCP transport — only those are LLM-callable). McpParams ride along
    // in Capability.params, no Connect needed.
    let _ = consumer_id; // currently unused; kept on the signature for future channel-tracked discovery
    let initial_caps = discovery::discover(atlas)
        .await
        .map_err(|e| anyhow::anyhow!("atlas capability discovery failed: {e}"))?;
    // Pilot binds to the canonical contract_id, not the LLM-facing tool
    // name: the latter is just the contract_id leaf and a provider could
    // rename it freely. contract_id is the stable identity.
    let search_memory_target = initial_caps
        .iter()
        .find(|(_, cap)| cap.contract_id == "robonix/service/memory/search")
        .map(|(provider_id, cap)| (provider_id.clone(), cap.contract_id.clone()));

    // 1b. Pre-fetch long-term memory
    // Silently dispatches search_memory before the first VLM call so that
    // relevant past context is available from the start of the turn.
    let mut system_prompt = if skip_memory_prefetch(&task.text) {
        base_prompt
    } else {
        match memory::prefetch(&task.text, executor, search_memory_target).await {
            Some(mem) => format!(
                "{base_prompt}\n\n## Relevant past memories (System Context)\n\n{mem}\n\n---\n\n"
            ),
            None => base_prompt,
        }
    };

    // 1c. Append the per-capability docs index. Each provider that registered
    // a `capability_md_path` shows up here as a one-liner pointing at its
    // CAPABILITY.md; the LLM is instructed to lazy-load those via the
    // `read_file` builtin when it actually needs that provider. This keeps the
    // system prompt tiny while still giving the LLM full per-provider context
    // when relevant. Errors here are non-fatal — providers that didn't register
    // a path simply don't appear in the block.
    if let Ok(docs) = discovery::cap_md_index(atlas).await
        && !docs.is_empty()
    {
        let mut block = String::from(
            "\n\n## Capability docs (lazy-load via `read_file`)\n\
             Each capability below ships a CAPABILITY.md describing its operations \
             and the recommended usage pattern. Read the relevant one with the \
             `read_file` capability BEFORE you start using the capability — the \
             short descriptions in your capability list are intentionally terse.\n\n",
        );
        for d in &docs {
            block.push_str(&format!(
                "- `{}` ({}): `{}`\n",
                d.provider_id, d.namespace, d.md_path
            ));
        }
        system_prompt.push_str(&block);
    }

    // Voice-mode brevity hint. Liaison stamps `context_json.modality =
    // "voice"` for every voice-path Task; in that case we ask the VLM
    // for a short reply because the user is going to *hear* it via TTS,
    // not read a Markdown wall. Threshold is intentionally tight (~30
    // Chinese chars / ~50 English words) — barge-in matters more than
    // exhaustive coverage and the user can always ask follow-ups.
    if task_modality(task).as_deref() == Some("voice") {
        system_prompt.push_str(
            "\n\n## Voice mode\n\n\
             The user is interacting via voice; this reply will be\n\
             spoken back through TTS. Keep the response short (≤ ~30\n\
             characters Chinese / ~50 words English), no markdown\n\
             lists, no headings, no code blocks, plain conversational\n\
             tone. If the answer genuinely needs structure, summarise\n\
             out loud and offer to elaborate when asked.\n",
        );
    }
    let system_prompt = system_prompt;

    // 2. Add user message to history
    history.push(Message::user(&task.text));
    history::trim(history, MAX_HISTORY);

    let max_rounds = max_tool_rounds();
    let mut round: u32 = 0;

    // 3. RTDL planning/execution loop. This preserves the old tool loop
    // shape, but the VLM now emits JSON `{content,rtdl}` instead of
    // assistant.tool_calls.
    loop {
        // Check for interrupt at the top of every round.
        if *cancel_rx.borrow() {
            return_interrupted!();
        }

        // Re-discover capabilities from atlas every round so providers that
        // registered mid-turn are visible in the next call.
        let cap_list = discovery::discover(atlas)
            .await
            .map_err(|e| anyhow::anyhow!("atlas capability discovery failed: {e}"))?;

        let display_caps = build_display_capabilities(&cap_list);
        let target_map = build_capability_target_map(&display_caps);
        let rtdl_prompt = build_rtdl_prompt(&display_caps)?;

        let mut messages = vec![Message::system(&format!(
            "{system_prompt}\n\n{rtdl_prompt}"
        ))];

        messages.extend(history::sanitize_for_vlm(history));

        let (content, raw_tool_calls) = {
            let mut stream = vlm
                .chat_stream(&messages, &[])
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

        if !raw_tool_calls.is_empty() {
            anyhow::bail!("VLM returned tool_calls in RTDL mode");
        }

        let raw_content = content.unwrap_or_default();
        log::debug!("raw_content: {}", raw_content);
        let (assistant_content, rtdl) =
            parse_rtdl_assistant_response(&raw_content).with_context(|| {
                format!(
                    "parse RTDL assistant response: {}",
                    raw_preview(&raw_content)
                )
            })?;
        log::debug!(
            "[pilot/rtdl] parsed assistant JSON round={} content_chars={}",
            round,
            assistant_content.chars().count()
        );
        let plan_id = Uuid::new_v4().to_string();
        let graph = expand_rtdl_to_plan(
            &rtdl,
            &target_map,
            plan_id.clone(),
            session_id.clone(),
            round,
        )
        .context("expand RTDL to Plan")?;
        log::info!(
            "[pilot/rtdl] expanded plan_id={} round={} calls={}",
            graph.plan_id,
            graph.round,
            plan_call_count(&graph)
        );

        if plan_call_count(&graph) == 0 {
            log::info!(
                "[pilot/rtdl] empty sequence plan_id={} round={} final_text=true",
                graph.plan_id,
                graph.round
            );
            if !assistant_content.is_empty() {
                history.push(Message::assistant(&assistant_content));
            }
            let _ = tx
                .send(Ok(service::pack(
                    &session_id,
                    PilotStreamBody::FinalText(assistant_content),
                )))
                .await;
            break;
        }

        if !assistant_content.is_empty() {
            history.push(Message::assistant(&assistant_content));
            let _ = tx
                .send(Ok(service::pack(
                    &session_id,
                    PilotStreamBody::TextChunk(assistant_content.clone()),
                )))
                .await;
        }

        // Notify Liaison about the outgoing task graph slice.
        log::debug!(
            "[pilot/rtdl] sending plan_id={} calls={} to liaison/executor",
            graph.plan_id,
            plan_call_count(&graph)
        );
        let _ = tx
            .send(Ok(service::pack(
                &session_id,
                PilotStreamBody::Plan(graph.clone()),
            )))
            .await;

        // ── 6. Dispatch to Executor ───────────────────────────────────────────
        let submitted_plan = graph.clone();
        let mut exec_stream = executor
            .graph
            .execute(Request::new(graph))
            .await
            .map_err(|e| anyhow::anyhow!("Executor Stream RPC failed: {e}"))?
            .into_inner();

        let mut results: Vec<CapabilityCallResult> = Vec::new();

        while let Some(event) = exec_stream
            .message()
            .await
            .map_err(|e| anyhow::anyhow!("Executor stream recv: {e}"))?
        {
            if event.event_kind == EXECUTOR_EVT_NODE_STATE
                && let Some(ns) = event.node_state
                && is_terminal_executor_state(ns.state)
            {
                let r = executor_node_state_to_result(&submitted_plan, ns);
                if r.success {
                    let preview: String = r.output.chars().take(120).collect();
                    let ellipsis = if r.output.len() > 120 { "..." } else { "" };
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

        // ── 7. Feed results back into history ─────────────────────────────────
        let mut deferred_followups: Vec<Message> = Vec::new();
        for r in &results {
            let mapped = rtdl_result_to_messages(r);
            history.extend(mapped.tool_messages);
            deferred_followups.extend(mapped.followup_messages);
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

fn build_display_capabilities(
    cap_list: &[(String, atlas_pb::Capability)],
) -> Vec<DisplayCapability<'_>> {
    cap_list
        .iter()
        .map(|(provider_id, cap)| DisplayCapability {
            display_name: format!("{}.{}", provider_id, llm_name(&cap.contract_id)),
            provider_id: provider_id.as_str(),
            cap,
        })
        .collect()
}

fn build_capability_target_map(display_caps: &[DisplayCapability<'_>]) -> CapabilityTargetMap {
    let mut out = HashMap::new();
    for cap in display_caps {
        out.insert(
            cap.display_name.clone(),
            (cap.provider_id.to_string(), cap.cap.contract_id.clone()),
        );
    }
    out
}

fn build_rtdl_prompt(display_caps: &[DisplayCapability<'_>]) -> Result<String> {
    // Compile-time embedded; edit `rtdl_protocol.md` in this crate root.
    let mut p = String::from(include_str!("../rtdl_protocol.md"));
    p.push_str("\n## Available capabilities\n\n");

    for cap in display_caps {
        let c = cap.cap;
        let Some(atlas_pb::transport_params::Kind::Mcp(mcp)) =
            c.params.as_ref().and_then(|p| p.kind.as_ref())
        else {
            continue;
        };
        let schema: serde_json::Value =
            serde_json::from_str(&mcp.input_schema_json).unwrap_or(serde_json::Value::Null);
        p.push_str(&format!(
            "- capability_name: {}\n  - description: {}\n  - args_schema: `{}`\n",
            cap.display_name,
            c.description.trim(),
            schema
        ));
    }

    // log::debug!("[pilot/rtdl] rtdl prompt:\n{}", p);
    Ok(p)
}

/// Parses one VLM reply in RTDL envelope form.
///
/// The model must emit a JSON **object** (single root) whose only keys are
/// `content` and `rtdl`: `content` is user-facing narration (string); `rtdl`
/// is the declarative ops tree (another JSON object) consumed by
/// [`expand_rtdl_to_plan`].
///
/// Returns `(content, rtdl)` on success — the string is cloned for callers;
/// `rtdl` is a [`serde_json::Value`] object subtree (not validated beyond
/// "`rtdl` is an object").
///
/// Fails if `raw` is not valid JSON, the root is not an object, the key set
/// is not exactly `{content, rtdl}`, `content` is not a JSON string, or
/// `rtdl` is not a JSON object.
fn parse_rtdl_assistant_response(raw: &str) -> Result<(String, serde_json::Value)> {
    let v: serde_json::Value = serde_json::from_str(raw)?;
    let obj = v
        .as_object()
        .ok_or_else(|| anyhow::anyhow!("assistant response must be a JSON object"))?;
    if obj.len() != 2 || !obj.contains_key("content") || !obj.contains_key("rtdl") {
        anyhow::bail!("assistant response must contain exactly `content` and `rtdl`");
    }
    let content = obj
        .get("content")
        .and_then(|x| x.as_str())
        .ok_or_else(|| anyhow::anyhow!("assistant `content` must be a string"))?
        .to_string();
    let rtdl = obj
        .get("rtdl")
        .filter(|x| x.is_object())
        .ok_or_else(|| anyhow::anyhow!("assistant `rtdl` must be an object"))?
        .clone();
    Ok((content, rtdl))
}

fn raw_preview(raw: &str) -> String {
    if raw.is_empty() {
        return "assistant content was empty".to_string();
    }
    let preview: String = raw.chars().take(240).collect();
    let ellipsis = if raw.chars().count() > 240 { "..." } else { "" };
    format!("assistant content preview: {preview:?}{ellipsis}")
}

fn expand_rtdl_to_plan(
    rtdl: &serde_json::Value,
    target_map: &CapabilityTargetMap,
    plan_id: String,
    session_id: String,
    round: u32,
) -> Result<Plan> {
    let mut nodes = Vec::new();
    let mut next_call = 0usize;
    let root_index = expand_rtdl_node(rtdl, "$", target_map, &plan_id, &mut next_call, &mut nodes)?;
    Ok(Plan {
        plan_id,
        session_id,
        round,
        nodes,
        root_index,
    })
}

fn expand_rtdl_node(
    node: &serde_json::Value,
    path: &str,
    target_map: &CapabilityTargetMap,
    plan_id: &str,
    next_call: &mut usize,
    nodes: &mut Vec<RtdlNode>,
) -> Result<u32> {
    let obj = node
        .as_object()
        .ok_or_else(|| anyhow::anyhow!("{path}: RTDL node must be an object"))?;
    let op = obj
        .get("op")
        .and_then(|x| x.as_str())
        .ok_or_else(|| anyhow::anyhow!("{path}.op must be a string"))?;

    match op {
        "sequence" | "parallel" => {
            if obj.len() != 2 || !obj.contains_key("children") {
                anyhow::bail!("{path}: {op} node must contain only `op` and `children`");
            }
            let children = obj
                .get("children")
                .and_then(|x| x.as_array())
                .ok_or_else(|| anyhow::anyhow!("{path}.children must be an array"))?;
            let node_index = nodes.len() as u32;
            let node_kind = if op == "sequence" {
                RTDL_SEQUENCE
            } else {
                RTDL_PARALLEL
            };
            nodes.push(RtdlNode {
                node_kind,
                children: Vec::new(),
                call: None,
            });
            let mut child_indices = Vec::with_capacity(children.len());
            for (idx, child) in children.iter().enumerate() {
                let child_index = expand_rtdl_node(
                    child,
                    &format!("{path}.children[{idx}]"),
                    target_map,
                    plan_id,
                    next_call,
                    nodes,
                )?;
                child_indices.push(child_index);
            }
            nodes[node_index as usize].children = child_indices;
            Ok(node_index)
        }
        "do" => {
            if obj.len() != 3 || !obj.contains_key("cap") || !obj.contains_key("args") {
                anyhow::bail!("{path}: do node must contain only `op`, `cap`, and `args`");
            }
            let cap = obj
                .get("cap")
                .and_then(|x| x.as_str())
                .ok_or_else(|| anyhow::anyhow!("{path}.cap must be a string"))?;
            let args = obj
                .get("args")
                .filter(|x| x.is_object())
                .ok_or_else(|| anyhow::anyhow!("{path}.args must be an object"))?;
            let (provider_id, contract_id) = target_map
                .get(cap)
                .cloned()
                .ok_or_else(|| anyhow::anyhow!("{path}.cap unknown capability `{cap}`"))?;
            let call_index = *next_call;
            *next_call += 1;
            let node_index = nodes.len() as u32;
            nodes.push(RtdlNode {
                node_kind: RTDL_DO,
                children: Vec::new(),
                call: Some(CapabilityCall {
                    call_id: format!("{plan_id}:{call_index}"),
                    provider_id,
                    contract_id,
                    args_json: serde_json::to_string(args)?,
                }),
            });
            Ok(node_index)
        }
        other => anyhow::bail!("{path}.op unknown operator `{other}`"),
    }
}

fn plan_call_count(plan: &Plan) -> usize {
    plan.nodes
        .iter()
        .filter(|node| node.node_kind == RTDL_DO && node.call.is_some())
        .count()
}

fn is_terminal_executor_state(state: u32) -> bool {
    matches!(
        state,
        EXECUTOR_STATE_SUCCEEDED
            | EXECUTOR_STATE_FAILED
            | EXECUTOR_STATE_CANCELED
            | EXECUTOR_STATE_TIMEOUT
    )
}

/// Convert an Executor terminal node event into the result record shape exposed
/// on `BatchResult`. `do` nodes carry the concrete capability call result in
/// the event; the plan lookup is only a fallback for malformed or missing results.
fn executor_node_state_to_result(
    plan: &Plan,
    ns: crate::pb::executor::RtdlNodeState,
) -> CapabilityCallResult {
    if let Some(result) = ns.leaf_result {
        return result;
    }
    let call = plan
        .nodes
        .get(ns.node_index as usize)
        .and_then(|node| node.call.as_ref());
    let state = ns.state;
    let success = state == EXECUTOR_STATE_SUCCEEDED;
    let operator_detail = ns.operator_detail;
    CapabilityCallResult {
        call_id: call.map(|c| c.call_id.clone()).unwrap_or_default(),
        provider_id: call.map(|c| c.provider_id.clone()).unwrap_or_default(),
        contract_id: call.map(|c| c.contract_id.clone()).unwrap_or_default(),
        success,
        output: operator_detail.clone(),
        error: if success {
            String::new()
        } else {
            operator_detail
        },
    }
}

fn rtdl_result_to_messages(r: &CapabilityCallResult) -> history::ToolResultHistory {
    if !r.success {
        return history::ToolResultHistory {
            tool_messages: vec![Message::user(&format!(
                "Executor feedback for the current task (not a new user request): {}",
                r.output
            ))],
            followup_messages: vec![],
        };
    }

    let mapped = history::tool_result_to_messages(&r.call_id, &r.output);
    let tool_messages = mapped
        .tool_messages
        .into_iter()
        .map(|msg| {
            let content = msg.content.unwrap_or_default();
            Message::user(&format!(
                "Executor feedback for the current task (not a new user request): {}",
                content
            ))
        })
        .collect();

    history::ToolResultHistory {
        tool_messages,
        followup_messages: mapped.followup_messages,
    }
}

// ── System prompt + SOUL ──────────────────────────────────────────────────────
// Optional `SOUL.md` (agent personality) is read from `$ROBONIX_PILOT_SOUL`,
// then `~/.robonix/SOUL.md`. There is no skill index — skill providers surface as
// regular tools through `executor.list_tools`, with descriptions sourced from
// each provider's CAPABILITY.md.

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
by planning capability calls available to you.

## Operating principles
- ACT immediately using available capabilities. Do not ask the user to run things themselves.
- Each capability call you plan is dispatched to the Executor runtime, which handles the
  actual robot hardware or service call.
- Do NOT claim missing capabilities unless verified from the current capability list/results.
  - If `memory_search` / `memory_save` / `memory_compact` capabilities are available,
    treat long-term memory as available via those capabilities.
- Prefer structured output; report capability results concisely.
- If a capability returns an error, diagnose and retry, or report to the user.
- Some later messages may be labelled `Executor feedback for the current task`.
  Treat those as results of capability calls you already planned, not as new
  user requests.
- If executor feedback already contains enough information to answer the
  user's request, answer in `content` and output an empty RTDL sequence. Do
  not repeat the same observation capability just to confirm unchanged data.

## Persistence (READ THIS — most common failure mode)
The agent loop ENDS the turn the moment you produce an empty RTDL sequence.
Do NOT output an empty RTDL sequence until the user's task is *verifiably*
complete. Concretely:

- Define a clear success criterion at the start of the turn (e.g. for
  'turn around': yaw delta ≈ 180° from the starting pose; for 'find the
  door': a door is visible in a camera observation).
- For pure observation or visual question-answering tasks, one successful
  observation is usually enough. After answering from that observation, end
  with an empty RTDL sequence.
- For tasks that change robot or world state, loop plan-then-reason —
  observe, act, RE-observe — until the success criterion holds. After every
  physical action take a fresh camera observation to confirm the state actually
  changed the way you expected.
- A single short chassis movement burst typically rotates ~0.4–0.8 rad
  (≈ 25–45°) or translates ~0.1–0.2 m. To turn 180° you need MULTIPLE
  bursts; do not assume one call finishes the rotation.
- Only emit an empty RTDL sequence once the criterion is met OR you've
  exhausted reasonable attempts and need to report a blocker. 'Done.'
  with no verification is wrong — verify first.
- On the very rare case where the user explicitly cancels, you may stop
  early; otherwise keep going.
",
    );
    p
}

#[cfg(test)]
mod tests {
    use super::{
        CapabilityTargetMap, RTDL_DO, RTDL_PARALLEL, RTDL_SEQUENCE, expand_rtdl_to_plan,
        parse_rtdl_assistant_response, skip_memory_prefetch, task_is_session_end,
    };
    use crate::pb::pilot::Task;
    use serde_json::json;

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
    }

    #[test]
    fn no_skip_prefetch_real_query() {
        assert!(!skip_memory_prefetch("open the door"));
        assert!(!skip_memory_prefetch("find me a red cup"));
    }

    #[test]
    fn rtdl_response_requires_exact_top_level_keys() {
        let err = parse_rtdl_assistant_response(
            r#"{"content":"x","rtdl":{"op":"sequence","children":[]},"extra":1}"#,
        )
        .unwrap_err();
        assert!(err.to_string().contains("exactly `content` and `rtdl`"));
    }

    #[test]
    fn rtdl_expands_sequence_to_plan_calls() {
        let mut targets = CapabilityTargetMap::new();
        targets.insert(
            "camera_snapshot".to_string(),
            (
                "cap-camera".to_string(),
                "robonix/primitive/camera/snapshot".to_string(),
            ),
        );
        targets.insert(
            "chassis_move".to_string(),
            (
                "cap-chassis".to_string(),
                "robonix/primitive/chassis/move".to_string(),
            ),
        );

        let rtdl = json!({
            "op": "sequence",
            "children": [
                { "op": "do", "cap": "camera_snapshot", "args": {} },
                { "op": "do", "cap": "chassis_move", "args": { "linear": 0.1 } }
            ]
        });
        let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 7).unwrap();

        assert_eq!(plan.plan_id, "p");
        assert_eq!(plan.session_id, "s");
        assert_eq!(plan.round, 7);
        assert_eq!(plan.nodes.len(), 3);
        assert_eq!(plan.root_index, 0);
        assert_eq!(plan.nodes[0].node_kind, RTDL_SEQUENCE);
        assert_eq!(plan.nodes[0].children, vec![1, 2]);
        let first = plan.nodes[1].call.as_ref().unwrap();
        let second = plan.nodes[2].call.as_ref().unwrap();
        assert_eq!(plan.nodes[1].node_kind, RTDL_DO);
        assert_eq!(first.call_id, "p:0");
        assert_eq!(first.provider_id, "cap-camera");
        assert_eq!(first.contract_id, "robonix/primitive/camera/snapshot");
        assert_eq!(first.args_json, "{}");
        assert_eq!(second.call_id, "p:1");
        assert_eq!(second.args_json, r#"{"linear":0.1}"#);
    }

    #[test]
    fn rtdl_expands_parallel_root() {
        let mut targets = CapabilityTargetMap::new();
        targets.insert(
            "camera_snapshot".to_string(),
            (
                "cap-camera".to_string(),
                "robonix/primitive/camera/snapshot".to_string(),
            ),
        );
        targets.insert(
            "read_temp".to_string(),
            (
                "cap-temp".to_string(),
                "robonix/primitive/sensor/temp".to_string(),
            ),
        );

        let rtdl = json!({
            "op": "parallel",
            "children": [
                { "op": "do", "cap": "camera_snapshot", "args": {} },
                { "op": "do", "cap": "read_temp", "args": { "unit": "c" } }
            ]
        });
        let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 1).unwrap();

        assert_eq!(plan.root_index, 0);
        assert_eq!(plan.nodes[0].node_kind, RTDL_PARALLEL);
        assert_eq!(plan.nodes[0].children, vec![1, 2]);
        assert_eq!(plan.nodes[1].call.as_ref().unwrap().call_id, "p:0");
        assert_eq!(plan.nodes[2].call.as_ref().unwrap().call_id, "p:1");
    }

    #[test]
    fn rtdl_nested_call_ids_follow_json_traversal_order() {
        let mut targets = CapabilityTargetMap::new();
        for name in ["a", "b", "c"] {
            targets.insert(
                name.to_string(),
                (format!("provider-{name}"), format!("robonix/test/{name}")),
            );
        }

        let rtdl = json!({
            "op": "sequence",
            "children": [
                { "op": "do", "cap": "a", "args": {} },
                {
                    "op": "parallel",
                    "children": [
                        { "op": "do", "cap": "b", "args": {} },
                        { "op": "do", "cap": "c", "args": {} }
                    ]
                }
            ]
        });
        let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 1).unwrap();
        let calls: Vec<_> = plan
            .nodes
            .iter()
            .filter_map(|node| node.call.as_ref())
            .map(|call| call.call_id.as_str())
            .collect();
        assert_eq!(calls, vec!["p:0", "p:1", "p:2"]);
    }

    #[test]
    fn rtdl_empty_sequence_generates_root_node() {
        let targets = CapabilityTargetMap::new();
        let rtdl = json!({ "op": "sequence", "children": [] });
        let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0).unwrap();

        assert_eq!(plan.root_index, 0);
        assert_eq!(plan.nodes.len(), 1);
        assert_eq!(plan.nodes[0].node_kind, RTDL_SEQUENCE);
        assert!(plan.nodes[0].children.is_empty());
    }

    #[test]
    fn rtdl_rejects_out_field() {
        let mut targets = CapabilityTargetMap::new();
        targets.insert(
            "camera_snapshot".to_string(),
            (
                "cap-camera".to_string(),
                "robonix/primitive/camera/snapshot".to_string(),
            ),
        );
        let rtdl = json!({
            "op": "do",
            "cap": "camera_snapshot",
            "args": {},
            "out": { "image": "img" }
        });
        let err = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0).unwrap_err();
        assert!(err.to_string().contains("only `op`, `cap`, and `args`"));
    }

    #[test]
    fn rtdl_rejects_parallel_non_array_children() {
        let targets = CapabilityTargetMap::new();
        let rtdl = json!({ "op": "parallel", "children": {} });
        let err = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0).unwrap_err();
        assert!(err.to_string().contains("children must be an array"));
    }

    #[test]
    fn rtdl_rejects_do_non_object_args() {
        let mut targets = CapabilityTargetMap::new();
        targets.insert(
            "camera_snapshot".to_string(),
            (
                "cap-camera".to_string(),
                "robonix/primitive/camera/snapshot".to_string(),
            ),
        );
        let rtdl = json!({ "op": "do", "cap": "camera_snapshot", "args": [] });
        let err = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0).unwrap_err();
        assert!(err.to_string().contains("args must be an object"));
    }

    #[test]
    fn rtdl_rejects_unknown_op() {
        let targets = CapabilityTargetMap::new();
        let rtdl = json!({ "op": "race", "children": [] });
        let err = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0).unwrap_err();
        assert!(err.to_string().contains("unknown operator"));
    }
}
