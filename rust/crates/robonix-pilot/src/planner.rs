// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
use crate::discovery::{self, llm_name};
use crate::history;
use crate::memory;
use crate::pb::contracts::system_executor_client::SystemExecutorClient;
use crate::pb::pilot::{
    BatchResult, CapabilityCall, CapabilityCallResult, PilotEvent, Plan, SessionStatusEvent, Task,
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
    pub graph: SystemExecutorClient<Channel>,
}

type CapabilityTarget = (String, String);
type CapabilityTargetMap = HashMap<String, CapabilityTarget>;

struct DisplayCapability<'a> {
    display_name: String,
    cap_id: &'a str,
    iface: &'a atlas_pb::InterfaceMetadata,
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
    // in InterfaceMetadata.params, no Connect needed.
    let _ = consumer_id; // currently unused; kept on the signature for future channel-tracked discovery
    let initial_caps = discovery::discover(atlas)
        .await
        .map_err(|e| anyhow::anyhow!("atlas capability discovery failed: {e}"))?;
    // Pilot binds to the canonical contract_id, not the LLM-facing tool
    // name: the latter is just the contract_id leaf and a provider could
    // rename it freely. contract_id is the stable identity.
    let search_memory_target = initial_caps
        .iter()
        .find(|(_, iface)| iface.contract_id == "robonix/system/memory/search")
        .map(|(cap_id, iface)| (cap_id.clone(), iface.contract_id.clone()));

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

    // 1c. Append the per-capability docs index. Each cap that registered
    // a `capability_md_path` shows up here as a one-liner pointing at its
    // CAPABILITY.md; the LLM is instructed to lazy-load those via the
    // `read_file` builtin when it actually needs that cap. This keeps the
    // system prompt tiny while still giving the LLM full per-cap context
    // when relevant. Errors here are non-fatal — caps that didn't register
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
                d.cap_id, d.namespace, d.md_path
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

        // Re-discover capabilities from atlas every round so caps that
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
            graph.calls.len()
        );

        if graph.calls.is_empty() {
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
            graph.calls.len()
        );
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
    cap_list: &[(String, atlas_pb::InterfaceMetadata)],
) -> Vec<DisplayCapability<'_>> {
    cap_list
        .iter()
        .map(|(cap_id, iface)| DisplayCapability {
            display_name: format!("{}.{}", cap_id, llm_name(&iface.contract_id)),
            cap_id,
            iface,
        })
        .collect()
}

fn build_capability_target_map(display_caps: &[DisplayCapability<'_>]) -> CapabilityTargetMap {
    let mut out = HashMap::new();
    for cap in display_caps {
        out.insert(
            cap.display_name.clone(),
            (cap.cap_id.to_string(), cap.iface.contract_id.clone()),
        );
    }
    out
}

fn build_rtdl_prompt(display_caps: &[DisplayCapability<'_>]) -> Result<String> {
    let mut p = String::from(
        "\
## RTDL output protocol

Return a valid JSON object with exactly these top-level keys:
- `content`: a string visible to the user.
- `rtdl`: a Robot Task Description Language JSON AST.

The whole assistant message MUST begin with `{` and end with `}`.
Do not output markdown fences, headings, explanations, or any text outside the JSON object.

RTDL nodes are JSON objects with an `op` string. MVP supports only:
- `sequence`: fields `op` and `children`; `children` is an array of RTDL nodes executed in order.
- `do`: fields `op`, `cap`, and `args`.
  - `cap` MUST be copied exactly from the `capability_name` field of one Available capabilities entry.
  - `args` MUST be a JSON object whose keys and value shapes come from that capability's `args_schema`.

Rules:
1. Use ONLY capabilities listed in the Available capabilities section.
2. In RTDL `do.cap`, use ONLY the listed `capability_name` value. Do NOT use path fragments with `/`, hidden provider ids, contract ids, or natural-language aliases.
3. Build RTDL `do.args` from the listed `args_schema`. Do NOT invent argument keys.
4. Do NOT invent new capabilities, robots, objects, locations, or relations.
5. The value of `rtdl` MUST be a JSON object, not a string.
6. Do not output `out`, variables, expressions, or any operator other than `sequence` and `do`.
7. If no capability call is needed, output an empty sequence: {\"op\":\"sequence\",\"children\":[]}.

Example JSON format:
{
  \"content\": \"I will inspect the current scene.\",
  \"rtdl\": {
    \"op\": \"sequence\",
    \"children\": [
      {
        \"op\": \"do\",
        \"cap\": \"camera_snapshot\",
        \"args\": {}
      }
    ]
  }
}

## Available capabilities

",
    );

    for cap in display_caps {
        let iface = cap.iface;
        let Some(atlas_pb::transport_params::Kind::Mcp(mcp)) =
            iface.params.as_ref().and_then(|p| p.kind.as_ref())
        else {
            continue;
        };
        let schema: serde_json::Value =
            serde_json::from_str(&mcp.input_schema_json).unwrap_or(serde_json::Value::Null);
        p.push_str(&format!(
            "- capability_name: `{}`\n  - description: {}\n  - args_schema: `{}`\n",
            cap.display_name,
            mcp.description.trim(),
            schema
        ));
    }

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
    let mut calls = Vec::new();
    let mut next_call = 0usize;
    expand_rtdl_node(rtdl, "$", target_map, &plan_id, &mut next_call, &mut calls)?;
    Ok(Plan {
        plan_id,
        session_id,
        round,
        calls,
    })
}

fn expand_rtdl_node(
    node: &serde_json::Value,
    path: &str,
    target_map: &CapabilityTargetMap,
    plan_id: &str,
    next_call: &mut usize,
    calls: &mut Vec<CapabilityCall>,
) -> Result<()> {
    let obj = node
        .as_object()
        .ok_or_else(|| anyhow::anyhow!("{path}: RTDL node must be an object"))?;
    let op = obj
        .get("op")
        .and_then(|x| x.as_str())
        .ok_or_else(|| anyhow::anyhow!("{path}.op must be a string"))?;

    match op {
        "sequence" => {
            if obj.len() != 2 || !obj.contains_key("children") {
                anyhow::bail!("{path}: sequence node must contain only `op` and `children`");
            }
            let children = obj
                .get("children")
                .and_then(|x| x.as_array())
                .ok_or_else(|| anyhow::anyhow!("{path}.children must be an array"))?;
            for (idx, child) in children.iter().enumerate() {
                expand_rtdl_node(
                    child,
                    &format!("{path}.children[{idx}]"),
                    target_map,
                    plan_id,
                    next_call,
                    calls,
                )?;
            }
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
            let (cap_id, contract_id) = target_map
                .get(cap)
                .cloned()
                .ok_or_else(|| anyhow::anyhow!("{path}.cap unknown capability `{cap}`"))?;
            let call_index = *next_call;
            *next_call += 1;
            calls.push(CapabilityCall {
                call_id: format!("{plan_id}:{call_index}"),
                cap_id,
                contract_id,
                args_json: serde_json::to_string(args)?,
            });
        }
        other => anyhow::bail!("{path}.op unknown operator `{other}`"),
    }

    Ok(())
}

fn rtdl_result_to_messages(r: &CapabilityCallResult) -> history::ToolResultHistory {
    if !r.success {
        return history::ToolResultHistory {
            tool_messages: vec![Message::user(&format!(
                "Executor feedback for the current task (not a new user request): capability call '{}' ({}) failed: {}",
                r.call_id, r.contract_id, r.error
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
                "Executor feedback for the current task (not a new user request): capability call '{}' ({}) succeeded: {}",
                r.call_id, r.contract_id, content
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
// then `~/.robonix/SOUL.md`. Capability docs are surfaced in the RTDL prompt
// with descriptions sourced from each cap's CAPABILITY.md.

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
        CapabilityTargetMap, expand_rtdl_to_plan,
        parse_rtdl_assistant_response, skip_memory_prefetch, task_is_session_end,
    };
    use crate::pb::pilot::Task;
    use robonix_atlas::pb as atlas_pb;
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
        assert!(!skip_memory_prefetch("帮我找个红色杯子"));
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
        assert_eq!(plan.calls.len(), 2);
        assert_eq!(plan.calls[0].call_id, "p:0");
        assert_eq!(plan.calls[0].cap_id, "cap-camera");
        assert_eq!(
            plan.calls[0].contract_id,
            "robonix/primitive/camera/snapshot"
        );
        assert_eq!(plan.calls[0].args_json, "{}");
        assert_eq!(plan.calls[1].call_id, "p:1");
        assert_eq!(plan.calls[1].args_json, r#"{"linear":0.1}"#);
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

    fn iface(contract_id: &str) -> atlas_pb::InterfaceMetadata {
        atlas_pb::InterfaceMetadata {
            contract_id: contract_id.to_string(),
            transport: atlas_pb::Transport::Mcp as i32,
            params: Some(atlas_pb::TransportParams {
                kind: Some(atlas_pb::transport_params::Kind::Mcp(atlas_pb::McpParams {
                    description: String::new(),
                    input_schema_json: "{}".to_string(),
                })),
            }),
        }
    }
}
