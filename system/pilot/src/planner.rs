// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
use crate::discovery::{self, llm_name};
use crate::history;
use crate::memory;
use crate::pb::contracts::robonix_system_executor_execute_client::RobonixSystemExecutorExecuteClient;
use crate::pb::pilot::{
    BatchResult, CapabilityCall, CapabilityCallResult, PilotEvent, PilotNodeState, Plan, RtdlNode,
    SessionStatusEvent, Task, TaskStateEvent,
};
use crate::service::{self, PilotStreamBody, SessionState};
use crate::vlm::{Message, VlmClient, VlmStreamItem};
use anyhow::{Context, Result};
use futures_util::StreamExt;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{debug, info, warn};
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use tokio::sync::{mpsc, watch};
use tonic::Request;
use tonic::transport::Channel;

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
const EXECUTOR_EVT_PLAN_COMPLETE: u32 = 2;
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

/// The LLM's persistent overall task for the turn, parsed from the
/// `task_update` field of the RTDL envelope. `null` in the envelope means
/// "keep the current task unchanged" and is represented as `None` at the
/// call site, not as a `TaskState`.
#[derive(Clone, Debug, PartialEq, Eq)]
struct TaskState {
    goal: String,
    success_criterion: String,
    status: String,
}

impl TaskState {
    /// Whether the LLM has declared the overall task complete. This is the
    /// authoritative completion signal — an empty RTDL tree alone does not end
    /// the turn.
    fn is_done(&self) -> bool {
        self.status == "done"
    }

    /// Render the current task as a system-prompt block so the LLM always sees
    /// its own standing goal and success criterion.
    fn prompt_block(&self) -> String {
        format!(
            "\n\n## Current overall task\n- goal: {}\n- success_criterion: {}\n- status: {}\n",
            self.goal, self.success_criterion, self.status
        )
    }
}

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

/// Metadata for one in-flight RTDL tree in the forest. Trees are keyed by
/// pilot-assigned `plan_id`; this carries what the supervisor and the LLM need
/// to reason about a running tree (and, later, what the chat UI renders).
struct TreeMeta {
    /// LLM-supplied `rtdl_description` (sub-task label).
    description: String,
    /// True when this tree is purely control ops (only `cancel_plan` /
    /// `cancel_all_plans`). Such trees are NOT advertised to the LLM as
    /// cancellable in-flight work — a cancel is not itself a task tree, and
    /// listing it makes the model cancel its own cancels in a loop.
    control_only: bool,
}

/// Events fed from per-tree driver tasks back to the supervisor loop. One
/// `drive_plan` task runs per dispatched tree and streams these.
enum ForestEvent {
    /// A node changed state — forwarded for live visualisation. Carries the
    /// originating tree's `plan_id`. The state is boxed because it is much
    /// larger than the other variant's payload.
    NodeState {
        plan_id: String,
        node_state: Box<crate::pb::executor::RtdlNodeState>,
    },
    /// A tree finished (or its Execute stream ended/errored). Carries the
    /// terminal capability results collected from the tree.
    PlanDone {
        plan_id: String,
        results: Vec<CapabilityCallResult>,
        any_failed: bool,
    },
}

/// Drive one dispatched plan's Execute stream to completion, forwarding node
/// states for visualisation and collecting terminal results. Sends exactly one
/// `PlanDone` when the stream ends. Runs as its own task so the supervisor loop
/// never blocks on a single tree — concurrent trees form the forest.
async fn drive_plan(
    plan: Plan,
    mut client: RobonixSystemExecutorExecuteClient<Channel>,
    events_tx: mpsc::Sender<ForestEvent>,
) {
    let plan_id = plan.plan_id.clone();
    let submitted = plan.clone();
    let mut stream = match client.execute(Request::new(plan)).await {
        Ok(resp) => resp.into_inner(),
        Err(e) => {
            warn!("[pilot/forest] plan_id={plan_id} Execute RPC failed: {e}");
            let _ = events_tx
                .send(ForestEvent::PlanDone {
                    plan_id,
                    results: Vec::new(),
                    any_failed: true,
                })
                .await;
            return;
        }
    };

    let mut results: Vec<CapabilityCallResult> = Vec::new();
    let mut any_failed = false;
    loop {
        match stream.message().await {
            Ok(Some(event)) => {
                if event.event_kind == EXECUTOR_EVT_PLAN_COMPLETE
                    && let Some(pc) = event.plan_complete
                {
                    any_failed |= pc.any_failed;
                    continue;
                }
                if event.event_kind == EXECUTOR_EVT_NODE_STATE
                    && let Some(ns) = event.node_state
                {
                    // Forward every node state for live viz.
                    let _ = events_tx
                        .send(ForestEvent::NodeState {
                            plan_id: plan_id.clone(),
                            node_state: Box::new(ns.clone()),
                        })
                        .await;
                    if is_terminal_executor_state(ns.state) {
                        let r = executor_node_state_to_result(&submitted, ns);
                        if !r.success {
                            any_failed = true;
                        }
                        results.push(r);
                    }
                }
            }
            Ok(None) => break,
            Err(e) => {
                warn!("[pilot/forest] plan_id={plan_id} stream recv error: {e}");
                any_failed = true;
                break;
            }
        }
    }

    let _ = events_tx
        .send(ForestEvent::PlanDone {
            plan_id,
            results,
            any_failed,
        })
        .await;
}

/// Render the in-flight forest as a system-prompt block so the LLM can see what
/// is still running and reference a `plan_id` to cancel it. Empty when no tree
/// is running. Trees are ordered by numeric plan id for stable output.
/// True when every `do` node is a control builtin (`cancel_plan` /
/// `cancel_all_plans`) and there is at least one. A cancel is not itself a
/// cancellable task tree; advertising it makes the model cancel its own cancels.
fn is_control_only(plan: &Plan) -> bool {
    let mut has_do = false;
    for n in &plan.nodes {
        if n.node_kind != RTDL_DO {
            continue;
        }
        has_do = true;
        let leaf = n
            .call
            .as_ref()
            .map(|c| c.contract_id.rsplit('/').next().unwrap_or(""))
            .unwrap_or("");
        if !matches!(leaf, "cancel_plan" | "cancel_all_plans") {
            return false;
        }
    }
    has_do
}

fn build_forest_block(forest: &HashMap<String, TreeMeta>) -> String {
    // Only real task trees are cancellable in-flight work; hide pure control
    // (cancel-only) trees so the model never tries to cancel its own cancels.
    let mut entries: Vec<(&String, &TreeMeta)> =
        forest.iter().filter(|(_, m)| !m.control_only).collect();
    if entries.is_empty() {
        return String::new();
    }
    entries.sort_by_key(|(plan_id, _)| plan_id.parse::<u64>().unwrap_or(u64::MAX));
    let mut block = String::from(
        "\n\n## In-flight trees\n\
         These RTDL trees you dispatched earlier are still running concurrently. \
         To stop one, call `builtin_cancel_plan` with its exact `plan_id` below \
         (or `executor_cancel_all_plans` to stop everything at once). Cancel each \
         plan_id at most once — a cancel that returned is already stopping; do NOT \
         re-issue it. Only the plan_ids listed here are running; never cancel an id \
         not in this list. Do not reuse these ids for new trees.\n",
    );
    for (plan_id, meta) in entries {
        block.push_str(&format!(
            "- plan_id={} running: {}\n",
            plan_id, meta.description
        ));
    }
    block
}

/// Pull every queued mid-task steer into the LLM history as fresh user input.
///
/// A steer is just a `Task` the user submitted while the turn was already
/// running. Draining is non-blocking; returns whether anything was pulled so
/// the caller knows to re-plan. The model decides for itself whether the steer
/// requires cancelling an in-flight tree (via `builtin_cancel_plan`).
fn drain_steers(steer_rx: &mut mpsc::Receiver<Task>, history: &mut Vec<Message>) -> bool {
    let mut pulled = false;
    while let Ok(task) = steer_rx.try_recv() {
        let text = task.text.trim();
        if text.is_empty() {
            continue;
        }
        info!("[pilot/steer] mid-task input: {text}");
        history.push(Message::user(text));
        pulled = true;
    }
    if pulled {
        history::trim(history, MAX_HISTORY);
    }
    pulled
}

/// Approx history size (in chars; ~4 chars/token) past which we compact. Tuned
/// to keep the working window small without compacting on every short turn.
const HISTORY_COMPACT_TRIGGER_CHARS: usize = 24_000;
/// Most recent messages always kept verbatim through a compaction.
const HISTORY_KEEP_RECENT: usize = 12;

/// Claude-Code-style rolling compaction. When the running history grows past
/// `HISTORY_COMPACT_TRIGGER_CHARS`, summarize everything except the most recent
/// `HISTORY_KEEP_RECENT` messages into a single summary note (preserving goal,
/// decisions, observations, and current state) and keep the recent turns
/// verbatim. This shrinks the per-round prompt for the rest of the turn instead
/// of re-shipping the full transcript every round.
///
/// Best-effort: any VLM error leaves history untouched (the `MAX_HISTORY` trim
/// still bounds it). Self-limiting: after compaction the total drops below the
/// trigger, so it won't fire again until history regrows.
async fn compact_history(history: &mut Vec<Message>, vlm: &VlmClient) {
    let total: usize = history
        .iter()
        .map(|m| m.content.as_deref().map_or(0, str::len))
        .sum();
    if total < HISTORY_COMPACT_TRIGGER_CHARS || history.len() <= HISTORY_KEEP_RECENT + 4 {
        return;
    }

    let split = history.len() - HISTORY_KEEP_RECENT;
    let mut msgs = vec![Message::system(
        "You compact a robot agent's working memory. Summarize the conversation so far \
         into a concise but COMPLETE note that preserves: the user's goal(s) and any \
         success criteria, key decisions, important tool results / observations, the \
         current state of the task, and anything needed to keep going. Compact plain text, \
         no markdown headings. Do not invent facts.",
    )];
    msgs.extend(history::sanitize_for_vlm(&history[..split]));
    msgs.push(Message::user("Summarize the above conversation now."));

    let summary = match collect_vlm_text(vlm, &msgs).await {
        Some(s) if !s.trim().is_empty() => s,
        _ => return,
    };

    let before = history.len();
    let mut compacted = Vec::with_capacity(HISTORY_KEEP_RECENT + 1);
    compacted.push(Message::user(&format!(
        "[summary of earlier conversation — treat as established context]\n{}",
        summary.trim()
    )));
    compacted.extend_from_slice(&history[split..]);
    *history = compacted;
    info!(
        "[pilot] compacted history {before} -> {} messages (was ~{total} chars)",
        history.len()
    );
}

/// Run one non-streaming VLM completion and return the full text (drains the
/// stream). Returns `None` on any stream error.
async fn collect_vlm_text(vlm: &VlmClient, messages: &[Message]) -> Option<String> {
    let mut stream = vlm.chat_stream(messages, &[]).await.ok()?;
    let mut text = String::new();
    while let Some(item) = stream.next().await {
        if let Ok(VlmStreamItem::TextDelta(d)) = item {
            text.push_str(&d);
        }
    }
    Some(text)
}

/// Feed one finished tree's terminal results into the LLM history, mirroring
/// the per-round feedback the blocking loop used to produce.
fn feed_results_into_history(history: &mut Vec<Message>, results: &[CapabilityCallResult]) {
    let mut deferred_followups: Vec<Message> = Vec::new();
    for r in results {
        let mapped = rtdl_result_to_messages(r);
        history.extend(mapped.tool_messages);
        deferred_followups.extend(mapped.followup_messages);
    }
    history.extend(deferred_followups);
    history::trim(history, MAX_HISTORY);
}

#[allow(clippy::too_many_arguments)]
pub async fn run_turn(
    task: &Task,
    history: &mut Vec<Message>,
    vlm: &VlmClient,
    executor: &mut ExecutorConn,
    atlas: &mut AtlasClient,
    consumer_id: &str,
    tx: &mpsc::Sender<Result<PilotEvent, tonic::Status>>,
    mut cancel_rx: watch::Receiver<bool>,
    mut steer_rx: mpsc::Receiver<Task>,
    plan_seq: Arc<AtomicU64>,
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
        info!("[pilot] session_end: invoking compact_memory if available");
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
            "\n\n## Capability docs (lazy-load via `read_capability_doc`)\n\
             The providers below ship a CAPABILITY.md manual. Read one by calling \
             the `read_capability_doc` builtin with its `provider_id` (shown in \
             backticks). IMPORTANT: before the FIRST time you call a capability of \
             a provider marked `[skill]`, read that provider's CAPABILITY.md first \
             — skills have multi-step usage (e.g. start → poll status → cancel) and \
             constraints the terse description omits. For primitives/services, \
             reading is optional. Never use `read_file` and never guess a file path \
             for docs; `read_capability_doc` is the only way, and only the providers \
             listed here have one.\n\n",
        );
        for d in &docs {
            let tag = if d.namespace.contains("/skill/") || d.namespace.starts_with("skill/") {
                " `[skill]`"
            } else {
                ""
            };
            block.push_str(&format!("- `{}` ({}){}\n", d.provider_id, d.namespace, tag));
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

    // The LLM's standing overall task. `None` until the model first emits a
    // `task_update`; thereafter it persists across rounds and is replaced only
    // when the model sends a fresh `task_update`.
    let mut current_task: Option<TaskState> = None;

    // Pilot-assigned plan ids: monotonic from 1, never reused — and shared
    // across every turn of this session (the counter lives in the service's
    // per-session map), so a new user message never resets numbering. The LLM
    // never chooses a plan id (it sends -1, which pilot ignores); pilot fills
    // the real id here. See the "Plan IDs" section of rtdl_protocol.md.

    // 3. Forest supervisor loop.
    //
    // Each dispatched RTDL tree runs in its own `drive_plan` task; the loop
    // never blocks on a single tree, so trees dispatched across rounds run
    // concurrently — the forest. The loop wakes when a planning round is due
    // (`should_plan`), a running tree emits an event, or a cancel arrives. It
    // re-plans when a tree finishes; it ends only when the overall task is
    // `done` (or was never set, i.e. chit-chat) AND no tree is still running.
    let (forest_tx, mut forest_rx) = mpsc::channel::<ForestEvent>(256);
    let mut forest: HashMap<String, TreeMeta> = HashMap::new();
    let mut should_plan = true;
    // Last user-facing narration; surfaced as FinalText when the turn ends.
    let mut last_content = String::new();

    loop {
        // Check for hard interrupt at the top of every iteration.
        if *cancel_rx.borrow() {
            return_interrupted!();
        }

        if !should_plan {
            // No planning due. Either wait for a running tree, or end the turn.
            let task_done = current_task
                .as_ref()
                .map(TaskState::is_done)
                .unwrap_or(false);
            if forest.is_empty() {
                if task_done || current_task.is_none() {
                    let _ = tx
                        .send(Ok(service::pack(
                            &session_id,
                            PilotStreamBody::FinalText(last_content.clone()),
                        )))
                        .await;
                    break;
                }
                // Not done and nothing running: plan again to make progress.
                should_plan = true;
                continue;
            }
            // A tree is still running: block until it emits an event, a steer
            // arrives, or a cancel.
            tokio::select! {
                biased;
                _ = cancel_rx.changed() => {
                    return_interrupted!();
                }
                steer = steer_rx.recv() => {
                    if let Some(task) = steer {
                        let text = task.text.trim();
                        if !text.is_empty() {
                            info!("[pilot/steer] mid-task input: {text}");
                            history.push(Message::user(text));
                            history::trim(history, MAX_HISTORY);
                            // Re-plan now so the model can react (and decide
                            // whether to cancel any in-flight tree).
                            should_plan = true;
                        }
                    }
                }
                ev = forest_rx.recv() => {
                    match ev {
                        Some(ForestEvent::NodeState { plan_id, node_state }) => {
                            // Forward to the chat UI for the live forest highlight.
                            let _ = tx
                                .send(Ok(service::pack(
                                    &session_id,
                                    PilotStreamBody::NodeState(PilotNodeState {
                                        plan_id: plan_id.clone(),
                                        node_index: node_state.node_index,
                                        node_kind: node_state.node_kind,
                                        state: node_state.state,
                                        op_id: node_state.op_id.clone(),
                                        description: node_state.description.clone(),
                                    }),
                                )))
                                .await;
                            // Feed every node's result into context the moment it
                            // reaches a terminal state (2=SUCCEEDED 3=FAILED
                            // 4=CANCELED 5=TIMEOUT) so the model's view is always
                            // current — but do NOT re-plan mid-tree. The model
                            // decides at tree completion (PlanDone, which carries
                            // any_failed) or when the user steers; a steer-triggered
                            // re-plan then sees these partial results. Re-planning
                            // per node caused runaway loops (a wall of repeated
                            // narration; retrying a failed cancel forever). The
                            // tree-level feed in PlanDone is dropped to avoid
                            // double-feeding — every leaf result already arrives here.
                            const TERMINAL: [u32; 4] = [2, 3, 4, 5];
                            if TERMINAL.contains(&node_state.state)
                                && let Some(r) = node_state.leaf_result.as_ref()
                            {
                                feed_results_into_history(history, std::slice::from_ref(r));
                            }
                            // An error escalates to the VLM immediately rather than
                            // waiting for the whole tree to finish (PlanDone): the
                            // failure is already in context (a leaf result is fed
                            // above), so re-plan now and let the model recover or
                            // abort without blocking on still-running sibling
                            // branches. Only failures escalate — successes still
                            // batch at tree completion, which avoids the per-node
                            // re-plan storms that plain "re-plan on every node"
                            // caused.
                            if node_state.state == EXECUTOR_STATE_FAILED {
                                should_plan = true;
                            }
                            debug!(
                                "[pilot/forest] node_state plan_id={} node={} state={}",
                                plan_id,
                                node_state.node_index,
                                node_state.state
                            );
                        }
                        Some(ForestEvent::PlanDone { plan_id, results, any_failed }) => {
                            forest.remove(&plan_id);
                            // Leaf results were already fed per-node (see above);
                            // only surface the batch to the chat UI here.
                            let batch = BatchResult {
                                plan_id: plan_id.clone(),
                                session_id: session_id.clone(),
                                round,
                                results,
                                any_failed,
                            };
                            let _ = tx
                                .send(Ok(service::pack(
                                    &session_id,
                                    PilotStreamBody::BatchResult(batch),
                                )))
                                .await;
                            info!(
                                "[pilot/forest] plan_id={plan_id} done (failed={any_failed}); replanning"
                            );
                            should_plan = true;
                        }
                        None => {
                            // run_turn still holds forest_tx, so a closed channel
                            // means no producers — fall back to planning if idle.
                            should_plan = forest.is_empty();
                        }
                    }
                }
            }
            continue;
        }

        // ── Planning round ────────────────────────────────────────────────────
        should_plan = false;

        // Pull any steers that landed while we were busy (e.g. during the
        // previous VLM stream) so this round plans with the latest user input.
        drain_steers(&mut steer_rx, history);

        // Roll up old history into a summary once it gets large, so the rest of
        // the turn plans against a compact window instead of the full transcript.
        compact_history(history, vlm).await;

        // Re-discover capabilities from atlas every round so providers that
        // registered mid-turn are visible in the next call.
        let cap_list = discovery::discover(atlas)
            .await
            .map_err(|e| anyhow::anyhow!("atlas capability discovery failed: {e}"))?;

        let display_caps = build_display_capabilities(&cap_list);
        let target_map = build_capability_target_map(&display_caps);
        let rtdl_prompt = build_rtdl_prompt(&display_caps, round == 0)?;

        let task_block = current_task
            .as_ref()
            .map(TaskState::prompt_block)
            .unwrap_or_default();
        let forest_block = build_forest_block(&forest);
        // Plan with a single corrective retry (merged from dev #88): if the
        // VLM's RTDL fails to parse or expand, feed the error back and let it
        // fix the reply once; a second failure ends the turn gracefully (empty
        // recovery plan) instead of crashing the whole turn. The loop yields a
        // valid (narration, tree label, plan, id) tuple for the forest dispatch.
        let mut correction: Option<String> = None;
        let (assistant_content, rtdl_description, graph, plan_id, task_update, recovered) = loop {
            let mut messages = vec![Message::system(&format!(
                "{system_prompt}\n\n{rtdl_prompt}{task_block}{forest_block}"
            ))];
            messages.extend(history::sanitize_for_vlm(history));
            if let Some(ref correction) = correction {
                messages.push(Message::user(correction));
            }

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
                                VlmStreamItem::TextDelta(delta) => full_text.push_str(&delta),
                                VlmStreamItem::ToolCall(tc) => tool_calls.push(tc),
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
            debug!("raw_content: {}", raw_content);
            let parsed = parse_rtdl_assistant_response(&raw_content).with_context(|| {
                format!(
                    "parse RTDL assistant response: {}",
                    raw_preview(&raw_content)
                )
            });
            let RtdlEnvelope {
                content: assistant_content,
                rtdl_description,
                rtdl,
                task_update,
            } = match parsed {
                Ok(env) => env,
                Err(e) if correction.is_none() => {
                    warn!("[pilot/rtdl] parse failed round={round}, retrying once: {e:#}");
                    correction = Some(build_rtdl_retry_prompt(&e, &raw_content, &display_caps));
                    continue;
                }
                Err(e) => {
                    warn!(
                        "[pilot/rtdl] parse failed again round={round}, ending turn gracefully: {e:#}"
                    );
                    let plan_id = (plan_seq.load(Ordering::Relaxed) + 1).to_string();
                    let graph = empty_sequence_plan(plan_id.clone(), session_id.clone(), round);
                    break (
                        rtdl_recovery_final_text(),
                        String::new(),
                        graph,
                        plan_id,
                        None,
                        true,
                    );
                }
            };

            // Tentative id: committed to `plan_seq` only if this round actually
            // dispatches a tree, so empty-rtdl rounds don't burn a plan id and
            // the ids stay contiguous with the trees the user sees and cancels.
            let plan_id = (plan_seq.load(Ordering::Relaxed) + 1).to_string();
            match expand_rtdl_to_plan(
                &rtdl,
                &target_map,
                plan_id.clone(),
                session_id.clone(),
                round,
                &rtdl_description,
            )
            .context("expand RTDL to Plan")
            {
                // Carry `task_update` out so it is applied ONLY after a tree
                // expands — never on a recovery path, where it could falsely
                // mark the turn done for a plan that never ran.
                Ok(graph) => {
                    break (
                        assistant_content,
                        rtdl_description,
                        graph,
                        plan_id,
                        task_update,
                        false,
                    );
                }
                Err(e) if correction.is_none() => {
                    warn!("[pilot/rtdl] expand failed round={round}, retrying once: {e:#}");
                    correction = Some(build_rtdl_retry_prompt(&e, &raw_content, &display_caps));
                }
                Err(e) => {
                    warn!(
                        "[pilot/rtdl] expand failed again round={round}, ending turn gracefully: {e:#}"
                    );
                    let plan_id = (plan_seq.load(Ordering::Relaxed) + 1).to_string();
                    let graph = empty_sequence_plan(plan_id.clone(), session_id.clone(), round);
                    break (
                        rtdl_recovery_final_text(),
                        String::new(),
                        graph,
                        plan_id,
                        None,
                        true,
                    );
                }
            }
        };

        // RTDL recovery gave up after a retry: surface the user-facing message
        // once and END the turn. Without this the empty recovery plan would fall
        // through to "no new tree this round" and re-plan forever.
        if recovered {
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

        // Apply the task update now that we have a real expanded tree (recovery
        // breaks carry `None`). `None` keeps the standing task unchanged.
        if let Some(updated) = task_update {
            info!(
                "[pilot/rtdl] task_update goal='{}' status='{}'",
                updated.goal, updated.status
            );
            let _ = tx
                .send(Ok(service::pack(
                    &session_id,
                    PilotStreamBody::TaskState(TaskStateEvent {
                        goal: updated.goal.clone(),
                        success_criterion: updated.success_criterion.clone(),
                        status: updated.status.clone(),
                    }),
                )))
                .await;
            current_task = Some(updated);
        }

        let calls = plan_call_count(&graph);
        info!(
            "[pilot/rtdl] round={} plan_id={} calls={}",
            round, graph.plan_id, calls
        );

        // Narration enters history once; it streams to the client as a
        // TextChunk while the turn continues, or as FinalText when it ends.
        if !assistant_content.is_empty() {
            history.push(Message::assistant(&assistant_content));
            last_content = assistant_content.clone();
        }

        round += 1;
        let hit_cap = round as usize >= max_rounds;
        let task_done = current_task
            .as_ref()
            .map(TaskState::is_done)
            .unwrap_or(false);

        if calls == 0 {
            // No new tree this round. End only when nothing is running and the
            // task is done (or was never set — chit-chat). Otherwise the model
            // is waiting on the forest: keep its narration flowing and loop back
            // to the wait arm.
            if forest.is_empty() && (task_done || current_task.is_none() || hit_cap) {
                if hit_cap && !(task_done || current_task.is_none()) {
                    warn!("[pilot] hit max tool rounds ({max_rounds}), stopping turn");
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
                let _ = tx
                    .send(Ok(service::pack(
                        &session_id,
                        PilotStreamBody::TextChunk(assistant_content),
                    )))
                    .await;
            }
            if hit_cap {
                warn!("[pilot] hit max tool rounds ({max_rounds}), stopping turn");
                break;
            }
            // should_plan stays false: wait for forest events (or, when the
            // forest is empty and the task isn't done, the loop top re-plans).
            continue;
        }

        // Non-empty tree: narrate, hand the structure to the client, and dispatch
        // it to the forest without blocking. Trees added across rounds (e.g. by a
        // mid-task steer) run side by side.
        if !assistant_content.is_empty() {
            let _ = tx
                .send(Ok(service::pack(
                    &session_id,
                    PilotStreamBody::TextChunk(assistant_content),
                )))
                .await;
        }
        let _ = tx
            .send(Ok(service::pack(
                &session_id,
                PilotStreamBody::Plan(graph.clone()),
            )))
            .await;
        // Commit the plan id now that a real tree is being dispatched.
        plan_seq.fetch_add(1, Ordering::Relaxed);
        forest.insert(
            plan_id.clone(),
            TreeMeta {
                description: rtdl_description,
                control_only: is_control_only(&graph),
            },
        );
        tokio::spawn(drive_plan(graph, executor.graph.clone(), forest_tx.clone()));
        info!(
            "[pilot/forest] dispatched plan_id={plan_id}; forest size now {}",
            forest.len()
        );

        if hit_cap {
            warn!("[pilot] hit max tool rounds ({max_rounds}), stopping turn");
            break;
        }
        // should_plan stays false: wait for this tree (and any others) to report.
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

/// One-line protocol reminder sent on every round after the first. The full
/// ~9KB `rtdl_protocol.md` is sent only on round 0 (and the model's own prior
/// envelopes stay in history), so later rounds re-anchor cheaply instead of
/// re-shipping the whole spec — a large latency/token win on multi-round turns.
const RTDL_PROTOCOL_REMINDER: &str = "## RTDL output (reminder — same format as your earlier turns)\n\
Reply with exactly ONE JSON object, keys EXACTLY these four: \
`content`, `rtdl_description`, `rtdl`, `task_update`. No other top-level keys.\n\
`rtdl` is a tree; every node is EXACTLY one of:\n\
- {\"op\":\"sequence\",\"children\":[ ...nodes... ]}   (run children in order)\n\
- {\"op\":\"parallel\",\"children\":[ ...nodes... ]}   (run children concurrently)\n\
- {\"op\":\"do\",\"cap\":\"<capability_name>\",\"args\":{ ... }}   (one capability call)\n\
A capability name goes ONLY in a do node's `cap` — NEVER as an `op`. \
Do NOT add any other fields to a node (no `plan_id`, no `out`, no `id`). \
Use ONLY capability_name values from the list below; never invent names.\n\
`task_update`: null keeps the current goal, or {\"goal\",\"success_criterion\",\"status\"} with \
status \"done\" only when the success_criterion verifiably holds AND no tree in the \
\"In-flight trees\" list is still running (cancelling a tree does not make the task done — wait \
for it to leave the list first).\n\
Compose multi-step trees; don't drip one node per round. No new capability call this round = \
{\"op\":\"sequence\",\"children\":[]}.\n\
To stop a running tree, add a do node calling `builtin_cancel_plan` with the exact plan_id from \
the In-flight trees list.\n\
Example: {\"content\":\"listing\",\"rtdl_description\":\"list tmp\",\"rtdl\":{\"op\":\"sequence\",\
\"children\":[{\"op\":\"do\",\"cap\":\"list_dir\",\"args\":{\"path\":\"/tmp\"}}]},\"task_update\":null}\n";

/// Build the per-round protocol + capability catalog. `full_protocol` ships the
/// complete spec (round 0); otherwise a one-line reminder. The capability
/// catalog is always included because providers can change mid-turn.
fn build_rtdl_prompt(
    display_caps: &[DisplayCapability<'_>],
    full_protocol: bool,
) -> Result<String> {
    // Compile-time embedded; edit `rtdl_protocol.md` in this crate root.
    let mut p = if full_protocol {
        String::from(include_str!("../rtdl_protocol.md"))
    } else {
        String::from(RTDL_PROTOCOL_REMINDER)
    };
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

    // debug!("[pilot/rtdl] rtdl prompt:\n{}", p);
    Ok(p)
}

/// One parsed RTDL envelope from the VLM.
#[derive(Debug)]
struct RtdlEnvelope {
    /// User-facing narration.
    content: String,
    /// Short label for the dispatched tree (sub-task name); may be empty only
    /// when `rtdl` is an empty sequence.
    rtdl_description: String,
    /// The declarative ops tree to dispatch this round.
    rtdl: serde_json::Value,
    /// Overall-task update. `None` means "keep the current task unchanged"
    /// (envelope `task_update: null`).
    task_update: Option<TaskState>,
}

/// Parses one VLM reply in RTDL envelope form.
///
/// The model must emit a JSON **object** whose only keys are exactly
/// `content`, `rtdl_description`, `rtdl`, and `task_update`. See
/// `rtdl_protocol.md` for the field contract.
///
/// Fails if `raw` is not valid JSON, the root is not an object, the key set is
/// not exactly those four, `content` / `rtdl_description` are not strings,
/// `rtdl` is not an object, or `task_update` is neither `null` nor a valid task
/// object.
fn parse_rtdl_assistant_response(raw: &str) -> Result<RtdlEnvelope> {
    let v: serde_json::Value = serde_json::from_str(raw)?;
    let obj = v
        .as_object()
        .ok_or_else(|| anyhow::anyhow!("assistant response must be a JSON object"))?;
    const KEYS: [&str; 4] = ["content", "rtdl_description", "rtdl", "task_update"];
    if obj.len() != KEYS.len() || !KEYS.iter().all(|k| obj.contains_key(*k)) {
        anyhow::bail!(
            "assistant response must contain exactly `content`, `rtdl_description`, `rtdl`, and `task_update`"
        );
    }
    let content = obj
        .get("content")
        .and_then(|x| x.as_str())
        .ok_or_else(|| anyhow::anyhow!("assistant `content` must be a string"))?
        .to_string();
    let rtdl_description = obj
        .get("rtdl_description")
        .and_then(|x| x.as_str())
        .ok_or_else(|| anyhow::anyhow!("assistant `rtdl_description` must be a string"))?
        .to_string();
    let rtdl = obj
        .get("rtdl")
        .filter(|x| x.is_object())
        .ok_or_else(|| anyhow::anyhow!("assistant `rtdl` must be an object"))?
        .clone();
    let task_update = match obj.get("task_update") {
        None | Some(serde_json::Value::Null) => None,
        Some(v) => Some(parse_task_update(v)?),
    };
    Ok(RtdlEnvelope {
        content,
        rtdl_description,
        rtdl,
        task_update,
    })
}

/// Parse a non-null `task_update` object into a [`TaskState`].
///
/// Requires exactly `goal`, `success_criterion`, and `status` (all strings),
/// with `status` constrained to `"in_progress"` or `"done"`.
fn parse_task_update(v: &serde_json::Value) -> Result<TaskState> {
    let obj = v
        .as_object()
        .ok_or_else(|| anyhow::anyhow!("`task_update` must be null or an object"))?;
    const KEYS: [&str; 3] = ["goal", "success_criterion", "status"];
    if obj.len() != KEYS.len() || !KEYS.iter().all(|k| obj.contains_key(*k)) {
        anyhow::bail!(
            "`task_update` object must contain exactly `goal`, `success_criterion`, and `status`"
        );
    }
    let get = |key: &str| -> Result<String> {
        obj.get(key)
            .and_then(|x| x.as_str())
            .map(str::to_string)
            .ok_or_else(|| anyhow::anyhow!("`task_update.{key}` must be a string"))
    };
    let status = get("status")?;
    if status != "in_progress" && status != "done" {
        anyhow::bail!("`task_update.status` must be \"in_progress\" or \"done\"");
    }
    Ok(TaskState {
        goal: get("goal")?,
        success_criterion: get("success_criterion")?,
        status,
    })
}

fn raw_preview(raw: &str) -> String {
    if raw.is_empty() {
        return "assistant content was empty".to_string();
    }
    let preview: String = raw.chars().take(240).collect();
    let ellipsis = if raw.chars().count() > 240 { "..." } else { "" };
    format!("assistant content preview: {preview:?}{ellipsis}")
}

// ── RTDL recovery (merged from dev #88) ────────────────────────────────────────
// When the VLM emits an RTDL that fails to parse or expand, feed the error back
// once and let it self-correct; a second failure ends the turn gracefully.

/// Corrective prompt appended to the next VLM round after a parse/expand failure.
fn build_rtdl_retry_prompt(
    err: &anyhow::Error,
    raw_content: &str,
    display_caps: &[DisplayCapability<'_>],
) -> String {
    let mut p = format!(
        "Your previous RTDL response could not be parsed or expanded by Pilot.\n\
         Error: {err:#}\n\
         Previous response preview: {}\n\n\
         Fix the RTDL error and retry the same user request exactly once. If the error \
         mentions an unknown capability, do not repeat that capability. Return ONLY a JSON object \
         with exactly `content`, `rtdl_description`, `rtdl`, and `task_update`. Use only \
         capability_name values from this list; do not invent provider names, method names, or \
         aliases:\n",
        raw_preview(raw_content)
    );
    for cap in display_caps {
        p.push_str("- ");
        p.push_str(&cap.display_name);
        p.push('\n');
    }
    p.push_str(
        "\nIf no further capability call is needed, use \
         {\"op\":\"sequence\",\"children\":[]} as `rtdl`. If the user's requested action cannot \
         be performed using the listed capabilities, explain the missing capability in `content` \
         and return an empty RTDL sequence instead of inventing a capability.\n",
    );
    p
}

/// A single empty-sequence root plan, used as the no-op plan when a turn ends in
/// RTDL recovery. Carries non-empty `op_id`/`description` so executor's
/// `validate_plan` accepts it.
fn empty_sequence_plan(plan_id: String, session_id: String, round: u32) -> Plan {
    Plan {
        plan_id,
        session_id,
        round,
        nodes: vec![RtdlNode {
            node_kind: RTDL_SEQUENCE,
            children: Vec::new(),
            call: None,
            op_id: "recovery".to_string(),
            description: "recovery: no valid plan produced".to_string(),
        }],
        root_index: 0,
    }
}

/// User-facing message when RTDL recovery gives up — never leaks the internal error.
fn rtdl_recovery_final_text() -> String {
    "I couldn't produce a valid robot plan after retrying once. Please try again or rephrase the request."
        .to_string()
}

/// Process-wide monotonic source of node `op_id`s. The LLM-emitted RTDL does
/// not carry a usable op_id (it defaults to 0), so pilot assigns one itself
/// while parsing: a globally-unique, auto-incrementing id starting at 1.
/// "Global" = across every plan/round in this pilot process, so each node in
/// the live task-graph forest is uniquely addressable for steering and result
/// correlation — not merely unique within one plan.
static OP_ID_SEQ: AtomicU64 = AtomicU64::new(0);

/// Allocate the next global op_id (1, 2, 3, …) as a decimal string.
fn next_op_id() -> String {
    (OP_ID_SEQ.fetch_add(1, Ordering::Relaxed) + 1).to_string()
}

fn expand_rtdl_to_plan(
    rtdl: &serde_json::Value,
    target_map: &CapabilityTargetMap,
    plan_id: String,
    session_id: String,
    round: u32,
    root_description: &str,
) -> Result<Plan> {
    let mut nodes = Vec::new();
    let mut next_call = 0usize;
    let root_index = expand_rtdl_node(
        rtdl,
        "$",
        target_map,
        plan_id.as_str(),
        root_description,
        &mut next_call,
        &mut nodes,
    )?;
    Ok(Plan {
        plan_id,
        session_id,
        round,
        nodes,
        root_index,
    })
}

/// Pick a node's `description`. The root node uses the LLM's tree label
/// (`rtdl_description`) when it is non-empty; every other node — and an
/// unlabelled root — falls back to a synthesized description. Executor's
/// `validate_plan` requires a non-empty description on every node.
fn node_description(path: &str, root_description: &str, synthesized: String) -> String {
    if path == "$" && !root_description.is_empty() {
        root_description.to_string()
    } else {
        synthesized
    }
}

fn expand_rtdl_node(
    node: &serde_json::Value,
    path: &str,
    target_map: &CapabilityTargetMap,
    plan_id: &str,
    root_description: &str,
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
            let description = node_description(
                path,
                root_description,
                format!("{op} of {} step(s)", children.len()),
            );
            nodes.push(RtdlNode {
                node_kind,
                children: Vec::new(),
                call: None,
                op_id: next_op_id(),
                description,
            });
            let mut child_indices = Vec::with_capacity(children.len());
            for (idx, child) in children.iter().enumerate() {
                let child_index = expand_rtdl_node(
                    child,
                    &format!("{path}.children[{idx}]"),
                    target_map,
                    plan_id,
                    root_description,
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
            let description = node_description(path, root_description, format!("call {cap}"));
            nodes.push(RtdlNode {
                node_kind: RTDL_DO,
                children: Vec::new(),
                call: Some(CapabilityCall {
                    call_id: format!("{plan_id}:{call_index}"),
                    provider_id,
                    contract_id,
                    args_json: serde_json::to_string(args)?,
                }),
                op_id: next_op_id(),
                description,
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
- COMPOSE multi-step RTDL trees. When you already know several steps that don't
  depend on each other's results, put them ALL in one `sequence` (ordered) or
  `parallel` (independent) tree in a single round — that is the entire point of
  RTDL. Emitting one single-node tree per round (ReAct-style drip) is wrong
  UNLESS the next step genuinely needs to see the previous step's result.
- Do NOT claim missing capabilities unless verified from the current capability list/results.
  - If `memory_search` / `memory_save` / `memory_compact` capabilities are available,
    treat long-term memory as available via those capabilities.
- Prefer structured output; report capability results concisely.
- If a capability returns an error, diagnose and retry, or report to the user.
- Some later messages may be labelled `Executor feedback for the current task`.
  Treat those as results of capability calls you already planned, not as new
  user requests.
- If executor feedback already contains enough information to answer the
  user's request, answer in `content`, set `task_update.status` to `done`, and
  output an empty RTDL sequence. Do not repeat the same observation capability
  just to confirm unchanged data.

## Persistence (READ THIS — most common failure mode)
The turn ends only when your overall task is `done` (you set
`task_update.status: \"done\"`), every tree you dispatched has finished, and
there is no pending user input. An empty RTDL sequence alone does NOT end the
turn — it just means you are dispatching no new tree this round (for example,
while you wait for an in-flight tree). Do NOT mark the task `done` until it is
*verifiably* complete. Concretely:

- Set a concrete `task_update.success_criterion` as soon as you understand the
  goal (e.g. for 'turn around': yaw delta ≈ 180° from the starting pose; for
  'find the door': a door is visible in a camera observation).
- For pure observation or visual question-answering tasks, one successful
  observation is usually enough. After answering from that observation, mark
  `status: \"done\"` with an empty RTDL sequence.
- For tasks that change robot or world state, batch the steps you can already
  foresee into one tree, then verify at meaningful checkpoints — not after
  literally every action. Re-observe and re-plan when the NEXT step depends on
  what you'd see (e.g. you must confirm an object moved before grasping it), not
  as a reflex after each call.
- A single short chassis movement burst typically rotates ~0.4–0.8 rad
  (≈ 25–45°) or translates ~0.1–0.2 m. To turn 180° you need MULTIPLE
  bursts; do not assume one call finishes the rotation.
- Only mark `status: \"done\"` once the criterion is met OR you've exhausted
  reasonable attempts and need to report a blocker. 'Done.' with no
  verification is wrong — verify first.
- On the very rare case where the user explicitly cancels, you may stop
  early; otherwise keep going.
",
    );
    p
}

#[cfg(test)]
mod tests {
    use super::{
        CapabilityTargetMap, RTDL_DO, RTDL_PARALLEL, RTDL_SEQUENCE, TaskState, expand_rtdl_to_plan,
        parse_rtdl_assistant_response, parse_task_update, rtdl_recovery_final_text,
        skip_memory_prefetch, task_is_session_end,
    };
    use crate::pb::pilot::Task;
    use serde_json::json;

    #[test]
    fn rtdl_recovery_final_text_hides_internal_error() {
        let text = rtdl_recovery_final_text();
        assert!(text.contains("valid robot plan"));
        assert!(!text.contains("expand RTDL"));
        assert!(!text.contains("capability call"));
        assert!(!text.contains("assistant content preview"));
    }

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
        // Old two-key envelope is now rejected.
        let err = parse_rtdl_assistant_response(
            r#"{"content":"x","rtdl":{"op":"sequence","children":[]}}"#,
        )
        .unwrap_err();
        assert!(
            err.to_string()
                .contains("exactly `content`, `rtdl_description`, `rtdl`, and `task_update`")
        );
    }

    #[test]
    fn rtdl_response_parses_full_envelope() {
        let env = parse_rtdl_assistant_response(
            r#"{
                "content":"on it",
                "rtdl_description":"fetch water",
                "rtdl":{"op":"sequence","children":[]},
                "task_update":{"goal":"bring water","success_criterion":"cup by user","status":"in_progress"}
            }"#,
        )
        .unwrap();
        assert_eq!(env.content, "on it");
        assert_eq!(env.rtdl_description, "fetch water");
        assert!(env.rtdl.is_object());
        assert_eq!(
            env.task_update,
            Some(TaskState {
                goal: "bring water".into(),
                success_criterion: "cup by user".into(),
                status: "in_progress".into(),
            })
        );
    }

    #[test]
    fn rtdl_response_task_update_null_is_none() {
        let env = parse_rtdl_assistant_response(
            r#"{"content":"x","rtdl_description":"","rtdl":{"op":"sequence","children":[]},"task_update":null}"#,
        )
        .unwrap();
        assert!(env.task_update.is_none());
    }

    #[test]
    fn task_update_rejects_unknown_status() {
        let err = parse_task_update(&json!({
            "goal":"g","success_criterion":"c","status":"paused"
        }))
        .unwrap_err();
        assert!(err.to_string().contains("status"));
    }

    #[test]
    fn task_update_rejects_missing_field() {
        let err = parse_task_update(&json!({ "goal":"g","status":"done" })).unwrap_err();
        assert!(err.to_string().contains("exactly"));
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
        let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 7, "").unwrap();

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
        let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 1, "").unwrap();

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
        let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 1, "").unwrap();
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
        let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0, "").unwrap();

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
        let err = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0, "").unwrap_err();
        assert!(err.to_string().contains("only `op`, `cap`, and `args`"));
    }

    #[test]
    fn rtdl_rejects_parallel_non_array_children() {
        let targets = CapabilityTargetMap::new();
        let rtdl = json!({ "op": "parallel", "children": {} });
        let err = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0, "").unwrap_err();
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
        let err = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0, "").unwrap_err();
        assert!(err.to_string().contains("args must be an object"));
    }

    #[test]
    fn rtdl_rejects_unknown_op() {
        let targets = CapabilityTargetMap::new();
        let rtdl = json!({ "op": "race", "children": [] });
        let err = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0, "").unwrap_err();
        assert!(err.to_string().contains("unknown operator"));
    }
}
