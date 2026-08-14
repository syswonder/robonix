// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
use crate::discovery::{self, llm_name};
use crate::history;
use crate::memory;
use crate::pb::contracts::robonix_system_executor_control_plan_client::RobonixSystemExecutorControlPlanClient;
use crate::pb::contracts::robonix_system_executor_execute_client::RobonixSystemExecutorExecuteClient;
use crate::pb::contracts::robonix_system_executor_list_active_plans_client::RobonixSystemExecutorListActivePlansClient;
use crate::pb::executor::rtdl_event::RtdlEventEnum;
use crate::pb::executor::{ControlPlanRequest, ListActivePlansRequest};
use crate::pb::pilot::rtdl_node_state::RtdlNodeStateEnum;
use crate::pb::pilot::{
    BatchResult, CapabilityCall, CapabilityCallResult, PilotEvent, Plan, RtdlNode, RtdlNodeState,
    SessionStatusEvent, Task, TaskStateEvent,
};
use crate::service::{self, PilotStreamBody, SessionState};
use crate::state_context;
use crate::vlm::{Message, VlmClient, VlmStreamItem};
use anyhow::{Context, Result};
use futures_util::StreamExt;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{debug, info, warn};
use std::collections::hash_map::DefaultHasher;
use std::collections::{HashMap, HashSet};
use std::hash::{Hash, Hasher};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;
use tokio::sync::{mpsc, watch};
use tonic::Request;
use tonic::transport::Channel;
use uuid::Uuid;

/// gRPC client for executor's plan-dispatch contract. Pilot only ever calls
/// `Execute(Plan)` — discovery happens directly against atlas now.
pub struct ExecutorConn {
    pub graph: RobonixSystemExecutorExecuteClient<Channel>,
    pub control: RobonixSystemExecutorControlPlanClient<Channel>,
    pub active: RobonixSystemExecutorListActivePlansClient<Channel>,
}

type CapabilityTarget = (String, String);
type CapabilityTargetMap = HashMap<String, CapabilityTarget>;

const RTDL_SEQUENCE: u32 = 0;
const RTDL_PARALLEL: u32 = 1;
const RTDL_DO: u32 = 2;

struct DisplayCapability<'a> {
    display_name: String,
    provider_id: &'a str,
    cap: &'a atlas_pb::Capability,
}

#[derive(Default)]
struct CapabilityPromptCache {
    fingerprint: u64,
    catalog: String,
    initialized: bool,
}

struct PromptSection<'a> {
    name: &'static str,
    content: &'a str,
}

impl CapabilityPromptCache {
    /// Reuse the rendered catalog while Atlas reports the same provider,
    /// contract, description, and input-schema data. Discovery still runs on
    /// every round, so a registration change invalidates the cache immediately.
    fn render<'a>(&'a mut self, caps: &[DisplayCapability<'_>]) -> (&'a str, bool) {
        let fingerprint = capability_prompt_fingerprint(caps);
        let hit = self.initialized && self.fingerprint == fingerprint;
        if !hit {
            self.catalog = render_capability_prompt(caps);
            self.fingerprint = fingerprint;
            self.initialized = true;
        }
        (&self.catalog, hit)
    }
}

fn estimated_text_tokens(bytes: usize) -> usize {
    bytes.div_ceil(4)
}

/// Assemble one request while emitting a bounded, machine-readable breakdown
/// of every prompt section. Section token counts are explicit four-byte
/// estimates; the provider-reported total is logged separately when available.
fn assemble_planning_messages(
    round: u32,
    capability_cache_hit: bool,
    sections: &[PromptSection<'_>],
    history_messages: &[Message],
    correction: Option<&str>,
) -> Vec<Message> {
    let system_bytes = sections.iter().map(|section| section.content.len()).sum();
    let mut system = String::with_capacity(system_bytes);
    for section in sections {
        system.push_str(section.content);
    }
    let sanitized_history = history::sanitize_for_vlm(history_messages);
    let history_bytes: usize = sanitized_history
        .iter()
        .map(|message| message.content.as_deref().map_or(0, str::len))
        .sum();
    let correction_bytes = correction.map_or(0, str::len);
    let prompt_bytes = system.len() + history_bytes + correction_bytes;
    let mut section_metrics = sections
        .iter()
        .map(|section| {
            serde_json::json!({
                "name": section.name,
                "bytes": section.content.len(),
                "estimated_tokens": estimated_text_tokens(section.content.len()),
            })
        })
        .collect::<Vec<_>>();
    section_metrics.push(serde_json::json!({
        "name": "history",
        "bytes": history_bytes,
        "estimated_tokens": estimated_text_tokens(history_bytes),
    }));
    section_metrics.push(serde_json::json!({
        "name": "correction",
        "bytes": correction_bytes,
        "estimated_tokens": estimated_text_tokens(correction_bytes),
    }));
    info!(
        "[pilot/prompt] {}",
        serde_json::json!({
            "round": round,
            "prompt_text_bytes": prompt_bytes,
            "estimated_input_tokens": estimated_text_tokens(prompt_bytes),
            "history_bytes": history_bytes,
            "correction_bytes": correction_bytes,
            "capability_catalog_render_cache_hit": capability_cache_hit,
            "sections": section_metrics,
        })
    );

    let mut messages = Vec::with_capacity(sanitized_history.len() + 2);
    messages.push(Message::system(&system));
    messages.extend(sanitized_history);
    if let Some(correction) = correction {
        messages.push(Message::user(correction));
    }
    messages
}

fn max_tool_rounds() -> usize {
    std::env::var("ROBONIX_PILOT_MAX_TOOL_ROUNDS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(64)
}

fn vlm_idle_timeout() -> Duration {
    configured_vlm_idle_timeout(
        std::env::var("ROBONIX_PILOT_VLM_IDLE_TIMEOUT_SECS")
            .ok()
            .as_deref(),
    )
}

fn configured_vlm_idle_timeout(value: Option<&str>) -> Duration {
    let seconds = value
        .and_then(|value| value.parse::<u64>().ok())
        .unwrap_or(30)
        .clamp(5, 300);
    Duration::from_secs(seconds)
}

const MAX_HISTORY: usize = 200;

/// Harness-owned state for the latest user interaction. Long-running work is
/// represented independently by the RTDL forest; it must not keep older user
/// text welded into the current goal forever.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct TaskState {
    goal: String,
    success_criterion: String,
    status: String,
}

const DEFAULT_SUCCESS_CRITERION: &str =
    "The user's request is completed and the result has been verified.";

impl TaskState {
    /// Whether the LLM has declared the overall task complete. This is the
    /// authoritative completion signal — an empty RTDL tree alone does not end
    /// the turn.
    fn is_done(&self) -> bool {
        self.status == "done"
    }

    /// Render the latest interaction separately from the in-flight forest.
    fn prompt_block(&self) -> String {
        format!(
            "\n\n## Current user interaction\n- instruction: {}\n\
             - success_criterion: {}\n- status: {}\n",
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
    /// Canonical provider/contract/args signatures for calls in this tree.
    /// The harness rejects a second tree containing an identical call while
    /// the first is still in flight; execution latency must not duplicate a
    /// physical or external command.
    call_signatures: HashSet<String>,
    /// Ordered executable leaves from the original RTDL graph. Keeping these
    /// visible lets the model target any semantic boundary in one control call
    /// instead of querying live state first or guessing what "current" means.
    steps: Vec<TreeStep>,
}

struct TreeStep {
    op_id: String,
    description: String,
    capability: String,
}

/// Events fed from per-tree driver tasks back to the supervisor loop. One
/// `drive_plan` task runs per dispatched tree and streams these.
enum ForestEvent {
    /// A node changed state — forwarded for live visualisation. Carries the
    /// originating tree's `plan_id`. The state is boxed because it is much
    /// larger than the other variant's payload.
    NodeState {
        plan_id: String,
        node_state: Box<RtdlNodeState>,
    },
    /// A tree finished (or its Execute stream ended/errored). Carries one
    /// full `RtdlNodeState` record for every node that reached a terminal
    /// state (leaf and non-leaf), collected from the tree.
    PlanDone {
        plan_id: String,
        results: Vec<RtdlNodeState>,
        any_failed: bool,
        /// True when this tree ended because it was canceled (a node reached
        /// CANCELED), as opposed to running to natural success/failure. A
        /// cancellation fulfils a prior decision and carries no new info, so the
        /// supervisor must NOT trigger a fresh planning round for it — otherwise
        /// "cancel old plan → PlanDone → replan → model re-cancels" becomes a
        /// self-sustaining storm with monotonically growing plan ids.
        canceled: bool,
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
    forest_revision: Arc<AtomicU64>,
) {
    let plan_id = plan.plan_id.clone();
    let mut stream = match client.execute(Request::new(plan)).await {
        Ok(resp) => resp.into_inner(),
        Err(e) => {
            warn!("[pilot/forest] plan_id={plan_id} Execute RPC failed: {e}");
            forest_revision.fetch_add(1, Ordering::Release);
            let _ = events_tx
                .send(ForestEvent::PlanDone {
                    plan_id,
                    results: Vec::new(),
                    any_failed: true,
                    canceled: false,
                })
                .await;
            return;
        }
    };

    let mut results: Vec<RtdlNodeState> = Vec::new();
    let mut any_failed = false;
    let mut canceled = false;
    loop {
        match stream.message().await {
            Ok(Some(event)) => {
                if event.event_kind == RtdlEventEnum::PlanComplete as u32
                    && let Some(pc) = event.plan_complete
                {
                    any_failed |= pc.any_failed;
                    continue;
                }
                if event.event_kind == RtdlEventEnum::NodeState as u32
                    && let Some(ns) = event.node_state
                {
                    // Forward every node state for live viz.
                    let _ = events_tx
                        .send(ForestEvent::NodeState {
                            plan_id: plan_id.clone(),
                            node_state: Box::new(ns.clone()),
                        })
                        .await;
                    // Collect the full RtdlNodeState for every node that reaches
                    // a terminal state (leaf and non-leaf). A non-success
                    // terminal state marks the round as failed.
                    if is_terminal_executor_state(ns.state) {
                        forest_revision.fetch_add(1, Ordering::Release);
                        if ns.state == RtdlNodeStateEnum::Canceled as u32 {
                            // Cancellation is not a failure to recover from — it
                            // is the model's own stop request taking effect. Flag
                            // it so the supervisor suppresses the post-cancel
                            // replan that would otherwise feed a cancel storm.
                            canceled = true;
                        } else if ns.state != RtdlNodeStateEnum::Succeeded as u32 {
                            any_failed = true;
                        }
                        results.push(ns);
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

    forest_revision.fetch_add(1, Ordering::Release);
    let _ = events_tx
        .send(ForestEvent::PlanDone {
            plan_id,
            results,
            any_failed,
            canceled,
        })
        .await;
}

/// Cancel every real task tree owned by this turn before reporting the Pilot
/// session interrupted. Dropping the Execute stream alone only detaches Pilot;
/// Executor continues the plan (and synchronous tools such as run_command)
/// unless its PlanRuntime receives an explicit cancel_plan request.
async fn cancel_forest_plans(
    executor: &mut ExecutorConn,
    forest: &HashMap<String, TreeMeta>,
    _session_id: &str,
) {
    let targets: Vec<String> = forest
        .iter()
        .filter(|(_, meta)| !meta.control_only)
        .map(|(plan_id, _)| plan_id.clone())
        .collect();
    for target in targets {
        let cancel = executor
            .control
            .control_plan(Request::new(ControlPlanRequest {
                action: "cancel".to_string(),
                plan_id: target.clone(),
                op_id: String::new(),
                when: String::new(),
                wait_ms: 5_000,
            }));
        match tokio::time::timeout(std::time::Duration::from_secs(7), cancel).await {
            Ok(Ok(response)) => {
                let response = response.into_inner();
                if response.success {
                    info!("[pilot] canceled executor plan {target} on abort_turn");
                } else {
                    warn!(
                        "[pilot] cancel executor plan {target} rejected: {}",
                        response.error
                    );
                }
            }
            Ok(Err(error)) => {
                warn!("[pilot] cancel executor plan {target} failed: {error}")
            }
            Err(_) => warn!("[pilot] cancel executor plan {target} timed out"),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum MetaPlanOp {
    Cancel {
        plan_id: String,
        wait_ms: u64,
    },
    CancelAll {
        wait_ms: u64,
    },
    StopAt {
        plan_id: String,
        op_id: String,
        when: String,
    },
}

impl MetaPlanOp {
    fn cancellation_targets(&self, forest: &HashMap<String, TreeMeta>) -> Vec<String> {
        match self {
            Self::Cancel { plan_id, .. } => vec![plan_id.clone()],
            Self::CancelAll { .. } => forest
                .iter()
                .filter(|(_, meta)| !meta.control_only)
                .map(|(plan_id, _)| plan_id.clone())
                .collect::<Vec<_>>(),
            Self::StopAt { plan_id, .. } => vec![plan_id.clone()],
        }
    }
}

fn parse_meta_plan_op(rtdl: &serde_json::Value) -> Result<Option<MetaPlanOp>> {
    let Some(obj) = rtdl.as_object() else {
        return Ok(None);
    };
    let Some(op) = obj.get("op").and_then(|value| value.as_str()) else {
        return Ok(None);
    };
    let string = |key: &str| -> Result<String> {
        let value = obj
            .get(key)
            .and_then(|value| value.as_str())
            .map(str::to_string)
            .ok_or_else(|| anyhow::anyhow!("meta op `{op}` requires string `{key}`"))?;
        if value.trim().is_empty() {
            anyhow::bail!("meta op `{op}` requires non-empty `{key}`");
        }
        Ok(value)
    };
    let wait_ms = || {
        obj.get("wait_ms")
            .and_then(|value| value.as_u64())
            .unwrap_or(5_000)
            .min(30_000)
    };
    let parsed = match op {
        "cancel_plan" => MetaPlanOp::Cancel {
            plan_id: string("plan_id")?,
            wait_ms: wait_ms(),
        },
        "cancel_all" => MetaPlanOp::CancelAll { wait_ms: wait_ms() },
        "stop_plan_at" => {
            let when = obj
                .get("when")
                .and_then(|value| value.as_str())
                .unwrap_or("on_complete")
                .to_string();
            if !matches!(when.as_str(), "on_enter" | "on_complete") {
                anyhow::bail!("meta op `stop_plan_at` requires when=on_enter or on_complete");
            }
            MetaPlanOp::StopAt {
                plan_id: string("plan_id")?,
                op_id: string("target_op_id")?,
                when,
            }
        }
        _ => return Ok(None),
    };
    Ok(Some(parsed))
}

async fn execute_meta_plan_op(executor: &mut ExecutorConn, op: &MetaPlanOp) -> Result<String> {
    let request = match op {
        MetaPlanOp::Cancel { plan_id, wait_ms } => ControlPlanRequest {
            action: "cancel".to_string(),
            plan_id: plan_id.clone(),
            op_id: String::new(),
            when: String::new(),
            wait_ms: *wait_ms,
        },
        MetaPlanOp::CancelAll { wait_ms } => ControlPlanRequest {
            action: "cancel_all".to_string(),
            plan_id: String::new(),
            op_id: String::new(),
            when: String::new(),
            wait_ms: *wait_ms,
        },
        MetaPlanOp::StopAt {
            plan_id,
            op_id,
            when,
        } => ControlPlanRequest {
            action: "stop_at".to_string(),
            plan_id: plan_id.clone(),
            op_id: op_id.clone(),
            when: when.clone(),
            wait_ms: 0,
        },
    };
    let timeout = Duration::from_millis(request.wait_ms.saturating_add(2_000).max(2_000));
    let response = tokio::time::timeout(
        timeout,
        executor.control.control_plan(Request::new(request)),
    )
    .await
    .context("Executor plan-control RPC timed out")??
    .into_inner();
    if !response.success {
        anyhow::bail!(response.error);
    }
    Ok(response.message)
}

/// Render the in-flight forest as a system-prompt block so the LLM can see what
/// is still running and reference a `plan_id` to cancel it. Empty when no tree
/// is running. Trees are ordered by numeric plan id for stable output.
/// True when every `do` node is a plan-control builtin and there is at least
/// one. Plan-control trees are not themselves cancellable task work; advertising
/// them makes the model inspect or cancel its own control actions.
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
        if !matches!(
            leaf,
            "cancel_plan"
                | "cancel_all_plans"
                | "get_all_plans"
                | "get_plan_status"
                | "stop_plan_at"
        ) {
            return false;
        }
    }
    has_do
}

fn plan_steps(plan: &Plan) -> Vec<TreeStep> {
    plan.nodes
        .iter()
        .filter(|node| node.node_kind == RTDL_DO)
        .filter_map(|node| {
            let call = node.call.as_ref()?;
            Some(TreeStep {
                op_id: node.op_id.clone(),
                description: node.description.clone(),
                capability: call
                    .contract_id
                    .rsplit('/')
                    .next()
                    .unwrap_or(&call.contract_id)
                    .to_string(),
            })
        })
        .collect()
}

fn build_forest_block(
    forest: &HashMap<String, TreeMeta>,
    cancel_requested: &HashSet<String>,
) -> String {
    // Only real task trees are cancellable in-flight work; hide pure control
    // (cancel-only) trees so the model never tries to cancel its own cancels.
    let mut entries: Vec<(&String, &TreeMeta)> = forest
        .iter()
        .filter(|(plan_id, meta)| !meta.control_only && !cancel_requested.contains(*plan_id))
        .collect();
    if entries.is_empty() {
        return String::new();
    }
    entries.sort_by_key(|(plan_id, _)| plan_id.parse::<u64>().unwrap_or(u64::MAX));
    let mut block = String::from(
        "\n\n## In-flight trees\n\
         These RTDL trees you dispatched earlier are still running concurrently. \
         Plan control is NOT a capability call and must never be placed inside a \
         sequence, parallel, or do node. To stop one immediately, emit a root \
         `cancel_plan` meta op with its exact `plan_id` below; to stop all work, \
         emit a root `cancel_all` meta op. Every plan's ordered executable steps \
         are listed below. To stop at a requested semantic boundary (for example \
         after step 8 or after reaching the restaurant), emit a root \
         `stop_plan_at` meta op with that `plan_id`, the chosen `target_op_id`, \
         and `when` (`on_enter` to stop \
         before that op runs, `on_complete` to stop after it finishes). Do not \
         assume the target is the currently running step, and do not query status \
         first when the requested boundary is already present in this list. Bind \
         the user's named boundary literally: `after X` means X/on_complete and \
         `before X` means X/on_enter. Never rewrite `after X` as `before` its \
         successor because those are not equivalent in branching/parallel trees. It \
         cancels the whole plan when execution reaches that op. Cancel/stop each \
         plan_id at most once — a cancel that returned is already stopping; do NOT \
         re-issue it. Do not reuse these ids for new trees. If an in-flight plan \
         is already executing the same goal, do not cancel or re-issue it; wait \
         for it to finish. This block contains trees owned by the current Pilot \
         supervisor only. The authoritative Executor snapshot below may contain \
         additional plans started by an earlier interaction. Never use this \
         local block alone to answer how many tasks are running.\n",
    );
    for (plan_id, meta) in entries {
        block.push_str(&format!(
            "- plan_id={} running: {}\n",
            plan_id, meta.description
        ));
        for (index, step) in meta.steps.iter().enumerate() {
            block.push_str(&format!(
                "  {}. op_id={} [{}] {}\n",
                index + 1,
                step.op_id,
                step.capability,
                step.description
            ));
        }
    }
    block
}

fn build_executor_active_block(plans_json: Option<&str>) -> String {
    let Some(raw) = plans_json else {
        return String::from(
            "\n\n## Executor active plans (authoritative live snapshot)\n\
             - status: unavailable\n\
             The live query failed. Never guess a task count or claim that no \
             task is running. Tell the user that current execution state could \
             not be verified.\n",
        );
    };
    let Ok(value) = serde_json::from_str::<serde_json::Value>(raw) else {
        return build_executor_active_block(None);
    };
    let Some(plans) = value.get("plans").and_then(serde_json::Value::as_array) else {
        return build_executor_active_block(None);
    };
    let normalized = serde_json::json!({
        "count": plans.len(),
        "plans": plans,
    });
    format!(
        "\n\n## Executor active plans (authoritative live snapshot)\n\
         This is the source of truth for every currently running RTDL plan, \
         including long-running skills started by earlier interactions. For \
         questions about running task count, names, state, or cancellation \
         targets, answer from this snapshot rather than conversation history or \
         the local forest. A plan remains running while listed here even when \
         its provider is internally idle or motion-gated. Never say that no task \
         is running unless count is exactly 0.\n\
         snapshot_json: {}\n",
        normalized
    )
}

async fn fetch_executor_active_block(executor: &mut ExecutorConn) -> String {
    let request = executor
        .active
        .list_active_plans(Request::new(ListActivePlansRequest::default()));
    match tokio::time::timeout(Duration::from_secs(2), request).await {
        Ok(Ok(response)) => {
            let response = response.into_inner();
            if response.success {
                build_executor_active_block(Some(&response.plans_json))
            } else {
                warn!(
                    "[pilot/state] Executor active-plan query rejected: {}",
                    response.error
                );
                build_executor_active_block(None)
            }
        }
        Ok(Err(error)) => {
            warn!("[pilot/state] Executor active-plan query failed: {error}");
            build_executor_active_block(None)
        }
        Err(_) => {
            warn!("[pilot/state] Executor active-plan query timed out");
            build_executor_active_block(None)
        }
    }
}

/// Pull every queued mid-task steer into the LLM history as fresh user input.
///
/// A steer is just a `Task` the user submitted while the turn was already
/// running. Draining is non-blocking; returns whether anything was pulled so
/// the caller knows to re-plan. The model decides for itself whether the steer
/// requires a root plan-control meta op.
fn append_steer(
    task: Task,
    history: &mut Vec<Message>,
    current_task: &mut Option<TaskState>,
) -> bool {
    let text = task.text.trim();
    if text.is_empty() {
        return false;
    }
    info!("[pilot/steer] mid-task input: {text}");
    history.push(Message::user(text));
    *current_task = Some(TaskState {
        goal: text.to_string(),
        success_criterion: DEFAULT_SUCCESS_CRITERION.to_string(),
        status: "in_progress".to_string(),
    });
    true
}

fn drain_steers(
    steer_rx: &mut mpsc::Receiver<Task>,
    history: &mut Vec<Message>,
    current_task: &mut Option<TaskState>,
) -> bool {
    let mut pulled = false;
    while let Ok(task) = steer_rx.try_recv() {
        pulled |= append_steer(task, history, current_task);
    }
    if pulled {
        history::trim(history, MAX_HISTORY);
    }
    pulled
}

fn start_or_resume_task(current_task: &mut Option<TaskState>, user_text: &str) {
    let text = user_text.trim();
    if text.is_empty() {
        return;
    }
    *current_task = Some(TaskState {
        goal: text.to_string(),
        success_criterion: DEFAULT_SUCCESS_CRITERION.to_string(),
        status: "in_progress".to_string(),
    });
}

/// Apply only progress fields from the model. The user-owned goal is immutable
/// within the standing task; steering is appended by the harness above. The
/// model may refine the default success criterion once, but cannot erase or
/// replace an established criterion. Completion is accepted only at a harness
/// safe point with no new or in-flight execution.
fn apply_task_update(
    current_task: &mut Option<TaskState>,
    update: TaskState,
    can_finish: bool,
) -> bool {
    let Some(state) = current_task.as_mut() else {
        return false;
    };
    let before = state.clone();
    if update.goal != state.goal {
        warn!(
            "[pilot/rtdl] ignoring model goal replacement {:?}; harness goal remains {:?}",
            update.goal, state.goal
        );
        // The response was sampled for an older interaction. Applying even
        // its status or success criterion can falsely complete and discard a
        // newer steer, so reject the entire stale update.
        return false;
    }
    if state.success_criterion == DEFAULT_SUCCESS_CRITERION
        && !update.success_criterion.trim().is_empty()
    {
        state.success_criterion = update.success_criterion;
    }
    state.status = if update.status == "done" && can_finish {
        "done".to_string()
    } else {
        "in_progress".to_string()
    };
    *state != before
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
    let mut stream = vlm.chat_stream(messages, &[], None).await.ok()?;
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
fn feed_results_into_history(
    history: &mut Vec<Message>,
    plan_id: &str,
    plan_description: &str,
    results: &[CapabilityCallResult],
) {
    history.push(Message::user(&format!(
        "Executor feedback scope: plan_id={plan_id}, independent RTDL tree={plan_description:?}. \
         Attribute the following results only to this tree. A failure here blocks dependent \
         steps in this tree, but does not cancel or invalidate other in-flight trees."
    )));
    let mut deferred_followups: Vec<Message> = Vec::new();
    for r in results {
        let mut bounded = r.clone();
        if !history::is_image_output(&bounded.output) {
            bounded.output = compact_tool_result(&bounded.contract_id, &bounded.output, 4096);
        }
        let mapped = rtdl_result_to_messages(&bounded);
        history.extend(mapped.tool_messages);
        deferred_followups.extend(mapped.followup_messages);
    }
    history.extend(deferred_followups);
    history::trim(history, MAX_HISTORY);
}

/// Persist the exact capability calls handed to Executor so a later planning
/// round can correlate terminal results with work it already dispatched.
///
/// RTDL is a custom planning protocol rather than an OpenAI tool call, so the
/// model's structured plan is otherwise lost when only `content` is appended to
/// chat history. That made a successful physical step look unexecuted on the
/// mandatory post-PlanDone round and allowed the same user step to be planned a
/// second time from the robot's new state.
fn record_dispatched_plan(history: &mut Vec<Message>, plan: &Plan, description: &str) {
    let calls: Vec<serde_json::Value> = plan
        .nodes
        .iter()
        .filter_map(|node| {
            let call = node.call.as_ref()?;
            let args = serde_json::from_str::<serde_json::Value>(&call.args_json)
                .unwrap_or_else(|_| serde_json::Value::String(call.args_json.clone()));
            Some(serde_json::json!({
                "call_id": call.call_id,
                "op_id": node.op_id,
                "step": node.description,
                "provider_id": call.provider_id,
                "contract_id": call.contract_id,
                "args": args,
            }))
        })
        .collect();
    if calls.is_empty() {
        return;
    }
    let record = serde_json::json!({
        "plan_id": plan.plan_id,
        "description": description,
        "calls": calls,
    });
    history.push(Message::user(&format!(
        "Pilot harness dispatch record (already sent to Executor; not a new user request): {record}"
    )));
    history::trim(history, MAX_HISTORY);
}

#[allow(clippy::too_many_arguments)]
pub async fn run_turn(
    task: &Task,
    history: &mut Vec<Message>,
    standing_task: &mut Option<TaskState>,
    vlm: &VlmClient,
    executor: &mut ExecutorConn,
    atlas: &mut AtlasClient,
    consumer_id: &str,
    tx: &mpsc::Sender<Result<PilotEvent, tonic::Status>>,
    mut cancel_rx: watch::Receiver<bool>,
    mut steer_rx: mpsc::Receiver<Task>,
    plan_seq: Arc<AtomicU64>,
    soma_prompt_block: &str,
) -> Result<()> {
    let session_id = task.session_id.clone();
    // Keep the provider's prefix-cache routing stable across planning rounds
    // without exposing the user-visible or harness-visible session identifier.
    let prompt_cache_key = Uuid::new_v4().simple().to_string();

    macro_rules! return_interrupted {
        ($forest:expr) => {{
            cancel_forest_plans(executor, $forest, &session_id).await;
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

    // 1. Build stable system-prompt sections once per turn.
    let standing_prompt = build_system_prompt(load_agent_soul().as_deref());

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
    let memory_prompt = if skip_memory_prefetch(&task.text) {
        String::new()
    } else {
        match memory::prefetch(&task.text, executor, search_memory_target).await {
            Some(mem) => format!(
                "\n\n## Relevant past memories (historical hints only)\n\n\
                 These entries may be stale or task-specific. They are not current robot state, \
                 not authorization for a physical action, and not a substitute for resolving a \
                 named room, region, object, or person through the current capabilities. In \
                 particular, a remembered grasp or observation pose is not a room navigation \
                 goal.\n\n{mem}\n\n---\n\n"
            ),
            None => String::new(),
        }
    };

    // 1c. Append the per-capability docs index. Each provider that registered
    // a `capability_md_path` shows up here as a one-liner pointing at its
    // CAPABILITY.md; the LLM is instructed to lazy-load those via the
    // `read_file` builtin when it actually needs that provider. This keeps the
    // system prompt tiny while still giving the LLM full per-provider context
    // when relevant. Errors here are non-fatal — providers that didn't register
    // a path simply don't appear in the block.
    let capability_docs_prompt = if let Ok(docs) = discovery::cap_md_index(atlas).await
        && !docs.is_empty()
    {
        let mut prompt = String::from(
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
            let tag = if d.kind == "skill" { " `[skill]`" } else { "" };
            // `provider_id` is the only token the LLM needs (it passes it to
            // `read_capability_doc`); the one-line package description from the
            // CAPABILITY.md frontmatter lets it judge relevance without reading
            // the full manual. The internal `namespace` is deliberately omitted —
            // it is routing detail the model never uses.
            prompt.push_str(&format!(
                "- `{}`{}: {}\n",
                d.provider_id, tag, d.description
            ));
        }
        prompt
    } else {
        String::new()
    };

    // Voice-mode brevity hint. Liaison stamps `context_json.modality =
    // "voice"` for every voice-path Task; in that case we ask the VLM
    // for a short reply because the user is going to *hear* it via TTS,
    // not read a Markdown wall. Threshold is intentionally tight (~30
    // Chinese chars / ~50 English words) — barge-in matters more than
    // exhaustive coverage and the user can always ask follow-ups.
    let voice_prompt = if task_modality(task).as_deref() == Some("voice") {
        "\n\n## Voice mode\n\n\
             The user is interacting via voice; this reply will be\n\
             spoken back through TTS. Keep the response short (≤ ~30\n\
             characters Chinese / ~50 words English), no markdown\n\
             lists, no headings, no code blocks, plain conversational\n\
             tone. If the answer genuinely needs structure, summarise\n\
             out loud and offer to elaborate when asked.\n"
    } else {
        ""
    };

    // 2. Add user message to history
    history.push(Message::user(&task.text));
    history::trim(history, MAX_HISTORY);
    start_or_resume_task(standing_task, &task.text);
    if let Some(state) = standing_task.as_ref() {
        let _ = tx
            .send(Ok(service::pack(
                &session_id,
                PilotStreamBody::TaskState(TaskStateEvent {
                    goal: state.goal.clone(),
                    success_criterion: state.success_criterion.clone(),
                    status: state.status.clone(),
                }),
            )))
            .await;
    }

    let max_rounds = max_tool_rounds();
    let mut round: u32 = 0;

    // Pilot-assigned plan ids come from one process-global atomic counter.
    // They are reserved only for normal RTDL trees, never for meta operations,
    // and are unique even when different sessions dispatch concurrently.

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
    let mut cancel_requested: HashSet<String> = HashSet::new();
    let forest_revision = Arc::new(AtomicU64::new(0));
    let mut should_plan = true;
    let mut capability_prompt_cache = CapabilityPromptCache::default();
    // Last user-facing narration; surfaced as FinalText when the turn ends.
    let mut last_content = String::new();

    'supervisor: loop {
        // Check for hard interrupt at the top of every iteration.
        if *cancel_rx.borrow() {
            return_interrupted!(&forest);
        }

        if !should_plan {
            // No planning due. Either wait for a running tree, or end the turn.
            let task_done = standing_task
                .as_ref()
                .map(TaskState::is_done)
                .unwrap_or(false);
            if forest.is_empty() {
                if task_done || standing_task.is_none() {
                    let _ = tx
                        .send(Ok(service::pack(
                            &session_id,
                            PilotStreamBody::FinalText(last_content.clone()),
                        )))
                        .await;
                    break;
                }
                // An in-progress task with no running tree and no planning event
                // is deliberately waiting for operator input. Replanning here
                // turns an empty "wait for instructions" response (or a completed
                // cancel-only tree) into an unbounded VLM/reply/cancel loop.
                tokio::select! {
                    biased;
                    _ = cancel_rx.changed() => {
                        return_interrupted!(&forest);
                    }
                    steer = steer_rx.recv() => {
                        match steer {
                            Some(task) => {
                                if append_steer(task, history, standing_task) {
                                    history::trim(history, MAX_HISTORY);
                                    should_plan = true;
                                }
                            }
                            None => break,
                        }
                    }
                }
                continue;
            }
            // A tree is still running: block until it emits an event, a steer
            // arrives, or a cancel.
            tokio::select! {
                biased;
                _ = cancel_rx.changed() => {
                    return_interrupted!(&forest);
                }
                steer = steer_rx.recv() => {
                    if let Some(task) = steer
                        && append_steer(task, history, standing_task)
                    {
                        history::trim(history, MAX_HISTORY);
                        // Re-plan now so the model can react (and decide
                        // whether to cancel any in-flight tree).
                        should_plan = true;
                    }
                }
                ev = forest_rx.recv() => {
                    match ev {
                        Some(ForestEvent::NodeState { plan_id, node_state }) => {
                            let mut ns = *node_state;
                            // Carry the originating tree's id (the executor sets
                            // this too, but be explicit so the live view always
                            // correlates with the Plan already sent).
                            ns.plan_id = plan_id.clone();
                            // Feed every node's result into context the moment it
                            // reaches a terminal state, using names rather than
                            // numeric RTDL state codes in logs. Successful nodes wait
                            // for PlanDone before replanning; non-success terminal
                            // nodes replan immediately below. The tree-level feed in
                            // PlanDone is dropped to avoid double-feeding — every
                            // leaf result already arrives here.
                            const TERMINAL: [u32; 4] = [2, 3, 4, 5];
                            if TERMINAL.contains(&ns.state)
                                && let Some(r) = ns.leaf_result.as_ref()
                            {
                                let description = forest
                                    .get(&plan_id)
                                    .map(|meta| meta.description.as_str())
                                    .unwrap_or("unknown tree");
                                feed_results_into_history(
                                    history,
                                    &plan_id,
                                    description,
                                    std::slice::from_ref(r),
                                );
                            }
                            // Any non-success terminal outcome escalates to the VLM
                            // immediately rather than waiting for the whole tree to
                            // finish (PlanDone): the result is already in context
                            // above, so re-plan now and let the model recover or
                            // abort without blocking on still-running sibling
                            // branches. Successes still batch at tree completion,
                            // which avoids the per-node re-plan storms that plain
                            // "re-plan on every node" caused.
                            if is_terminal_executor_state(ns.state)
                                && ns.state != RtdlNodeStateEnum::Succeeded as u32
                                && !cancel_requested.contains(&plan_id)
                                && standing_task.as_ref().is_some_and(|state| !state.is_done())
                            {
                                should_plan = true;
                            }
                            log_node_state(&plan_id, &ns);
                            // Forward to the chat UI for the live forest highlight.
                            // Moving `ns` last avoids cloning its (possibly large)
                            // leaf_result on every node tick.
                            let _ = tx
                                .send(Ok(service::pack(
                                    &session_id,
                                    PilotStreamBody::NodeState(ns),
                                )))
                                .await;
                        }
                        Some(ForestEvent::PlanDone { plan_id, results, any_failed, canceled }) => {
                            forest.remove(&plan_id);
                            let requested_cancellation = cancel_requested.remove(&plan_id);
                            if requested_cancellation {
                                history.push(Message::user(&format!(
                                    "Pilot harness event: the requested cancellation of RTDL plan \
                                     {plan_id} is complete. Do not query or cancel that plan again. \
                                     Unrelated in-flight trees remain independent. If the current \
                                     interaction requested only this stop and has no successor action, \
                                     mark it done and report the completed stop now."
                                )));
                                history::trim(history, MAX_HISTORY);
                            }
                            // Leaf results were already fed per-node (see above);
                            // only surface the batch to the chat UI here.
                            log_plan_complete(&plan_id, &results, any_failed);
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
                            // Re-plan after natural completion or exactly once
                            // when a cancellation explicitly requested by this
                            // supervisor is fulfilled. Unsolicited canceled
                            // events stay quiet, preventing the old self-feeding
                            // cancel storm across unrelated sibling trees.
                            if should_replan_after_plan_done(
                                canceled,
                                requested_cancellation,
                                cancel_requested.is_empty(),
                                standing_task.as_ref().is_some_and(|state| !state.is_done()),
                            ) {
                                should_plan = true;
                            }
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
        drain_steers(&mut steer_rx, history, standing_task);

        // Roll up old history into a summary once it gets large, so the rest of
        // the turn plans against a compact window instead of the full transcript.
        compact_history(history, vlm).await;

        // Re-discover capabilities from atlas every round so providers that
        // registered mid-turn are visible in the next call.
        let cap_list = discovery::discover(atlas)
            .await
            .map_err(|e| anyhow::anyhow!("atlas capability discovery failed: {e}"))?;

        let embodiment_block =
            crate::soma_context::fetch_runtime_prompt_block(atlas, consumer_id).await;
        let environment_block = state_context::collect(executor, atlas, &cap_list).await;

        let display_caps = build_display_capabilities(&cap_list);
        let target_map = build_capability_target_map(&display_caps);
        let protocol_prompt = rtdl_protocol(round == 0);
        let (capability_prompt, capability_cache_hit) =
            capability_prompt_cache.render(&display_caps);

        let task_block = standing_task
            .as_ref()
            .map(TaskState::prompt_block)
            .unwrap_or_default();
        let forest_block = build_forest_block(&forest, &cancel_requested);
        let executor_active_block = fetch_executor_active_block(executor).await;
        let _ = tx
            .send(Ok(service::pack(
                &session_id,
                PilotStreamBody::Status(SessionStatusEvent {
                    session_id: session_id.clone(),
                    state: SessionState::Active as u32,
                    message: "Planning the next step".to_string(),
                }),
            )))
            .await;
        // Plan with a single corrective retry (merged from dev #88): if the
        // VLM's RTDL fails to parse or expand, feed the error back and let it
        // fix the reply once; a second failure ends the turn gracefully (empty
        // recovery plan) instead of crashing the whole turn. The loop yields a
        // valid (narration, tree label, plan, id) tuple for the forest dispatch.
        let mut correction: Option<String> = None;
        let (assistant_content, rtdl_description, graph, meta_op, plan_id, task_update, recovered) = loop {
            let sections = [
                PromptSection {
                    name: "standing_system",
                    content: &standing_prompt,
                },
                PromptSection {
                    name: "embodiment_static",
                    content: soma_prompt_block,
                },
                PromptSection {
                    name: "memory",
                    content: &memory_prompt,
                },
                PromptSection {
                    name: "capability_docs",
                    content: &capability_docs_prompt,
                },
                PromptSection {
                    name: "voice",
                    content: voice_prompt,
                },
                PromptSection {
                    name: "rtdl_protocol",
                    content: protocol_prompt,
                },
                PromptSection {
                    name: "capability_catalog",
                    content: capability_prompt,
                },
                PromptSection {
                    name: "task",
                    content: &task_block,
                },
                PromptSection {
                    name: "in_flight_trees",
                    content: &forest_block,
                },
                PromptSection {
                    name: "executor_state",
                    content: &executor_active_block,
                },
                PromptSection {
                    name: "embodiment_live",
                    content: &embodiment_block,
                },
                PromptSection {
                    name: "environment_live",
                    content: &environment_block,
                },
            ];
            let messages = assemble_planning_messages(
                round,
                capability_cache_hit,
                &sections,
                history,
                correction.as_deref(),
            );

            let planning_revision = forest_revision.load(Ordering::Acquire);
            let mut vlm_attempt = 0_u8;
            let (content, raw_tool_calls) = loop {
                let mut stream = match tokio::time::timeout(
                    vlm_idle_timeout(),
                    vlm.chat_stream(&messages, &[], Some(&prompt_cache_key)),
                )
                .await
                {
                    Ok(Ok(stream)) => stream,
                    Ok(Err(error)) if vlm_attempt == 0 => {
                        warn!("[pilot/vlm] opening stream failed; retrying once: {error:#}");
                        vlm_attempt += 1;
                        continue;
                    }
                    Ok(Err(error)) => {
                        return Err(anyhow::anyhow!("VLM stream error: {error:#}"));
                    }
                    Err(_) if vlm_attempt == 0 => {
                        warn!("[pilot/vlm] opening stream timed out; retrying once");
                        vlm_attempt += 1;
                        continue;
                    }
                    Err(_) => return Err(anyhow::anyhow!("VLM stream open timed out")),
                };
                let mut full_text = String::new();
                let mut tool_calls: Vec<crate::vlm::ToolCall> = Vec::new();

                let receive_result: anyhow::Result<()> = loop {
                    tokio::select! {
                        biased;
                        // Cancel takes priority — checked before every new VLM token.
                        _ = cancel_rx.changed() => {
                            drop(stream);
                            return_interrupted!(&forest);
                        }
                        steer = steer_rx.recv() => {
                            if let Some(task) = steer {
                                append_steer(task, history, standing_task);
                                drain_steers(&mut steer_rx, history, standing_task);
                                history::trim(history, MAX_HISTORY);
                            }
                            // The response being sampled was built without this
                            // input. Drop it before parsing or dispatching any
                            // call, then sample again from the updated history.
                            drop(stream);
                            should_plan = true;
                            continue 'supervisor;
                        }
                        item = stream.next() => {
                            let item = match item {
                                Some(Ok(it)) => it,
                                Some(Err(error)) => break Err(anyhow::anyhow!("VLM stream recv: {error:#}")),
                                None => break Ok(()),
                            };
                            match item {
                                VlmStreamItem::TextDelta(delta) => full_text.push_str(&delta),
                                VlmStreamItem::ToolCall(tc) => tool_calls.push(tc),
                                VlmStreamItem::Usage(usage) => info!(
                                    "[pilot/prompt] {}",
                                    serde_json::json!({
                                        "round": round,
                                        "provider_prompt_tokens": usage.prompt_tokens,
                                        "provider_completion_tokens": usage.completion_tokens,
                                        "provider_cached_tokens": usage.cached_tokens,
                                    })
                                ),
                                VlmStreamItem::Finish => {}
                            }
                        }
                        _ = tokio::time::sleep(vlm_idle_timeout()) => {
                            break Err(anyhow::anyhow!("VLM stream idle timeout"));
                        }
                    }
                };

                if let Err(error) = receive_result {
                    if vlm_attempt == 0 {
                        warn!("[pilot/vlm] {error:#}; retrying once");
                        let _ = tx
                            .send(Ok(service::pack(
                                &session_id,
                                PilotStreamBody::Status(SessionStatusEvent {
                                    session_id: session_id.clone(),
                                    state: SessionState::Active as u32,
                                    message: "VLM response delayed; retrying once".to_string(),
                                }),
                            )))
                            .await;
                        vlm_attempt += 1;
                        continue;
                    }
                    return Err(error);
                }

                let content = if full_text.is_empty() {
                    None
                } else {
                    Some(full_text)
                };
                break (content, tool_calls);
            };

            if forest_revision.load(Ordering::Acquire) != planning_revision {
                // Executor state changed while the model was thinking. Never
                // dispatch a plan based on the stale in-flight snapshot. Return
                // to the event arm, consume the queued state, then re-plan.
                should_plan = false;
                continue 'supervisor;
            }

            if !raw_tool_calls.is_empty() {
                anyhow::bail!("VLM returned tool_calls in RTDL mode");
            }

            let raw_content = content.unwrap_or_default();
            debug!("[pilot/rtdl/raw] raw_content={raw_content}");
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
                    let plan_id = String::new();
                    let graph = empty_sequence_plan(plan_id.clone(), session_id.clone(), round);
                    break (
                        rtdl_recovery_final_text(),
                        String::new(),
                        Some(graph),
                        None,
                        plan_id,
                        None,
                        true,
                    );
                }
            };

            debug!(
                "[pilot/rtdl/raw] model_rtdl={}",
                serde_json::to_string(&rtdl).unwrap_or_else(|_| "<unserializable>".into())
            );

            match parse_meta_plan_op(&rtdl).context("parse RTDL meta op") {
                Ok(Some(meta_op)) => {
                    break (
                        assistant_content,
                        rtdl_description,
                        None,
                        Some(meta_op),
                        String::new(),
                        task_update,
                        false,
                    );
                }
                Ok(None) => {}
                Err(e) if correction.is_none() => {
                    warn!("[pilot/rtdl] meta op invalid round={round}, retrying once: {e:#}");
                    correction = Some(build_rtdl_retry_prompt(&e, &raw_content, &display_caps));
                    continue;
                }
                Err(e) => {
                    warn!(
                        "[pilot/rtdl] meta op invalid again round={round}, ending turn gracefully: {e:#}"
                    );
                    let plan_id = String::new();
                    let graph = empty_sequence_plan(plan_id.clone(), session_id.clone(), round);
                    break (
                        rtdl_recovery_final_text(),
                        String::new(),
                        Some(graph),
                        None,
                        plan_id,
                        None,
                        true,
                    );
                }
            }

            // Reserve an id atomically only for normal RTDL. Concurrent sessions
            // cannot observe or dispatch the same id. A failed expansion may
            // leave a harmless gap, but an id is never reused.
            let plan_id = (plan_seq.fetch_add(1, Ordering::Relaxed) + 1).to_string();
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
                        Some(graph),
                        None,
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
                    let plan_id = String::new();
                    let graph = empty_sequence_plan(plan_id.clone(), session_id.clone(), round);
                    break (
                        rtdl_recovery_final_text(),
                        String::new(),
                        Some(graph),
                        None,
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

        if let Some(meta_op) = meta_op {
            let targets = meta_op.cancellation_targets(&forest);
            if let Some(target) = invalid_cancel_target(&targets, &forest, &cancel_requested) {
                warn!("[pilot/harness] suppressed stale or duplicate meta op for plan {target}");
                history.push(Message::user(&format!(
                    "Pilot harness feedback: plan-control target {target} is not active or is already stopping. Re-read In-flight trees and choose a currently listed plan_id. Do not retry a completed control operation."
                )));
                history::trim(history, MAX_HISTORY);
                should_plan = true;
                continue 'supervisor;
            }
            if let MetaPlanOp::StopAt { plan_id, op_id, .. } = &meta_op
                && forest
                    .get(plan_id)
                    .is_none_or(|meta| !meta.steps.iter().any(|step| step.op_id == *op_id))
            {
                warn!("[pilot/harness] suppressed stop_at for unknown op {plan_id}/{op_id}");
                history.push(Message::user(&format!(
                    "Pilot harness feedback: RTDL plan {plan_id} has no listed target_op_id {op_id}. Copy an exact op_id from In-flight trees and do not guess which step is current."
                )));
                history::trim(history, MAX_HISTORY);
                should_plan = true;
                continue 'supervisor;
            }

            if let Some(updated) = task_update {
                let changed = apply_task_update(standing_task, updated, false);
                if changed && let Some(state) = standing_task.as_ref() {
                    let _ = tx
                        .send(Ok(service::pack(
                            &session_id,
                            PilotStreamBody::TaskState(TaskStateEvent {
                                goal: state.goal.clone(),
                                success_criterion: state.success_criterion.clone(),
                                status: state.status.clone(),
                            }),
                        )))
                        .await;
                }
            }
            if !assistant_content.trim().is_empty() {
                history.push(Message::assistant(&assistant_content));
                history::trim(history, MAX_HISTORY);
                last_content = assistant_content.clone();
                let _ = tx
                    .send(Ok(service::pack(
                        &session_id,
                        PilotStreamBody::TextChunk(assistant_content),
                    )))
                    .await;
            }

            cancel_requested.extend(targets.iter().cloned());
            let result = execute_meta_plan_op(executor, &meta_op).await;
            round += 1;
            match result {
                Ok(message) => {
                    info!("[pilot/control] {message}");
                    history.push(Message::user(&format!(
                        "Pilot plan-control result: {message} This was an out-of-band meta operation, not an RTDL tree. Do not issue it again."
                    )));
                    history::trim(history, MAX_HISTORY);
                    let _ = tx
                        .send(Ok(service::pack(
                            &session_id,
                            PilotStreamBody::Status(SessionStatusEvent {
                                session_id: session_id.clone(),
                                state: SessionState::Active as u32,
                                message: "Plan control accepted".to_string(),
                            }),
                        )))
                        .await;
                    // PlanDone is the durable boundary. Replan only after every
                    // target in this control batch has left the forest.
                    should_plan = targets.is_empty();
                }
                Err(error) => {
                    warn!("[pilot/control] meta operation failed: {error:#}");
                    for target in &targets {
                        cancel_requested.remove(target);
                    }
                    history.push(Message::user(&format!(
                        "Pilot plan-control failure: {error:#}. The operation was not accepted; inspect the current In-flight trees before deciding whether to retry."
                    )));
                    history::trim(history, MAX_HISTORY);
                    should_plan = true;
                }
            }
            continue 'supervisor;
        }

        let graph = graph.expect("non-meta RTDL response must carry a graph");

        let calls = plan_call_count(&graph);
        let call_signatures = plan_call_signatures(&graph);
        let cancel_targets = plan_cancel_targets(&graph);
        if mixes_control_inspection_with_action(&graph) {
            warn!("[pilot/harness] suppressed mixed control inspection and action tree");
            history.push(Message::user(
                "Pilot harness feedback: legacy plan-control builtins cannot be mixed with business RTDL. Use a root cancel_plan, cancel_all, or stop_plan_at meta op instead; dispatch successor work only after control completion.",
            ));
            history::trim(history, MAX_HISTORY);
            should_plan = true;
            continue 'supervisor;
        }
        if let Some(target) = invalid_cancel_target(&cancel_targets, &forest, &cancel_requested) {
            warn!("[pilot/harness] suppressed stale or duplicate cancel for plan {target}");
            history.push(Message::user(
                "Pilot harness feedback: that legacy cancel target is not cancellable now. Re-read In-flight trees and use one root plan-control meta op; do not retry a finished target or create a cancel RTDL tree.",
            ));
            history::trim(history, MAX_HISTORY);
            should_plan = true;
            continue 'supervisor;
        }
        if let Some(duplicate) = duplicate_in_flight_signature(&call_signatures, &forest) {
            warn!("[pilot/harness] suppressed duplicate in-flight call: {duplicate}");
            history.push(Message::user(
                "Pilot harness feedback: that exact capability call is already in flight. Do not dispatch or cancel it again; wait for its result.",
            ));
            history::trim(history, MAX_HISTORY);
            should_plan = false;
            continue 'supervisor;
        }

        // Apply progress only after the harness knows whether this response can
        // safely finish. A model cannot mark a task done while it is also
        // dispatching work or while an older tree remains in flight.
        if let Some(updated) = task_update {
            info!(
                "[pilot/rtdl] task_update goal='{}' status='{}'",
                updated.goal, updated.status
            );
            let changed = apply_task_update(standing_task, updated, calls == 0);
            if changed && let Some(state) = standing_task.as_ref() {
                let _ = tx
                    .send(Ok(service::pack(
                        &session_id,
                        PilotStreamBody::TaskState(TaskStateEvent {
                            goal: state.goal.clone(),
                            success_criterion: state.success_criterion.clone(),
                            status: state.status.clone(),
                        }),
                    )))
                    .await;
            }
        }

        log_plan_start(&graph, &rtdl_description, round, calls);

        // Retain model narration for later planning. Action-producing RTDL
        // rounds are also streamed below so the current user sees and hears
        // progress instead of receiving only a final burst after a long task.
        if !assistant_content.is_empty() {
            history.push(Message::assistant(&assistant_content));
            last_content = assistant_content.clone();
        }

        round += 1;
        let hit_cap = round as usize >= max_rounds;
        let task_done = standing_task
            .as_ref()
            .map(TaskState::is_done)
            .unwrap_or(false);

        if calls == 0 {
            // With no tree left, this is either a final answer or a deliberate
            // request for more user input. End this transport turn exactly once;
            // an in-progress standing task remains persisted by the service and
            // resumes on the next user message.
            if forest.is_empty() {
                if hit_cap && !(task_done || standing_task.is_none()) {
                    warn!("[pilot] hit max tool rounds ({max_rounds}), stopping turn");
                }
                let reply = if assistant_content.trim().is_empty() && !task_done {
                    "I need more information before I can continue.".to_string()
                } else {
                    assistant_content
                };
                let _ = tx
                    .send(Ok(service::pack(
                        &session_id,
                        PilotStreamBody::FinalText(reply),
                    )))
                    .await;
                break;
            }
            // A completed interaction may close while unrelated long-running
            // work remains. If this interaction is still in progress, keep its
            // stream open: surface the model text as progress and wait for the
            // relevant plan result before producing FinalText.
            if !assistant_content.trim().is_empty() {
                let body = if task_done {
                    PilotStreamBody::FinalText(assistant_content.clone())
                } else {
                    PilotStreamBody::TextChunk(assistant_content.clone())
                };
                let _ = tx.send(Ok(service::pack(&session_id, body))).await;
                if !task_done {
                    // Status is the narration boundary consumed by Liaison:
                    // display/TTS the complete progress text now without
                    // closing the SubmitTask stream.
                    let _ = tx
                        .send(Ok(service::pack(
                            &session_id,
                            PilotStreamBody::Status(SessionStatusEvent {
                                session_id: session_id.clone(),
                                state: SessionState::Active as u32,
                                message: "Waiting for in-flight work".to_string(),
                            }),
                        )))
                        .await;
                }
            }
            if hit_cap {
                warn!("[pilot] hit max tool rounds ({max_rounds}), stopping turn");
                break;
            }
            // should_plan stays false: wait for a forest event, or for a steer
            // when an in-progress task has intentionally produced no new tree.
            continue;
        }

        if !assistant_content.trim().is_empty() {
            let _ = tx
                .send(Ok(service::pack(
                    &session_id,
                    PilotStreamBody::TextChunk(assistant_content.clone()),
                )))
                .await;
        }

        // Non-empty tree: hand the structure to the client and dispatch it to
        // the forest after its user-facing narration above.
        let _ = tx
            .send(Ok(service::pack(
                &session_id,
                PilotStreamBody::Plan(graph.clone()),
            )))
            .await;
        cancel_requested.extend(cancel_targets);
        record_dispatched_plan(history, &graph, &rtdl_description);
        forest.insert(
            plan_id.clone(),
            TreeMeta {
                description: rtdl_description,
                control_only: is_control_only(&graph),
                call_signatures,
                steps: plan_steps(&graph),
            },
        );
        tokio::spawn(drive_plan(
            graph,
            executor.graph.clone(),
            forest_tx.clone(),
            Arc::clone(&forest_revision),
        ));
        info!(
            "[pilot/forest] plan_id={plan_id} dispatched forest_size={}",
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

/// Convert Atlas rows to provider-qualified model names and sort them so an
/// unchanged catalog remains byte-identical even if discovery order varies.
fn build_display_capabilities(
    cap_list: &[(String, atlas_pb::Capability)],
) -> Vec<DisplayCapability<'_>> {
    let mut display = cap_list
        .iter()
        .filter(|(_, cap)| !is_legacy_plan_control_contract(&cap.contract_id))
        .map(|(provider_id, cap)| DisplayCapability {
            display_name: format!("{}.{}", provider_id, llm_name(&cap.contract_id)),
            provider_id: provider_id.as_str(),
            cap,
        })
        .collect::<Vec<_>>();
    display.sort_by(|left, right| left.display_name.cmp(&right.display_name));
    display
}

fn is_legacy_plan_control_contract(contract_id: &str) -> bool {
    if !contract_id.starts_with("robonix/system/executor/builtin/") {
        return false;
    }
    matches!(
        contract_id.rsplit('/').next().unwrap_or_default(),
        "cancel_plan" | "cancel_all_plans" | "stop_plan_at" | "get_all_plans" | "get_plan_status"
    )
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

/// Compact contract reminder for rounds after the first. The complete frozen
/// protocol is sent on round zero; later requests keep only the wire grammar
/// and harness invariants that are required to parse and admit the next plan.
const RTDL_PROTOCOL_REMINDER: &str = r#"## RTDL output (same frozen contract as round 0)
Reply with exactly one JSON object and no surrounding prose:
{"content":"...","rtdl_description":"...","rtdl":<node>,"task_update":null|{"goal":"...","success_criterion":"...","status":"in_progress"|"done"}}

Nodes are exactly one of:
- {"op":"sequence","op_id":0,"description":"...","children":[...]}
- {"op":"parallel","op_id":0,"description":"...","children":[...]}
- {"op":"do","op_id":0,"description":"...","cap":"<exact capability_name>","args":{...}}
Write op_id=0; Pilot assigns the real ID. Do not add node fields. Compose every currently-known dependent step into one sequence and every independent step into one parallel tree; do not drip one known call per round. With no new call, return an empty sequence.

Plan control is the entire rtdl value, never a nested node or capability: cancel_plan, cancel_all, or stop_plan_at using an exact listed plan_id/op_id. Never repeat a completed/cancelled control operation and never cancel an unrelated tree after another tree fails.

Resolve a named room through current Scene regions before navigation; never use a remembered grasp or observation pose as a room goal. A navigation SUCCEEDED result proves only the resolved requested destination; a zero-distance result does not prove that the robot moved. Never call a skill's cancel capability; Executor propagates RTDL cancellation.

Copy each cap exactly from the current catalog. Executor feedback and dispatch records are scoped by plan_id/call_id and prove what was already sent; do not repeat a successful call. task_update.goal must exactly copy Current user interaction. Use status=done only when its success criterion is verified and its relevant tree is no longer running.
"#;

fn rtdl_protocol(full: bool) -> &'static str {
    if full {
        include_str!("../rtdl_protocol.md")
    } else {
        RTDL_PROTOCOL_REMINDER
    }
}

/// Hash the exact Atlas fields used by the prompt so registration changes
/// invalidate the rendered catalog without relying on provider list identity.
fn capability_prompt_fingerprint(display_caps: &[DisplayCapability<'_>]) -> u64 {
    let mut hasher = DefaultHasher::new();
    for cap in display_caps {
        cap.display_name.hash(&mut hasher);
        cap.provider_id.hash(&mut hasher);
        cap.cap.contract_id.hash(&mut hasher);
        cap.cap.description.hash(&mut hasher);
        if let Some(atlas_pb::transport_params::Kind::Mcp(mcp)) = cap
            .cap
            .params
            .as_ref()
            .and_then(|params| params.kind.as_ref())
        {
            mcp.input_schema_json.hash(&mut hasher);
        }
    }
    hasher.finish()
}

/// Render the complete capability catalog in a compact, deterministic shape.
/// Names remain on their own line for the CI fake VLM and descriptions are
/// JSON-escaped so embedded whitespace cannot inflate or corrupt the catalog.
fn render_capability_prompt(display_caps: &[DisplayCapability<'_>]) -> String {
    let mut prompt = String::from("\n## Available capabilities\n\n");
    for cap in display_caps {
        let c = cap.cap;
        let Some(atlas_pb::transport_params::Kind::Mcp(mcp)) =
            c.params.as_ref().and_then(|params| params.kind.as_ref())
        else {
            continue;
        };
        let schema: serde_json::Value =
            serde_json::from_str(&mcp.input_schema_json).unwrap_or(serde_json::Value::Null);
        let description =
            serde_json::to_string(c.description.trim()).unwrap_or_else(|_| "\"\"".to_string());
        prompt.push_str(&format!(
            "- capability_name: {}\n  description: {}\n  args_schema: {}\n",
            cap.display_name, description, schema
        ));
    }
    prompt
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
/// Extract the first balanced top-level JSON object from `raw`, ignoring any
/// prose before or after it (e.g. a narration line the model emitted before the
/// JSON, or a trailing comment). Returns the `{...}` slice, or `None` if there
/// is no `{` or no matching close brace.
///
/// Brace depth is counted only outside JSON string literals, so braces inside
/// strings don't affect it. Scanning by bytes is UTF-8-safe here because `{`,
/// `}`, `"`, and `\` are all ASCII and never collide with multibyte
/// continuation bytes (which are all >= 0x80).
fn extract_json_object(raw: &str) -> Option<&str> {
    let bytes = raw.as_bytes();
    let start = bytes.iter().position(|&b| b == b'{')?;
    let mut depth = 0usize;
    let mut in_str = false;
    let mut escaped = false;
    for (i, &b) in bytes.iter().enumerate().skip(start) {
        if in_str {
            if escaped {
                escaped = false;
            } else if b == b'\\' {
                escaped = true;
            } else if b == b'"' {
                in_str = false;
            }
            continue;
        }
        match b {
            b'"' => in_str = true,
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(&raw[start..=i]);
                }
            }
            _ => {}
        }
    }
    None
}

/// Tolerates a prose preamble or trailing commentary around the JSON object
/// (a common model habit, e.g. a narration line then the JSON on the next
/// line) by extracting the first balanced `{...}` before parsing; the raw
/// string is used unchanged when no object is found, so a genuinely
/// JSON-less reply still surfaces the original parse error.
///
/// Fails if `raw` is not valid JSON, the root is not an object, the key set is
/// not exactly those four, `content` / `rtdl_description` are not strings,
/// `rtdl` is not an object, or `task_update` is neither `null` nor a valid task
/// object.
fn parse_rtdl_assistant_response(raw: &str) -> Result<RtdlEnvelope> {
    let candidate = extract_json_object(raw).unwrap_or(raw);
    let v: serde_json::Value = serde_json::from_str(candidate)?;
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
         with exactly `content`, `rtdl_description`, `rtdl`, and `task_update`. The reply MUST begin \
         with `{{` and end with `}}`: no prose, narration, or markdown fences before or after it (put \
         any user-facing text inside `content`). Use only \
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

/// Pick a node's `description`, in priority order:
/// 1. the LLM's own per-node `description` field when present and non-empty;
/// 2. the LLM's tree label (`rtdl_description`) for an otherwise-unlabelled root;
/// 3. a synthesized fallback (e.g. `call camera_snapshot`).
///
/// The model is asked to author a node-level `description` for every node (see
/// `rtdl_protocol.md`); the fallbacks keep a sloppy or older reply from failing
/// the turn, since executor's `validate_plan` requires a non-empty description.
fn pick_description(
    obj: &serde_json::Map<String, serde_json::Value>,
    path: &str,
    root_description: &str,
    synthesized: String,
) -> String {
    if let Some(d) = obj.get("description").and_then(|x| x.as_str()) {
        let d = d.trim();
        if !d.is_empty() {
            return d.to_string();
        }
    }
    if path == "$" && !root_description.is_empty() {
        return root_description.to_string();
    }
    synthesized
}

/// Reject node fields outside the allowed set and require the structural ones.
///
/// `required` lists the keys an operator must carry beyond `op` (e.g.
/// `children` for sequence/parallel; `cap` + `args` for do). `op_id` and
/// `description` are always optional — the model emits them (op_id defaults to
/// 0, which pilot ignores and reassigns), but a reply that omits them still
/// parses. Any other key (`out`, `id`, `plan_id`, …) is an error.
fn reject_unknown_node_keys(
    obj: &serde_json::Map<String, serde_json::Value>,
    path: &str,
    op: &str,
    required: &[&str],
) -> Result<()> {
    const OPTIONAL: [&str; 2] = ["op_id", "description"];
    for key in obj.keys() {
        let known =
            key == "op" || required.contains(&key.as_str()) || OPTIONAL.contains(&key.as_str());
        if !known {
            anyhow::bail!("{path}: {op} node has unexpected field `{key}`");
        }
    }
    for req in required {
        if !obj.contains_key(*req) {
            anyhow::bail!("{path}: {op} node must contain `{req}`");
        }
    }
    Ok(())
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
            reject_unknown_node_keys(obj, path, op, &["children"])?;
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
            let description = pick_description(
                obj,
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
            reject_unknown_node_keys(obj, path, op, &["cap", "args"])?;
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
            let description = pick_description(obj, path, root_description, format!("call {cap}"));
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

fn mixes_control_inspection_with_action(plan: &Plan) -> bool {
    let leaves: Vec<&str> = plan
        .nodes
        .iter()
        .filter_map(|node| node.call.as_ref())
        .filter_map(|call| call.contract_id.rsplit('/').next())
        .collect();
    let has_inspection = leaves
        .iter()
        .any(|leaf| matches!(*leaf, "get_plan_status" | "get_all_plans"));
    has_inspection
        && leaves
            .iter()
            .any(|leaf| !matches!(*leaf, "get_plan_status" | "get_all_plans"))
}

fn plan_call_signatures(plan: &Plan) -> HashSet<String> {
    plan.nodes
        .iter()
        .filter_map(|node| node.call.as_ref())
        .map(|call| {
            let args = serde_json::from_str::<serde_json::Value>(&call.args_json)
                .ok()
                .and_then(|value| serde_json::to_string(&value).ok())
                .unwrap_or_else(|| call.args_json.clone());
            format!("{}|{}|{args}", call.provider_id, call.contract_id)
        })
        .collect()
}

fn duplicate_in_flight_signature(
    signatures: &HashSet<String>,
    forest: &HashMap<String, TreeMeta>,
) -> Option<String> {
    signatures.iter().find_map(|signature| {
        forest
            .values()
            .any(|meta| meta.call_signatures.contains(signature))
            .then(|| signature.clone())
    })
}

fn plan_cancel_targets(plan: &Plan) -> Vec<String> {
    plan.nodes
        .iter()
        .filter_map(|node| node.call.as_ref())
        .filter(|call| call.contract_id.rsplit('/').next() == Some("cancel_plan"))
        .filter_map(|call| serde_json::from_str::<serde_json::Value>(&call.args_json).ok())
        .filter_map(|args| {
            args.get("plan_id")
                .and_then(|value| value.as_str())
                .map(str::to_string)
        })
        .collect()
}

fn invalid_cancel_target(
    targets: &[String],
    forest: &HashMap<String, TreeMeta>,
    cancel_requested: &HashSet<String>,
) -> Option<String> {
    targets.iter().find_map(|target| {
        let invalid = cancel_requested.contains(target)
            || forest.get(target).is_none_or(|meta| meta.control_only);
        invalid.then(|| target.clone())
    })
}

fn should_replan_after_plan_done(
    canceled: bool,
    requested_cancellation: bool,
    cancellation_batch_complete: bool,
    interaction_active: bool,
) -> bool {
    interaction_active && (!canceled || (requested_cancellation && cancellation_batch_complete))
}

/// Render an RTDL node state as a stable human-readable name for logs.
fn rtdl_state_name(state: u32) -> String {
    match RtdlNodeStateEnum::try_from(state as i32) {
        Ok(RtdlNodeStateEnum::Pending) => "Pending".to_string(),
        Ok(RtdlNodeStateEnum::Running) => "Running".to_string(),
        Ok(RtdlNodeStateEnum::Succeeded) => "Succeeded".to_string(),
        Ok(RtdlNodeStateEnum::Failed) => "Failed".to_string(),
        Ok(RtdlNodeStateEnum::Canceled) => "Canceled".to_string(),
        Ok(RtdlNodeStateEnum::Timeout) => "Timeout".to_string(),
        Ok(RtdlNodeStateEnum::Paused) => "Paused".to_string(),
        Err(_) => format!("Unknown({state})"),
    }
}

/// Render an RTDL node kind as the tree operator name used in plan logs.
fn rtdl_node_kind_name(kind: u32) -> String {
    match kind {
        RTDL_SEQUENCE => "sequence".to_string(),
        RTDL_PARALLEL => "parallel".to_string(),
        RTDL_DO => "do".to_string(),
        _ => format!("unknown({kind})"),
    }
}

/// Shorten free-form payloads so one log event stays readable on one line.
fn compact_preview(value: &str, max_chars: usize) -> String {
    let flattened = value.replace('\n', "\\n");
    let mut preview: String = flattened.chars().take(max_chars).collect();
    if flattened.chars().count() > max_chars {
        preview.push_str("...");
    }
    preview
}

/// Bound a tool result without turning structured JSON into an invalid prefix.
/// Scene list contracts keep the identifiers needed for a targeted follow-up
/// while explicitly reporting whether any records were omitted.
fn compact_tool_result(contract_id: &str, value: &str, max_chars: usize) -> String {
    let original_chars = value.chars().count();
    if original_chars <= max_chars {
        return value.to_string();
    }

    let projection = match contract_id {
        "robonix/system/scene/list_objects" => Some(("objects", &["id", "label"][..])),
        "robonix/system/scene/list_regions" => Some((
            "regions",
            &["id", "kind", "name", "stale", "stale_reason"][..],
        )),
        _ => None,
    };
    if let (Some((array_key, fields)), Ok(parsed)) =
        (projection, serde_json::from_str::<serde_json::Value>(value))
        && let Some(items) = parsed.get(array_key).and_then(|entry| entry.as_array())
    {
        let mut projected: Vec<serde_json::Value> = items
            .iter()
            .map(|item| {
                let mut record = serde_json::Map::new();
                if let Some(source) = item.as_object() {
                    for field in fields {
                        if let Some(field_value) = source.get(*field) {
                            record.insert((*field).to_string(), field_value.clone());
                        }
                    }
                }
                serde_json::Value::Object(record)
            })
            .collect();
        loop {
            let returned = projected.len();
            let mut root = serde_json::Map::new();
            root.insert(
                array_key.to_string(),
                serde_json::Value::Array(projected.clone()),
            );
            for key in ["map_id", "stamp_unix"] {
                if let Some(field_value) = parsed.get(key) {
                    root.insert(key.to_string(), field_value.clone());
                }
            }
            root.insert(
                "_robonix_truncation".to_string(),
                serde_json::json!({
                    "truncated": true,
                    "complete_record_index": returned == items.len(),
                    "original_chars": original_chars,
                    "total_records": items.len(),
                    "returned_records": returned,
                    "omitted_fields": true,
                    "instruction": "Use a narrower capability for full record details; never infer absence when complete_record_index is false."
                }),
            );
            let encoded = serde_json::Value::Object(root).to_string();
            if encoded.chars().count() <= max_chars {
                return encoded;
            }
            if projected.is_empty() {
                break;
            }
            projected.pop();
        }
    }

    let mut preview_chars = max_chars / 3;
    loop {
        let encoded = serde_json::json!({
            "_robonix_truncation": {
                "truncated": true,
                "original_chars": original_chars,
                "complete": false,
                "instruction": "The preview is incomplete; do not infer that an omitted value is absent."
            },
            "preview": compact_preview(value, preview_chars),
        })
        .to_string();
        if encoded.chars().count() <= max_chars || preview_chars == 0 {
            return encoded;
        }
        preview_chars /= 2;
    }
}

/// Recover the LLM-facing capability name from an expanded capability call.
fn call_display_name(call: &CapabilityCall) -> String {
    format!("{}.{}", call.provider_id, llm_name(&call.contract_id))
}

/// Append one node and its descendants to the human-readable plan summary.
fn append_plan_node_summary(plan: &Plan, node_index: usize, depth: usize, out: &mut Vec<String>) {
    let Some(node) = plan.nodes.get(node_index) else {
        out.push(format!("{}[{node_index}] missing-node", "  ".repeat(depth)));
        return;
    };
    let indent = "  ".repeat(depth);
    let mut line = format!(
        "{indent}[{node_index}] {} op_id={} desc='{}'",
        rtdl_node_kind_name(node.node_kind),
        node.op_id,
        compact_preview(&node.description, 160),
    );
    if let Some(call) = node.call.as_ref() {
        line.push_str(&format!(
            " cap={} args={}",
            call_display_name(call),
            compact_preview(&call.args_json, 240)
        ));
    }
    out.push(line);
    for child in &node.children {
        append_plan_node_summary(plan, *child as usize, depth + 1, out);
    }
}

/// Format a plan as an indented tree instead of exposing arena child arrays.
fn format_plan_summary(plan: &Plan) -> Vec<String> {
    let mut lines = Vec::new();
    append_plan_node_summary(plan, plan.root_index as usize, 0, &mut lines);
    lines
}

/// Emit the compact plan-start log block for one expanded RTDL plan.
fn log_plan_start(plan: &Plan, description: &str, round: u32, calls: usize) {
    info!(
        "[pilot/rtdl] -- plan start plan_id={} round={} calls={} --",
        plan.plan_id, round, calls
    );
    info!(
        "[pilot/rtdl] rtdl_plan_description='{}'",
        compact_preview(description, 240)
    );
    info!("[pilot/rtdl] rtdl_plan:");
    for line in format_plan_summary(plan) {
        info!("[pilot/rtdl] {line}");
    }
}

/// Build compact extra detail for non-success terminal node states.
fn terminal_node_detail(ns: &RtdlNodeState) -> String {
    if !is_terminal_executor_state(ns.state) || ns.state == RtdlNodeStateEnum::Succeeded as u32 {
        return String::new();
    }
    let Some(result) = ns.leaf_result.as_ref() else {
        return String::new();
    };
    if !result.error.trim().is_empty() {
        return format!(" error='{}'", compact_preview(&result.error, 180));
    }
    if !result.output.trim().is_empty() {
        return format!(" output='{}'", compact_preview(&result.output, 180));
    }
    String::new()
}

/// Emit one readable node-state event without numeric state or kind codes.
fn log_node_state(plan_id: &str, ns: &RtdlNodeState) {
    let mut line = format!(
        "[pilot/forest] plan_id={} node={} op_id={} state={} desc='{}'",
        plan_id,
        ns.node_index,
        ns.op_id,
        rtdl_state_name(ns.state),
        compact_preview(&ns.description, 160),
    );
    if !ns.operator_detail.trim().is_empty() {
        line.push_str(&format!(
            " detail='{}'",
            compact_preview(&ns.operator_detail, 180)
        ));
    }
    line.push_str(&terminal_node_detail(ns));
    debug!("{line}");
}

/// Pick the plan-level completion state shown in the forest completion log.
fn plan_completion_state(results: &[RtdlNodeState], any_failed: bool) -> String {
    if !any_failed {
        return "Succeeded".to_string();
    }
    for preferred in [
        RtdlNodeStateEnum::Failed as u32,
        RtdlNodeStateEnum::Timeout as u32,
        RtdlNodeStateEnum::Canceled as u32,
    ] {
        if results.iter().any(|ns| ns.state == preferred) {
            return rtdl_state_name(preferred);
        }
    }
    results
        .iter()
        .find(|ns| ns.state != RtdlNodeStateEnum::Succeeded as u32)
        .map(|ns| rtdl_state_name(ns.state))
        .unwrap_or_else(|| "Failed".to_string())
}

/// Emit the readable plan completion line, including non-success terminal nodes.
fn log_plan_complete(plan_id: &str, results: &[RtdlNodeState], any_failed: bool) {
    let state = plan_completion_state(results, any_failed);
    let mut line = format!(
        "[pilot/forest] plan_id={} complete state={} terminal_nodes={}",
        plan_id,
        state,
        results.len()
    );
    let non_success: Vec<String> = results
        .iter()
        .filter(|ns| ns.state != RtdlNodeStateEnum::Succeeded as u32)
        .map(|ns| format!("node={} state={}", ns.node_index, rtdl_state_name(ns.state)))
        .collect();
    if !non_success.is_empty() {
        line.push_str(" non_success=[");
        line.push_str(&non_success.join(", "));
        line.push(']');
    }
    line.push_str("; replanning");
    info!("{line}");
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

fn rtdl_result_to_messages(r: &CapabilityCallResult) -> history::ToolResultHistory {
    let mapped = if r.success {
        history::tool_result_to_messages(&r.call_id, &r.output)
    } else {
        history::ToolResultHistory {
            tool_messages: vec![Message::user(&r.output)],
            followup_messages: vec![],
        }
    };

    let tool_messages = mapped
        .tool_messages
        .into_iter()
        .map(|msg| {
            let output = msg.content.unwrap_or_default();
            let feedback = serde_json::json!({
                "leaf_result": {
                    "call_id": r.call_id,
                    "contract_id": r.contract_id,
                    "success": r.success,
                    "output": output,
                    "error": r.error,
                }
            });
            Message::user(&format!(
                "Executor feedback for the current RTDL leaf (not a new user request): {}",
                feedback
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
- Scope every result to the `plan_id` and independent RTDL tree named in its
  Executor feedback. If a capability fails, times out, returns success=false,
  or gives an unsafe/unexpected result, stop only steps that depend on that
  result. Report that branch failure, but let unrelated in-flight trees continue.
  Never cancel a different in-flight tree merely because this tree failed.
- Cancel a running tree only when the latest user steer explicitly asks to stop
  work covered by that tree, or when continuing that same tree is unsafe. A
  failure in an independent monitoring, greeting, observation, or query branch
  is not permission to cancel navigation or another physical task.
- For any boundary stop, select the explicitly requested step from the ordered
  in-flight RTDL step list and call `builtin_stop_plan_at` once. The target may
  be any step in the plan; never assume it means the currently running step.
  Use `on_complete` for 'after step X' and `on_enter` for 'before step X'.
  Bind X itself; never substitute X's predecessor or successor.
- Do not execute a later physical step unless its required earlier steps have succeeded.
- For semantic navigation, resolve names through Scene before calling navigation:
  - call Scene `list_regions` first to discover the stable ID for a named room
    or region, and call `list_objects` for a physical object; pass that exact
    full ID to the goal tool, never its label, room number, or a guessed ID;
    use `get_scene_graph` only when object relationships are needed;
  - named rooms or regions MUST use Scene `goal_room`; never use `goal_near`,
    Memory coordinates, or guessed coordinates for a room destination;
  - physical objects MUST use Scene `goal_near` to obtain an approach pose;
  - call navigation only when Scene returns `reachable=true`.
- Long-term Memory is historical context, not live spatial state. It may help
  recall what the user called a place, but it never replaces current Scene
  resolution for a named destination and never turns a task-specific grasp or
  observation pose into a room goal.
- After navigation, relate the result to the resolved requested destination.
  A transport/action status of `SUCCEEDED` does not by itself prove that the
  requested movement occurred: if the submitted pose was already the current
  pose, state that the robot was already there instead of claiming it moved.
- Some later messages may be labelled `Executor feedback for the current task`.
  Treat those as results of capability calls you already planned, not as new
  user requests.
- `Pilot harness dispatch record` messages are the authoritative record of RTDL
  calls already sent to Executor. Correlate each result by `plan_id` and
  `call_id`. When a recorded step succeeds, do not plan that same user-requested
  step again from newly observed state; use the recorded args and result to
  decide whether the success criterion is met. A genuinely different dependent
  step may still use the same capability.
- If executor feedback already contains enough information to answer the
  user's request, answer in `content`, set `task_update.status` to `done`, and
  output an empty RTDL sequence. Do not repeat the same observation capability
  just to confirm unchanged data.

## Interaction and execution lifetime
The harness owns the latest instruction shown in \"Current user interaction\".
Copy it exactly into a non-null `task_update.goal`; `task_update` reports
progress and never replaces user intent. Older conversation remains in message
history. Independently running work appears only in \"In-flight RTDL trees\"
and may outlive this interaction. Preserve unrelated trees; if the latest
instruction conflicts with one, target that specific plan with cancel/stop
before dispatching its replacement.

Mark the current interaction `done` once its own requested outcome is verified,
even when an unrelated long-running tree remains active. An empty RTDL sequence
alone does not prove completion; it may also mean waiting for an in-flight tree.
Concretely:

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
- For every action-producing RTDL response, put one concise user-facing progress
  update in `content` that says what is happening now. Do not repeat an unchanged
  update. When the task completes or needs clarification, use `content` for the
  concise final result or question.
",
    );
    p
}

#[cfg(test)]
mod tests {
    use super::{
        CapabilityPromptCache, CapabilityTargetMap, DEFAULT_SUCCESS_CRITERION, MetaPlanOp, RTDL_DO,
        RTDL_PARALLEL, RTDL_PROTOCOL_REMINDER, RTDL_SEQUENCE, TaskState, TreeMeta, TreeStep,
        append_steer, apply_task_update, build_capability_target_map, build_display_capabilities,
        build_executor_active_block, build_forest_block, compact_tool_result,
        configured_vlm_idle_timeout, duplicate_in_flight_signature, expand_rtdl_to_plan,
        extract_json_object, feed_results_into_history, format_plan_summary, invalid_cancel_target,
        is_control_only, is_legacy_plan_control_contract, mixes_control_inspection_with_action,
        parse_meta_plan_op, parse_rtdl_assistant_response, parse_task_update, plan_call_signatures,
        record_dispatched_plan, rtdl_node_kind_name, rtdl_recovery_final_text, rtdl_state_name,
        should_replan_after_plan_done, skip_memory_prefetch, start_or_resume_task,
        task_is_session_end,
    };
    use crate::pb::pilot::{CapabilityCall, CapabilityCallResult, Plan, RtdlNode, Task};
    use robonix_atlas::pb as atlas_pb;
    use serde_json::json;
    use std::collections::{HashMap, HashSet};
    use std::time::Duration;

    #[test]
    fn vlm_idle_timeout_is_bounded_and_has_a_responsive_default() {
        assert_eq!(configured_vlm_idle_timeout(None), Duration::from_secs(30));
        assert_eq!(
            configured_vlm_idle_timeout(Some("1")),
            Duration::from_secs(5)
        );
        assert_eq!(
            configured_vlm_idle_timeout(Some("600")),
            Duration::from_secs(300)
        );
        assert_eq!(
            configured_vlm_idle_timeout(Some("bad")),
            Duration::from_secs(30)
        );
    }

    fn test_capability(provider: &str, leaf: &str) -> (String, atlas_pb::Capability) {
        (
            provider.to_string(),
            atlas_pb::Capability {
                provider_id: provider.to_string(),
                contract_id: format!("robonix/service/test/{leaf}"),
                transport: atlas_pb::Transport::Mcp as i32,
                params: Some(atlas_pb::TransportParams {
                    kind: Some(atlas_pb::transport_params::Kind::Mcp(atlas_pb::McpParams {
                        input_schema_json: format!(
                            r#"{{"type":"object","properties":{{"{leaf}":{{"type":"string"}}}}}}"#
                        ),
                    })),
                }),
                description: format!("Run {leaf}"),
                ..Default::default()
            },
        )
    }

    #[test]
    fn later_round_protocol_is_compact_but_keeps_admission_rules() {
        assert!(RTDL_PROTOCOL_REMINDER.len() < 2_000);
        for required in [
            "capability_name",
            "sequence",
            "parallel",
            "cancel_plan",
            "plan_id/call_id",
            "task_update.goal",
            "Scene regions",
            "Never call a skill's cancel capability",
        ] {
            assert!(RTDL_PROTOCOL_REMINDER.contains(required));
        }
    }

    #[test]
    fn stable_catalog_is_cached_and_three_step_tree_stays_one_plan() {
        let capabilities = vec![
            test_capability("demo", "observe"),
            test_capability("demo", "remember"),
            test_capability("demo", "report"),
        ];
        let display = build_display_capabilities(&capabilities);
        let mut cache = CapabilityPromptCache::default();
        let (first, first_hit) = cache.render(&display);
        let first = first.to_string();
        let (second, second_hit) = cache.render(&display);
        assert!(!first_hit);
        assert!(second_hit);
        assert_eq!(first, second);

        let targets = build_capability_target_map(&display);
        let rtdl = json!({
            "op": "sequence",
            "op_id": 0,
            "description": "observe, remember, then report",
            "children": [
                {"op":"do","op_id":0,"description":"observe","cap":"demo.test_observe","args":{"observe":"room"}},
                {"op":"do","op_id":0,"description":"remember","cap":"demo.test_remember","args":{"remember":"room"}},
                {"op":"do","op_id":0,"description":"report","cap":"demo.test_report","args":{"report":"room"}}
            ]
        });
        let plan =
            expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 1, "multi-step").unwrap();
        assert_eq!(super::plan_call_count(&plan), 3);
        assert_eq!(plan.round, 1);
    }

    #[test]
    fn large_region_results_remain_valid_json_and_keep_stable_ids() {
        let regions: Vec<_> = (0..24)
            .map(|index| {
                json!({
                    "id": format!("scene.room.anno.{index}"),
                    "kind": "room",
                    "name": format!("room {index}"),
                    "points_xy": vec![index as f64; 300],
                    "stale": false,
                    "stale_reason": "",
                })
            })
            .collect();
        let original = json!({
            "regions": regions,
            "map_id": "3f_demo",
            "stamp_unix": 123.0,
        })
        .to_string();
        assert!(original.chars().count() > 4096);

        let compact = compact_tool_result("robonix/system/scene/list_regions", &original, 4096);
        let parsed: serde_json::Value = serde_json::from_str(&compact).unwrap();
        let compact_regions = parsed["regions"].as_array().unwrap();
        assert_eq!(compact_regions.len(), 24);
        assert_eq!(compact_regions[23]["id"], "scene.room.anno.23");
        assert_eq!(parsed["_robonix_truncation"]["complete_record_index"], true);
        assert!(compact.chars().count() <= 4096);
    }

    #[test]
    fn arbitrary_large_text_is_marked_incomplete_in_valid_json() {
        let compact = compact_tool_result("example/large", &"x".repeat(9000), 4096);
        let parsed: serde_json::Value = serde_json::from_str(&compact).unwrap();
        assert_eq!(parsed["_robonix_truncation"]["truncated"], true);
        assert_eq!(parsed["_robonix_truncation"]["complete"], false);
        assert!(compact.chars().count() <= 4096);
    }

    #[test]
    fn malformed_image_shape_is_bounded_as_text() {
        let original = json!({
            "width": 640,
            "height": 480,
            "encoding": "error",
            "data": "x".repeat(9000),
        })
        .to_string();
        assert!(!crate::history::is_image_output(&original));
        let compact = compact_tool_result("camera/snapshot", &original, 4096);
        assert!(compact.chars().count() <= 4096);
    }

    #[test]
    fn oversized_projected_scalar_and_escaped_preview_stay_bounded() {
        let original = json!({
            "regions": [{
                "id": format!("scene.room.{}", "\\\"\n".repeat(3000)),
                "kind": "room",
                "name": "large",
            }],
            "map_id": "demo",
        })
        .to_string();
        let compact = compact_tool_result("robonix/system/scene/list_regions", &original, 4096);
        serde_json::from_str::<serde_json::Value>(&compact).unwrap();
        assert!(compact.chars().count() <= 4096);
    }

    #[test]
    fn executor_snapshot_is_authoritative_across_turns() {
        let block = build_executor_active_block(Some(
            r#"{"count":99,"plans":[{"plan_id":"8","description":"greet","ops":[]}]}"#,
        ));
        assert!(block.contains("\"count\":1"));
        assert!(block.contains("\"plan_id\":\"8\""));
        assert!(block.contains("long-running skills started by earlier interactions"));
    }

    #[test]
    fn unavailable_executor_snapshot_forbids_guessing_zero() {
        let block = build_executor_active_block(None);
        assert!(block.contains("status: unavailable"));
        assert!(block.contains("Never guess a task count"));
    }

    #[test]
    fn only_requested_cancellation_replans_for_final_confirmation() {
        assert!(should_replan_after_plan_done(false, false, true, true));
        assert!(should_replan_after_plan_done(true, true, true, true));
        assert!(!should_replan_after_plan_done(true, true, false, true));
        assert!(!should_replan_after_plan_done(true, false, true, true));
        assert!(!should_replan_after_plan_done(true, true, true, false));
    }

    #[test]
    fn root_meta_ops_parse_without_becoming_rtdl_nodes() {
        assert_eq!(
            parse_meta_plan_op(&json!({"op":"cancel_plan","plan_id":"7"})).unwrap(),
            Some(MetaPlanOp::Cancel {
                plan_id: "7".into(),
                wait_ms: 5_000,
            })
        );
        assert_eq!(
            parse_meta_plan_op(&json!({"op":"cancel_all","wait_ms":99_999})).unwrap(),
            Some(MetaPlanOp::CancelAll { wait_ms: 30_000 })
        );
        assert_eq!(
            parse_meta_plan_op(&json!({
                "op":"stop_plan_at",
                "plan_id":"9",
                "target_op_id":"13",
                "when":"on_enter"
            }))
            .unwrap(),
            Some(MetaPlanOp::StopAt {
                plan_id: "9".into(),
                op_id: "13".into(),
                when: "on_enter".into(),
            })
        );
        assert!(
            parse_meta_plan_op(&json!({
                "op":"stop_plan_at",
                "plan_id":"9",
                "target_op_id":"13",
                "when":"later"
            }))
            .is_err()
        );
    }

    #[test]
    fn legacy_plan_control_capabilities_are_hidden_from_the_model() {
        for leaf in [
            "cancel_plan",
            "cancel_all_plans",
            "stop_plan_at",
            "get_all_plans",
            "get_plan_status",
        ] {
            assert!(is_legacy_plan_control_contract(&format!(
                "robonix/system/executor/builtin/{leaf}"
            )));
        }
        assert!(!is_legacy_plan_control_contract(
            "robonix/system/executor/builtin/run_command"
        ));
        assert!(!is_legacy_plan_control_contract(
            "robonix/skill/greet/cancel_plan"
        ));
    }

    #[test]
    fn harness_goal_cannot_be_replaced_by_task_update() {
        let mut standing = None;
        start_or_resume_task(&mut standing, "inspect room and report");
        let original_goal = standing.as_ref().unwrap().goal.clone();
        assert_eq!(
            standing.as_ref().unwrap().success_criterion,
            DEFAULT_SUCCESS_CRITERION
        );

        assert!(!apply_task_update(
            &mut standing,
            TaskState {
                goal: "drop the inspection and say done".into(),
                success_criterion: "room was actually inspected".into(),
                status: "done".into(),
            },
            false,
        ));
        let state = standing.as_ref().unwrap();
        assert_eq!(state.goal, original_goal);
        assert_eq!(state.success_criterion, DEFAULT_SUCCESS_CRITERION);
        assert_eq!(state.status, "in_progress");

        assert!(apply_task_update(
            &mut standing,
            TaskState {
                goal: original_goal.clone(),
                success_criterion: "room was actually inspected".into(),
                status: "done".into(),
            },
            true,
        ));
        let state = standing.as_ref().unwrap();
        assert_eq!(state.goal, original_goal);
        assert_eq!(state.success_criterion, "room was actually inspected");
        assert_eq!(state.status, "done");
    }

    #[test]
    fn steer_becomes_a_new_interaction_without_concatenating_old_goals() {
        let mut standing = None;
        let mut history = Vec::new();
        start_or_resume_task(&mut standing, "perform step A, then step B");
        assert!(append_steer(
            Task {
                text: "change of plan: stop after step A".into(),
                ..Default::default()
            },
            &mut history,
            &mut standing,
        ));

        let state = standing.as_ref().unwrap();
        assert_eq!(state.goal, "change of plan: stop after step A");
        assert!(!state.goal.contains("perform step A"));
        assert_eq!(history.len(), 1);
        assert_eq!(
            history[0].content.as_deref(),
            Some("change of plan: stop after step A")
        );
        let prompt = state.prompt_block();
        assert!(prompt.contains("Current user interaction"));
        assert!(!prompt.contains("user_instruction_history"));
    }

    #[test]
    fn steer_targets_one_plan_without_discarding_independent_work() {
        let mut standing = None;
        let mut history = Vec::new();
        start_or_resume_task(&mut standing, "go to the meeting room");

        let mut forest = HashMap::new();
        for (plan_id, description, capability) in [
            ("11", "navigate to the meeting room", "navigation_navigate"),
            ("5", "watch for passersby", "greet_greet"),
        ] {
            forest.insert(
                plan_id.to_string(),
                TreeMeta {
                    description: description.into(),
                    control_only: false,
                    call_signatures: HashSet::new(),
                    steps: vec![TreeStep {
                        op_id: format!("op-{plan_id}"),
                        description: description.into(),
                        capability: capability.into(),
                    }],
                },
            );
        }

        assert!(append_steer(
            Task {
                text: "cancel the meeting-room trip and return to room 315".into(),
                ..Default::default()
            },
            &mut history,
            &mut standing,
        ));
        assert_eq!(
            standing.as_ref().unwrap().goal,
            "cancel the meeting-room trip and return to room 315"
        );

        let prompt = build_forest_block(&forest, &HashSet::new());
        assert!(prompt.contains("plan_id=11"));
        assert!(prompt.contains("plan_id=5"));
        assert!(prompt.contains("navigate to the meeting room"));
        assert!(prompt.contains("watch for passersby"));
        assert_eq!(
            invalid_cancel_target(&["11".into()], &forest, &HashSet::new()),
            None
        );
        assert_eq!(
            invalid_cancel_target(&["5".into()], &forest, &HashSet::new()),
            None
        );

        let requested = HashSet::from(["11".to_string()]);
        assert_eq!(
            invalid_cancel_target(&["11".into()], &forest, &requested),
            Some("11".to_string())
        );
        assert_eq!(
            invalid_cancel_target(&["5".into()], &forest, &requested),
            None
        );
    }

    #[test]
    fn forest_prompt_distinguishes_immediate_cancel_from_boundary_stop() {
        let mut forest = HashMap::new();
        forest.insert(
            "4".to_string(),
            TreeMeta {
                description: "ordered multi-step task".into(),
                control_only: false,
                call_signatures: HashSet::new(),
                steps: vec![
                    TreeStep {
                        op_id: "op-restaurant".into(),
                        description: "move to restaurant".into(),
                        capability: "navigate".into(),
                    },
                    TreeStep {
                        op_id: "op-meeting".into(),
                        description: "move to meeting room".into(),
                        capability: "navigate".into(),
                    },
                ],
            },
        );
        let prompt = build_forest_block(&forest, &HashSet::new());
        assert!(prompt.contains("op_id=op-restaurant"));
        assert!(prompt.contains("move to meeting room"));
        assert!(prompt.contains("target is the currently running step"));
        assert!(prompt.contains("do not query status"));
        assert!(prompt.contains("on_complete"));
        assert!(prompt.contains("on_enter"));
    }

    #[test]
    fn executor_feedback_is_scoped_to_its_independent_tree() {
        let mut history = Vec::new();
        feed_results_into_history(
            &mut history,
            "9",
            "start greet watch",
            &[CapabilityCallResult {
                call_id: "9:0".into(),
                contract_id: "robonix/skill/greet/greet".into(),
                success: false,
                error: "activation failed".into(),
                ..Default::default()
            }],
        );
        let scope = history[0].content.as_deref().unwrap_or_default();
        assert!(scope.contains("plan_id=9"));
        assert!(scope.contains("start greet watch"));
        assert!(scope.contains("does not cancel or invalidate other in-flight trees"));
    }

    #[test]
    fn completed_plan_context_preserves_original_call_before_replanning() {
        let original_target = 1.4430711285352669;
        let plan = Plan {
            plan_id: "5".into(),
            nodes: vec![RtdlNode {
                node_kind: RTDL_DO,
                op_id: "6".into(),
                description: "navigate to the original one-metre target".into(),
                call: Some(CapabilityCall {
                    call_id: "5:0".into(),
                    provider_id: "nav2".into(),
                    contract_id: "robonix/service/navigation/navigate".into(),
                    args_json: json!({
                        "goal": {
                            "header": {"frame_id": "map"},
                            "pose": {"position": {"x": original_target, "y": -0.0019468723}}
                        }
                    })
                    .to_string(),
                }),
                ..Default::default()
            }],
            ..Default::default()
        };
        let mut history = Vec::new();
        record_dispatched_plan(&mut history, &plan, "move forward one metre");
        feed_results_into_history(
            &mut history,
            "5",
            "move forward one metre",
            &[CapabilityCallResult {
                call_id: "5:0".into(),
                contract_id: "robonix/service/navigation/navigate".into(),
                success: true,
                output: r#"{"state":"SUCCEEDED","detail":"last_pose=(1.160,-0.050)"}"#.into(),
                ..Default::default()
            }],
        );

        let visible = crate::history::sanitize_for_vlm(&history);
        let context = visible
            .iter()
            .filter_map(|message| message.content.as_deref())
            .collect::<Vec<_>>()
            .join("\n");
        assert!(context.contains("Pilot harness dispatch record"));
        assert!(context.contains("\"plan_id\":\"5\""));
        assert!(context.contains("\"call_id\":\"5:0\""));
        assert!(context.contains(&original_target.to_string()));
        assert!(context.contains("SUCCEEDED"));
    }

    #[test]
    fn duplicate_in_flight_calls_are_detected_by_canonical_signature() {
        let plan = Plan {
            plan_id: "2".into(),
            nodes: vec![RtdlNode {
                node_kind: RTDL_DO,
                call: Some(CapabilityCall {
                    provider_id: "executor".into(),
                    contract_id: "test/run".into(),
                    args_json: r#"{"b":2,"a":1}"#.into(),
                    ..Default::default()
                }),
                ..Default::default()
            }],
            ..Default::default()
        };
        let signatures = plan_call_signatures(&plan);
        let mut forest = HashMap::new();
        forest.insert(
            "1".to_string(),
            TreeMeta {
                description: "same call".into(),
                control_only: false,
                call_signatures: signatures.clone(),
                steps: Vec::new(),
            },
        );
        assert!(duplicate_in_flight_signature(&signatures, &forest).is_some());
    }

    #[test]
    fn inspection_result_must_arrive_before_new_action_is_admitted() {
        let plan = Plan {
            nodes: vec![
                RtdlNode {
                    node_kind: RTDL_DO,
                    call: Some(CapabilityCall {
                        contract_id: "robonix/system/executor/builtin/get_plan_status".into(),
                        ..Default::default()
                    }),
                    ..Default::default()
                },
                RtdlNode {
                    node_kind: RTDL_DO,
                    call: Some(CapabilityCall {
                        contract_id: "robonix/system/executor/builtin/run_command".into(),
                        ..Default::default()
                    }),
                    ..Default::default()
                },
            ],
            ..Default::default()
        };
        assert!(mixes_control_inspection_with_action(&plan));

        let inspection_only = Plan {
            nodes: vec![plan.nodes[0].clone()],
            ..Default::default()
        };
        assert!(!mixes_control_inspection_with_action(&inspection_only));
    }

    #[test]
    fn cancel_target_must_be_live_and_not_already_requested() {
        let mut forest = HashMap::new();
        forest.insert(
            "7".to_string(),
            TreeMeta {
                description: "drive".into(),
                control_only: false,
                call_signatures: HashSet::new(),
                steps: Vec::new(),
            },
        );
        let targets = vec!["7".to_string()];
        assert!(invalid_cancel_target(&targets, &forest, &HashSet::new()).is_none());
        assert_eq!(
            invalid_cancel_target(&targets, &forest, &HashSet::from(["7".to_string()])),
            Some("7".to_string())
        );
        assert_eq!(
            invalid_cancel_target(&["8".to_string()], &forest, &HashSet::new()),
            Some("8".to_string())
        );
    }

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

    fn single_do_plan(contract_leaf: &str) -> Plan {
        Plan {
            plan_id: "p".into(),
            session_id: "s".into(),
            round: 0,
            root_index: 0,
            nodes: vec![RtdlNode {
                node_kind: RTDL_DO,
                children: vec![],
                call: Some(CapabilityCall {
                    call_id: "p:0".into(),
                    provider_id: "executor".into(),
                    contract_id: format!("robonix/system/executor/builtin/{contract_leaf}"),
                    args_json: "{}".into(),
                }),
                op_id: "op_1".into(),
                description: "control action".into(),
            }],
        }
    }

    #[test]
    fn plan_control_builtins_are_control_only() {
        for leaf in [
            "cancel_plan",
            "cancel_all_plans",
            "get_all_plans",
            "get_plan_status",
            "stop_plan_at",
        ] {
            assert!(is_control_only(&single_do_plan(leaf)), "{leaf}");
        }
        assert!(!is_control_only(&single_do_plan("list_dir")));
    }

    #[test]
    fn rtdl_state_names_are_human_readable() {
        assert_eq!(rtdl_state_name(0), "Pending");
        assert_eq!(rtdl_state_name(2), "Succeeded");
        assert_eq!(rtdl_state_name(3), "Failed");
        assert_eq!(rtdl_state_name(4), "Canceled");
        assert_eq!(rtdl_state_name(5), "Timeout");
        assert_eq!(rtdl_state_name(999), "Unknown(999)");
    }

    #[test]
    fn rtdl_node_kind_names_are_human_readable() {
        assert_eq!(rtdl_node_kind_name(RTDL_SEQUENCE), "sequence");
        assert_eq!(rtdl_node_kind_name(RTDL_PARALLEL), "parallel");
        assert_eq!(rtdl_node_kind_name(RTDL_DO), "do");
        assert_eq!(rtdl_node_kind_name(99), "unknown(99)");
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
    fn rtdl_response_tolerates_prose_preamble() {
        // Observed real failure: the model narrates a line, then emits the JSON
        // on the next line. The leading prose must be stripped, not rejected.
        let env = parse_rtdl_assistant_response(
            "Let me take a photo to check the scene, then turn left.\n{\"content\":\"on it\",\"rtdl_description\":\"turn\",\"rtdl\":{\"op\":\"sequence\",\"children\":[]},\"task_update\":null}",
        )
        .unwrap();
        assert_eq!(env.content, "on it");
        assert!(env.task_update.is_none());
    }

    #[test]
    fn extract_json_object_skips_prose_and_braces_in_strings() {
        // Leading prose dropped; a `}` inside a string value does not end it.
        let got = extract_json_object("hi: {\"a\":\"x}y\",\"b\":1} trailing");
        assert_eq!(got, Some("{\"a\":\"x}y\",\"b\":1}"));
        // No object at all → None, so the caller still hits the real parse error.
        assert_eq!(extract_json_object("no json here"), None);
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
    fn format_plan_summary_uses_tree_shape_and_compact_cap_names() {
        let mut targets = CapabilityTargetMap::new();
        targets.insert(
            "nav2.navigation_status".to_string(),
            (
                "nav2".to_string(),
                "robonix/service/navigation/status".to_string(),
            ),
        );
        let rtdl = json!({
            "op": "sequence",
            "description": "poll navigation status",
            "children": [
                {
                    "op": "do",
                    "description": "check current navigation goal",
                    "cap": "nav2.navigation_status",
                    "args": { "goal_id": "" }
                }
            ]
        });
        let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 1, "").unwrap();
        let summary = format_plan_summary(&plan).join("\n");

        assert!(summary.contains("[0] sequence"));
        assert!(summary.contains("[1] do"));
        assert!(summary.contains("cap=nav2.navigation_status"));
        assert!(summary.contains(r#"args={"goal_id":""}"#));
        assert!(summary.contains("  [1] do"));
        assert!(!summary.contains("kind="));
        assert!(!summary.contains("state="));
        assert!(!summary.contains("children"));
        assert!(!summary.contains("robonix/service/navigation/status"));
        assert!(!summary.contains("call_id"));
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
        assert!(err.to_string().contains("unexpected field `out`"));
    }

    #[test]
    fn rtdl_uses_model_node_description_over_synthesized() {
        let mut targets = CapabilityTargetMap::new();
        targets.insert(
            "camera_snapshot".to_string(),
            (
                "cap-camera".to_string(),
                "robonix/primitive/camera/snapshot".to_string(),
            ),
        );
        // Every node carries op_id (always 0 — pilot reassigns) plus a
        // model-authored node-level description.
        let rtdl = json!({
            "op": "sequence",
            "op_id": 0,
            "description": "inspect the doorway",
            "children": [
                {
                    "op": "do",
                    "op_id": 0,
                    "description": "take a camera snapshot of the door",
                    "cap": "camera_snapshot",
                    "args": {}
                }
            ]
        });
        let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0, "").unwrap();
        assert_eq!(plan.nodes[0].description, "inspect the doorway");
        assert_eq!(
            plan.nodes[1].description,
            "take a camera snapshot of the door"
        );
        // The model's op_id=0 is ignored; pilot assigns non-empty unique ids.
        assert!(!plan.nodes[0].op_id.is_empty());
        assert_ne!(plan.nodes[0].op_id, plan.nodes[1].op_id);
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
