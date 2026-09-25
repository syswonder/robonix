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
use crate::prompt::{
    CatalogView, UsageTotals, assemble_planning_messages, capability_entries,
    render_capability_docs, render_context_sections, render_full_catalog,
};
use crate::service::{self, PilotStreamBody, SessionState};
use crate::state_context;
use crate::vlm::{Message, ReplyShape, VlmClient, VlmStreamItem};
use anyhow::{Context, Result};
use futures_util::StreamExt;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{debug, info, warn};
use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;
use tokio::sync::{mpsc, watch};
use tonic::Request;
use tonic::transport::Channel;

mod catalog;
mod compaction;
mod control;
mod feedback;
mod forest;
mod rtdl;
mod system_prompt;
mod task;
#[cfg(test)]
mod tests;

pub(crate) use catalog::DisplayCapability;
use catalog::*;
use compaction::*;
pub use compaction::{CONTEXT_RESERVE_TOKENS, HistoryBudget};
use control::*;
use feedback::*;
use forest::*;
use rtdl::*;
use system_prompt::*;
pub(crate) use task::TaskState;
use task::*;

/// gRPC client for executor's plan-dispatch contract. Pilot only ever calls
/// `Execute(Plan)` — discovery happens directly against atlas now.
pub struct ExecutorConn {
    pub graph: RobonixSystemExecutorExecuteClient<Channel>,
    pub control: RobonixSystemExecutorControlPlanClient<Channel>,
    pub active: RobonixSystemExecutorListActivePlansClient<Channel>,
}

/// Persist exactly the user-side suffix sent for a completed planning request.
/// This makes the next request an extension of the actual prior prompt:
/// `system -> history -> runtime context -> assistant -> new runtime context`.
fn append_request_context(
    history: &mut Vec<Message>,
    runtime_context: &str,
    correction: Option<&str>,
) {
    if !runtime_context.is_empty() {
        history.push(Message::user(runtime_context));
    }
    if let Some(correction) = correction {
        history.push(Message::user(correction));
    }
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
    history_budget: &HistoryBudget,
) -> Result<()> {
    let session_id = task.session_id.clone();

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

    // 1. Pilot's capability catalog comes straight from Atlas. MCP params ride
    // along in Capability.params, and contract metadata below decides which
    // of those capabilities the planning model may see; no Connect is needed.
    let initial_caps = discovery::discover(atlas)
        .await
        .map_err(|e| anyhow::anyhow!("atlas capability discovery failed: {e}"))?;
    // Contract metadata is immutable for one Atlas process, so resolve the
    // model-facing exclusions once per turn. This does not affect Executor or
    // any other Atlas consumer's ability to resolve and call the capability.
    let non_llm_callable_contract_ids = discovery::non_llm_callable_contract_ids(atlas)
        .await
        .map_err(|e| anyhow::anyhow!("atlas contract discovery failed: {e}"))?;

    // Build deployment-independent system instructions once per turn.
    // Provider-specific semantics belong to the discovered capability catalog
    // and lazily loaded provider documentation, not Pilot's standing prefix.
    let standing_prompt = build_system_prompt(load_agent_soul().as_deref());
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

    // 2. Preserve the user goal verbatim in the ordered history. The label is
    // also an explicit retention anchor: bounded working history may discard
    // stale narration, but never this task record.
    history.push(Message::user(&format!(
        "User task (authoritative): {}\n{}",
        task.text,
        response_mode(task)
    )));
    if !memory_prompt.is_empty() {
        history.push(Message::user(&memory_prompt));
    }
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
    let mut catalog_view = CatalogView::default();
    let mut usage_totals = UsageTotals::default();
    let mut cache_epoch = 0_u64;
    let mut last_soma_body = String::new();
    let mut last_capability_docs = String::new();
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
                            // VERIFYING is deliberately non-terminal. Feed only a
                            // leaf's post-verification final state so history never
                            // contains an optimistic result that must be corrected.
                            if is_terminal_executor_state(ns.state)
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
                            }
                            // Leaf results were already upserted per node event.
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

        // Re-discover capabilities from atlas every round so providers that
        // registered mid-turn are visible in the next call.
        let cap_list = discovery::discover(atlas)
            .await
            .map_err(|e| anyhow::anyhow!("atlas capability discovery failed: {e}"))?;

        let embodiment_block =
            crate::soma_context::fetch_runtime_prompt_block(atlas, consumer_id).await;
        let environment_block = state_context::collect(executor, atlas, &cap_list).await;
        let soma_body = crate::soma_context::fetch_system_prompt_block(atlas, consumer_id)
            .await
            .ok()
            .flatten()
            .unwrap_or_else(|| {
                "\n\n## Robot Body Context (from Soma)\nUnavailable; any earlier body description may be stale.\n".to_string()
            });
        let capability_docs = discovery::cap_md_index(atlas)
            .await
            .map(|docs| render_capability_docs(&docs))
            .unwrap_or_else(|_| {
                "\n\n## Capability docs\nUnavailable; use the current capability catalog only.\n"
                    .to_string()
            });
        let display_caps = build_display_capabilities(&cap_list, &non_llm_callable_contract_ids);
        let target_map = build_capability_target_map(&display_caps);
        // The RTDL protocol is part of the cacheable system prefix. Starting
        // with its compact form avoids the old full-on-round-zero rewrite that
        // made the second provider request cold.
        let protocol_prompt = rtdl_protocol(false);
        let forest_block = build_forest_block(&forest, &cancel_requested);
        let executor_active_block = fetch_executor_active_block(executor).await;
        // Only instructions that never change sit before history.
        let sections = [
            ("standing_system", standing_prompt.as_str()),
            ("rtdl_protocol", protocol_prompt),
        ];
        // Budget as if every snapshot were shown in full: after a compaction
        // they all are.
        let full_catalog = render_full_catalog(&capability_entries(&display_caps));
        let compaction_non_history_tokens = [
            standing_prompt.as_str(),
            protocol_prompt,
            full_catalog.as_str(),
            soma_body.as_str(),
            capability_docs.as_str(),
            forest_block.as_str(),
            executor_active_block.as_str(),
            embodiment_block.as_str(),
            environment_block.as_str(),
        ]
        .iter()
        .map(|content| content.len().div_ceil(4))
        .sum();
        if compact_history(
            history,
            vlm,
            history_budget,
            compaction_non_history_tokens,
            cache_epoch,
        )
        .await
        {
            cache_epoch = cache_epoch.saturating_add(1);
            // The summary may have absorbed the last full snapshots; show
            // them again rather than send changes against a missing baseline.
            catalog_view.reset();
            last_soma_body.clear();
            last_capability_docs.clear();
        }
        let catalog_update = catalog_view.update(&display_caps);
        let soma_body_update = if soma_body == last_soma_body {
            ""
        } else {
            soma_body.as_str()
        };
        let capability_docs_update = if capability_docs == last_capability_docs {
            ""
        } else {
            capability_docs.as_str()
        };
        // Current observations follow history, in a fixed order.
        let live_sections = [
            ("capability_catalog", catalog_update.as_str()),
            ("embodiment_description", soma_body_update),
            ("capability_docs", capability_docs_update),
            ("in_flight_trees", forest_block.as_str()),
            ("executor_state", executor_active_block.as_str()),
            ("embodiment_live", embodiment_block.as_str()),
            ("environment_live", environment_block.as_str()),
        ];
        let runtime_context = render_context_sections(&live_sections);
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
            let mut request_history = history.clone();
            if !runtime_context.is_empty() {
                request_history.push(Message::user(&runtime_context));
            }
            if let Some(correction) = correction.as_deref() {
                request_history.push(Message::user(correction));
            }
            let messages = assemble_planning_messages(
                round,
                catalog_update.is_empty(),
                &sections,
                &request_history,
            );

            let planning_revision = forest_revision.load(Ordering::Acquire);
            let mut vlm_attempt = 0_u8;
            let (content, raw_tool_calls) = loop {
                let mut stream = match tokio::time::timeout(
                    vlm_idle_timeout(),
                    vlm.chat_stream(
                        &messages,
                        &[],
                        Some(vlm.prompt_cache_key()),
                        ReplyShape::RtdlEnvelope,
                    ),
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
                                    usage_totals.record(round, cache_epoch, &usage)
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

        // Persist the sent context before its reply to preserve prefix reuse.
        append_request_context(history, &runtime_context, correction.as_deref());
        // Only now have these snapshots reached history; later rounds send
        // what changes against them.
        catalog_view.commit();
        last_soma_body = soma_body;
        last_capability_docs = capability_docs;

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
                should_plan = true;
                continue 'supervisor;
            }

            let task_state_changed = if let Some(updated) = task_update {
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
                changed
            } else {
                false
            };
            if !assistant_content.trim().is_empty() {
                history.push(Message::assistant(&assistant_content));
                last_content = assistant_content.clone();
                let _ = tx
                    .send(Ok(service::pack(
                        &session_id,
                        PilotStreamBody::TextChunk(assistant_content),
                    )))
                    .await;
            }
            if task_state_changed && let Some(state) = standing_task.as_ref() {
                append_task_state_record(history, state);
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
            should_plan = true;
            continue 'supervisor;
        }
        if let Some(target) = invalid_cancel_target(&cancel_targets, &forest, &cancel_requested) {
            warn!("[pilot/harness] suppressed stale or duplicate cancel for plan {target}");
            history.push(Message::user(
                "Pilot harness feedback: that legacy cancel target is not cancellable now. Re-read In-flight trees and use one root plan-control meta op; do not retry a finished target or create a cancel RTDL tree.",
            ));
            should_plan = true;
            continue 'supervisor;
        }
        if let Some(duplicate) = duplicate_in_flight_signature(&call_signatures, &forest) {
            warn!("[pilot/harness] suppressed duplicate in-flight call: {duplicate}");
            history.push(Message::user(
                "Pilot harness feedback: that exact capability call is already in flight. Do not dispatch or cancel it again; wait for its result.",
            ));
            should_plan = false;
            continue 'supervisor;
        }

        // Apply progress only after the harness knows whether this response can
        // safely finish. A model cannot mark a task done while it is also
        // dispatching work or while an older tree remains in flight.
        let task_state_changed = if let Some(updated) = task_update {
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
            changed
        } else {
            false
        };

        log_plan_start(&graph, &rtdl_description, round, calls);

        // Retain model narration for later planning. Action-producing RTDL
        // rounds are also streamed below so the current user sees and hears
        // progress instead of receiving only a final burst after a long task.
        if !assistant_content.is_empty() {
            history.push(Message::assistant(&assistant_content));
            last_content = assistant_content.clone();
        }
        if task_state_changed && let Some(state) = standing_task.as_ref() {
            append_task_state_record(history, state);
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
