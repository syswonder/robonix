// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// The RTDL forest: trees dispatched to Executor and still running, how they are
// driven and cancelled, and how their state is shown to the model.

use super::*;

/// Metadata for one in-flight RTDL tree in the forest. Trees are keyed by
/// pilot-assigned `plan_id`; this carries what the supervisor and the LLM need
/// to reason about a running tree (and, later, what the chat UI renders).
pub(super) struct TreeMeta {
    /// LLM-supplied `rtdl_description` (sub-task label).
    pub(super) description: String,
    /// True when this tree is purely control ops (only `cancel_plan` /
    /// `cancel_all_plans`). Such trees are NOT advertised to the LLM as
    /// cancellable in-flight work — a cancel is not itself a task tree, and
    /// listing it makes the model cancel its own cancels in a loop.
    pub(super) control_only: bool,
    /// Canonical provider/contract/args signatures for calls in this tree.
    /// The harness rejects a second tree containing an identical call while
    /// the first is still in flight; execution latency must not duplicate a
    /// physical or external command.
    pub(super) call_signatures: HashSet<String>,
    /// Ordered executable leaves from the original RTDL graph. Keeping these
    /// visible lets the model target any semantic boundary in one control call
    /// instead of querying live state first or guessing what "current" means.
    pub(super) steps: Vec<TreeStep>,
}

pub(super) struct TreeStep {
    pub(super) op_id: String,
    pub(super) description: String,
    pub(super) capability: String,
}

/// Events fed from per-tree driver tasks back to the supervisor loop. One
/// `drive_plan` task runs per dispatched tree and streams these.
pub(super) enum ForestEvent {
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
pub(super) async fn drive_plan(
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
                        upsert_terminal_result(&mut results, ns);
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

/// Keep one latest terminal state per RTDL node so verification can correct it.
pub(super) fn upsert_terminal_result(results: &mut Vec<RtdlNodeState>, state: RtdlNodeState) {
    if let Some(existing) = results
        .iter_mut()
        .find(|existing| existing.node_index == state.node_index)
    {
        *existing = state;
    } else {
        results.push(state);
    }
}

/// Cancel every real task tree owned by this turn before reporting the Pilot
/// session interrupted. Dropping the Execute stream alone only detaches Pilot;
/// Executor continues the plan (and synchronous tools such as run_command)
/// unless its PlanRuntime receives an explicit cancel_plan request.
pub(super) async fn cancel_forest_plans(
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

pub(super) fn plan_steps(plan: &Plan) -> Vec<TreeStep> {
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

pub(super) fn build_forest_block(
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

pub(super) fn build_executor_active_block(plans_json: Option<&str>) -> String {
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

pub(super) async fn fetch_executor_active_block(executor: &mut ExecutorConn) -> String {
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

pub(super) fn should_replan_after_plan_done(
    canceled: bool,
    requested_cancellation: bool,
    cancellation_batch_complete: bool,
    interaction_active: bool,
) -> bool {
    interaction_active && (!canceled || (requested_cancellation && cancellation_batch_complete))
}

pub(super) fn is_terminal_executor_state(state: u32) -> bool {
    matches!(
        RtdlNodeStateEnum::try_from(state as i32),
        Ok(RtdlNodeStateEnum::Succeeded
            | RtdlNodeStateEnum::Failed
            | RtdlNodeStateEnum::Canceled
            | RtdlNodeStateEnum::Timeout)
    )
}
