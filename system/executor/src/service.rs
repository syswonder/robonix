// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// gRPC contract handlers for executor plan execution and cancellation.

use crate::dispatch::{async_poll, async_registry};
use crate::pb::contracts::robonix_system_executor_cancel_all_plans_server::RobonixSystemExecutorCancelAllPlans;
use crate::pb::contracts::robonix_system_executor_control_plan_server::RobonixSystemExecutorControlPlan;
use crate::pb::contracts::robonix_system_executor_execute_server::RobonixSystemExecutorExecute;
use crate::pb::contracts::robonix_system_executor_get_health_server::RobonixSystemExecutorGetHealth;
use crate::pb::contracts::robonix_system_executor_list_active_plans_server::RobonixSystemExecutorListActivePlans;
use crate::pb::executor::{
    CancelAllResponse, ControlPlanResponse, ListActivePlansResponse, RtdlEvent,
};
use crate::pb::module_health::{
    GetModuleHealthRequest, GetModuleHealthResponse, ModuleHealth, ModuleHealthReport,
};
use crate::pb::pilot::rtdl_node_state::RtdlNodeStateEnum;
use crate::pb::pilot::{CapabilityCall, CapabilityCallResult, Plan};
use crate::plan_runtime::{PlanRuntime, StopWhen};
use crate::rtdl_wire::{self, NodeEventContext};
use crate::verification::{self, VerificationPolicy};
use robonix_atlas::client::AtlasClient;
use robonix_scribe::{info, warn};
use std::collections::HashSet;
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use std::time::Instant;
use tokio::sync::mpsc::Sender;
use tokio::task::JoinHandle;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

const RTDL_SEQUENCE: u32 = 0;
const RTDL_PARALLEL: u32 = 1;
const RTDL_DO: u32 = 2;
const RTDL_IF: u32 = 3;
const RTDL_REPEAT: u32 = 4;
const RTDL_WHILE: u32 = 5;
const MAX_ITERATIONS: u32 = 10_000;
const MODULE_HEALTH_SCHEMA_VERSION: u32 = 1;
const MODULE_HEALTH_OK: u32 = 0;
const MODULE_HEALTH_TTL_MS: u32 = 5000;

/// Immediate node outcome plus overlap verifiers owned by its RTDL subtree.
struct NodeExecution {
    failed: bool,
    verifiers: Vec<JoinHandle<bool>>,
}

impl NodeExecution {
    /// Build an outcome with no background verification still pending.
    fn finished(failed: bool) -> Self {
        Self {
            failed,
            verifiers: Vec::new(),
        }
    }
}

/// Wait for a subtree's verifiers and report whether any one failed.
async fn wait_for_verifiers(verifiers: Vec<JoinHandle<bool>>) -> bool {
    let mut any_failed = false;
    for verifier in verifiers {
        match verifier.await {
            Ok(failed) => any_failed |= failed,
            Err(error) => {
                any_failed = true;
                warn!("[executor/verification] overlap verifier failed to join: {error}");
            }
        }
    }
    any_failed
}

/// Publish an operator immediately when final, or defer its final state until
/// every verifier in its subtree has completed.
#[allow(clippy::too_many_arguments)]
async fn complete_operator(
    verifiers: Vec<JoinHandle<bool>>,
    immediate_failed: bool,
    cancelled: bool,
    tx: Sender<Result<RtdlEvent, Status>>,
    node: NodeEventContext,
    runtime: PlanRuntime,
    atlas: AtlasClient,
    provider_id: String,
) -> NodeExecution {
    if verifiers.is_empty() {
        let failed = immediate_failed || cancelled;
        send_operator_final(&tx, &node, failed, cancelled, &runtime).await;
        return NodeExecution::finished(failed);
    }

    send_operator_state(
        &tx,
        &node,
        RtdlNodeStateEnum::Verifying as u32,
        "is waiting for descendant verification",
        &runtime,
    )
    .await;
    let known_failed = immediate_failed || cancelled;
    let verifier = tokio::spawn(async move {
        let verification_failed = wait_for_verifiers(verifiers).await;
        let cancelled = cancelled || runtime.is_cancelled(&node.plan_id).await;
        let failed = immediate_failed || verification_failed || cancelled;
        send_operator_final(&tx, &node, failed, cancelled, &runtime).await;
        trigger_stop_on_complete(&runtime, &node, &provider_id, atlas).await;
        failed
    });
    NodeExecution {
        failed: known_failed,
        verifiers: vec![verifier],
    }
}

/// Owned inputs for one background verifier.
struct OverlapVerifier {
    call: CapabilityCall,
    node: NodeEventContext,
    tx: Sender<Result<RtdlEvent, Status>>,
    atlas: AtlasClient,
    provider_id: String,
    runtime: PlanRuntime,
    policy: Arc<VerificationPolicy>,
    original: CapabilityCallResult,
    started_at: Instant,
}

/// `AtlasClient` is cheap to clone — each Execute RPC clones it so per-plan
/// dispatch runs without serialising on a single mutex.
#[derive(Clone)]
pub struct ExecutorServiceImpl {
    atlas: AtlasClient,
    /// Executor's own provider_id. Two roles:
    ///   1. consumer_id passed to atlas on every ConnectCapability so the
    ///      channel record reflects who is using each downstream provider.
    ///   2. self-detection: when a CapabilityCall in the plan targets this
    ///      provider_id, dispatch short-circuits to the in-process builtin
    ///      handlers instead of going through MCP loopback.
    provider_id: String,
    runtime: PlanRuntime,
    verification: Arc<VerificationPolicy>,
}

impl ExecutorServiceImpl {
    pub fn new(
        atlas: AtlasClient,
        provider_id: String,
        verification: Arc<VerificationPolicy>,
    ) -> Self {
        Self {
            atlas,
            provider_id,
            runtime: PlanRuntime::default(),
            verification,
        }
    }
}

#[tonic::async_trait]
impl RobonixSystemExecutorExecute for ExecutorServiceImpl {
    type ExecuteStream = ReceiverStream<Result<RtdlEvent, Status>>;

    async fn execute(
        &self,
        request: Request<Plan>,
    ) -> Result<Response<Self::ExecuteStream>, Status> {
        let plan = request.into_inner();
        if let Err(error) = validate_plan(&plan) {
            warn!(
                "[executor/rtdl] plan_id={} plan rejected: {}",
                plan.plan_id, error
            );
            return Err(Status::invalid_argument(error));
        }
        let (tx, rx) = tokio::sync::mpsc::channel(64);
        let atlas = self.atlas.clone();
        let provider_id = self.provider_id.clone();
        let runtime = self.runtime.clone();
        let verification = Arc::clone(&self.verification);

        tokio::spawn(async move {
            let started_at = Instant::now();
            let plan_id = plan.plan_id.clone();
            let call_count = plan.nodes.iter().filter(|node| node.call.is_some()).count();
            info!(
                "[executor/rtdl] plan_id={} session_id={} round={} plan start root={} nodes={} calls={}",
                plan_id,
                plan.session_id,
                plan.round,
                plan.root_index,
                plan.nodes.len(),
                call_count
            );
            let plan = Arc::new(plan);
            runtime.register_plan(&plan_id).await;
            runtime.record_plan_ops(&plan).await;
            let _ = tx.send(Ok(rtdl_wire::plan_started(plan_id.clone()))).await;
            let execution = execute_node(
                Arc::clone(&plan),
                plan.root_index as usize,
                tx.clone(),
                atlas,
                provider_id,
                runtime.clone(),
                verification,
            )
            .await;
            let verification_failed = wait_for_verifiers(execution.verifiers).await;
            let cancelled = runtime.is_cancelled(&plan_id).await;
            runtime.complete_plan(&plan_id).await;

            let any_failed = execution.failed || verification_failed || cancelled;
            let state = if cancelled {
                "Canceled"
            } else if any_failed {
                "Failed"
            } else {
                "Succeeded"
            };
            info!(
                "[executor/rtdl] plan_id={} plan end state={} duration_ms={} execution_failed={} verification_failed={} canceled={}",
                plan_id,
                state,
                started_at.elapsed().as_millis(),
                execution.failed,
                verification_failed,
                cancelled
            );

            let _ = tx
                .send(Ok(rtdl_wire::plan_complete(
                    plan_id,
                    any_failed,
                )))
                .await;
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

type ExecuteNodeFuture = Pin<Box<dyn Future<Output = NodeExecution> + Send + 'static>>;

fn execute_node(
    plan: Arc<Plan>,
    node_index: usize,
    tx: Sender<Result<RtdlEvent, Status>>,
    atlas: AtlasClient,
    provider_id: String,
    runtime: PlanRuntime,
    verification: Arc<VerificationPolicy>,
) -> ExecuteNodeFuture {
    Box::pin(async move {
        let node = &plan.nodes[node_index];
        let node_ctx = node_event_context(&plan, node_index);
        let op_id = node.op_id.clone();
        info!(
            "[executor/rtdl] plan_id={} node={} op_id={} kind={} enter children={} desc='{}'",
            plan.plan_id,
            node_index,
            op_id,
            rtdl_node_kind_name(node.node_kind),
            node.children.len(),
            log_preview(&node.description, 160)
        );
        if runtime.is_cancelled(&plan.plan_id).await {
            info!(
                "[executor/rtdl] plan_id={} node={} op_id={} kind={} skipped reason='plan canceled before entry'",
                plan.plan_id,
                node_index,
                op_id,
                rtdl_node_kind_name(node.node_kind)
            );
            if is_operator_node(node.node_kind) {
                send_operator_state(
                    &tx,
                    &node_ctx,
                    RtdlNodeStateEnum::Canceled as u32,
                    "canceled",
                    &runtime,
                )
                .await;
            }
            return NodeExecution::finished(true);
        }
        if runtime
            .should_stop_at(&plan.plan_id, &op_id, StopWhen::OnEnter)
            .await
        {
            let mut atlas = atlas;
            runtime
                .trigger_stop(&plan.plan_id, &provider_id, &mut atlas)
                .await;
            send_stop_on_enter(&tx, &node_ctx, &runtime).await;
            return NodeExecution::finished(true);
        }
        let atlas_after = atlas.clone();
        let execution = match node.node_kind {
            RTDL_SEQUENCE => {
                let mut any_failed = false;
                let mut cancelled = false;
                let mut verifiers = Vec::new();
                for child in &node.children {
                    if runtime.is_cancelled(&plan.plan_id).await {
                        cancelled = true;
                        any_failed = true;
                        break;
                    }
                    let child_execution = execute_node(
                        Arc::clone(&plan),
                        *child as usize,
                        tx.clone(),
                        atlas.clone(),
                        provider_id.clone(),
                        runtime.clone(),
                        verification.clone(),
                    )
                    .await;
                    any_failed |= child_execution.failed;
                    verifiers.extend(child_execution.verifiers);
                    if any_failed {
                        break;
                    }
                }
                let cancelled = cancelled || runtime.is_cancelled(&plan.plan_id).await;
                complete_operator(
                    verifiers,
                    any_failed,
                    cancelled,
                    tx.clone(),
                    node_ctx.clone(),
                    runtime.clone(),
                    atlas.clone(),
                    provider_id.clone(),
                )
                .await
            }
            RTDL_PARALLEL => {
                let mut handles = Vec::with_capacity(node.children.len());
                for child in &node.children {
                    let child_plan = Arc::clone(&plan);
                    let child_tx = tx.clone();
                    let child_atlas = atlas.clone();
                    let child_provider_id = provider_id.clone();
                    let child_runtime = runtime.clone();
                    let child_verification = verification.clone();
                    let child_index = *child as usize;
                    handles.push(tokio::spawn(async move {
                        execute_node(
                            child_plan,
                            child_index,
                            child_tx,
                            child_atlas,
                            child_provider_id,
                            child_runtime,
                            child_verification,
                        )
                        .await
                    }));
                }
                let mut any_failed = false;
                let mut verifiers = Vec::new();
                for handle in handles {
                    match handle.await {
                        Ok(child_execution) => {
                            any_failed |= child_execution.failed;
                            verifiers.extend(child_execution.verifiers);
                        }
                        Err(e) => {
                            any_failed = true;
                            warn!("[executor] parallel branch task failed: {e}");
                        }
                    }
                }
                let cancelled = runtime.is_cancelled(&plan.plan_id).await;
                complete_operator(
                    verifiers,
                    any_failed,
                    cancelled,
                    tx.clone(),
                    node_ctx.clone(),
                    runtime.clone(),
                    atlas.clone(),
                    provider_id.clone(),
                )
                .await
            }
            RTDL_IF | RTDL_REPEAT | RTDL_WHILE => {
                let context = runtime.context(&plan.plan_id).await;
                let condition = if node.condition_json.is_empty() {
                    serde_json::Value::Bool(true)
                } else {
                    serde_json::from_str(&node.condition_json).expect("validated condition")
                };
                let limit = match node.node_kind {
                    RTDL_IF => 1,
                    RTDL_REPEAT => node.repeat_count,
                    _ => node.max_iterations,
                };
                let kind = match node.node_kind {
                    RTDL_IF => "if",
                    RTDL_REPEAT => "repeat",
                    _ => "while",
                };
                info!(
                    "[executor/rtdl] plan_id={} node={} op_id={} {kind} start limit={limit}",
                    plan.plan_id, node_index, op_id
                );
                let mut iterations = 0;
                let mut failed = false;
                let mut exit_reason = "count completed";
                loop {
                    if runtime.is_cancelled(&plan.plan_id).await {
                        exit_reason = "plan canceled";
                        break;
                    }
                    let chosen = if node.node_kind == RTDL_REPEAT {
                        (iterations < limit).then_some(0)
                    } else {
                        match crate::expression::eval_condi(&condition, &context.snapshot().await) {
                            Ok(value) => {
                                let branch = if value {
                                    Some(0)
                                } else if node.node_kind == RTDL_IF && node.children.len() == 2 {
                                    Some(1)
                                } else {
                                    None
                                };
                                let selected = match (node.node_kind, branch) {
                                    (RTDL_IF, Some(0)) => "then",
                                    (RTDL_IF, Some(1)) => "else",
                                    (_, Some(_)) => "body",
                                    _ => "none",
                                };
                                info!(
                                    "[executor/rtdl] plan_id={} node={} op_id={} {kind} condition={value} completed_iterations={iterations} selected={selected}",
                                    plan.plan_id, node_index, op_id
                                );
                                if branch.is_none() {
                                    exit_reason = "condition false";
                                }
                                branch
                            }
                            Err(error) => {
                                warn!(
                                    "[executor/rtdl] plan_id={} node={} op_id={} {kind} failed completed_iterations={iterations}: condition evaluation: {error}",
                                    plan.plan_id, node_index, op_id
                                );
                                send_operator_state(
                                    &tx,
                                    &node_ctx,
                                    RtdlNodeStateEnum::Failed as u32,
                                    &error,
                                    &runtime,
                                )
                                .await;
                                return NodeExecution::finished(true);
                            }
                        }
                    };
                    let Some(chosen) = chosen else {
                        break;
                    };
                    if iterations >= limit {
                        warn!(
                            "[executor/rtdl] plan_id={} node={} op_id={} {kind} failed completed_iterations={iterations} max_iterations={limit}: condition remains true",
                            plan.plan_id, node_index, op_id
                        );
                        send_operator_state(
                            &tx,
                            &node_ctx,
                            RtdlNodeStateEnum::Failed as u32,
                            "while condition remains true at max_iterations",
                            &runtime,
                        )
                        .await;
                        return NodeExecution::finished(true);
                    }
                    info!(
                        "[executor/rtdl] plan_id={} node={} op_id={} {kind} iteration={} limit={limit} child={} start",
                        plan.plan_id,
                        node_index,
                        op_id,
                        iterations + 1,
                        node.children[chosen]
                    );
                    let execution = execute_node(
                        plan.clone(),
                        node.children[chosen] as usize,
                        tx.clone(),
                        atlas.clone(),
                        provider_id.clone(),
                        runtime.clone(),
                        verification.clone(),
                    )
                    .await;
                    failed = execution.failed | wait_for_verifiers(execution.verifiers).await;
                    iterations += 1;
                    info!(
                        "[executor/rtdl] plan_id={} node={} op_id={} {kind} iteration={iterations} child={} complete failed={failed}",
                        plan.plan_id, node_index, op_id, node.children[chosen]
                    );
                    if failed || node.node_kind == RTDL_IF {
                        exit_reason = if failed {
                            "child failed or canceled"
                        } else {
                            "branch completed"
                        };
                        break;
                    }
                    tokio::task::yield_now().await;
                }
                let canceled = runtime.is_cancelled(&plan.plan_id).await;
                let state = if canceled {
                    "Canceled"
                } else if failed {
                    "Failed"
                } else {
                    "Succeeded"
                };
                info!(
                    "[executor/rtdl] plan_id={} node={} op_id={} {kind} end state={state} completed_iterations={iterations} reason={}",
                    plan.plan_id,
                    node_index,
                    op_id,
                    if canceled {
                        "plan canceled"
                    } else {
                        exit_reason
                    }
                );
                complete_operator(
                    Vec::new(),
                    failed,
                    canceled,
                    tx.clone(),
                    node_ctx.clone(),
                    runtime.clone(),
                    atlas.clone(),
                    provider_id.clone(),
                )
                .await
            }
            RTDL_DO => {
                let original = node
                    .call
                    .as_ref()
                    .expect("validated do node must contain call");
                let context = runtime.context(&plan.plan_id).await;
                let args: serde_json::Value =
                    serde_json::from_str(&original.args_json).expect("validated arguments");
                let args = match crate::expression::eval_value(&args, &context.snapshot().await) {
                    Ok(args) => args,
                    Err(error) => {
                        warn!(
                            "[executor/rtdl] plan_id={} node={} op_id={} kind=do failed before dispatch: argument evaluation: {}",
                            plan.plan_id, node_index, op_id, error
                        );
                        let result = CapabilityCallResult {
                            call_id: original.call_id.clone(),
                            provider_id: original.provider_id.clone(),
                            contract_id: original.contract_id.clone(),
                            success: false,
                            output: String::new(),
                            error,
                        };
                        runtime
                            .record_op_state(
                                &plan.plan_id,
                                &op_id,
                                RtdlNodeStateEnum::Failed as u32,
                            )
                            .await;
                        let _ = tx
                            .send(Ok(rtdl_wire::node_state_from_result(
                                &node_ctx,
                                result,
                                RtdlNodeStateEnum::Failed as u32,
                            )))
                            .await;
                        return NodeExecution::finished(true);
                    }
                };
                let mut call = original.clone();
                call.args_json = args.to_string();
                execute_call(
                    &call,
                    node_ctx.clone(),
                    tx,
                    atlas,
                    provider_id.clone(),
                    runtime.clone(),
                    verification,
                    &node.bind,
                )
                .await
            }
            _ => {
                warn!(
                    "[executor] invalid node_kind={} reached after validation",
                    node.node_kind
                );
                NodeExecution::finished(true)
            }
        };
        if execution.verifiers.is_empty() {
            trigger_stop_on_complete(&runtime, &node_ctx, &provider_id, atlas_after).await;
        }
        execution
    })
}

/// Build the wire context copied into every node_state event.
fn node_event_context(plan: &Plan, node_index: usize) -> NodeEventContext {
    let node = &plan.nodes[node_index];
    NodeEventContext {
        plan_id: plan.plan_id.clone(),
        node_index: node_index as u32,
        node_kind: node.node_kind,
        op_id: node.op_id.clone(),
        description: node.description.clone(),
    }
}

/// Return whether a node kind is an RTDL operator rather than a leaf call.
fn is_operator_node(node_kind: u32) -> bool {
    matches!(
        node_kind,
        RTDL_SEQUENCE | RTDL_PARALLEL | RTDL_IF | RTDL_REPEAT | RTDL_WHILE
    )
}

fn rtdl_node_kind_name(node_kind: u32) -> &'static str {
    match node_kind {
        RTDL_SEQUENCE => "sequence",
        RTDL_PARALLEL => "parallel",
        RTDL_DO => "do",
        RTDL_IF => "if",
        RTDL_REPEAT => "repeat",
        RTDL_WHILE => "while",
        _ => "unknown",
    }
}

fn rtdl_state_name(state: u32) -> &'static str {
    match RtdlNodeStateEnum::try_from(state as i32) {
        Ok(RtdlNodeStateEnum::Pending) => "Pending",
        Ok(RtdlNodeStateEnum::Running) => "Running",
        Ok(RtdlNodeStateEnum::Verifying) => "Verifying",
        Ok(RtdlNodeStateEnum::Succeeded) => "Succeeded",
        Ok(RtdlNodeStateEnum::Failed) => "Failed",
        Ok(RtdlNodeStateEnum::Canceled) => "Canceled",
        Ok(RtdlNodeStateEnum::Timeout) => "Timeout",
        Ok(RtdlNodeStateEnum::Paused) => "Paused",
        Err(_) => "Unknown",
    }
}

fn log_preview(value: &str, max_chars: usize) -> String {
    let mut chars = value.chars();
    let mut preview: String = chars.by_ref().take(max_chars).collect();
    if chars.next().is_some() {
        preview.push('…');
    }
    preview
}

/// Map a completed leaf call to its RTDL state. Cancellation wins over the
/// provider result: a command terminated by cancel_plan normally returns
/// success=false, but that is an expected CANCELED outcome, not a FAILED tool.
fn leaf_terminal_state(success: bool, cancelled: bool) -> u32 {
    if cancelled {
        RtdlNodeStateEnum::Canceled as u32
    } else if success {
        RtdlNodeStateEnum::Succeeded as u32
    } else {
        RtdlNodeStateEnum::Failed as u32
    }
}

/// Emit the terminal event for an `on_enter` stop point on any RTDL node.
async fn send_stop_on_enter(
    tx: &Sender<Result<RtdlEvent, Status>>,
    node: &NodeEventContext,
    runtime: &PlanRuntime,
) {
    let detail = format!("stopped on entering op_id={}: plan cancelled", node.op_id);
    runtime
        .record_op_state(
            &node.plan_id,
            &node.op_id,
            RtdlNodeStateEnum::Canceled as u32,
        )
        .await;
    if is_operator_node(node.node_kind) {
        let _ = tx
            .send(Ok(rtdl_wire::operator_node_state(
                node,
                RtdlNodeStateEnum::Canceled as u32,
                detail,
            )))
            .await;
    } else {
        let _ = tx
            .send(Ok(rtdl_wire::node_state(
                node,
                RtdlNodeStateEnum::Canceled as u32,
                detail,
                None,
            )))
            .await;
    }
}

/// Stream a state event for a non-leaf RTDL operator node and record it.
async fn send_operator_state(
    tx: &Sender<Result<RtdlEvent, Status>>,
    node: &NodeEventContext,
    state: u32,
    reason: &str,
    runtime: &PlanRuntime,
) {
    let op = match node.node_kind {
        RTDL_SEQUENCE => "sequence",
        RTDL_PARALLEL => "parallel",
        RTDL_IF => "if",
        RTDL_REPEAT => "repeat",
        RTDL_WHILE => "while",
        _ => "operator",
    };
    let detail = format!(
        "RTDL {op} op_id={} {reason}: {}",
        node.op_id, node.description
    );
    info!(
        "[executor/rtdl] plan_id={} node={} op_id={} kind={} state={} detail='{}'",
        node.plan_id,
        node.node_index,
        node.op_id,
        op,
        rtdl_state_name(state),
        log_preview(&detail, 240)
    );
    runtime
        .record_op_state(&node.plan_id, &node.op_id, state)
        .await;
    let _ = tx
        .send(Ok(rtdl_wire::operator_node_state(node, state, detail)))
        .await;
}

/// Select and publish one non-leaf operator's final state after execution and verification.
async fn send_operator_final(
    tx: &Sender<Result<RtdlEvent, Status>>,
    node: &NodeEventContext,
    failed: bool,
    cancelled: bool,
    runtime: &PlanRuntime,
) {
    let (state, reason) = if cancelled {
        (RtdlNodeStateEnum::Canceled as u32, "was canceled")
    } else if failed {
        (
            RtdlNodeStateEnum::Failed as u32,
            "failed because one or more child nodes failed",
        )
    } else {
        (
            RtdlNodeStateEnum::Succeeded as u32,
            "completed successfully",
        )
    };
    send_operator_state(tx, node, state, reason, runtime).await;
}

/// Apply an `on_complete` stop point only after a node reaches its final state.
// TODO: Decide whether an overlap node's on_complete boundary should be its
// capability completion or its post-verification final state. It currently
// fires after verification, while parent execution may already have continued.
async fn trigger_stop_on_complete(
    runtime: &PlanRuntime,
    node: &NodeEventContext,
    provider_id: &str,
    mut atlas: AtlasClient,
) {
    if runtime
        .should_stop_at(&node.plan_id, &node.op_id, StopWhen::OnComplete)
        .await
    {
        runtime
            .trigger_stop(&node.plan_id, provider_id, &mut atlas)
            .await;
    }
}

/// Dispatch one RTDL `do` node and stream node_state events.
#[allow(clippy::too_many_arguments)]
async fn execute_call(
    call: &CapabilityCall,
    node: NodeEventContext,
    tx: Sender<Result<RtdlEvent, Status>>,
    mut atlas: AtlasClient,
    provider_id: String,
    runtime: PlanRuntime,
    verification: Arc<VerificationPolicy>,
    bind: &str,
) -> NodeExecution {
    let started_at = Instant::now();
    // Log the args too (bounded) so the log shows what each call requested —
    // essential for debugging plan-control builtins (stop_plan_at / cancel_plan)
    // and any cap call. Truncated to keep large payloads (images, file content)
    // from bloating the log.
    info!(
        "[executor/rtdl] plan_id={} node={} op_id={} kind=do dispatching call_id={} provider='{}' contract='{}' bind='{}' args={}",
        node.plan_id,
        node.node_index,
        node.op_id,
        call.call_id,
        call.provider_id,
        call.contract_id,
        bind,
        log_preview(&call.args_json, 256),
    );

    // Mark the op running so get_plan_status shows the in-flight node. Live
    // async states may replace it, then this function records one final state.
    runtime
        .record_op_state(
            &node.plan_id,
            &node.op_id,
            RtdlNodeStateEnum::Running as u32,
        )
        .await;

    let async_group = if call.provider_id == provider_id {
        Ok(None)
    } else {
        async_registry::resolve_async_group(&mut atlas, &call.provider_id, &call.contract_id).await
    };

    let (mut result, mut state) = match async_group {
        Err(error) => {
            let r = CapabilityCallResult {
                call_id: call.call_id.clone(),
                provider_id: call.provider_id.clone(),
                contract_id: call.contract_id.clone(),
                success: false,
                output: String::new(),
                error,
            };
            (r, RtdlNodeStateEnum::Failed as u32)
        }
        Ok(Some(group)) => {
            async_poll::run_until_terminal(call, &group, &provider_id, &mut atlas, &node, &runtime)
                .await
        }
        Ok(None) => {
            let r =
                crate::dispatch::dispatch(call, &provider_id, &mut atlas, &runtime, &node.plan_id)
                    .await;
            let cancelled = runtime.is_cancelled(&node.plan_id).await;
            let state = leaf_terminal_state(r.success, cancelled);
            (r, state)
        }
    };

    let eligible_for_verification = result.success
        && state == RtdlNodeStateEnum::Succeeded as u32
        && !runtime.is_cancelled(&node.plan_id).await;
    let overlap = bind.is_empty()
        && eligible_for_verification
        && verification.overlap()
        && verification.rule_for(call).is_some();

    if overlap {
        runtime
            .record_op_state(
                &node.plan_id,
                &node.op_id,
                RtdlNodeStateEnum::Verifying as u32,
            )
            .await;
        let _ = tx
            .send(Ok(rtdl_wire::node_state_from_result(
                &node,
                result.clone(),
                RtdlNodeStateEnum::Verifying as u32,
            )))
            .await;
        log_success(call, &result);
        info!(
            "[executor/rtdl] plan_id={} node={} op_id={} kind=do call_id={} state=Verifying duration_ms={} provider_success=true output_bytes={}",
            node.plan_id,
            node.node_index,
            node.op_id,
            call.call_id,
            started_at.elapsed().as_millis(),
            result.output.len()
        );

        let verifier = tokio::spawn(
            OverlapVerifier {
                call: call.clone(),
                node,
                tx,
                atlas,
                provider_id,
                runtime,
                policy: verification,
                original: result,
                started_at,
            }
            .run(),
        );
        return NodeExecution {
            failed: false,
            verifiers: vec![verifier],
        };
    }

    if eligible_for_verification {
        result = verification::verify_result(
            verification.as_ref(),
            call,
            &node,
            result,
            &provider_id,
            &mut atlas,
            &runtime,
        )
        .await;
        if !result.success {
            state = RtdlNodeStateEnum::Failed as u32;
        }
    }

    if runtime.is_cancelled(&node.plan_id).await {
        state = RtdlNodeStateEnum::Canceled as u32;
    }
    if state == RtdlNodeStateEnum::Succeeded as u32 && !bind.is_empty() {
        runtime
            .context(&node.plan_id)
            .await
            .store(bind, &result.output)
            .await;
        let value_type = match serde_json::from_str::<serde_json::Value>(&result.output) {
            Ok(serde_json::Value::Null) => "null",
            Ok(serde_json::Value::Bool(_)) => "boolean",
            Ok(serde_json::Value::Number(_)) => "number",
            Ok(serde_json::Value::String(_)) => "string",
            Ok(serde_json::Value::Array(_)) => "array",
            Ok(serde_json::Value::Object(_)) => "object",
            Err(_) => "string",
        };
        info!(
            "[executor/rtdl] plan_id={} node={} op_id={} kind=do bind stored name='{}' value_type={} output_bytes={}",
            node.plan_id,
            node.node_index,
            node.op_id,
            bind,
            value_type,
            result.output.len()
        );
    }
    runtime
        .record_op_state(&node.plan_id, &node.op_id, state)
        .await;
    let _ = tx
        .send(Ok(rtdl_wire::node_state_from_result(
            &node,
            result.clone(),
            state,
        )))
        .await;
    let failed = !result.success;

    info!(
        "[executor/rtdl] plan_id={} node={} op_id={} kind=do end call_id={} state={} duration_ms={} provider_success={} output_bytes={} error='{}'",
        node.plan_id,
        node.node_index,
        node.op_id,
        call.call_id,
        rtdl_state_name(state),
        started_at.elapsed().as_millis(),
        result.success,
        result.output.len(),
        log_preview(&result.error, 240)
    );

    if result.success {
        log_success(call, &result);
    } else {
        warn!("[executor] '{}' failed: {}", call.contract_id, result.error);
    }

    NodeExecution::finished(failed)
}

impl OverlapVerifier {
    /// Run the verifier and publish the leaf's final state without side effects.
    async fn run(mut self) -> bool {
        let verified = verification::verify_result(
            self.policy.as_ref(),
            &self.call,
            &self.node,
            self.original,
            &self.provider_id,
            &mut self.atlas,
            &self.runtime,
        )
        .await;
        // Entering VERIFYING means capability execution already completed.
        // A later plan cancellation therefore cannot replace this leaf's
        // verifier-owned SUCCEEDED/FAILED final state.
        let state = leaf_terminal_state(verified.success, false);
        self.runtime
            .record_op_state(&self.node.plan_id, &self.node.op_id, state)
            .await;
        let _ = self
            .tx
            .send(Ok(rtdl_wire::node_state_from_result(
                &self.node,
                verified.clone(),
                state,
            )))
            .await;
        info!(
            "[executor/rtdl] plan_id={} node={} op_id={} kind=do verification end call_id={} state={} duration_ms={} success={} output_bytes={} error='{}'",
            self.node.plan_id,
            self.node.node_index,
            self.node.op_id,
            self.call.call_id,
            rtdl_state_name(state),
            self.started_at.elapsed().as_millis(),
            verified.success,
            verified.output.len(),
            log_preview(&verified.error, 240)
        );
        if !verified.success {
            warn!(
                "[executor] '{}' verification finalized call_id={} as failed: {}",
                self.call.contract_id, self.call.call_id, verified.error
            );
        }
        trigger_stop_on_complete(&self.runtime, &self.node, &self.provider_id, self.atlas).await;
        !verified.success
    }
}

/// Log one successful capability output with a bounded payload preview.
fn log_success(call: &CapabilityCall, result: &CapabilityCallResult) {
    let preview: String = result.output.chars().take(512).collect();
    let ellipsis = if result.output.len() > 512 { "..." } else { "" };
    info!(
        "[executor] '{}' ok: {}{}",
        call.contract_id, preview, ellipsis
    );
}

#[tonic::async_trait]
impl RobonixSystemExecutorCancelAllPlans for ExecutorServiceImpl {
    async fn cancel_all(
        &self,
        _request: Request<crate::pb::executor::CancelAllRequest>,
    ) -> Result<Response<CancelAllResponse>, Status> {
        let mut atlas = self.atlas.clone();
        let success = self
            .runtime
            .cancel_all_plans(&self.provider_id, &mut atlas)
            .await;
        Ok(Response::new(CancelAllResponse { success }))
    }
}

#[tonic::async_trait]
impl RobonixSystemExecutorListActivePlans for ExecutorServiceImpl {
    async fn list_active_plans(
        &self,
        _request: Request<crate::pb::executor::ListActivePlansRequest>,
    ) -> Result<Response<ListActivePlansResponse>, Status> {
        Ok(Response::new(ListActivePlansResponse {
            success: true,
            plans_json: self.runtime.active_plans_json().await,
            error: String::new(),
        }))
    }
}

#[tonic::async_trait]
impl RobonixSystemExecutorControlPlan for ExecutorServiceImpl {
    async fn control_plan(
        &self,
        request: Request<crate::pb::executor::ControlPlanRequest>,
    ) -> Result<Response<ControlPlanResponse>, Status> {
        let request = request.into_inner();
        let mut atlas = self.atlas.clone();
        let response = match request.action.as_str() {
            "cancel" => {
                let wait_ms = if request.wait_ms == 0 {
                    5_000
                } else {
                    request.wait_ms
                };
                let (completed, message) = self
                    .runtime
                    .cancel_plan_control(&request.plan_id, wait_ms, &self.provider_id, &mut atlas)
                    .await;
                ControlPlanResponse {
                    success: true,
                    completed,
                    message,
                    error: String::new(),
                }
            }
            "cancel_all" => {
                let wait_ms = if request.wait_ms == 0 {
                    5_000
                } else {
                    request.wait_ms
                };
                let (target_count, completed) = self
                    .runtime
                    .cancel_all_plans_except(&self.provider_id, &mut atlas, None, wait_ms)
                    .await;
                ControlPlanResponse {
                    success: true,
                    completed,
                    message: format!(
                        "Cancellation requested for all RTDL plans; target_count={target_count}, completed={completed}."
                    ),
                    error: String::new(),
                }
            }
            "stop_at" => match self
                .runtime
                .stop_plan_at_control(&request.plan_id, &request.op_id, &request.when)
                .await
            {
                Ok(message) => ControlPlanResponse {
                    success: true,
                    completed: true,
                    message,
                    error: String::new(),
                },
                Err(error) => ControlPlanResponse {
                    success: false,
                    completed: true,
                    message: String::new(),
                    error,
                },
            },
            action => ControlPlanResponse {
                success: false,
                completed: true,
                message: String::new(),
                error: format!("unknown plan-control action '{action}'"),
            },
        };
        Ok(Response::new(response))
    }
}

#[tonic::async_trait]
impl RobonixSystemExecutorGetHealth for ExecutorServiceImpl {
    async fn get_module_health(
        &self,
        _request: Request<GetModuleHealthRequest>,
    ) -> Result<Response<GetModuleHealthResponse>, Status> {
        Ok(Response::new(GetModuleHealthResponse {
            report: Some(executor_health_report(&self.provider_id)),
        }))
    }
}

fn executor_health_report(provider_id: &str) -> ModuleHealthReport {
    ModuleHealthReport {
        schema_version: MODULE_HEALTH_SCHEMA_VERSION,
        module: Some(ModuleHealth {
            module_key: String::new(),
            module_id: "executor".to_string(),
            provider_id: provider_id.to_string(),
            health: MODULE_HEALTH_OK,
            state: "active".to_string(),
            reason_code: "OK".to_string(),
            detail: "executor serving".to_string(),
            source: String::new(),
            received_ts_ns: 0,
            ttl_ms: MODULE_HEALTH_TTL_MS,
        }),
    }
}

/// Validate Plan arena shape before spawning execution work.
fn validate_plan(plan: &Plan) -> Result<(), String> {
    if plan.nodes.is_empty() {
        return Err("Plan.nodes must not be empty".to_string());
    }
    let root = plan.root_index as usize;
    if root >= plan.nodes.len() {
        return Err(format!(
            "Plan.root_index {} is out of bounds for {} nodes",
            plan.root_index,
            plan.nodes.len()
        ));
    }

    let mut op_ids = HashSet::new();
    for (idx, node) in plan.nodes.iter().enumerate() {
        let op_id = node.op_id.trim();
        if op_id.is_empty() {
            return Err(format!("node {idx} op_id must not be empty"));
        }
        if !op_ids.insert(op_id.to_string()) {
            return Err(format!("node {idx} has duplicate op_id '{op_id}'"));
        }
        if node.description.trim().is_empty() {
            return Err(format!("node {idx} description must not be empty"));
        }
        if !node.bind.is_empty() && (node.node_kind != RTDL_DO || node.bind.trim() != node.bind) {
            return Err(format!("node {idx} has invalid bind"));
        }
        if matches!(node.node_kind, RTDL_IF | RTDL_WHILE) {
            let condition = serde_json::from_str(&node.condition_json)
                .map_err(|error| format!("node {idx} invalid condition: {error}"))?;
            crate::expression::validate_expr(&condition)?;
        } else if !node.condition_json.is_empty() {
            return Err(format!("node {idx} must not contain a condition"));
        }
        if node.node_kind != RTDL_REPEAT && node.repeat_count != 0
            || node.node_kind != RTDL_WHILE && node.max_iterations != 0
        {
            return Err(format!("node {idx} has unexpected loop limits"));
        }
        match node.node_kind {
            RTDL_SEQUENCE | RTDL_PARALLEL | RTDL_IF | RTDL_REPEAT | RTDL_WHILE => {
                if node.call.is_some() {
                    return Err(format!("operator node {idx} must not contain a call"));
                }
                if node.node_kind == RTDL_IF && !(1..=2).contains(&node.children.len()) {
                    return Err(format!("if node {idx} requires one or two children"));
                }
                if matches!(node.node_kind, RTDL_REPEAT | RTDL_WHILE) && node.children.len() != 1 {
                    return Err(format!("loop node {idx} requires one body"));
                }
                if node.repeat_count > MAX_ITERATIONS
                    || node.max_iterations > MAX_ITERATIONS
                    || node.node_kind == RTDL_WHILE && node.max_iterations == 0
                {
                    return Err(format!(
                        "node {idx} loop limit must be at most {MAX_ITERATIONS}; while requires a positive limit"
                    ));
                }
                for child in &node.children {
                    if *child as usize >= plan.nodes.len() {
                        return Err(format!("node {idx} child index {child} is out of bounds"));
                    }
                }
            }
            RTDL_DO => {
                if !node.children.is_empty() {
                    return Err(format!("do node {idx} must not have children"));
                }
                let Some(call) = node.call.as_ref() else {
                    return Err(format!("do node {idx} must contain a call"));
                };
                validate_call(idx, call)?;
            }
            other => return Err(format!("node {idx} has invalid node_kind {other}")),
        }
    }

    let mut colors = vec![VisitColor::White; plan.nodes.len()];
    visit_for_cycles(root, plan, &mut colors)?;
    validate_parallel_bindings(root, plan, &mut vec![None; plan.nodes.len()]).map(|_| ())
}

fn expression_reads(value: &serde_json::Value, reads: &mut HashSet<String>) {
    match value {
        serde_json::Value::Object(fields) => {
            if crate::expression::is_reference(fields)
                && let Some(name) = fields.get("var").and_then(serde_json::Value::as_str)
            {
                reads.insert(name.into());
            }
            for value in fields.values() {
                expression_reads(value, reads);
            }
        }
        serde_json::Value::Array(items) => {
            for value in items {
                expression_reads(value, reads);
            }
        }
        _ => {}
    }
}

fn validate_parallel_bindings(
    index: usize,
    plan: &Plan,
    cache: &mut [Option<(HashSet<String>, HashSet<String>)>],
) -> Result<(HashSet<String>, HashSet<String>), String> {
    if let Some(sets) = &cache[index] {
        return Ok(sets.clone());
    }
    let node = &plan.nodes[index];
    let mut reads = HashSet::new();
    let mut writes = HashSet::new();
    if !node.bind.is_empty() {
        writes.insert(node.bind.clone());
    }
    if !node.condition_json.is_empty() {
        expression_reads(
            &serde_json::from_str(&node.condition_json).expect("validated condition"),
            &mut reads,
        );
    }
    if let Some(call) = &node.call {
        expression_reads(
            &serde_json::from_str(&call.args_json).expect("validated arguments"),
            &mut reads,
        );
    }
    let mut branches = Vec::new();
    for child in &node.children {
        let branch = validate_parallel_bindings(*child as usize, plan, cache)?;
        if node.node_kind == RTDL_PARALLEL {
            for (other_reads, other_writes) in &branches {
                if !branch.1.is_disjoint(other_writes)
                    || !branch.0.is_disjoint(other_writes)
                    || !branch.1.is_disjoint(other_reads)
                {
                    return Err(format!(
                        "parallel node {index} has conflicting variable dependencies"
                    ));
                }
            }
        }
        reads.extend(branch.0.iter().cloned());
        writes.extend(branch.1.iter().cloned());
        branches.push(branch);
    }
    cache[index] = Some((reads.clone(), writes.clone()));
    Ok((reads, writes))
}

fn validate_call(node_index: usize, call: &CapabilityCall) -> Result<(), String> {
    if call.call_id.is_empty() {
        return Err(format!("do node {node_index} call_id must not be empty"));
    }
    if call.provider_id.is_empty() {
        return Err(format!(
            "do node {node_index} provider_id must not be empty"
        ));
    }
    if call.contract_id.is_empty() {
        return Err(format!(
            "do node {node_index} contract_id must not be empty"
        ));
    }
    let args: serde_json::Value = serde_json::from_str(&call.args_json)
        .map_err(|error| format!("do node {node_index} invalid args: {error}"))?;
    if !args.is_object() {
        return Err(format!("do node {node_index} args must be an object"));
    }
    crate::expression::validate_expr(&args)
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum VisitColor {
    White,
    Gray,
    Black,
}

/// DFS cycle check on every operator child edge.
///
/// Uses White/Gray/Black marks: entering a Gray node means a back-edge to an ancestor.
/// `RTDL_DO` nodes have no children in this graph. Returns `Ok` when the subgraph from
/// `index` is acyclic; otherwise an error naming the node where the cycle was found.
fn visit_for_cycles(index: usize, plan: &Plan, colors: &mut [VisitColor]) -> Result<(), String> {
    match colors[index] {
        VisitColor::Gray => return Err(format!("cycle detected at node {index}")),
        VisitColor::Black => return Ok(()),
        VisitColor::White => {}
    }
    colors[index] = VisitColor::Gray;
    let node = &plan.nodes[index];
    if is_operator_node(node.node_kind) {
        for child in &node.children {
            visit_for_cycles(*child as usize, plan, colors)?;
        }
    }
    colors[index] = VisitColor::Black;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{
        ExecutorServiceImpl, MODULE_HEALTH_OK, MODULE_HEALTH_SCHEMA_VERSION, MODULE_HEALTH_TTL_MS,
        OverlapVerifier, PlanRuntime, RTDL_DO, RTDL_PARALLEL, RTDL_SEQUENCE, RtdlNodeStateEnum,
        complete_operator, execute_call, executor_health_report, leaf_terminal_state,
        send_operator_state, send_stop_on_enter, validate_plan, wait_for_verifiers,
    };
    use crate::config::VerificationRule;
    use crate::pb::contracts::robonix_system_executor_execute_server::RobonixSystemExecutorExecute;
    use crate::pb::executor::rtdl_event::RtdlEventEnum;
    use crate::pb::pilot::{CapabilityCall, CapabilityCallResult, Plan, RtdlNode};
    use crate::rtdl_wire::NodeEventContext;
    use crate::verification::VerificationPolicy;
    use robonix_atlas::client::AtlasClient;
    use robonix_atlas::service::{AtlasRegistry, serve_atlas};
    use std::net::SocketAddr;
    use std::sync::Arc;
    use std::time::Duration;
    use tokio_stream::StreamExt;
    use tonic::Request;

    fn call(id: &str) -> CapabilityCall {
        CapabilityCall {
            call_id: id.to_string(),
            provider_id: "provider".to_string(),
            contract_id: "robonix/test/cap".to_string(),
            args_json: "{}".to_string(),
        }
    }

    fn control_node(kind: u32, id: &str, children: Vec<u32>) -> RtdlNode {
        RtdlNode {
            node_kind: kind,
            op_id: id.into(),
            description: id.into(),
            children,
            ..Default::default()
        }
    }

    fn command_node(id: &str, command: &str, bind: &str) -> RtdlNode {
        RtdlNode {
            node_kind: RTDL_DO,
            op_id: id.into(),
            description: id.into(),
            bind: bind.into(),
            call: Some(CapabilityCall {
                call_id: id.into(),
                provider_id: "executor".into(),
                contract_id: "robonix/system/executor/builtin/run_command".into(),
                args_json: serde_json::json!({"command":command}).to_string(),
            }),
            ..Default::default()
        }
    }

    async fn run_control_plan(nodes: Vec<RtdlNode>) -> Vec<crate::pb::executor::RtdlEvent> {
        run_control_plan_with_cancel(nodes, false).await
    }

    async fn run_control_plan_with_cancel(
        nodes: Vec<RtdlNode>,
        cancel: bool,
    ) -> Vec<crate::pb::executor::RtdlEvent> {
        let address = reserve_address();
        let server = tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), address));
        let atlas = AtlasClient::connect_with_retry(
            format!("http://{address}"),
            50,
            Duration::from_millis(10),
        )
        .await
        .unwrap();
        let service = ExecutorServiceImpl::new(
            atlas,
            "executor".into(),
            Arc::new(VerificationPolicy::new(false, vec![])),
        );
        if cancel {
            let runtime = service.runtime.clone();
            let mut atlas = service.atlas.clone();
            tokio::spawn(async move {
                tokio::time::sleep(Duration::from_millis(50)).await;
                runtime.cancel_all_plans("executor", &mut atlas).await;
            });
        }
        let mut stream = service
            .execute(Request::new(plan(nodes, 0)))
            .await
            .unwrap()
            .into_inner();
        let mut events = Vec::new();
        while let Some(event) = stream.next().await {
            events.push(event.unwrap());
        }
        server.abort();
        events
    }

    #[tokio::test]
    async fn if_selects_else_and_zero_repeat_skips_body() {
        let mut branch = control_node(super::RTDL_IF, "branch", vec![1, 2]);
        branch.condition_json = "false".into();
        let events = run_control_plan(vec![
            branch,
            command_node("then", "exit 1", ""),
            command_node("else", "printf selected", ""),
        ])
        .await;
        assert!(
            !events
                .last()
                .unwrap()
                .plan_complete
                .as_ref()
                .unwrap()
                .any_failed
        );
        let leaves: Vec<_> = events
            .iter()
            .filter_map(|event| event.node_state.as_ref())
            .filter_map(|state| state.leaf_result.as_ref())
            .collect();
        assert_eq!(leaves.len(), 1);
        assert_eq!(leaves[0].call_id, "else");
        let events = run_control_plan(vec![
            control_node(super::RTDL_REPEAT, "zero", vec![1]),
            command_node("unreachable", "exit 1", ""),
        ])
        .await;
        assert!(
            !events
                .last()
                .unwrap()
                .plan_complete
                .as_ref()
                .unwrap()
                .any_failed
        );
        assert!(events.iter().all(|event| {
            event
                .node_state
                .as_ref()
                .is_none_or(|state| state.leaf_result.is_none())
        }));
    }

    #[tokio::test]
    async fn repeat_runs_exact_count_and_stops_after_body_failure() {
        let mut repeat = control_node(super::RTDL_REPEAT, "repeat", vec![1]);
        repeat.repeat_count = 3;
        let events = run_control_plan(vec![
            repeat.clone(),
            command_node("body", "printf ok", "result"),
        ])
        .await;
        assert_eq!(
            events
                .iter()
                .filter(|event| event
                    .node_state
                    .as_ref()
                    .is_some_and(|state| state.leaf_result.is_some()))
                .count(),
            3
        );
        assert!(
            !events
                .last()
                .unwrap()
                .plan_complete
                .as_ref()
                .unwrap()
                .any_failed
        );
        let mut failing = command_node("body", "true", "result");
        failing.call.as_mut().unwrap().args_json = "{}".into();
        let events = run_control_plan(vec![repeat, failing]).await;
        assert!(
            events
                .last()
                .unwrap()
                .plan_complete
                .as_ref()
                .unwrap()
                .any_failed
        );
        assert_eq!(
            events
                .iter()
                .filter(|event| event
                    .node_state
                    .as_ref()
                    .is_some_and(|state| state.leaf_result.is_some()))
                .count(),
            1
        );
    }

    #[tokio::test]
    async fn while_reads_updated_bindings_and_reports_iteration_limit() {
        let mut loop_node = control_node(super::RTDL_WHILE, "while", vec![3]);
        loop_node.max_iterations = 3;
        loop_node.condition_json =
            serde_json::json!({"op":"eq","args":[{"var":"observation"},"yes"]}).to_string();
        let nodes = vec![
            control_node(RTDL_SEQUENCE, "root", vec![1, 2]),
            command_node("init", "printf yes", "observation"),
            loop_node,
            command_node("refresh", "printf no", "observation"),
        ];
        let events = run_control_plan(nodes).await;
        assert!(
            !events
                .last()
                .unwrap()
                .plan_complete
                .as_ref()
                .unwrap()
                .any_failed
        );
        assert_eq!(
            events
                .iter()
                .filter(|event| event
                    .node_state
                    .as_ref()
                    .is_some_and(|state| state.op_id == "refresh" && state.leaf_result.is_some()))
                .count(),
            1
        );
        let mut infinite = control_node(super::RTDL_WHILE, "bounded", vec![1]);
        infinite.max_iterations = 2;
        infinite.condition_json = "true".into();
        let events = run_control_plan(vec![infinite, command_node("body", "printf ok", "")]).await;
        assert!(
            events
                .last()
                .unwrap()
                .plan_complete
                .as_ref()
                .unwrap()
                .any_failed
        );
        assert!(events.iter().any(|event| {
            event
                .node_state
                .as_ref()
                .is_some_and(|state| state.operator_detail.contains("max_iterations"))
        }));
    }

    #[test]
    fn rejects_malformed_loops_and_expression_cycles() {
        let mut loop_node = control_node(super::RTDL_WHILE, "loop", vec![1]);
        loop_node.condition_json = "true".into();
        assert!(
            validate_plan(&plan(
                vec![loop_node.clone(), command_node("body", "true", "")],
                0
            ))
            .is_err()
        );
        loop_node.max_iterations = 2;
        loop_node.children = vec![0];
        assert!(
            validate_plan(&plan(vec![loop_node], 0))
                .unwrap_err()
                .contains("cycle")
        );
    }

    #[tokio::test]
    async fn arguments_use_results_and_context_does_not_leak_between_plans() {
        let init = command_node("init", r#"printf '{"command":"printf done"}'"#, "previous");
        let mut consumer = command_node("consumer", "unused", "");
        consumer.call.as_mut().unwrap().args_json =
            serde_json::json!({"command":{"var":"previous","path":["command"]}}).to_string();
        let events = run_control_plan(vec![
            control_node(RTDL_SEQUENCE, "root", vec![1, 2]),
            init,
            consumer.clone(),
        ])
        .await;
        assert!(
            !events
                .last()
                .unwrap()
                .plan_complete
                .as_ref()
                .unwrap()
                .any_failed
        );
        assert!(events.iter().any(|event| {
            event
                .node_state
                .as_ref()
                .and_then(|state| state.leaf_result.as_ref())
                .is_some_and(|result| result.call_id == "consumer" && result.output == "done")
        }));
        let events = run_control_plan(vec![consumer]).await;
        assert!(
            events
                .last()
                .unwrap()
                .plan_complete
                .as_ref()
                .unwrap()
                .any_failed
        );
        assert!(events.iter().any(|event| {
            event
                .node_state
                .as_ref()
                .and_then(|state| state.leaf_result.as_ref())
                .is_some_and(|result| result.error.contains("undefined variable"))
        }));
    }

    #[tokio::test]
    async fn cancellation_stops_a_running_loop() {
        let mut repeat = control_node(super::RTDL_REPEAT, "repeat", vec![1]);
        repeat.repeat_count = 100;
        let events =
            run_control_plan_with_cancel(vec![repeat, command_node("body", "sleep 1", "")], true)
                .await;
        assert!(
            events
                .last()
                .unwrap()
                .plan_complete
                .as_ref()
                .unwrap()
                .any_failed
        );
        assert!(events.iter().any(|event| {
            event.node_state.as_ref().is_some_and(|state| {
                state.op_id == "repeat" && state.state == RtdlNodeStateEnum::Canceled as u32
            })
        }));
    }

    #[test]
    fn parallel_variables_cannot_race_but_join_consumers_are_valid() {
        let writer = command_node("writer", "printf ok", "shared");
        let mut reader = command_node("reader", "unused", "");
        reader.call.as_mut().unwrap().args_json =
            serde_json::json!({"command":{"var":"shared"}}).to_string();
        assert!(
            validate_plan(&plan(
                vec![
                    control_node(RTDL_PARALLEL, "parallel", vec![1, 2]),
                    writer.clone(),
                    reader.clone()
                ],
                0
            ))
            .unwrap_err()
            .contains("conflicting")
        );
        assert!(
            validate_plan(&plan(
                vec![
                    control_node(RTDL_SEQUENCE, "root", vec![1, 3]),
                    control_node(RTDL_PARALLEL, "parallel", vec![2]),
                    writer,
                    reader
                ],
                0
            ))
            .is_ok()
        );
    }

    #[test]
    fn shared_subgraphs_are_validated_once() {
        let mut nodes = Vec::new();
        for index in 0..50 {
            nodes.push(control_node(
                RTDL_SEQUENCE,
                &format!("node-{index}"),
                if index == 49 {
                    vec![]
                } else {
                    vec![index + 1, index + 1]
                },
            ));
        }
        assert!(validate_plan(&plan(nodes, 0)).is_ok());
    }

    fn reserve_address() -> SocketAddr {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("reserve port");
        listener.local_addr().expect("reserved address")
    }

    #[test]
    fn executor_health_report_uses_minimal_module_health_v1_fields() {
        let report = executor_health_report("executor");
        assert_eq!(report.schema_version, MODULE_HEALTH_SCHEMA_VERSION);

        let module = report.module.expect("module health");
        assert_eq!(module.module_id, "executor");
        assert_eq!(module.provider_id, "executor");
        assert_eq!(module.health, MODULE_HEALTH_OK);
        assert_eq!(module.state, "active");
        assert_eq!(module.reason_code, "OK");
        assert_eq!(module.detail, "executor serving");
        assert_eq!(module.ttl_ms, MODULE_HEALTH_TTL_MS);

        assert!(module.module_key.is_empty());
        assert!(module.source.is_empty());
        assert_eq!(module.received_ts_ns, 0);
    }

    /// Aggregate failures without serializing the background verifiers.
    #[tokio::test]
    async fn overlap_verifiers_report_any_background_failure() {
        let verifiers = vec![tokio::spawn(async { false }), tokio::spawn(async { true })];

        assert!(wait_for_verifiers(verifiers).await);
    }

    /// Operators publish VERIFYING without awaiting their descendant verifier.
    #[tokio::test]
    async fn operator_verification_is_deferred_without_blocking_its_parent() {
        let atlas_addr = reserve_address();
        let atlas_server =
            tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), atlas_addr));
        let atlas = AtlasClient::connect_with_retry(
            format!("http://{atlas_addr}"),
            50,
            Duration::from_millis(10),
        )
        .await
        .expect("connect test Atlas");
        let runtime = PlanRuntime::default();
        runtime.register_plan("p").await;
        let (release_tx, release_rx) = tokio::sync::oneshot::channel();
        let pending = tokio::spawn(async move {
            let _ = release_rx.await;
            false
        });
        let (tx, mut rx) = tokio::sync::mpsc::channel(4);
        let execution = complete_operator(
            vec![pending],
            false,
            false,
            tx,
            NodeEventContext {
                plan_id: "p".into(),
                node_index: 0,
                node_kind: RTDL_SEQUENCE,
                op_id: "sequence".into(),
                description: "continue before verification".into(),
            },
            runtime,
            atlas,
            "executor".into(),
        )
        .await;

        let verifying = rx.recv().await.unwrap().unwrap().node_state.unwrap();
        assert_eq!(verifying.state, RtdlNodeStateEnum::Verifying as u32);
        assert!(!execution.failed);
        assert_eq!(execution.verifiers.len(), 1);
        release_tx.send(()).unwrap();
        assert!(!wait_for_verifiers(execution.verifiers).await);
        let succeeded = rx.recv().await.unwrap().unwrap().node_state.unwrap();
        assert_eq!(succeeded.state, RtdlNodeStateEnum::Succeeded as u32);
        atlas_server.abort();
    }

    /// Global overlap=false retains one synchronous verified terminal event.
    #[tokio::test]
    async fn synchronous_verification_never_emits_verifying() {
        let atlas_addr = reserve_address();
        let atlas_server =
            tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), atlas_addr));
        let atlas = AtlasClient::connect_with_retry(
            format!("http://{atlas_addr}"),
            50,
            Duration::from_millis(10),
        )
        .await
        .expect("connect test Atlas");
        let runtime = PlanRuntime::default();
        runtime.register_plan("p").await;
        let contract_id = "robonix/system/executor/builtin/get_all_plans";
        let call = CapabilityCall {
            call_id: "p:1".into(),
            provider_id: "executor".into(),
            contract_id: contract_id.into(),
            args_json: "{}".into(),
        };
        let (tx, mut rx) = tokio::sync::mpsc::channel(2);
        let execution = execute_call(
            &call,
            NodeEventContext {
                plan_id: "p".into(),
                node_index: 1,
                node_kind: RTDL_DO,
                op_id: "op_1".into(),
                description: "verify synchronously".into(),
            },
            tx,
            atlas,
            "executor".into(),
            runtime,
            Arc::new(VerificationPolicy::new(
                false,
                vec![VerificationRule {
                    target_contract_id: contract_id.into(),
                    target_provider_id: None,
                    verifier_provider_id: "missing_verifier".into(),
                    verifier_args: serde_json::json!({}),
                }],
            )),
            "",
        )
        .await;

        assert!(execution.failed);
        assert!(execution.verifiers.is_empty());
        let terminal = rx.recv().await.unwrap().unwrap().node_state.unwrap();
        assert_eq!(terminal.state, RtdlNodeStateEnum::Failed as u32);
        assert!(rx.try_recv().is_err());
        atlas_server.abort();
    }

    /// A background verifier emits only its own node's final failure.
    #[tokio::test]
    async fn overlap_verifier_failure_finalizes_only_its_node() {
        let atlas_addr = reserve_address();
        let atlas_server =
            tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), atlas_addr));
        let atlas = AtlasClient::connect_with_retry(
            format!("http://{atlas_addr}"),
            50,
            Duration::from_millis(10),
        )
        .await
        .expect("connect test Atlas");
        let runtime = PlanRuntime::default();
        runtime.register_plan("p").await;
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);
        let target_call = call("p:1");
        let verifier = OverlapVerifier {
            call: target_call.clone(),
            node: NodeEventContext {
                plan_id: "p".into(),
                node_index: 1,
                node_kind: RTDL_DO,
                op_id: "op_1".into(),
                description: "verify this node only".into(),
            },
            tx,
            atlas,
            provider_id: "executor".into(),
            runtime,
            policy: Arc::new(VerificationPolicy::new(
                true,
                vec![VerificationRule {
                    target_contract_id: target_call.contract_id.clone(),
                    target_provider_id: None,
                    verifier_provider_id: "missing_verifier".into(),
                    verifier_args: serde_json::json!({}),
                }],
            )),
            original: CapabilityCallResult {
                call_id: target_call.call_id,
                provider_id: target_call.provider_id,
                contract_id: target_call.contract_id,
                success: true,
                output: "done".into(),
                error: String::new(),
            },
        };

        assert!(verifier.run().await);
        let event = rx.recv().await.unwrap().unwrap();
        let final_state = event.node_state.unwrap();
        assert_eq!(final_state.node_index, 1);
        assert_eq!(final_state.state, RtdlNodeStateEnum::Failed as u32);
        assert!(!final_state.leaf_result.unwrap().success);
        assert!(rx.try_recv().is_err());
        atlas_server.abort();
    }

    /// Cancellation does not overwrite a leaf that already entered VERIFYING.
    #[tokio::test]
    async fn canceled_plan_keeps_verifying_leaf_success() {
        let atlas_addr = reserve_address();
        let atlas_server =
            tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), atlas_addr));
        let mut atlas = AtlasClient::connect_with_retry(
            format!("http://{atlas_addr}"),
            50,
            Duration::from_millis(10),
        )
        .await
        .expect("connect test Atlas");
        let runtime = PlanRuntime::default();
        runtime.register_plan("p").await;
        runtime.trigger_stop("p", "executor", &mut atlas).await;
        assert!(runtime.is_cancelled("p").await);
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);
        let target_call = call("p:1");
        let verifier = OverlapVerifier {
            call: target_call.clone(),
            node: NodeEventContext {
                plan_id: "p".into(),
                node_index: 1,
                node_kind: RTDL_DO,
                op_id: "op_1".into(),
                description: "already completed capability".into(),
            },
            tx,
            atlas,
            provider_id: "executor".into(),
            runtime,
            policy: Arc::new(VerificationPolicy::new(true, Vec::new())),
            original: CapabilityCallResult {
                call_id: target_call.call_id,
                provider_id: target_call.provider_id,
                contract_id: target_call.contract_id,
                success: true,
                output: "done".into(),
                error: String::new(),
            },
        };

        assert!(!verifier.run().await);
        let final_state = rx.recv().await.unwrap().unwrap().node_state.unwrap();
        assert_eq!(final_state.state, RtdlNodeStateEnum::Succeeded as u32);
        assert!(final_state.leaf_result.unwrap().success);
        atlas_server.abort();
    }

    /// A sequence reports verification in progress before its final failed state.
    #[tokio::test]
    async fn overlap_stream_finalizes_the_leaf_before_failing_its_sequence() {
        let atlas_addr = reserve_address();
        let atlas_server =
            tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), atlas_addr));
        let atlas = AtlasClient::connect_with_retry(
            format!("http://{atlas_addr}"),
            50,
            Duration::from_millis(10),
        )
        .await
        .expect("connect test Atlas");
        let contract_id = "robonix/system/executor/builtin/get_all_plans";
        let service = ExecutorServiceImpl::new(
            atlas,
            "executor".into(),
            Arc::new(VerificationPolicy::new(
                true,
                vec![VerificationRule {
                    target_contract_id: contract_id.into(),
                    target_provider_id: None,
                    verifier_provider_id: "missing_verifier".into(),
                    verifier_args: serde_json::json!({}),
                }],
            )),
        );
        let plan = Plan {
            plan_id: "overlap-plan".into(),
            session_id: "test".into(),
            round: 0,
            nodes: vec![
                RtdlNode {
                    node_kind: RTDL_SEQUENCE,
                    children: vec![1, 2],
                    call: None,
                    op_id: "sequence".into(),
                    description: "run the verified sequence".into(),
                    ..Default::default()
                },
                RtdlNode {
                    node_kind: RTDL_DO,
                    children: Vec::new(),
                    call: Some(CapabilityCall {
                        call_id: "overlap-plan:1".into(),
                        provider_id: "executor".into(),
                        contract_id: contract_id.into(),
                        args_json: "{}".into(),
                    }),
                    op_id: "op_1".into(),
                    description: "list active plans".into(),
                    ..Default::default()
                },
                RtdlNode {
                    node_kind: RTDL_DO,
                    children: Vec::new(),
                    call: Some(CapabilityCall {
                        call_id: "overlap-plan:2".into(),
                        provider_id: "executor".into(),
                        contract_id: "robonix/system/executor/builtin/list_dir".into(),
                        args_json: r#"{"path":"."}"#.into(),
                    }),
                    op_id: "op_2".into(),
                    description: "continue with the next node".into(),
                    ..Default::default()
                },
            ],
            root_index: 0,
        };
        let mut stream = service
            .execute(Request::new(plan))
            .await
            .unwrap()
            .into_inner();
        let mut states = Vec::new();
        let mut plan_failed = None;
        while let Some(event) = stream.next().await {
            let event = event.unwrap();
            if let Some(state) = event.node_state {
                states.push((state.node_index, state.state));
            }
            if let Some(complete) = event.plan_complete {
                plan_failed = Some(complete.any_failed);
            }
        }

        assert_eq!(
            states.first(),
            Some(&(1, RtdlNodeStateEnum::Verifying as u32))
        );
        assert!(states.contains(&(0, RtdlNodeStateEnum::Verifying as u32)));
        assert!(states.contains(&(1, RtdlNodeStateEnum::Failed as u32)));
        assert!(states.contains(&(2, RtdlNodeStateEnum::Succeeded as u32)));
        assert_eq!(states.last(), Some(&(0, RtdlNodeStateEnum::Failed as u32)));
        assert_eq!(plan_failed, Some(true));
        atlas_server.abort();
    }

    #[test]
    fn canceled_leaf_is_not_reported_as_failed_provider_work() {
        assert_eq!(
            leaf_terminal_state(false, true),
            RtdlNodeStateEnum::Canceled as u32
        );
        assert_eq!(
            leaf_terminal_state(false, false),
            RtdlNodeStateEnum::Failed as u32
        );
        assert_eq!(
            leaf_terminal_state(true, false),
            RtdlNodeStateEnum::Succeeded as u32
        );
    }

    fn node(kind: u32, children: Vec<u32>, call: Option<CapabilityCall>) -> RtdlNode {
        node_with_identity("op", "test node", kind, children, call)
    }

    fn node_with_identity(
        op_id: &str,
        description: &str,
        kind: u32,
        children: Vec<u32>,
        call: Option<CapabilityCall>,
    ) -> RtdlNode {
        RtdlNode {
            node_kind: kind,
            children,
            call,
            op_id: op_id.to_string(),
            description: description.to_string(),
            ..Default::default()
        }
    }

    fn plan(nodes: Vec<RtdlNode>, root_index: u32) -> Plan {
        Plan {
            plan_id: "p".to_string(),
            session_id: "s".to_string(),
            round: 0,
            nodes,
            root_index,
        }
    }

    #[test]
    fn validates_sequence_and_parallel_nodes() {
        let p = plan(
            vec![
                node_with_identity("op_1", "run sequence", RTDL_SEQUENCE, vec![1, 2], None),
                node_with_identity("op_2", "call first cap", RTDL_DO, vec![], Some(call("p:0"))),
                node_with_identity("op_3", "run parallel", RTDL_PARALLEL, vec![3, 4], None),
                node_with_identity(
                    "op_4",
                    "call second cap",
                    RTDL_DO,
                    vec![],
                    Some(call("p:1")),
                ),
                node_with_identity("op_5", "call third cap", RTDL_DO, vec![], Some(call("p:2"))),
            ],
            0,
        );
        validate_plan(&p).unwrap();
    }

    #[test]
    fn rejects_empty_op_id() {
        let p = plan(
            vec![node_with_identity("", "root", RTDL_SEQUENCE, vec![], None)],
            0,
        );
        assert!(validate_plan(&p).unwrap_err().contains("op_id"));
    }

    #[test]
    fn rejects_empty_description() {
        let p = plan(
            vec![node_with_identity("op_1", "", RTDL_SEQUENCE, vec![], None)],
            0,
        );
        assert!(validate_plan(&p).unwrap_err().contains("description"));
    }

    #[test]
    fn rejects_duplicate_op_id() {
        let p = plan(
            vec![
                node_with_identity("op_1", "root", RTDL_SEQUENCE, vec![1], None),
                node_with_identity("op_1", "child", RTDL_DO, vec![], Some(call("p:0"))),
            ],
            0,
        );
        assert!(validate_plan(&p).unwrap_err().contains("duplicate op_id"));
    }

    #[test]
    fn rejects_invalid_root() {
        let p = plan(vec![node(RTDL_SEQUENCE, vec![], None)], 3);
        assert!(validate_plan(&p).unwrap_err().contains("root_index"));
    }

    #[test]
    fn rejects_out_of_bounds_child() {
        let p = plan(vec![node(RTDL_SEQUENCE, vec![9], None)], 0);
        assert!(validate_plan(&p).unwrap_err().contains("out of bounds"));
    }

    #[test]
    fn rejects_cycle() {
        let p = plan(
            vec![
                node_with_identity("op_1", "root", RTDL_SEQUENCE, vec![1], None),
                node_with_identity("op_2", "child", RTDL_PARALLEL, vec![0], None),
            ],
            0,
        );
        assert!(validate_plan(&p).unwrap_err().contains("cycle"));
    }

    #[test]
    fn rejects_do_without_call() {
        let p = plan(vec![node(RTDL_DO, vec![], None)], 0);
        assert!(
            validate_plan(&p)
                .unwrap_err()
                .contains("must contain a call")
        );
    }

    #[tokio::test]
    async fn operator_terminal_event_carries_node_identity() {
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);
        let node = NodeEventContext {
            plan_id: "p".to_string(),
            node_index: 0,
            node_kind: RTDL_SEQUENCE,
            op_id: "op_1".to_string(),
            description: "run the ordered checks".to_string(),
        };

        let runtime = PlanRuntime::default();
        send_operator_state(
            &tx,
            &node,
            RtdlNodeStateEnum::Succeeded as u32,
            "completed successfully",
            &runtime,
        )
        .await;

        let event = rx.recv().await.unwrap().unwrap();
        let ns = event.node_state.unwrap();
        assert_eq!(event.event_kind, RtdlEventEnum::NodeState as u32);
        assert_eq!(ns.op_id, "op_1");
        assert_eq!(ns.description, "run the ordered checks");
        assert_eq!(ns.state, RtdlNodeStateEnum::Succeeded as u32);
        assert!(ns.leaf_result.is_none());
        assert!(ns.operator_detail.contains("RTDL sequence op_id=op_1"));
    }

    #[tokio::test]
    async fn stop_on_enter_event_supports_operator_nodes() {
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);
        let runtime = PlanRuntime::default();
        runtime.register_plan("p").await;
        let node = NodeEventContext {
            plan_id: "p".to_string(),
            node_index: 0,
            node_kind: RTDL_PARALLEL,
            op_id: "op_1".to_string(),
            description: "run branches".to_string(),
        };

        send_stop_on_enter(&tx, &node, &runtime).await;

        let event = rx.recv().await.unwrap().unwrap();
        let ns = event.node_state.unwrap();
        assert_eq!(event.event_kind, RtdlEventEnum::NodeState as u32);
        assert_eq!(ns.op_id, "op_1");
        assert_eq!(ns.state, RtdlNodeStateEnum::Canceled as u32);
        assert!(ns.leaf_result.is_none());
        assert!(ns.operator_detail.contains("stopped on entering"));
    }
}
