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
use tokio::sync::Mutex;
use tokio::sync::mpsc::Sender;
use tokio::task::JoinHandle;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

const RTDL_SEQUENCE: u32 = 0;
const RTDL_PARALLEL: u32 = 1;
const RTDL_DO: u32 = 2;
const MODULE_HEALTH_SCHEMA_VERSION: u32 = 1;
const MODULE_HEALTH_OK: u32 = 0;
const MODULE_HEALTH_TTL_MS: u32 = 5000;

/// Per-plan background verifiers that must finish before `plan_complete`.
#[derive(Clone, Default)]
struct OverlapVerifiers {
    verifiers: Arc<Mutex<Vec<JoinHandle<bool>>>>,
}

impl OverlapVerifiers {
    /// Retain a verifier handle so it cannot outlive the Execute stream.
    async fn track(&self, verifier: JoinHandle<bool>) {
        self.verifiers.lock().await.push(verifier);
    }

    /// Wait for every tracked verifier and report whether any one failed.
    async fn wait_for_all(self) -> bool {
        let verifiers = std::mem::take(&mut *self.verifiers.lock().await);
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
}

/// Verification state shared by every node in one Execute request.
#[derive(Clone)]
struct ExecutionVerification {
    policy: Arc<VerificationPolicy>,
    overlap_verifiers: OverlapVerifiers,
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
        validate_plan(&plan).map_err(Status::invalid_argument)?;
        let (tx, rx) = tokio::sync::mpsc::channel(64);
        let atlas = self.atlas.clone();
        let provider_id = self.provider_id.clone();
        let runtime = self.runtime.clone();
        let verification = ExecutionVerification {
            policy: Arc::clone(&self.verification),
            overlap_verifiers: OverlapVerifiers::default(),
        };

        tokio::spawn(async move {
            let plan_id = plan.plan_id.clone();
            let plan = Arc::new(plan);
            runtime.register_plan(&plan_id).await;
            runtime.record_plan_ops(&plan).await;
            let _ = tx.send(Ok(rtdl_wire::plan_started(plan_id.clone()))).await;
            let any_failed = execute_node(
                Arc::clone(&plan),
                plan.root_index as usize,
                tx.clone(),
                atlas,
                provider_id,
                runtime.clone(),
                verification.clone(),
            )
            .await;
            let verification_failed = verification.overlap_verifiers.wait_for_all().await;
            let cancelled = runtime.is_cancelled(&plan_id).await;
            runtime.complete_plan(&plan_id).await;

            let _ = tx
                .send(Ok(rtdl_wire::plan_complete(
                    plan_id,
                    any_failed || verification_failed || cancelled,
                )))
                .await;
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

type ExecuteNodeFuture = Pin<Box<dyn Future<Output = bool> + Send + 'static>>;

fn execute_node(
    plan: Arc<Plan>,
    node_index: usize,
    tx: Sender<Result<RtdlEvent, Status>>,
    atlas: AtlasClient,
    provider_id: String,
    runtime: PlanRuntime,
    verification: ExecutionVerification,
) -> ExecuteNodeFuture {
    Box::pin(async move {
        let node = &plan.nodes[node_index];
        let node_ctx = node_event_context(&plan, node_index);
        let op_id = node.op_id.clone();
        if runtime.is_cancelled(&plan.plan_id).await {
            if is_operator_node(node.node_kind) {
                send_operator_terminal(
                    &tx,
                    &node_ctx,
                    RtdlNodeStateEnum::Canceled as u32,
                    "canceled",
                    &runtime,
                )
                .await;
            }
            return true;
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
            return true;
        }
        let mut atlas_after = atlas.clone();
        let failed = match node.node_kind {
            RTDL_SEQUENCE => {
                let mut any_failed = false;
                let mut cancelled = false;
                for child in &node.children {
                    if runtime.is_cancelled(&plan.plan_id).await {
                        cancelled = true;
                        any_failed = true;
                        break;
                    }
                    any_failed |= execute_node(
                        Arc::clone(&plan),
                        *child as usize,
                        tx.clone(),
                        atlas.clone(),
                        provider_id.clone(),
                        runtime.clone(),
                        verification.clone(),
                    )
                    .await;
                    if any_failed {
                        break;
                    }
                }
                let cancelled = cancelled || runtime.is_cancelled(&plan.plan_id).await;
                let state = if cancelled {
                    RtdlNodeStateEnum::Canceled as u32
                } else if any_failed {
                    RtdlNodeStateEnum::Failed as u32
                } else {
                    RtdlNodeStateEnum::Succeeded as u32
                };
                let reason = if cancelled {
                    "canceled before remaining children could run"
                } else if any_failed {
                    "failed because a child node failed"
                } else {
                    "completed successfully"
                };
                send_operator_terminal(&tx, &node_ctx, state, reason, &runtime).await;
                any_failed || cancelled
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
                for handle in handles {
                    match handle.await {
                        Ok(child_failed) => any_failed |= child_failed,
                        Err(e) => {
                            any_failed = true;
                            warn!("[executor] parallel branch task failed: {e}");
                        }
                    }
                }
                let cancelled = runtime.is_cancelled(&plan.plan_id).await;
                let state = if cancelled {
                    RtdlNodeStateEnum::Canceled as u32
                } else if any_failed {
                    RtdlNodeStateEnum::Failed as u32
                } else {
                    RtdlNodeStateEnum::Succeeded as u32
                };
                let reason = if cancelled {
                    "canceled"
                } else if any_failed {
                    "failed because one or more child nodes failed"
                } else {
                    "completed successfully"
                };
                send_operator_terminal(&tx, &node_ctx, state, reason, &runtime).await;
                any_failed || cancelled
            }
            RTDL_DO => {
                let call = node
                    .call
                    .as_ref()
                    .expect("validated do node must contain call");
                execute_call(
                    call,
                    node_ctx,
                    tx,
                    atlas,
                    provider_id.clone(),
                    runtime.clone(),
                    verification,
                )
                .await
            }
            _ => {
                warn!(
                    "[executor] invalid node_kind={} reached after validation",
                    node.node_kind
                );
                true
            }
        };
        if runtime
            .should_stop_at(&plan.plan_id, &op_id, StopWhen::OnComplete)
            .await
        {
            runtime
                .trigger_stop(&plan.plan_id, &provider_id, &mut atlas_after)
                .await;
        }
        failed
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
    matches!(node_kind, RTDL_SEQUENCE | RTDL_PARALLEL)
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

/// Stream the terminal event for a non-leaf RTDL operator node, and record the
/// state so `get_plan_status` reflects it.
async fn send_operator_terminal(
    tx: &Sender<Result<RtdlEvent, Status>>,
    node: &NodeEventContext,
    state: u32,
    reason: &str,
    runtime: &PlanRuntime,
) {
    let op = match node.node_kind {
        RTDL_SEQUENCE => "sequence",
        RTDL_PARALLEL => "parallel",
        _ => "operator",
    };
    let detail = format!(
        "RTDL {op} op_id={} {reason}: {}",
        node.op_id, node.description
    );
    runtime
        .record_op_state(&node.plan_id, &node.op_id, state)
        .await;
    let _ = tx
        .send(Ok(rtdl_wire::operator_node_state(node, state, detail)))
        .await;
}

/// Dispatch one RTDL `do` node and stream node_state events.
async fn execute_call(
    call: &CapabilityCall,
    node: NodeEventContext,
    tx: Sender<Result<RtdlEvent, Status>>,
    mut atlas: AtlasClient,
    provider_id: String,
    runtime: PlanRuntime,
    verification: ExecutionVerification,
) -> bool {
    // Log the args too (bounded) so the log shows what each call requested —
    // essential for debugging plan-control builtins (stop_plan_at / cancel_plan)
    // and any cap call. Truncated to keep large payloads (images, file content)
    // from bloating the log.
    let args_preview: String = call.args_json.chars().take(256).collect();
    let args_ellipsis = if call.args_json.len() > 256 {
        "…"
    } else {
        ""
    };
    info!(
        "[executor] dispatching call_id={} provider='{}' contract='{}' args={}{}",
        call.call_id, call.provider_id, call.contract_id, args_preview, args_ellipsis,
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
    let overlap = eligible_for_verification
        && verification
            .policy
            .rule_for(call)
            .is_some_and(|rule| rule.overlap);

    if overlap {
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
        log_success(call, &result);

        let verifier = tokio::spawn(
            OverlapVerifier {
                call: call.clone(),
                node,
                tx,
                atlas,
                provider_id,
                runtime,
                policy: verification.policy,
                original: result,
            }
            .run(),
        );
        verification.overlap_verifiers.track(verifier).await;
        return false;
    }

    if eligible_for_verification {
        result = verification::verify_result(
            verification.policy.as_ref(),
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

    if result.success {
        log_success(call, &result);
    } else {
        warn!("[executor] '{}' failed: {}", call.contract_id, result.error);
    }

    failed
}

impl OverlapVerifier {
    /// Run the verifier and publish a correction without affecting other nodes.
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
        if verified.success {
            return false;
        }

        self.runtime
            .record_op_state(
                &self.node.plan_id,
                &self.node.op_id,
                RtdlNodeStateEnum::Failed as u32,
            )
            .await;
        let _ = self
            .tx
            .send(Ok(rtdl_wire::node_state_from_result(
                &self.node,
                verified.clone(),
                RtdlNodeStateEnum::Failed as u32,
            )))
            .await;
        warn!(
            "[executor] '{}' verification corrected call_id={} to failed: {}",
            self.call.contract_id, self.call.call_id, verified.error
        );
        true
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
        match node.node_kind {
            RTDL_SEQUENCE | RTDL_PARALLEL => {
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
    visit_for_cycles(root, plan, &mut colors)
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
    Ok(())
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum VisitColor {
    White,
    Gray,
    Black,
}

/// DFS cycle check on the plan arena following only sequence/parallel child edges.
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
    if matches!(node.node_kind, RTDL_SEQUENCE | RTDL_PARALLEL) {
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
        OverlapVerifier, OverlapVerifiers, PlanRuntime, RTDL_DO, RTDL_PARALLEL, RTDL_SEQUENCE,
        RtdlNodeStateEnum, executor_health_report, leaf_terminal_state, send_operator_terminal,
        send_stop_on_enter, validate_plan,
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
        let verifiers = OverlapVerifiers::default();
        verifiers.track(tokio::spawn(async { false })).await;
        verifiers.track(tokio::spawn(async { true })).await;

        assert!(verifiers.wait_for_all().await);
    }

    /// A background verifier emits only its own node's failed correction.
    #[tokio::test]
    async fn overlap_verifier_failure_emits_a_correction_for_only_its_node() {
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
            policy: Arc::new(VerificationPolicy::new(vec![VerificationRule {
                target_contract_id: target_call.contract_id.clone(),
                target_provider_id: None,
                verifier_provider_id: "missing_verifier".into(),
                overlap: true,
                verifier_args: serde_json::json!({}),
            }])),
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
        let correction = event.node_state.unwrap();
        assert_eq!(correction.node_index, 1);
        assert_eq!(correction.state, RtdlNodeStateEnum::Failed as u32);
        assert!(!correction.leaf_result.unwrap().success);
        assert!(rx.try_recv().is_err());
        atlas_server.abort();
    }

    /// The Execute stream publishes optimistic success before its correction.
    #[tokio::test]
    async fn overlap_stream_orders_success_correction_and_failed_completion() {
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
            Arc::new(VerificationPolicy::new(vec![VerificationRule {
                target_contract_id: contract_id.into(),
                target_provider_id: None,
                verifier_provider_id: "missing_verifier".into(),
                overlap: true,
                verifier_args: serde_json::json!({}),
            }])),
        );
        let plan = Plan {
            plan_id: "overlap-plan".into(),
            session_id: "test".into(),
            round: 0,
            nodes: vec![RtdlNode {
                node_kind: RTDL_DO,
                children: Vec::new(),
                call: Some(CapabilityCall {
                    call_id: "overlap-plan:0".into(),
                    provider_id: "executor".into(),
                    contract_id: contract_id.into(),
                    args_json: "{}".into(),
                }),
                op_id: "op_0".into(),
                description: "list active plans".into(),
            }],
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
                states.push(state.state);
            }
            if let Some(complete) = event.plan_complete {
                plan_failed = Some(complete.any_failed);
            }
        }

        assert_eq!(
            states,
            vec![
                RtdlNodeStateEnum::Succeeded as u32,
                RtdlNodeStateEnum::Failed as u32,
            ]
        );
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
        send_operator_terminal(
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
