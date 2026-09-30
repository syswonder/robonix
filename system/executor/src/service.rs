// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// gRPC contract handlers for executor plan execution and cancellation.

use crate::dispatch::{async_poll, async_registry};
use crate::execution::{Actor, Input};
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
use std::sync::Arc;
use tokio::sync::mpsc::Sender;
use tokio::task::JoinHandle;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

const RTDL_SEQUENCE: u32 = 0;
const RTDL_PARALLEL: u32 = 1;
#[cfg(test)]
const RTDL_DO: u32 = 2;
const MODULE_HEALTH_SCHEMA_VERSION: u32 = 1;
const MODULE_HEALTH_OK: u32 = 0;
const MODULE_HEALTH_TTL_MS: u32 = 5000;

/// Immediate call outcome and its optional background result verification.
pub(crate) struct NodeExecution {
    pub(crate) failed: bool,
    pub(crate) verifier: Option<JoinHandle<bool>>,
}

impl NodeExecution {
    /// Build an outcome with no background verification still pending.
    fn finished(failed: bool) -> Self {
        Self {
            failed,
            verifier: None,
        }
    }
}

/// Wait for one verifier, treating a panicked or aborted task as failed verification.
pub(crate) async fn wait_for_verifier(verifier: JoinHandle<bool>) -> bool {
    match verifier.await {
        Ok(failed) => failed,
        Err(error) => {
            warn!("[executor/verification] overlap verifier failed to join: {error}");
            true
        }
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
}

/// `AtlasClient` is cheap to clone — each Execute RPC clones it so per-plan
/// dispatch runs without serialising on a single mutex.
#[derive(Clone)]
pub struct ExecutorServiceImpl {
    pub(crate) atlas: AtlasClient,
    /// Executor's own provider_id. Two roles:
    ///   1. consumer_id passed to atlas on every ConnectCapability so the
    ///      channel record reflects who is using each downstream provider.
    ///   2. self-detection: when a CapabilityCall in the plan targets this
    ///      provider_id, dispatch short-circuits to the in-process builtin
    ///      handlers instead of going through MCP loopback.
    pub(crate) provider_id: String,
    pub(crate) runtime: PlanRuntime,
    pub(crate) verification: Arc<VerificationPolicy>,
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

    /// Validate and reserve a plan before returning headers, then stream its execution.
    async fn execute(
        &self,
        request: Request<Plan>,
    ) -> Result<Response<Self::ExecuteStream>, Status> {
        let input = Input::from_plan(request.into_inner()).map_err(Status::invalid_argument)?;
        Ok(Response::new(
            Actor::start(self.clone(), input, tokio_stream::empty()).await?,
        ))
    }
}

/// Build the wire context copied into every node_state event.
pub(crate) fn node_event_context(plan: &Plan, node_index: usize) -> NodeEventContext {
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
pub(crate) async fn send_stop_on_enter(
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
pub(crate) async fn send_operator_state(
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

/// Select and publish one non-leaf operator's final state after execution and verification.
pub(crate) async fn send_operator_final(
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
pub(crate) async fn trigger_stop_on_complete(
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
pub(crate) async fn execute_call(
    call: &CapabilityCall,
    node: NodeEventContext,
    tx: Sender<Result<RtdlEvent, Status>>,
    mut atlas: AtlasClient,
    provider_id: String,
    runtime: PlanRuntime,
    verification: Arc<VerificationPolicy>,
) -> NodeExecution {
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
            }
            .run(),
        );
        return NodeExecution {
            failed: false,
            verifier: Some(verifier),
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

/// Require complete capability identity before dispatch or incremental admission.
pub(crate) fn validate_call(node_index: usize, call: &CapabilityCall) -> Result<(), String> {
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

#[cfg(test)]
mod tests {
    use super::{
        ExecutorServiceImpl, MODULE_HEALTH_OK, MODULE_HEALTH_SCHEMA_VERSION, MODULE_HEALTH_TTL_MS,
        OverlapVerifier, PlanRuntime, RTDL_DO, RTDL_PARALLEL, RTDL_SEQUENCE, RtdlNodeStateEnum,
        execute_call, executor_health_report, leaf_terminal_state, send_operator_state,
        send_stop_on_enter, wait_for_verifier,
    };
    use crate::config::VerificationRule;
    use crate::execution::Input;
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

    /// Preserve verifier outcomes and fail closed if its background task cannot finish.
    #[tokio::test]
    async fn overlap_verifier_reports_outcome_and_join_failure() {
        assert!(!wait_for_verifier(tokio::spawn(async { false })).await);
        assert!(wait_for_verifier(tokio::spawn(async { true })).await);
        assert!(wait_for_verifier(tokio::spawn(async { panic!("test verifier panic") })).await);
        let verifier = tokio::spawn(std::future::pending::<bool>());
        verifier.abort();
        assert!(wait_for_verifier(verifier).await);
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
        runtime.register_plan("p").await.unwrap();
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
        )
        .await;

        assert!(execution.failed);
        assert!(execution.verifier.is_none());
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
        runtime.register_plan("p").await.unwrap();
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
        runtime.register_plan("p").await.unwrap();
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
            Some(&(0, RtdlNodeStateEnum::Running as u32))
        );
        assert!(states.contains(&(1, RtdlNodeStateEnum::Verifying as u32)));
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
        Input::from_plan(p).unwrap();
    }

    #[test]
    fn rejects_empty_op_id() {
        let p = plan(
            vec![node_with_identity("", "root", RTDL_SEQUENCE, vec![], None)],
            0,
        );
        assert!(Input::from_plan(p).unwrap_err().contains("op_id"));
    }

    #[test]
    fn rejects_empty_description() {
        let p = plan(
            vec![node_with_identity("op_1", "", RTDL_SEQUENCE, vec![], None)],
            0,
        );
        assert!(Input::from_plan(p).unwrap_err().contains("description"));
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
        assert!(Input::from_plan(p).unwrap_err().contains("duplicate op_id"));
    }

    #[test]
    fn rejects_invalid_root() {
        let p = plan(vec![node(RTDL_SEQUENCE, vec![], None)], 3);
        assert!(Input::from_plan(p).unwrap_err().contains("root_index"));
    }

    #[test]
    fn rejects_out_of_bounds_child() {
        let p = plan(vec![node(RTDL_SEQUENCE, vec![9], None)], 0);
        assert!(Input::from_plan(p).unwrap_err().contains("out of bounds"));
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
        assert!(Input::from_plan(p).unwrap_err().contains("cycle"));
    }

    #[test]
    fn rejects_do_without_call() {
        let p = plan(vec![node(RTDL_DO, vec![], None)], 0);
        assert!(
            Input::from_plan(p)
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
        runtime.register_plan("p").await.unwrap();
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
