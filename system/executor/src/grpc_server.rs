// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//! Executor gRPC entry points for execution, plan control, queries, and health.
use crate::active_plans::PlanRuntime;
use crate::pb::contracts::robonix_system_executor_cancel_all_plans_server::RobonixSystemExecutorCancelAllPlans;
use crate::pb::contracts::robonix_system_executor_control_plan_server::RobonixSystemExecutorControlPlan;
use crate::pb::contracts::robonix_system_executor_execute_server::RobonixSystemExecutorExecute;
use crate::pb::contracts::robonix_system_executor_execute_stream_server::RobonixSystemExecutorExecuteStream;
use crate::pb::contracts::robonix_system_executor_get_health_server::RobonixSystemExecutorGetHealth;
use crate::pb::contracts::robonix_system_executor_list_active_plans_server::RobonixSystemExecutorListActivePlans;
use crate::pb::executor::ExecuteStreamInput;
use crate::pb::executor::{
    CancelAllResponse, ControlPlanResponse, ListActivePlansResponse, RtdlEvent,
};
use crate::pb::module_health::{
    GetModuleHealthRequest, GetModuleHealthResponse, ModuleHealth, ModuleHealthReport,
};
use crate::pb::pilot::Plan;
use crate::plan_input::PlanInput;
use crate::plan_scheduler::PlanScheduler;
use crate::sentinel_runtime::{SentinelIdentity, SentinelRuntime, resolve_session_identity};
use crate::verification::VerificationPolicy;
use robonix_atlas::client::AtlasClient;
use std::sync::Arc;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status, Streaming};

const MODULE_HEALTH_SCHEMA_VERSION: u32 = 1;
const MODULE_HEALTH_OK: u32 = 0;
const MODULE_HEALTH_TTL_MS: u32 = 5000;

/// Executor gRPC handlers sharing active plans and result verification policy.
///
/// `AtlasClient` is cheap to clone — each Execute RPC clones it so per-plan
/// dispatch runs without serialising on a single mutex.
#[derive(Clone)]
pub struct ExecutorServiceImpl {
    /// Atlas connection used to resolve capabilities for execution and cancellation.
    pub(crate) atlas: AtlasClient,
    /// Executor's own provider_id. Two roles:
    ///   1. consumer_id passed to atlas on every ConnectCapability so the
    ///      channel record reflects who is using each downstream provider.
    ///   2. self-detection: when a CapabilityCall in the plan targets this
    ///      provider_id, dispatch short-circuits to the in-process builtin
    ///      handlers instead of going through MCP loopback.
    pub(crate) provider_id: String,
    /// Shared plan progress and cancellation state used by all RPC handlers.
    pub(crate) runtime: PlanRuntime,
    /// Active robot-local policy checked before every capability dispatch.
    pub(crate) sentinel: SentinelRuntime,
    /// Whether missing Keystone sessions are rejected instead of treated as local.
    pub(crate) require_keystone: bool,
    /// Immutable verification timing and routing rules applied to submitted plans.
    pub(crate) verification: Arc<VerificationPolicy>,
}

impl ExecutorServiceImpl {
    /// Assemble RPC handlers sharing one active-plan table and verification policy.
    pub fn new(
        atlas: AtlasClient,
        provider_id: String,
        sentinel: SentinelRuntime,
        require_keystone: bool,
        verification: Arc<VerificationPolicy>,
    ) -> Self {
        Self {
            atlas,
            provider_id,
            runtime: PlanRuntime::default(),
            sentinel,
            require_keystone,
            verification,
        }
    }

    /// Resolve a request credential while allowing anonymous local execution only when configured.
    async fn authenticate_session(&self, token: &str) -> Result<SentinelIdentity, Status> {
        if token.is_empty() {
            if self.require_keystone {
                return Err(Status::unauthenticated("login session is required"));
            }
            return Ok(SentinelIdentity::default());
        }
        resolve_session_identity(&self.atlas, &self.provider_id, token).await
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
        let mut plan = request.into_inner();
        let token = std::mem::take(&mut plan.auth_session_token);
        let identity = self.authenticate_session(&token).await?;
        let input = PlanInput::from_plan(plan).map_err(Status::invalid_argument)?;
        Ok(Response::new(
            PlanScheduler::start(
                self.atlas.clone(),
                self.provider_id.clone(),
                self.runtime.clone(),
                self.sentinel.clone(),
                identity,
                self.verification.clone(),
                input,
                tokio_stream::empty(),
            )
            .await?,
        ))
    }
}

#[tonic::async_trait]
impl RobonixSystemExecutorExecuteStream for ExecutorServiceImpl {
    type ExecuteStreamStream = ReceiverStream<Result<RtdlEvent, Status>>;

    /// Require START before handing incremental input to the common execution loop.
    async fn execute_stream(
        &self,
        request: Request<Streaming<ExecuteStreamInput>>,
    ) -> Result<Response<Self::ExecuteStreamStream>, Status> {
        let mut stream = request.into_inner();
        let first = stream
            .message()
            .await?
            .ok_or_else(|| Status::invalid_argument("missing START"))?;
        let token = first.auth_session_token.clone();
        let identity = self.authenticate_session(&token).await?;
        let mut first = first;
        first.auth_session_token.clear();
        let mut input = PlanInput::default();
        input.accept(first).map_err(Status::invalid_argument)?;
        Ok(Response::new(
            PlanScheduler::start(
                self.atlas.clone(),
                self.provider_id.clone(),
                self.runtime.clone(),
                self.sentinel.clone(),
                identity,
                self.verification.clone(),
                input,
                stream,
            )
            .await?,
        ))
    }
}

#[tonic::async_trait]
impl RobonixSystemExecutorCancelAllPlans for ExecutorServiceImpl {
    /// Cancel active plans and return whether they finished within the default wait.
    async fn cancel_all(
        &self,
        request: Request<crate::pb::executor::CancelAllRequest>,
    ) -> Result<Response<CancelAllResponse>, Status> {
        self.authenticate_session(&request.into_inner().session_token)
            .await?;
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
    /// Return a snapshot without creating an execution plan for the query.
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
    /// Apply a cancel or cancel_all request and report completion within its wait limit.
    async fn control_plan(
        &self,
        request: Request<crate::pb::executor::ControlPlanRequest>,
    ) -> Result<Response<ControlPlanResponse>, Status> {
        let request = request.into_inner();
        self.authenticate_session(&request.session_token).await?;
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
    /// Report the health of the running Executor service.
    async fn get_module_health(
        &self,
        _request: Request<GetModuleHealthRequest>,
    ) -> Result<Response<GetModuleHealthResponse>, Status> {
        Ok(Response::new(GetModuleHealthResponse {
            report: Some(executor_health_report(&self.provider_id)),
        }))
    }
}

/// Build the minimal module-health report served to Vitals.
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Health responses retain the minimal schema and the configured provider identity.
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
}
