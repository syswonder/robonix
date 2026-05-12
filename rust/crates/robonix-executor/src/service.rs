// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// gRPC contract handler: RobonixSystemExecutor.Execute(Plan) → stream CapabilityCallEvent.

use crate::dispatch;
use crate::exec_wire;
use crate::pb::contracts::robonix_system_executor_server::RobonixSystemExecutor;
use crate::pb::executor::CapabilityCallEvent;
use crate::pb::pilot::Plan;
use robonix_atlas::client::AtlasClient;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

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
}

impl ExecutorServiceImpl {
    pub fn new(atlas: AtlasClient, provider_id: String) -> Self {
        Self { atlas, provider_id }
    }
}

#[tonic::async_trait]
impl RobonixSystemExecutor for ExecutorServiceImpl {
    type ExecuteStream = ReceiverStream<Result<CapabilityCallEvent, Status>>;

    async fn execute(
        &self,
        request: Request<Plan>,
    ) -> Result<Response<Self::ExecuteStream>, Status> {
        let plan = request.into_inner();
        let (tx, rx) = tokio::sync::mpsc::channel(64);
        let atlas = self.atlas.clone();
        let provider_id = self.provider_id.clone();

        tokio::spawn(async move {
            let plan_id = plan.plan_id.clone();
            let mut any_failed = false;

            for call in &plan.calls {
                let _ = tx
                    .send(Ok(exec_wire::started(
                        call.call_id.clone(),
                        call.provider_id.clone(),
                        call.contract_id.clone(),
                    )))
                    .await;

                log::info!(
                    "[executor] dispatching call_id={} provider='{}' contract='{}'",
                    call.call_id,
                    call.provider_id,
                    call.contract_id,
                );
                let mut atlas_for_call = atlas.clone();
                let result = dispatch::dispatch(call, &provider_id, &mut atlas_for_call).await;

                if result.success {
                    let preview: String = result.output.chars().take(120).collect();
                    let ellipsis = if result.output.len() > 120 { "…" } else { "" };
                    log::info!(
                        "[executor] '{}' ok: {}{}",
                        call.contract_id,
                        preview,
                        ellipsis
                    );
                } else {
                    any_failed = true;
                    log::warn!("[executor] '{}' failed: {}", call.contract_id, result.error);
                }

                let _ = tx.send(Ok(exec_wire::result(result))).await;
            }

            let _ = tx.send(Ok(exec_wire::complete(plan_id, any_failed))).await;
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}
