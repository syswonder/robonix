// SPDX-License-Identifier: MulanPSL-2.0
// gRPC contract handlers (SystemExecutor — Stream; SystemExecutorListTools — Call).

use crate::dispatch;
use crate::exec_wire;
use crate::pb::contracts::{
    system_executor_list_tools_server::SystemExecutorListTools,
    system_executor_server::SystemExecutor,
};
use crate::pb::executor::{
    CapabilityCallEvent, CapabilitySpec, ListToolsRequest, ListToolsResponse,
};
use crate::pb::pilot::Plan;
use crate::tools;
use robonix_atlas::client::AtlasClient;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

/// `AtlasClient` is cheap to clone — each Stream RPC clones it so per-turn
/// tool discovery runs without serialising on a single mutex.
#[derive(Clone)]
pub struct ExecutorServiceImpl {
    atlas: AtlasClient,
    /// Executor's own cap_id; passed to atlas as `consumer_id` whenever
    /// load_tools opens channels to MCP-providing caps.
    cap_id: String,
}

impl ExecutorServiceImpl {
    pub fn new(atlas: AtlasClient, cap_id: String) -> Self {
        Self { atlas, cap_id }
    }
}

#[tonic::async_trait]
impl SystemExecutor for ExecutorServiceImpl {
    type ExecuteStream = ReceiverStream<Result<CapabilityCallEvent, Status>>;

    async fn execute(
        &self,
        request: Request<Plan>,
    ) -> Result<Response<Self::ExecuteStream>, Status> {
        let graph = request.into_inner();
        let (tx, rx) = tokio::sync::mpsc::channel(64);
        let mut atlas = self.atlas.clone();
        let cap_id = self.cap_id.clone();

        tokio::spawn(async move {
            // Re-discover tools at the top of every Stream RPC so caps that
            // registered after pilot's last list_tools call are visible.
            let routing_map = match tools::load_tools(&mut atlas, &cap_id).await {
                Ok(list) => tools::routing_map(&list),
                Err(e) => {
                    log::warn!("failed to load tools: {e:#}");
                    Default::default()
                }
            };

            let plan_id = graph.plan_id.clone();
            let mut any_failed = false;

            for call in &graph.calls {
                let _ = tx
                    .send(Ok(exec_wire::started(
                        call.call_id.clone(),
                        call.capability_name.clone(),
                    )))
                    .await;

                log::info!(
                    "[executor] dispatching '{}' (call_id={})",
                    call.capability_name,
                    call.call_id
                );
                let result = dispatch::dispatch(call, &routing_map).await;

                if result.success {
                    let preview: String = result.output.chars().take(120).collect();
                    let ellipsis = if result.output.len() > 120 { "…" } else { "" };
                    log::info!(
                        "[executor] '{}' ok: {}{}",
                        call.capability_name,
                        preview,
                        ellipsis
                    );
                } else {
                    any_failed = true;
                    log::warn!(
                        "[executor] '{}' failed: {}",
                        call.capability_name,
                        result.error
                    );
                }

                let _ = tx.send(Ok(exec_wire::result(result))).await;
            }

            let _ = tx.send(Ok(exec_wire::complete(plan_id, any_failed))).await;
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

#[tonic::async_trait]
impl SystemExecutorListTools for ExecutorServiceImpl {
    async fn list_tools(
        &self,
        request: Request<ListToolsRequest>,
    ) -> Result<Response<ListToolsResponse>, Status> {
        let _refresh = request.into_inner().refresh;
        let mut atlas = self.atlas.clone();
        let tool_list = tools::load_tools(&mut atlas, &self.cap_id)
            .await
            .map_err(|e| Status::internal(e.to_string()))?;

        let specs = tool_list
            .into_iter()
            .map(|t| CapabilitySpec {
                capability_name: t.name,
                description: t.description,
                input_schema_json: t.input_schema.to_string(),
                routing: Some(t.routing),
            })
            .collect();

        Ok(Response::new(ListToolsResponse {
            capabilities: specs,
        }))
    }
}
