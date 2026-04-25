// SPDX-License-Identifier: MulanPSL-2.0
// gRPC contract handlers (SystemExecutor — Stream; SystemExecutorListTools — Call).

use crate::dispatch;
use crate::exec_wire;
use crate::pb::contracts::{
    system_executor_list_tools_server::SystemExecutorListTools,
    system_executor_server::SystemExecutor,
};
use crate::pb::executor::{ListToolsRequest, ListToolsResponse, TaskCallEvent, ToolSpec};
use crate::pb::pilot::TaskGraph;
use crate::tools;
use robonix_atlas::client::AtlasClient;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

/// `AtlasClient` is cheap to clone — each Stream RPC clones it so per-turn
/// tool discovery runs without serialising on a single mutex.
#[derive(Clone)]
pub struct ExecutorServiceImpl {
    atlas: AtlasClient,
}

impl ExecutorServiceImpl {
    pub fn new(atlas: AtlasClient) -> Self {
        Self { atlas }
    }
}

#[tonic::async_trait]
impl SystemExecutor for ExecutorServiceImpl {
    type StreamStream = ReceiverStream<Result<TaskCallEvent, Status>>;

    async fn stream(
        &self,
        request: Request<TaskGraph>,
    ) -> Result<Response<Self::StreamStream>, Status> {
        let graph = request.into_inner();
        let (tx, rx) = tokio::sync::mpsc::channel(64);
        let mut atlas = self.atlas.clone();

        tokio::spawn(async move {
            // Re-discover tools at the top of every Stream RPC so caps that
            // registered after pilot's last list_tools call are visible.
            let routing_map = match tools::load_tools(&mut atlas).await {
                Ok(list) => tools::routing_map(&list),
                Err(e) => {
                    log::warn!("failed to load tools: {e:#}");
                    Default::default()
                }
            };

            let graph_id = graph.graph_id.clone();
            let mut any_failed = false;

            for call in &graph.calls {
                let _ = tx
                    .send(Ok(exec_wire::started(
                        call.call_id.clone(),
                        call.tool_name.clone(),
                    )))
                    .await;

                log::info!(
                    "[executor] dispatching '{}' (call_id={})",
                    call.tool_name,
                    call.call_id
                );
                let result = dispatch::dispatch(call, &routing_map).await;

                if result.success {
                    let preview: String = result.output.chars().take(120).collect();
                    let ellipsis = if result.output.len() > 120 { "…" } else { "" };
                    log::info!(
                        "[executor] '{}' ok: {}{}",
                        call.tool_name,
                        preview,
                        ellipsis
                    );
                } else {
                    any_failed = true;
                    log::warn!("[executor] '{}' failed: {}", call.tool_name, result.error);
                }

                let _ = tx.send(Ok(exec_wire::result(result))).await;
            }

            let _ = tx.send(Ok(exec_wire::complete(graph_id, any_failed))).await;
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

#[tonic::async_trait]
impl SystemExecutorListTools for ExecutorServiceImpl {
    async fn call(
        &self,
        request: Request<ListToolsRequest>,
    ) -> Result<Response<ListToolsResponse>, Status> {
        let _refresh = request.into_inner().refresh;
        let mut atlas = self.atlas.clone();
        let tool_list = tools::load_tools(&mut atlas)
            .await
            .map_err(|e| Status::internal(e.to_string()))?;

        let specs = tool_list
            .into_iter()
            .map(|t| ToolSpec {
                tool_name: t.name,
                description: t.description,
                input_schema_json: t.input_schema.to_string(),
                routing: Some(t.routing),
            })
            .collect();

        Ok(Response::new(ListToolsResponse { tools: specs }))
    }
}
