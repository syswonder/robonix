// SPDX-License-Identifier: MulanPSL-2.0
// service.rs — gRPC contract services (SysRuntimeExecutor, SysRuntimeExecutorListTools)

use crate::contracts::{
    sys_runtime_executor_list_tools_server::SysRuntimeExecutorListTools,
    sys_runtime_executor_server::SysRuntimeExecutor,
};
use crate::dispatch;
use crate::exec_wire;
use crate::executor::{ListToolsRequest, ListToolsResponse, TaskCallEvent, ToolSpec};
use crate::pilot::TaskGraph;
use crate::tools;
use robonix_sdk::RobonixClient;
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

#[derive(Clone)]
pub struct ExecutorServiceImpl {
    sdk: Arc<Mutex<RobonixClient>>,
}

impl ExecutorServiceImpl {
    pub fn new(sdk: Arc<Mutex<RobonixClient>>) -> Self {
        Self { sdk }
    }
}

#[tonic::async_trait]
impl SysRuntimeExecutor for ExecutorServiceImpl {
    type StreamStream = ReceiverStream<Result<TaskCallEvent, Status>>;

    async fn stream(
        &self,
        request: Request<TaskGraph>,
    ) -> Result<Response<Self::StreamStream>, Status> {
        let graph = request.into_inner();
        let (tx, rx) = tokio::sync::mpsc::channel(64);
        let sdk = Arc::clone(&self.sdk);

        tokio::spawn(async move {
            let routing_map = {
                let mut sdk = sdk.lock().await;
                match tools::load_tools(&mut sdk).await {
                    Ok(list) => tools::routing_map(&list),
                    Err(e) => {
                        log::warn!("failed to load tools: {e:#}");
                        Default::default()
                    }
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
impl SysRuntimeExecutorListTools for ExecutorServiceImpl {
    async fn call(
        &self,
        request: Request<ListToolsRequest>,
    ) -> Result<Response<ListToolsResponse>, Status> {
        let _refresh = request.into_inner().refresh;
        let mut sdk = self.sdk.lock().await;
        let tool_list = tools::load_tools(&mut sdk)
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
