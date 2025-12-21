use crate::config::Config;
use anyhow::Result;
use robonix_core::task_manager::api::{
    SubmitTaskRequest, SubmitTaskResponse, TaskResultRequest, TaskResultResponse,
    TaskStatusRequest, TaskStatusResponse,
};
use ros2_client::{
    service::AService, Context, Name, Node, NodeName, NodeOptions, ServiceMapping, ServiceTypeName,
};
use rustdds::{policy, QosPolicyBuilder};
use std::sync::Arc;
use tokio::sync::Mutex;

pub struct TaskClient {
    _config: Config,
    node: Arc<Mutex<Option<Node>>>,
    submit_client: Arc<
        Mutex<
            Option<ros2_client::service::Client<AService<SubmitTaskRequest, SubmitTaskResponse>>>,
        >,
    >,
    status_client: Arc<
        Mutex<
            Option<ros2_client::service::Client<AService<TaskStatusRequest, TaskStatusResponse>>>,
        >,
    >,
    result_client: Arc<
        Mutex<
            Option<ros2_client::service::Client<AService<TaskResultRequest, TaskResultResponse>>>,
        >,
    >,
}

impl TaskClient {
    pub fn new(config: Config) -> Result<Self> {
        Ok(Self {
            _config: config,
            node: Arc::new(Mutex::new(None)),
            submit_client: Arc::new(Mutex::new(None)),
            status_client: Arc::new(Mutex::new(None)),
            result_client: Arc::new(Mutex::new(None)),
        })
    }

    async fn ensure_clients(&self) -> Result<()> {
        let mut node_guard = self.node.lock().await;
        let mut submit_guard = self.submit_client.lock().await;
        let mut status_guard = self.status_client.lock().await;
        let mut result_guard = self.result_client.lock().await;

        if node_guard.is_none() {
            let context = Context::new()
                .map_err(|e| anyhow::anyhow!("Failed to create ROS2 context: {:?}", e))?;

            let mut node = context
                .new_node(
                    NodeName::new("/rbnx", "rbnx_cli_task").unwrap(),
                    NodeOptions::new().enable_rosout(false),
                )
                .map_err(|e| anyhow::anyhow!("Failed to create ROS2 node: {:?}", e))?;

            // Start spinner in background
            let spinner = node
                .spinner()
                .map_err(|e| anyhow::anyhow!("Failed to get node spinner: {:?}", e))?;
            tokio::spawn(async move {
                let _ = spinner.spin().await;
            });

            // Create service clients
            let service_qos = QosPolicyBuilder::new()
                .reliability(policy::Reliability::Reliable {
                    max_blocking_time: rustdds::Duration::from_millis(100),
                })
                .history(policy::History::KeepLast { depth: 1 })
                .build();

            let submit_client = node
                .create_client::<AService<SubmitTaskRequest, SubmitTaskResponse>>(
                    ServiceMapping::Enhanced,
                    &Name::new("/rbnx/task", "submit").unwrap(),
                    &ServiceTypeName::new("robonix_sdk", "SubmitTask"),
                    service_qos.clone(),
                    service_qos.clone(),
                )
                .map_err(|e| anyhow::anyhow!("Failed to create submit_task client: {:?}", e))?;

            let status_client = node
                .create_client::<AService<TaskStatusRequest, TaskStatusResponse>>(
                    ServiceMapping::Enhanced,
                    &Name::new("/rbnx/task", "status").unwrap(),
                    &ServiceTypeName::new("robonix_sdk", "TaskStatus"),
                    service_qos.clone(),
                    service_qos.clone(),
                )
                .map_err(|e| anyhow::anyhow!("Failed to create task_status client: {:?}", e))?;

            let result_client = node
                .create_client::<AService<TaskResultRequest, TaskResultResponse>>(
                    ServiceMapping::Enhanced,
                    &Name::new("/rbnx/task", "result").unwrap(),
                    &ServiceTypeName::new("robonix_sdk", "TaskResult"),
                    service_qos.clone(),
                    service_qos,
                )
                .map_err(|e| anyhow::anyhow!("Failed to create task_result client: {:?}", e))?;

            // Wait for services to be available (with timeout)
            tracing::debug!("Waiting for task services to be available...");
            let wait_submit = submit_client.wait_for_service(&node);
            let wait_status = status_client.wait_for_service(&node);
            let wait_result = result_client.wait_for_service(&node);
            let timeout_future = tokio::time::sleep(tokio::time::Duration::from_secs(5));

            tokio::select! {
                _ = wait_submit => {
                    tracing::debug!("Submit task service is available");
                }
                _ = timeout_future => {
                    tracing::warn!("Submit task service not available after 5 seconds, continuing anyway...");
                }
            }
            tokio::select! {
                _ = wait_status => {
                    tracing::debug!("Task status service is available");
                }
                _ = tokio::time::sleep(tokio::time::Duration::from_secs(1)) => {
                    tracing::debug!("Task status service wait timeout, continuing...");
                }
            }
            tokio::select! {
                _ = wait_result => {
                    tracing::debug!("Task result service is available");
                }
                _ = tokio::time::sleep(tokio::time::Duration::from_secs(1)) => {
                    tracing::debug!("Task result service wait timeout, continuing...");
                }
            }

            *node_guard = Some(node);
            *submit_guard = Some(submit_client);
            *status_guard = Some(status_client);
            *result_guard = Some(result_client);
        }

        Ok(())
    }

    pub async fn submit(
        &self,
        description: String,
        params: serde_json::Value,
    ) -> Result<SubmitTaskResponse> {
        self.ensure_clients().await?;

        // Serialize params to JSON string
        let params_str = serde_json::to_string(&params).unwrap_or_else(|_| "{}".to_string());
        let request = SubmitTaskRequest {
            description,
            params: params_str,
        };

        let client_guard: tokio::sync::MutexGuard<
            '_,
            Option<ros2_client::service::Client<AService<SubmitTaskRequest, SubmitTaskResponse>>>,
        > = self.submit_client.lock().await;
        let client = client_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("Submit client not initialized"))?;

        let node_guard = self.node.lock().await;
        let _node = node_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("ROS2 node not initialized"))?;

        tracing::info!("Calling submit_task service");
        let call_result = tokio::time::timeout(
            tokio::time::Duration::from_secs(30),
            client.async_call_service(request),
        )
        .await;

        match call_result {
            Ok(Ok(response)) => {
                tracing::info!("Received response: task_id={}", response.task_id);
                Ok(response)
            }
            Ok(Err(e)) => {
                tracing::error!("Service call error: {:?}", e);
                anyhow::bail!("Service call error: {:?}", e);
            }
            Err(_) => {
                tracing::error!("Service call timeout after 30 seconds");
                anyhow::bail!(
                    "Service call timeout after 30 seconds. Make sure robonix-core is running: \
                    Run 'cargo run --bin robonix-core' in the robonix-core directory."
                );
            }
        }
    }

    pub async fn status(&self, task_id: String) -> Result<TaskStatusResponse> {
        self.ensure_clients().await?;

        let request = TaskStatusRequest { task_id };

        let client_guard: tokio::sync::MutexGuard<
            '_,
            Option<ros2_client::service::Client<AService<TaskStatusRequest, TaskStatusResponse>>>,
        > = self.status_client.lock().await;
        let client = client_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("Status client not initialized"))?;

        let node_guard = self.node.lock().await;
        let _node = node_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("ROS2 node not initialized"))?;

        tracing::info!("Calling task_status service");
        let call_result = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await;

        match call_result {
            Ok(Ok(response)) => {
                tracing::info!("Received response: status={}", response.status);
                Ok(response)
            }
            Ok(Err(e)) => {
                tracing::error!("Service call error: {:?}", e);
                anyhow::bail!("Service call error: {:?}", e);
            }
            Err(_) => {
                tracing::error!("Service call timeout after 10 seconds");
                anyhow::bail!("Service call timeout after 10 seconds");
            }
        }
    }

    pub async fn result(&self, task_id: String) -> Result<TaskResultResponse> {
        self.ensure_clients().await?;

        let request = TaskResultRequest { task_id };

        let client_guard: tokio::sync::MutexGuard<
            '_,
            Option<ros2_client::service::Client<AService<TaskResultRequest, TaskResultResponse>>>,
        > = self.result_client.lock().await;
        let client = client_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("Result client not initialized"))?;

        let node_guard = self.node.lock().await;
        let _node = node_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("ROS2 node not initialized"))?;

        tracing::info!("Calling task_result service");
        let call_result = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await;

        match call_result {
            Ok(Ok(response)) => {
                tracing::info!("Received response: result={:?}", response.result);
                Ok(response)
            }
            Ok(Err(e)) => {
                tracing::error!("Service call error: {:?}", e);
                anyhow::bail!("Service call error: {:?}", e);
            }
            Err(_) => {
                tracing::error!("Service call timeout after 10 seconds");
                anyhow::bail!("Service call timeout after 10 seconds");
            }
        }
    }
}
