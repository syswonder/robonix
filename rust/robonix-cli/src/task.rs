use crate::config::Config;
use anyhow::Result;
use robonix_core::planning::{
    CancelTaskRequest, CancelTaskResponse, CreateTaskRequest, CreateTaskResponse,
    GetTaskRequest, GetTaskResponse, ListTasksRequest, ListTasksResponse,
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
    create_client: Arc<
        Mutex<
            Option<
                ros2_client::service::Client<AService<CreateTaskRequest, CreateTaskResponse>>,
            >,
        >,
    >,
    get_client: Arc<
        Mutex<
            Option<ros2_client::service::Client<AService<GetTaskRequest, GetTaskResponse>>>,
        >,
    >,
    list_client: Arc<
        Mutex<
            Option<ros2_client::service::Client<AService<ListTasksRequest, ListTasksResponse>>>,
        >,
    >,
    cancel_client: Arc<
        Mutex<
            Option<ros2_client::service::Client<AService<CancelTaskRequest, CancelTaskResponse>>>,
        >,
    >,
}

impl TaskClient {
    pub fn new(config: Config) -> Result<Self> {
        Ok(Self {
            _config: config,
            node: Arc::new(Mutex::new(None)),
            create_client: Arc::new(Mutex::new(None)),
            get_client: Arc::new(Mutex::new(None)),
            list_client: Arc::new(Mutex::new(None)),
            cancel_client: Arc::new(Mutex::new(None)),
        })
    }

    async fn ensure_clients(&self) -> Result<()> {
        let mut node_guard = self.node.lock().await;
        let mut create_guard = self.create_client.lock().await;
        let mut get_guard = self.get_client.lock().await;
        let mut list_guard = self.list_client.lock().await;
        let mut cancel_guard = self.cancel_client.lock().await;

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

            let create_client = node
                .create_client::<AService<CreateTaskRequest, CreateTaskResponse>>(
                    ServiceMapping::Enhanced,
                    &Name::new("/rbnx/srv/planning", "create_task").unwrap(),
                    &ServiceTypeName::new("robonix_core", "CreateTask"),
                    service_qos.clone(),
                    service_qos.clone(),
                )
                .map_err(|e| anyhow::anyhow!("Failed to create create_task client: {:?}", e))?;

            let get_client = node
                .create_client::<AService<GetTaskRequest, GetTaskResponse>>(
                    ServiceMapping::Enhanced,
                    &Name::new("/rbnx/srv/planning", "get_task").unwrap(),
                    &ServiceTypeName::new("robonix_core", "GetTask"),
                    service_qos.clone(),
                    service_qos.clone(),
                )
                .map_err(|e| anyhow::anyhow!("Failed to create get_task client: {:?}", e))?;

            let list_client = node
                .create_client::<AService<ListTasksRequest, ListTasksResponse>>(
                    ServiceMapping::Enhanced,
                    &Name::new("/rbnx/srv/planning", "list_tasks").unwrap(),
                    &ServiceTypeName::new("robonix_core", "ListTasks"),
                    service_qos.clone(),
                    service_qos.clone(),
                )
                .map_err(|e| anyhow::anyhow!("Failed to create list_tasks client: {:?}", e))?;

            let cancel_client = node
                .create_client::<AService<CancelTaskRequest, CancelTaskResponse>>(
                    ServiceMapping::Enhanced,
                    &Name::new("/rbnx/srv/planning", "cancel_task").unwrap(),
                    &ServiceTypeName::new("robonix_core", "CancelTask"),
                    service_qos.clone(),
                    service_qos,
                )
                .map_err(|e| anyhow::anyhow!("Failed to create cancel_task client: {:?}", e))?;

            // Wait for services to be available (with timeout)
            tracing::debug!("Waiting for task services to be available...");
            let wait_create = create_client.wait_for_service(&node);
            let wait_get = get_client.wait_for_service(&node);
            let wait_list = list_client.wait_for_service(&node);
            let wait_cancel = cancel_client.wait_for_service(&node);
            let timeout_future = tokio::time::sleep(tokio::time::Duration::from_secs(5));
            
            tokio::select! {
                _ = wait_create => {
                    tracing::debug!("Create task service is available");
                }
                _ = timeout_future => {
                    tracing::warn!("Create task service not available after 5 seconds, continuing anyway...");
                }
            }
            tokio::select! {
                _ = wait_get => {
                    tracing::debug!("Get task service is available");
                }
                _ = tokio::time::sleep(tokio::time::Duration::from_secs(1)) => {
                    tracing::debug!("Get task service wait timeout, continuing...");
                }
            }
            tokio::select! {
                _ = wait_list => {
                    tracing::debug!("List tasks service is available");
                }
                _ = tokio::time::sleep(tokio::time::Duration::from_secs(1)) => {
                    tracing::debug!("List tasks service wait timeout, continuing...");
                }
            }
            tokio::select! {
                _ = wait_cancel => {
                    tracing::debug!("Cancel task service is available");
                }
                _ = tokio::time::sleep(tokio::time::Duration::from_secs(1)) => {
                    tracing::debug!("Cancel task service wait timeout, continuing...");
                }
            }

            *node_guard = Some(node);
            *create_guard = Some(create_client);
            *get_guard = Some(get_client);
            *list_guard = Some(list_client);
            *cancel_guard = Some(cancel_client);
        }

        Ok(())
    }

    pub async fn create(&self, natural_language: String) -> Result<CreateTaskResponse> {
        self.ensure_clients().await?;

        let request = CreateTaskRequest { natural_language };

        let client_guard = self.create_client.lock().await;
        let client = client_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("Create client not initialized"))?;

        let node_guard = self.node.lock().await;
        let _node = node_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("ROS2 node not initialized"))?;

        tracing::info!("Calling create_task service");
        let call_result = tokio::time::timeout(
            tokio::time::Duration::from_secs(30),
            client.async_call_service(request),
        )
        .await;

        match call_result {
            Ok(Ok(response)) => {
                tracing::info!(
                    "Received response: success={}, task_id={}",
                    response.success,
                    response.task_id
                );
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

    pub async fn get(&self, task_id: String) -> Result<GetTaskResponse> {
        self.ensure_clients().await?;

        let request = GetTaskRequest { task_id };

        let client_guard = self.get_client.lock().await;
        let client = client_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("Get client not initialized"))?;

        let node_guard = self.node.lock().await;
        let _node = node_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("ROS2 node not initialized"))?;

        tracing::info!("Calling get_task service");
        let call_result = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await;

        match call_result {
            Ok(Ok(response)) => {
                tracing::info!(
                    "Received response: success={}, task={:?}",
                    response.success,
                    response.task.is_some()
                );
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

    pub async fn list(&self) -> Result<ListTasksResponse> {
        self.ensure_clients().await?;

        let request = ListTasksRequest {};

        let client_guard = self.list_client.lock().await;
        let client = client_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("List client not initialized"))?;

        let node_guard = self.node.lock().await;
        let _node = node_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("ROS2 node not initialized"))?;

        tracing::info!("Calling list_tasks service");
        let call_result = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await;

        match call_result {
            Ok(Ok(response)) => {
                tracing::info!(
                    "Received response: success={}, tasks={}",
                    response.success,
                    response.tasks.len()
                );
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

    pub async fn cancel(&self, task_id: String) -> Result<CancelTaskResponse> {
        self.ensure_clients().await?;

        let request = CancelTaskRequest { task_id };

        let client_guard = self.cancel_client.lock().await;
        let client = client_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("Cancel client not initialized"))?;

        let node_guard = self.node.lock().await;
        let _node = node_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("ROS2 node not initialized"))?;

        tracing::info!("Calling cancel_task service");
        let call_result = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await;

        match call_result {
            Ok(Ok(response)) => {
                tracing::info!(
                    "Received response: success={}, error={}",
                    response.success,
                    response.error_message
                );
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

