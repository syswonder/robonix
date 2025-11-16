use crate::config::Config;
use anyhow::Result;
use robonix_core::messages::{
    ModelType, QueryModelRequest, QueryModelResponse, RegisterModelRequest, RegisterModelResponse,
};
use ros2_client::{
    service::AService, Context, Name, Node, NodeName, NodeOptions, ServiceMapping, ServiceTypeName,
};
use rustdds::{policy, QosPolicyBuilder};
use std::sync::Arc;
use tokio::sync::Mutex;

pub struct ModelClient {
    _config: Config,
    node: Arc<Mutex<Option<Node>>>,
    register_client: Arc<
        Mutex<
            Option<
                ros2_client::service::Client<AService<RegisterModelRequest, RegisterModelResponse>>,
            >,
        >,
    >,
    query_client: Arc<
        Mutex<
            Option<ros2_client::service::Client<AService<QueryModelRequest, QueryModelResponse>>>,
        >,
    >,
}

impl ModelClient {
    pub fn new(config: Config) -> Result<Self> {
        Ok(Self {
            _config: config,
            node: Arc::new(Mutex::new(None)),
            register_client: Arc::new(Mutex::new(None)),
            query_client: Arc::new(Mutex::new(None)),
        })
    }

    async fn ensure_clients(&self) -> Result<()> {
        let mut node_guard = self.node.lock().await;
        let mut register_guard = self.register_client.lock().await;
        let mut query_guard = self.query_client.lock().await;

        if node_guard.is_none() {
            let context = Context::new()
                .map_err(|e| anyhow::anyhow!("Failed to create ROS2 context: {:?}", e))?;

            let mut node = context
                .new_node(
                    NodeName::new("/rbnx", "rbnx_cli_model").unwrap(),
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

            let register_client = node
                .create_client::<AService<RegisterModelRequest, RegisterModelResponse>>(
                    ServiceMapping::Enhanced,
                    &Name::new("/rbnx/srv/mgmt", "register_model").unwrap(),
                    &ServiceTypeName::new("robonix_core", "RegisterModel"),
                    service_qos.clone(),
                    service_qos.clone(),
                )
                .map_err(|e| {
                    anyhow::anyhow!("Failed to create register model service client: {:?}", e)
                })?;

            let query_client = node
                .create_client::<AService<QueryModelRequest, QueryModelResponse>>(
                    ServiceMapping::Enhanced,
                    &Name::new("/rbnx/srv/mgmt", "query_model").unwrap(),
                    &ServiceTypeName::new("robonix_core", "QueryModel"),
                    service_qos.clone(),
                    service_qos,
                )
                .map_err(|e| {
                    anyhow::anyhow!("Failed to create query model service client: {:?}", e)
                })?;

            // Wait for services to be available (with timeout)
            tracing::debug!("Waiting for model services to be available...");
            let wait_register = register_client.wait_for_service(&node);
            let wait_query = query_client.wait_for_service(&node);
            let timeout_future = tokio::time::sleep(tokio::time::Duration::from_secs(5));
            tokio::select! {
                _ = wait_register => {
                    tracing::debug!("Register model service is available");
                }
                _ = timeout_future => {
                    tracing::warn!("Register model service not available after 5 seconds, continuing anyway...");
                }
            }
            tokio::select! {
                _ = wait_query => {
                    tracing::debug!("Query model service is available");
                }
                _ = tokio::time::sleep(tokio::time::Duration::from_secs(5)) => {
                    tracing::warn!("Query model service not available after 5 seconds, continuing anyway...");
                }
            }

            *node_guard = Some(node);
            *register_guard = Some(register_client);
            *query_guard = Some(query_client);
        }

        Ok(())
    }

    pub async fn register(
        &self,
        model_id: String,
        model_name: String,
        model_type: ModelType,
        provider: String,
        api_endpoint: String,
        api_key: Option<String>,
        description: String,
        capabilities: Vec<String>,
    ) -> Result<()> {
        self.ensure_clients().await?;

        let request = RegisterModelRequest {
            model_id,
            model_name,
            model_type,
            provider,
            api_endpoint,
            api_key,
            description,
            capabilities,
        };

        let client_guard = self.register_client.lock().await;
        let client = client_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("Register client not initialized"))?;

        let node_guard = self.node.lock().await;
        let _node = node_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("ROS2 node not initialized"))?;

        tracing::info!("Calling register model service");
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
                if !response.success {
                    anyhow::bail!("Registration failed: {}", response.error_message);
                }
                Ok(())
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

    pub async fn query(
        &self,
        model_id: Option<String>,
        model_type: Option<ModelType>,
        capability: Option<String>,
    ) -> Result<QueryModelResponse> {
        self.ensure_clients().await?;

        let request = QueryModelRequest {
            model_id,
            model_type,
            capability,
        };

        let client_guard = self.query_client.lock().await;
        let client = client_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("Query client not initialized"))?;

        let node_guard = self.node.lock().await;
        let _node = node_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("ROS2 node not initialized"))?;

        tracing::info!("Calling query model service");
        let call_result = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await;

        match call_result {
            Ok(Ok(response)) => {
                tracing::info!(
                    "Received response: success={}, models={}",
                    response.success,
                    response.models.len()
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
