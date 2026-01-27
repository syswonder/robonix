// SPDX-License-Identifier: MulanPSL-2.0
// Daemon ROS2 Module
//
// Manages persistent ROS2 node and service clients for daemon

use anyhow::Result;
use robonix_core::ros_idl::primitive::{RegisterPrimitiveRequest, RegisterPrimitiveResponse};
use robonix_core::ros_idl::service_registry::{RegisterServiceRequest, RegisterServiceResponse};
use robonix_core::ros_idl::skill::{RegisterSkillRequest, RegisterSkillResponse};
use robonix_core::ros_idl::task::{
    SubmitTaskRequest, SubmitTaskResponse, TaskDataRequest, TaskDataResponse,
};
use ros2_client::{
    Context, Name, Node, NodeName, NodeOptions, ServiceMapping, ServiceTypeName, service::AService,
};
use rustdds::{
    Duration, QosPolicyBuilder,
    policy::{self, Deadline, Lifespan},
};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use tokio::sync::Mutex;

pub struct DaemonRos2Clients {
    _node: Arc<Mutex<Node>>, // Keep node alive to maintain ROS2 connection
    primitive_client: Arc<
        Mutex<
            ros2_client::service::Client<
                AService<RegisterPrimitiveRequest, RegisterPrimitiveResponse>,
            >,
        >,
    >,
    service_client: Arc<
        Mutex<
            ros2_client::service::Client<AService<RegisterServiceRequest, RegisterServiceResponse>>,
        >,
    >,
    skill_client: Arc<
        Mutex<ros2_client::service::Client<AService<RegisterSkillRequest, RegisterSkillResponse>>>,
    >,
    submit_task_client:
        Arc<Mutex<ros2_client::service::Client<AService<SubmitTaskRequest, SubmitTaskResponse>>>>,
    task_data_client:
        Arc<Mutex<ros2_client::service::Client<AService<TaskDataRequest, TaskDataResponse>>>>,
    discovery_waited: Arc<AtomicBool>, // Track if we've already waited for service discovery
}

impl DaemonRos2Clients {
    pub async fn new() -> Result<Self> {
        let context = Context::new()
            .map_err(|e| anyhow::anyhow!("Failed to create ROS2 context: {:?}", e))?;

        let mut node = context
            .new_node(
                NodeName::new("/rbnx", "rbnx_daemon").unwrap(),
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
        // Match QoS settings with robonix-core server for compatibility
        let service_qos = QosPolicyBuilder::new()
            .history(policy::History::KeepLast { depth: 10 })
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: Duration::from_millis(100),
            })
            .durability(policy::Durability::Volatile)
            .deadline(Deadline(Duration::INFINITE))
            .lifespan(Lifespan {
                duration: Duration::INFINITE,
            })
            .liveliness(policy::Liveliness::Automatic {
                lease_duration: Duration::INFINITE,
            })
            .build();

        let primitive_client = node
            .create_client::<AService<RegisterPrimitiveRequest, RegisterPrimitiveResponse>>(
                ServiceMapping::Enhanced,
                &Name::new("/rbnx/prm", "register").unwrap(),
                &ServiceTypeName::new("robonix_sdk", "RegisterPrimitive"),
                service_qos.clone(),
                service_qos.clone(),
            )
            .map_err(|e| anyhow::anyhow!("Failed to create primitive register client: {:?}", e))?;

        let service_client = node
            .create_client::<AService<RegisterServiceRequest, RegisterServiceResponse>>(
                ServiceMapping::Enhanced,
                &Name::new("/rbnx/srv", "register").unwrap(),
                &ServiceTypeName::new("robonix_sdk", "RegisterService"),
                service_qos.clone(),
                service_qos.clone(),
            )
            .map_err(|e| anyhow::anyhow!("Failed to create service register client: {:?}", e))?;

        let skill_client = node
            .create_client::<AService<RegisterSkillRequest, RegisterSkillResponse>>(
                ServiceMapping::Enhanced,
                &Name::new("/rbnx/skl", "register").unwrap(),
                &ServiceTypeName::new("robonix_sdk", "RegisterSkill"),
                service_qos.clone(),
                service_qos.clone(),
            )
            .map_err(|e| anyhow::anyhow!("Failed to create skill register client: {:?}", e))?;

        let submit_task_client = node
            .create_client::<AService<SubmitTaskRequest, SubmitTaskResponse>>(
                ServiceMapping::Enhanced,
                &Name::new("/rbnx/task", "submit").unwrap(),
                &ServiceTypeName::new("robonix_sdk", "SubmitTask"),
                service_qos.clone(),
                service_qos.clone(),
            )
            .map_err(|e| anyhow::anyhow!("Failed to create submit_task client: {:?}", e))?;

        let task_data_client = node
            .create_client::<AService<TaskDataRequest, TaskDataResponse>>(
                ServiceMapping::Enhanced,
                &Name::new("/rbnx/task", "data").unwrap(),
                &ServiceTypeName::new("robonix_sdk", "TaskData"),
                service_qos.clone(),
                service_qos.clone(),
            )
            .map_err(|e| anyhow::anyhow!("Failed to create task_data client: {:?}", e))?;

        Ok(Self {
            _node: Arc::new(Mutex::new(node)),
            primitive_client: Arc::new(Mutex::new(primitive_client)),
            service_client: Arc::new(Mutex::new(service_client)),
            skill_client: Arc::new(Mutex::new(skill_client)),
            submit_task_client: Arc::new(Mutex::new(submit_task_client)),
            task_data_client: Arc::new(Mutex::new(task_data_client)),
            discovery_waited: Arc::new(AtomicBool::new(false)),
        })
    }

    pub async fn call_register_primitive(
        &self,
        request: RegisterPrimitiveRequest,
    ) -> Result<RegisterPrimitiveResponse> {
        // Wait for service discovery and retry if needed
        self.wait_for_service_discovery().await;

        // Retry logic: ROS2 service discovery can be slow
        let max_retries = 3;
        let mut last_error = None;

        for attempt in 0..max_retries {
            let client = self.primitive_client.lock().await;
            let result = tokio::time::timeout(
                tokio::time::Duration::from_secs(10),
                client.async_call_service(request.clone()),
            )
            .await;

            match result {
                Ok(Ok(response)) => return Ok(response),
                Ok(Err(e)) => {
                    last_error = Some(format!("Service call error: {:?}", e));
                    if attempt < max_retries - 1 {
                        tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
                        continue;
                    }
                }
                Err(_) => {
                    last_error = Some("Timeout: Service call timed out".to_string());
                    if attempt < max_retries - 1 {
                        tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
                        continue;
                    }
                }
            }
        }

        Err(anyhow::anyhow!(
            "Service call to /rbnx/prm/register failed after {} attempts. {}. \
            Please ensure robonix-core is running. Start it with: robonix-core",
            max_retries,
            last_error.unwrap_or_else(|| "Unknown error".to_string())
        ))
    }

    pub async fn call_register_service(
        &self,
        request: RegisterServiceRequest,
    ) -> Result<RegisterServiceResponse> {
        // Wait for service discovery and retry if needed
        self.wait_for_service_discovery().await;

        // Retry logic: ROS2 service discovery can be slow
        let max_retries = 3;
        let mut last_error = None;

        for attempt in 0..max_retries {
            let client = self.service_client.lock().await;
            let result = tokio::time::timeout(
                tokio::time::Duration::from_secs(10),
                client.async_call_service(request.clone()),
            )
            .await;

            match result {
                Ok(Ok(response)) => return Ok(response),
                Ok(Err(e)) => {
                    last_error = Some(format!("Service call error: {:?}", e));
                    // If it's a service discovery error, wait and retry
                    if attempt < max_retries - 1 {
                        tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
                        continue;
                    }
                }
                Err(_) => {
                    last_error = Some("Timeout: Service call timed out".to_string());
                    // If timeout, wait and retry (might be service discovery issue)
                    if attempt < max_retries - 1 {
                        tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
                        continue;
                    }
                }
            }
        }

        Err(anyhow::anyhow!(
            "Service call to /rbnx/srv/register failed after {} attempts. {}. \
            Please ensure robonix-core is running. Start it with: robonix-core",
            max_retries,
            last_error.unwrap_or_else(|| "Unknown error".to_string())
        ))
    }

    pub async fn call_register_skill(
        &self,
        request: RegisterSkillRequest,
    ) -> Result<RegisterSkillResponse> {
        // Wait for service discovery before calling
        self.wait_for_service_discovery().await;

        let client = self.skill_client.lock().await;
        let response = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await
        .map_err(|_| {
            anyhow::anyhow!(
                "Timeout: Service call to /rbnx/skl/register timed out after 10 seconds. \
                Please ensure robonix-core is running. Start it with: robonix-core"
            )
        })?
        .map_err(|e| anyhow::anyhow!("Service call error: {:?}", e))?;
        Ok(response)
    }

    pub async fn call_submit_task(&self, request: SubmitTaskRequest) -> Result<SubmitTaskResponse> {
        // Wait for service discovery before calling
        self.wait_for_service_discovery().await;

        let client = self.submit_task_client.lock().await;
        let response = tokio::time::timeout(
            tokio::time::Duration::from_secs(30),
            client.async_call_service(request),
        )
        .await
        .map_err(|_| {
            anyhow::anyhow!(
                "Timeout: Service call to /rbnx/task/submit timed out after 30 seconds. \
                Please ensure robonix-core is running. Start it with: robonix-core"
            )
        })?
        .map_err(|e| anyhow::anyhow!("Service call error: {:?}", e))?;
        Ok(response)
    }

    pub async fn call_task_data(&self, request: TaskDataRequest) -> Result<TaskDataResponse> {
        // Wait for service discovery before calling
        self.wait_for_service_discovery().await;

        let client = self.task_data_client.lock().await;
        let response = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await
        .map_err(|_| {
            anyhow::anyhow!(
                "Timeout: Service call to /rbnx/task/data timed out after 10 seconds. \
                Please ensure robonix-core is running. Start it with: robonix-core"
            )
        })?
        .map_err(|e| anyhow::anyhow!("Service call error: {:?}", e))?;
        Ok(response)
    }

    /// Wait for ROS2 service discovery to complete
    /// This gives time for the ROS2 DDS discovery process to find services
    /// Only waits once per DaemonRos2Clients instance (first call)
    async fn wait_for_service_discovery(&self) {
        // Only wait on the first call - subsequent calls don't need to wait
        // as service discovery is a one-time process
        if !self.discovery_waited.swap(true, Ordering::Relaxed) {
            // ROS2 service discovery typically takes 1-5 seconds, especially when
            // daemon starts before robonix-core or when they start simultaneously.
            // We wait a reasonable time to allow discovery to complete.
            // The spinner is already running in the background to help with discovery.
            tokio::time::sleep(tokio::time::Duration::from_secs(3)).await;
        }
    }
}
