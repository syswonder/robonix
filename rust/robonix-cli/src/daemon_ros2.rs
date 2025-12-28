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
    service::AService, Context, Name, Node, NodeName, NodeOptions, ServiceMapping, ServiceTypeName,
};
use rustdds::{policy, QosPolicyBuilder};
use std::sync::Arc;
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
        let service_qos = QosPolicyBuilder::new()
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: rustdds::Duration::from_millis(100),
            })
            .history(policy::History::KeepLast { depth: 1 })
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
        })
    }

    pub async fn call_register_primitive(
        &self,
        request: RegisterPrimitiveRequest,
    ) -> Result<RegisterPrimitiveResponse> {
        let client = self.primitive_client.lock().await;
        let response = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await
        .map_err(|e| anyhow::anyhow!("Timeout: {}", e))?
        .map_err(|e| anyhow::anyhow!("Service call error: {:?}", e))?;
        Ok(response)
    }

    pub async fn call_register_service(
        &self,
        request: RegisterServiceRequest,
    ) -> Result<RegisterServiceResponse> {
        let client = self.service_client.lock().await;
        let response = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await
        .map_err(|e| anyhow::anyhow!("Timeout: {}", e))?
        .map_err(|e| anyhow::anyhow!("Service call error: {:?}", e))?;
        Ok(response)
    }

    pub async fn call_register_skill(
        &self,
        request: RegisterSkillRequest,
    ) -> Result<RegisterSkillResponse> {
        let client = self.skill_client.lock().await;
        let response = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await
        .map_err(|e| anyhow::anyhow!("Timeout: {}", e))?
        .map_err(|e| anyhow::anyhow!("Service call error: {:?}", e))?;
        Ok(response)
    }

    pub async fn call_submit_task(&self, request: SubmitTaskRequest) -> Result<SubmitTaskResponse> {
        let client = self.submit_task_client.lock().await;
        let response = tokio::time::timeout(
            tokio::time::Duration::from_secs(30),
            client.async_call_service(request),
        )
        .await
        .map_err(|e| anyhow::anyhow!("Timeout: {}", e))?
        .map_err(|e| anyhow::anyhow!("Service call error: {:?}", e))?;
        Ok(response)
    }

    pub async fn call_task_data(&self, request: TaskDataRequest) -> Result<TaskDataResponse> {
        let client = self.task_data_client.lock().await;
        let response = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await
        .map_err(|e| anyhow::anyhow!("Timeout: {}", e))?
        .map_err(|e| anyhow::anyhow!("Service call error: {:?}", e))?;
        Ok(response)
    }
}
