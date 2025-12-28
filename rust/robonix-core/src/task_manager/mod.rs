// SPDX-License-Identifier: MulanPSL-2.0
// Task Manager Module
//
// Task Manager is the global scheduling and control core of the system,
// responsible for receiving user tasks, completing task parsing and planning,
// and coordinating task execution in the "planning-deduction-decision-execution-feedback" loop.

pub mod api;
pub mod context;
pub mod exception;
pub mod executor;
pub mod queue;
pub mod runtime;
pub mod task;

use crate::node::create_node;
use crate::primitive::PrimitiveRegistry;
use crate::service::ServiceRegistry;
use crate::skill_library::SkillLibrary;
use std::sync::Arc;
use tokio::sync::Mutex;

/// Task Manager - Global scheduling and control core
pub struct TaskManager {
    task_store: Arc<task::TaskStore>,
    context_store: Arc<context::TaskContextStore>,
    task_queue: Arc<queue::TaskQueue>,
    #[allow(dead_code)] // Runtime runs in background, needs to be kept alive
    executor: Arc<executor::RtdlExecutor>,
    #[allow(dead_code)] // Runtime runs in background, needs to be kept alive
    runtime: Arc<runtime::TaskRuntime>,
    skill_library: Arc<SkillLibrary>,
    service_registry: Arc<ServiceRegistry>,
    primitive_registry: Arc<PrimitiveRegistry>,
    #[allow(dead_code)] // Node needs to be kept alive for ROS2 context
    node: Arc<Mutex<ros2_client::Node>>,
}

impl TaskManager {
    pub fn new(
        skill_library: Arc<SkillLibrary>,
        service_registry: Arc<ServiceRegistry>,
        primitive_registry: Arc<PrimitiveRegistry>,
    ) -> Self {
        let context_store = Arc::new(context::TaskContextStore::new());
        let task_queue = Arc::new(queue::TaskQueue::new());
        let executor = Arc::new(executor::RtdlExecutor::new(skill_library.clone()));
        let node = Arc::new(Mutex::new(create_node()));

        let runtime = Arc::new(runtime::TaskRuntime::new(
            context_store.clone(),
            task_queue.clone(),
            executor.clone(),
            service_registry.clone(),
            node.clone(),
        ));

        // Start runtime loop in background
        let runtime_clone = runtime.clone();
        tokio::spawn(async move {
            runtime_clone.run().await;
        });

        Self {
            task_store: Arc::new(task::TaskStore::new()),
            context_store,
            task_queue,
            executor,
            runtime,
            skill_library,
            service_registry,
            primitive_registry,
            node,
        }
    }

    pub fn get_task_store(&self) -> Arc<task::TaskStore> {
        self.task_store.clone()
    }

    pub fn get_skill_library(&self) -> Arc<SkillLibrary> {
        self.skill_library.clone()
    }

    pub fn get_service_registry(&self) -> Arc<ServiceRegistry> {
        self.service_registry.clone()
    }

    pub fn get_primitive_registry(&self) -> Arc<PrimitiveRegistry> {
        self.primitive_registry.clone()
    }

    /// Submit a new task
    pub async fn submit_task(&self, req: api::SubmitTaskRequest) -> api::SubmitTaskResponse {
        use log::debug;

        debug!(
            "submitting new task: description={}, params={}",
            req.description, req.params
        );

        // Parse params JSON string
        let params: serde_json::Value = match serde_json::from_str(&req.params) {
            Ok(v) => {
                debug!("parsed task params successfully");
                v
            }
            Err(e) => {
                debug!("failed to parse task params: {}, using empty object", e);
                serde_json::json!({}) // Default to empty object if parsing fails
            }
        };

        // Extract priority from params (default to 0)
        let priority = params.get("priority").and_then(|p| p.as_i64()).unwrap_or(0) as i32;
        debug!("task priority: {}", priority);

        // Create task in store
        let task_id = self
            .task_store
            .create_task(req.description.clone(), params)
            .await;
        debug!("created task in store: task_id={}", task_id);

        // Create task context
        self.context_store
            .create_context(task_id.clone(), req.description, priority)
            .await;
        debug!("created task context: task_id={}", task_id);

        // Enqueue task
        let task = self
            .task_store
            .get_task(&task_id)
            .await
            .expect("Task should exist after creation");
        self.task_queue
            .enqueue(task_id.clone(), priority, task.created_at)
            .await;
        debug!(
            "enqueued task: task_id={}, priority={}, queue_size={}",
            task_id,
            priority,
            self.task_queue.len().await
        );

        api::SubmitTaskResponse { task_id }
    }

    /// Get task data
    pub async fn get_task_data(&self, req: api::TaskDataRequest) -> api::TaskDataResponse {
        use log::debug;

        debug!("getting task result: task_id={}", req.task_id);

        // Build comprehensive task information
        let mut task_info = serde_json::json!({});

        // Get task from store
        if let Some(task) = self.task_store.get_task(&req.task_id).await {
            debug!("found task {} in task store", req.task_id);
            task_info["task_id"] = serde_json::json!(task.task_id);
            task_info["description"] = serde_json::json!(task.description);
            task_info["state"] = serde_json::json!(format!("{:?}", task.state));
            task_info["created_at"] = serde_json::json!(task.created_at);
            task_info["updated_at"] = serde_json::json!(task.updated_at);
            if let Some(ref error_msg) = task.error_message {
                task_info["error_message"] = serde_json::json!(error_msg);
            }
            if let Some(ref result) = task.result {
                task_info["result"] = result.clone();
            }
        } else {
            debug!("task {} not found in task store", req.task_id);
            task_info["error"] = serde_json::json!("Task not found");
        }

        // Get context information (more detailed runtime info)
        if let Some(context) = self.context_store.get_context(&req.task_id).await {
            debug!("found task {} in context store", req.task_id);
            task_info["execution_state"] =
                serde_json::json!(format!("{:?}", context.execution_state));
            task_info["priority"] = serde_json::json!(context.priority);
            task_info["retry_count"] = serde_json::json!(context.retry_count);
            task_info["rtdl_instruction_pointer"] =
                serde_json::json!(context.rtdl_instruction_pointer);

            if let Some(ref rtdl) = context.rtdl {
                task_info["rtdl"] = serde_json::json!(rtdl);
            }
            if let Some(ref rtdl_type) = context.rtdl_type {
                task_info["rtdl_type"] = serde_json::json!(rtdl_type);
            }

            // Object graph information
            task_info["object_graph"] = context.object_graph.clone();
            task_info["object_graph_updated_at"] =
                serde_json::json!(context.object_graph_updated_at);
            if let Some(arr) = context.object_graph.as_array() {
                task_info["object_graph_count"] = serde_json::json!(arr.len());
            } else {
                task_info["object_graph_count"] = serde_json::json!(0);
            }

            // Exception information
            if let Some(ref exception) = context.last_exception {
                task_info["last_exception"] = serde_json::json!(exception);
            }
        }

        // Serialize to JSON string
        let result = serde_json::to_string(&task_info).unwrap_or_else(|_| "{}".to_string());
        debug!(
            "task {} result serialized: {} bytes",
            req.task_id,
            result.len()
        );
        api::TaskDataResponse { data: result }
    }
}

// Re-export task types (from ros_idl)
pub use api::{SubmitTaskRequest, SubmitTaskResponse, TaskDataRequest, TaskDataResponse};
pub use task::{Task, TaskState};
