// SPDX-License-Identifier: MulanPSL-2.0
// Task Manager Module
//
// Task Manager is the global scheduling and control core of the system,
// responsible for receiving user tasks, completing task parsing and planning,
// and coordinating task execution in the "planning-deduction-decision-execution-feedback" loop.

pub mod api;
pub mod task;

use crate::skill_library::SkillLibrary;
use crate::service::ServiceRegistry;
use crate::primitive::PrimitiveRegistry;
use std::sync::Arc;

/// Task Manager - Global scheduling and control core
pub struct TaskManager {
    task_store: Arc<task::TaskStore>,
    skill_library: Arc<SkillLibrary>,
    service_registry: Arc<ServiceRegistry>,
    primitive_registry: Arc<PrimitiveRegistry>,
}

impl TaskManager {
    pub fn new(
        skill_library: Arc<SkillLibrary>,
        service_registry: Arc<ServiceRegistry>,
        primitive_registry: Arc<PrimitiveRegistry>,
    ) -> Self {
        Self {
            task_store: Arc::new(task::TaskStore::new()),
            skill_library,
            service_registry,
            primitive_registry,
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
        // Parse params JSON string
        let params: serde_json::Value = match serde_json::from_str(&req.params) {
            Ok(v) => v,
            Err(_) => serde_json::json!({}), // Default to empty object if parsing fails
        };
        let task_id = self.task_store.create_task(req.description, params).await;
        api::SubmitTaskResponse { task_id }
    }

    /// Get task status
    pub async fn get_task_status(&self, req: api::TaskStatusRequest) -> api::TaskStatusResponse {
        let status = if let Some(task) = self.task_store.get_task(&req.task_id).await {
            match task.state {
                task::TaskState::Pending => "pending".to_string(),
                task::TaskState::Planning => "planning".to_string(),
                task::TaskState::Running => "running".to_string(),
                task::TaskState::Finished => "finished".to_string(),
                task::TaskState::Failed => "failed".to_string(),
                task::TaskState::Cancelled => "cancelled".to_string(),
            }
        } else {
            "not_found".to_string()
        };
        api::TaskStatusResponse { status }
    }

    /// Get task result
    pub async fn get_task_result(&self, req: api::TaskResultRequest) -> api::TaskResultResponse {
        let result_value = if let Some(task) = self.task_store.get_task(&req.task_id).await {
            task.result.unwrap_or(serde_json::json!({}))
        } else {
            serde_json::json!({"error": "Task not found"})
        };
        // Serialize to JSON string
        let result = serde_json::to_string(&result_value).unwrap_or_else(|_| "{}".to_string());
        api::TaskResultResponse { result }
    }
}

// Re-export task types
pub use task::{Task, TaskState};
pub use api::{SubmitTaskRequest, SubmitTaskResponse, TaskStatusRequest, TaskStatusResponse, TaskResultRequest, TaskResultResponse};

