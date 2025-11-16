// SPDX-License-Identifier: MulanPSL-2.0
// Cognitive Planning Module
//
// This module manages tasks: natural language input -> Model -> DSL -> skill execution

pub mod dsl;
pub mod task;

use crate::mgmt::ManagementModule;
use crate::perception::PerceptionModule;
use dsl::DSLGenerator;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use task::TaskManager;
use tracing::{error, info};

/// Cognitive Planning Module
pub struct PlanningModule {
    task_manager: Arc<TaskManager>,
    dsl_generator: Arc<DSLGenerator>,
    mgmt: Option<Arc<ManagementModule>>,
    perception: Option<Arc<PerceptionModule>>,
}

impl PlanningModule {
    pub fn new() -> Self {
        Self {
            task_manager: Arc::new(TaskManager::new()),
            dsl_generator: Arc::new(DSLGenerator::new()),
            mgmt: None,
            perception: None,
        }
    }

    /// Set references to other modules
    pub fn set_mgmt(&mut self, mgmt: Arc<ManagementModule>) {
        self.mgmt = Some(mgmt.clone());
        self.dsl_generator.set_mgmt(mgmt);
    }

    pub fn set_perception(&mut self, perception: Arc<PerceptionModule>) {
        self.perception = Some(perception.clone());
        self.dsl_generator.set_perception(perception);
    }

    /// Create a new task from natural language
    pub async fn create_task(&self, natural_language: String) -> String {
        let task_id = self.task_manager.create_task(natural_language).await;
        info!(task_id = %task_id, "Created new task");
        task_id
    }

    /// Get task by ID
    pub async fn get_task(&self, task_id: &str) -> Option<task::Task> {
        self.task_manager.get_task(task_id).await
    }

    /// Get all tasks
    pub async fn get_all_tasks(&self) -> Vec<task::Task> {
        self.task_manager.get_all_tasks().await
    }

    /// Update task state
    pub async fn update_task_state(
        &self,
        task_id: &str,
        state: TaskState,
        error_message: Option<String>,
    ) -> bool {
        self.task_manager.update_task_state(task_id, state, error_message).await
    }

    /// Generate DSL code for a task using model
    pub async fn generate_dsl(&self, task_id: &str) -> Result<String, String> {
        let task = match self.task_manager.get_task(task_id).await {
            Some(t) => t,
            None => return Err(format!("Task {} not found", task_id)),
        };

        // Update state to generating
        self.task_manager
            .update_task_state(task_id, TaskState::Generating, None)
            .await;

        // Generate DSL using model
        match self.dsl_generator.generate_dsl(&task.natural_language).await {
            Ok(dsl_code) => {
                // Save DSL code to task
                self.task_manager.set_task_dsl(task_id, dsl_code.clone()).await;
                self.task_manager
                    .update_task_state(task_id, TaskState::Parsing, None)
                    .await;
                info!(task_id = %task_id, "Generated DSL code");
                Ok(dsl_code)
            }
            Err(e) => {
                error!(task_id = %task_id, error = %e, "Failed to generate DSL");
                self.task_manager
                    .update_task_state(task_id, TaskState::Failed, Some(e.clone()))
                    .await;
                Err(e)
            }
        }
    }

    /// Cancel a task
    pub async fn cancel_task(&self, task_id: &str) -> bool {
        self.task_manager
            .update_task_state(task_id, TaskState::Cancelled, None)
            .await
    }
}

// Re-export types
pub use task::{Task, TaskState};

// Service message types

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CreateTaskRequest {
    pub natural_language: String,
}
impl ros2_client::Message for CreateTaskRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CreateTaskResponse {
    pub success: bool,
    pub error_message: String,
    pub task_id: String,
}
impl ros2_client::Message for CreateTaskResponse {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetTaskRequest {
    pub task_id: String,
}
impl ros2_client::Message for GetTaskRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetTaskResponse {
    pub success: bool,
    pub error_message: String,
    pub task: Option<task::Task>,
}
impl ros2_client::Message for GetTaskResponse {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ListTasksRequest {}
impl ros2_client::Message for ListTasksRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ListTasksResponse {
    pub success: bool,
    pub error_message: String,
    pub tasks: Vec<task::Task>,
}
impl ros2_client::Message for ListTasksResponse {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CancelTaskRequest {
    pub task_id: String,
}
impl ros2_client::Message for CancelTaskRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CancelTaskResponse {
    pub success: bool,
    pub error_message: String,
}
impl ros2_client::Message for CancelTaskResponse {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GenerateDSLRequest {
    pub task_id: String,
}
impl ros2_client::Message for GenerateDSLRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GenerateDSLResponse {
    pub success: bool,
    pub error_message: String,
    pub dsl_code: String,
}
impl ros2_client::Message for GenerateDSLResponse {}

