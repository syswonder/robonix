// SPDX-License-Identifier: MulanPSL-2.0
// Task Store Module
//
// Manages task lifecycle and state.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::info;

// Task states according to robonix spec
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum TaskState {
    Pending,    // Task created, waiting for processing
    Planning,   // Task planning phase
    Running,    // Task execution phase
    Finished,   // Task completed successfully
    Failed,     // Task failed
    Cancelled,  // Task cancelled
}

// Task structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Task {
    pub task_id: String,
    pub description: String,        // Natural language task description
    pub params: serde_json::Value, // Optional parameters (JSON)
    pub state: TaskState,
    pub result: Option<serde_json::Value>, // Task result (JSON)
    pub error_message: Option<String>,
    pub created_at: u64,  // Unix timestamp in nanoseconds
    pub updated_at: u64,
}

/// Task Store - Manages task storage and lifecycle
pub struct TaskStore {
    tasks: Arc<RwLock<HashMap<String, Task>>>,
    task_counter: Arc<AtomicU64>,
}

impl TaskStore {
    pub fn new() -> Self {
        Self {
            tasks: Arc::new(RwLock::new(HashMap::new())),
            task_counter: Arc::new(AtomicU64::new(0)),
        }
    }

    /// Create a new task from description and optional parameters
    pub async fn create_task(&self, description: String, params: serde_json::Value) -> String {
        let counter = self.task_counter.fetch_add(1, Ordering::SeqCst);
        let task_id = format!("task_{}", counter);
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        let task = Task {
            task_id: task_id.clone(),
            description,
            params,
            state: TaskState::Pending,
            result: None,
            error_message: None,
            created_at: now,
            updated_at: now,
        };

        let mut tasks = self.tasks.write().await;
        tasks.insert(task_id.clone(), task);

        info!(task_id = %task_id, "created new task");
        task_id
    }

    /// Get task by ID
    pub async fn get_task(&self, task_id: &str) -> Option<Task> {
        let tasks = self.tasks.read().await;
        tasks.get(task_id).cloned()
    }

    /// Get all tasks
    pub async fn get_all_tasks(&self) -> Vec<Task> {
        let tasks = self.tasks.read().await;
        tasks.values().cloned().collect()
    }

    /// Update task state
    pub async fn update_task_state(
        &self,
        task_id: &str,
        state: TaskState,
        error_message: Option<String>,
    ) -> bool {
        let mut tasks = self.tasks.write().await;
        if let Some(task) = tasks.get_mut(task_id) {
            task.state = state;
            task.error_message = error_message;
            task.updated_at = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;
            true
        } else {
            false
        }
    }

    /// Set task result
    pub async fn set_task_result(&self, task_id: &str, result: serde_json::Value) -> bool {
        let mut tasks = self.tasks.write().await;
        if let Some(task) = tasks.get_mut(task_id) {
            task.result = Some(result);
            task.updated_at = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;
            true
        } else {
            false
        }
    }
}

