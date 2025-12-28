// SPDX-License-Identifier: MulanPSL-2.0
// Task Store Module
//
// Manages task lifecycle and state.

use log::{debug, info};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use tokio::sync::RwLock;

// Task states according to robonix spec
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum TaskState {
    Pending,   // Task created, waiting for processing
    Planning,  // Task planning phase
    Running,   // Task execution phase
    Finished,  // Task completed successfully
    Failed,    // Task failed
    Cancelled, // Task cancelled
}

// Task structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Task {
    pub task_id: String,
    pub description: String,       // Natural language task description
    pub params: serde_json::Value, // Optional parameters (JSON)
    pub state: TaskState,
    pub result: Option<serde_json::Value>, // Task result (JSON)
    pub error_message: Option<String>,
    pub created_at: u64, // Unix timestamp in nanoseconds
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
        debug!(
            "creating new task: task_id={}, counter={}",
            task_id, counter
        );

        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        let param_keys: Vec<String> = params
            .as_object()
            .map(|o| o.keys().cloned().collect())
            .unwrap_or_default();
        debug!(
            "task {} params: {} keys, description_length={}",
            task_id,
            param_keys.len(),
            description.len()
        );

        let task = Task {
            task_id: task_id.clone(),
            description: description.clone(),
            params: params.clone(),
            state: TaskState::Pending,
            result: None,
            error_message: None,
            created_at: now,
            updated_at: now,
        };

        let mut tasks = self.tasks.write().await;
        let old_size = tasks.len();
        tasks.insert(task_id.clone(), task);
        let new_size = tasks.len();
        debug!("task store size: {} -> {}", old_size, new_size);

        info!("created new task: task_id={}", task_id);
        debug!(
            "task {} created successfully: created_at={}, state=Pending",
            task_id, now
        );
        task_id
    }

    /// Get task by ID
    pub async fn get_task(&self, task_id: &str) -> Option<Task> {
        debug!("getting task from store: task_id={}", task_id);
        let tasks = self.tasks.read().await;
        let result = tasks.get(task_id).cloned();
        if result.is_some() {
            debug!("task {} found in store", task_id);
        } else {
            debug!("task {} not found in store", task_id);
        }
        result
    }

    /// Get all tasks
    pub async fn get_all_tasks(&self) -> Vec<Task> {
        debug!("getting all tasks from store");
        let tasks = self.tasks.read().await;
        let result: Vec<Task> = tasks.values().cloned().collect();
        debug!("retrieved {} tasks from store", result.len());
        result
    }

    /// Update task state
    pub async fn update_task_state(
        &self,
        task_id: &str,
        state: TaskState,
        error_message: Option<String>,
    ) -> bool {
        debug!(
            "updating task state: task_id={}, new_state={:?}, error_message={:?}",
            task_id, state, error_message
        );

        let mut tasks = self.tasks.write().await;
        if let Some(task) = tasks.get_mut(task_id) {
            let old_state = task.state.clone();
            task.state = state.clone();
            task.error_message = error_message.clone();
            task.updated_at = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;
            debug!(
                "task {} state updated: {:?} -> {:?}",
                task_id, old_state, state
            );
            true
        } else {
            debug!("task {} not found, cannot update state", task_id);
            false
        }
    }

    /// Set task result
    pub async fn set_task_result(&self, task_id: &str, result: serde_json::Value) -> bool {
        debug!(
            "setting task result: task_id={}, result_size={} bytes",
            task_id,
            serde_json::to_string(&result).unwrap_or_default().len()
        );

        let mut tasks = self.tasks.write().await;
        if let Some(task) = tasks.get_mut(task_id) {
            let had_result = task.result.is_some();
            task.result = Some(result.clone());
            task.updated_at = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;
            debug!(
                "task {} result set: had_result={}, result={:?}",
                task_id, had_result, result
            );
            true
        } else {
            debug!("task {} not found, cannot set result", task_id);
            false
        }
    }
}
