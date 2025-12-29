// SPDX-License-Identifier: MulanPSL-2.0
// Task Store Module
//
// Manages task lifecycle and state.

use log::debug;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt::Display;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use tokio::sync::RwLock;

// Task states according to robonix spec
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum TaskState {
    Pending,   // Task created, waiting for processing
    Planning,  // Task planning phase
    Running,   // Task execution phase
    Suspended, // Task suspended (preempted by higher priority task)
    Finished,  // Task completed successfully
    Failed,    // Task failed
    Cancelled, // Task cancelled
}

impl Display for TaskState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

/// Task Context - Runtime execution context for a task
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskContext {
    // Priority for scheduling
    pub priority: i32, // Higher number = higher priority (for preemption)

    // Object graph from semantic map (updated periodically)
    pub object_graph: serde_json::Value,
    pub object_graph_updated_at: u64, // Unix timestamp in nanoseconds

    // RTDL program from task planning
    pub rtdl: Option<String>,            // RTDL code
    pub rtdl_type: Option<String>,       // RTDL style (e.g., "list")
    pub rtdl_instruction_pointer: usize, // Current instruction index in RTDL

    // Exception handling
    pub last_exception: Option<String>,
    pub retry_count: u32,
}

impl TaskContext {
    pub fn new(priority: i32) -> Self {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        Self {
            priority,
            object_graph: serde_json::json!([]), // Empty array initially
            object_graph_updated_at: now,
            rtdl: None,
            rtdl_type: None,
            rtdl_instruction_pointer: 0,
            last_exception: None,
            retry_count: 0,
        }
    }

    /// Update object graph from semantic map service
    pub fn update_object_graph(&mut self, object_graph: serde_json::Value) {
        let old_obj_count = if let Some(arr) = self.object_graph.as_array() {
            arr.len()
        } else {
            0
        };
        let new_obj_count = if let Some(arr) = object_graph.as_array() {
            arr.len()
        } else {
            0
        };

        debug!(
            "updating object graph: {} -> {} objects",
            old_obj_count, new_obj_count
        );

        self.object_graph = object_graph;
        self.object_graph_updated_at = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
    }

    /// Set RTDL program from task planning
    pub fn set_rtdl(&mut self, rtdl: String, rtdl_type: String) {
        debug!(
            "setting RTDL: type={}, length={} chars",
            rtdl_type,
            rtdl.len()
        );
        self.rtdl = Some(rtdl);
        self.rtdl_type = Some(rtdl_type);
        self.rtdl_instruction_pointer = 0; // Reset instruction pointer
    }

    /// Advance instruction pointer
    pub fn advance_instruction_pointer(&mut self) {
        let old_pointer = self.rtdl_instruction_pointer;
        self.rtdl_instruction_pointer += 1;
        debug!(
            "instruction pointer advanced: {} -> {}",
            old_pointer, self.rtdl_instruction_pointer
        );
    }

    /// Set instruction pointer (for resuming after preemption)
    pub fn set_instruction_pointer(&mut self, pointer: usize) {
        self.rtdl_instruction_pointer = pointer;
    }

    /// Record exception
    pub fn record_exception(&mut self, exception: String) {
        let old_retry_count = self.retry_count;
        self.last_exception = Some(exception.clone());
        self.retry_count += 1;
        debug!(
            "exception recorded: retry_count {} -> {}, exception: {}",
            old_retry_count, self.retry_count, exception
        );
    }

    /// Clear exception
    pub fn clear_exception(&mut self) {
        self.last_exception = None;
    }
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

    // Execution context
    pub context: TaskContext,
}

impl Task {
    pub fn new(
        task_id: String,
        description: String,
        params: serde_json::Value,
        priority: i32,
    ) -> Self {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        Self {
            task_id,
            description,
            params,
            state: TaskState::Pending,
            result: None,
            error_message: None,
            created_at: now,
            updated_at: now,
            context: TaskContext::new(priority),
        }
    }

    /// Transition state
    pub fn transition_state(&mut self, new_state: TaskState) {
        let old_state = self.state.clone();
        if old_state != new_state {
            debug!(
                "task {}: [{}] -> [{}]",
                self.task_id,
                format!("{:?}", old_state).to_uppercase(),
                format!("{:?}", new_state).to_uppercase()
            );
        }
        self.state = new_state;
        self.updated_at = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
    }
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
    pub async fn create_task(
        &self,
        description: String,
        params: serde_json::Value,
        priority: i32,
    ) -> String {
        let counter = self.task_counter.fetch_add(1, Ordering::SeqCst);
        let task_id = format!("task_{}", counter);
        debug!(
            "creating new task: task_id={}, counter={}, priority={}",
            task_id, counter, priority
        );

        let task = Task::new(task_id.clone(), description, params, priority);

        let mut tasks = self.tasks.write().await;
        let old_size = tasks.len();
        tasks.insert(task_id.clone(), task);
        let new_size = tasks.len();
        debug!("task store size: {} -> {}", old_size, new_size);
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

    /// Update task with a closure
    pub async fn update_task<F>(&self, task_id: &str, f: F) -> bool
    where
        F: FnOnce(&mut Task),
    {
        debug!("updating task: task_id={}", task_id);
        let mut tasks = self.tasks.write().await;
        if let Some(task) = tasks.get_mut(task_id) {
            f(task);
            true
        } else {
            debug!("task {} not found, cannot update", task_id);
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
