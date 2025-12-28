// SPDX-License-Identifier: MulanPSL-2.0
// Task Context Module
//
// Task Context maintains runtime state for task execution:
// - Object graph from semantic map service
// - RTDL program from task planning
// - State machine state
// - Execution progress (RTDL instruction pointer)

use crate::task_manager::task::TaskState;
use log::debug;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use tokio::sync::RwLock;

/// Task execution state machine states
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExecutionState {
    Init,      // Initial state, waiting to start
    Plan,      // Planning phase - calling task_plan service
    Execute,   // Executing RTDL program
    Finish,    // Task completed successfully
    Suspended, // Task suspended (preempted by higher priority task)
    Failed,    // Task failed
}

/// Task Context - Runtime execution context for a task
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskContext {
    pub task_id: String,

    // State machine
    pub execution_state: ExecutionState,

    // Object graph from semantic map (updated periodically)
    pub object_graph: serde_json::Value,
    pub object_graph_updated_at: u64, // Unix timestamp in nanoseconds

    // RTDL program from task planning
    pub rtdl: Option<String>,            // RTDL code
    pub rtdl_type: Option<String>,       // RTDL style (e.g., "list")
    pub rtdl_instruction_pointer: usize, // Current instruction index in RTDL

    // Task metadata
    pub description: String,
    pub priority: i32, // Higher number = higher priority (for preemption)
    pub created_at: u64,
    pub updated_at: u64,

    // Exception handling
    pub last_exception: Option<String>,
    pub retry_count: u32,
}

impl TaskContext {
    pub fn new(task_id: String, description: String, priority: i32) -> Self {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        Self {
            task_id,
            execution_state: ExecutionState::Init,
            object_graph: serde_json::json!([]), // Empty array initially
            object_graph_updated_at: now,
            rtdl: None,
            rtdl_type: None,
            rtdl_instruction_pointer: 0,
            description,
            priority,
            created_at: now,
            updated_at: now,
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
            "updating object graph for task {}: {} -> {} objects",
            self.task_id, old_obj_count, new_obj_count
        );

        self.object_graph = object_graph;
        self.object_graph_updated_at = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
        self.updated_at = self.object_graph_updated_at;

        debug!(
            "task {} object graph updated at: {}",
            self.task_id, self.object_graph_updated_at
        );
    }

    /// Set RTDL program from task planning
    pub fn set_rtdl(&mut self, rtdl: String, rtdl_type: String) {
        debug!(
            "task {} setting RTDL: type={}, length={} chars",
            self.task_id,
            rtdl_type,
            rtdl.len()
        );
        self.rtdl = Some(rtdl);
        self.rtdl_type = Some(rtdl_type);
        self.rtdl_instruction_pointer = 0; // Reset instruction pointer
        debug!("task {} RTDL instruction pointer reset to 0", self.task_id);
        self.updated_at = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
    }

    /// Advance instruction pointer
    pub fn advance_instruction_pointer(&mut self) {
        let old_pointer = self.rtdl_instruction_pointer;
        self.rtdl_instruction_pointer += 1;
        debug!(
            "task {} instruction pointer advanced: {} -> {}",
            self.task_id, old_pointer, self.rtdl_instruction_pointer
        );
        self.updated_at = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
    }

    /// Set instruction pointer (for resuming after preemption)
    pub fn set_instruction_pointer(&mut self, pointer: usize) {
        self.rtdl_instruction_pointer = pointer;
        self.updated_at = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
    }

    /// Transition execution state
    pub fn transition_state(&mut self, new_state: ExecutionState) {
        let old_state = self.execution_state.clone();
        if old_state != new_state {
            debug!(
                "task {} state transition: {:?} -> {:?}",
                self.task_id, old_state, new_state
            );
        }
        self.execution_state = new_state;
        self.updated_at = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
    }

    /// Record exception
    pub fn record_exception(&mut self, exception: String) {
        let old_retry_count = self.retry_count;
        self.last_exception = Some(exception.clone());
        self.retry_count += 1;
        debug!(
            "task {} exception recorded: retry_count {} -> {}, exception: {}",
            self.task_id, old_retry_count, self.retry_count, exception
        );
        self.updated_at = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
    }

    /// Clear exception
    pub fn clear_exception(&mut self) {
        self.last_exception = None;
        self.updated_at = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
    }

    /// Convert to TaskState for compatibility
    pub fn to_task_state(&self) -> TaskState {
        match self.execution_state {
            ExecutionState::Init => TaskState::Pending,
            ExecutionState::Plan => TaskState::Planning,
            ExecutionState::Execute | ExecutionState::Suspended => TaskState::Running,
            ExecutionState::Finish => TaskState::Finished,
            ExecutionState::Failed => TaskState::Failed,
        }
    }
}

/// Task Context Store - Manages all task contexts
pub struct TaskContextStore {
    contexts: Arc<RwLock<std::collections::HashMap<String, TaskContext>>>,
}

impl TaskContextStore {
    pub fn new() -> Self {
        Self {
            contexts: Arc::new(RwLock::new(std::collections::HashMap::new())),
        }
    }

    pub async fn create_context(&self, task_id: String, description: String, priority: i32) {
        debug!(
            "creating task context: task_id={}, priority={}, description_length={}",
            task_id,
            priority,
            description.len()
        );
        let context = TaskContext::new(task_id.clone(), description, priority);
        let mut contexts = self.contexts.write().await;
        let old_size = contexts.len();
        contexts.insert(task_id.clone(), context);
        let new_size = contexts.len();
        debug!(
            "context store size: {} -> {}, task {} context created",
            old_size, new_size, task_id
        );
    }

    pub async fn get_context(&self, task_id: &str) -> Option<TaskContext> {
        debug!("getting task context: task_id={}", task_id);
        let contexts = self.contexts.read().await;
        let result = contexts.get(task_id).cloned();
        if result.is_some() {
            debug!("task {} context found", task_id);
        } else {
            debug!("task {} context not found", task_id);
        }
        result
    }

    pub async fn update_context<F>(&self, task_id: &str, f: F) -> bool
    where
        F: FnOnce(&mut TaskContext),
    {
        debug!("updating task context: task_id={}", task_id);
        let mut contexts = self.contexts.write().await;
        if let Some(context) = contexts.get_mut(task_id) {
            let old_state = context.execution_state.clone();
            let old_retry_count = context.retry_count;
            let old_instruction_pointer = context.rtdl_instruction_pointer;
            f(context);
            debug!(
                "task {} context updated: state {:?} -> {:?}, retry_count {} -> {}, instruction_pointer {} -> {}",
                task_id,
                old_state,
                context.execution_state,
                old_retry_count,
                context.retry_count,
                old_instruction_pointer,
                context.rtdl_instruction_pointer
            );
            true
        } else {
            debug!("task {} context not found, cannot update", task_id);
            false
        }
    }

    pub async fn remove_context(&self, task_id: &str) -> bool {
        let mut contexts = self.contexts.write().await;
        contexts.remove(task_id).is_some()
    }

    pub async fn get_all_contexts(&self) -> Vec<TaskContext> {
        debug!("getting all task contexts");
        let contexts = self.contexts.read().await;
        let result: Vec<TaskContext> = contexts.values().cloned().collect();
        debug!("retrieved {} task contexts", result.len());
        result
    }
}
