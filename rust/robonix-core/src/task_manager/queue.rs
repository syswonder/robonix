// SPDX-License-Identifier: MulanPSL-2.0
// Task Queue Module
//
// Task Queue with priority-based preemptive scheduling

use log::{debug, trace};
use std::collections::BinaryHeap;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Task Queue Entry - Wrapper for priority-based scheduling
#[derive(Debug, Clone)]
struct TaskQueueEntry {
    pub task_id: String,
    pub priority: i32,
    pub created_at: u64,
}

impl PartialEq for TaskQueueEntry {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority && self.created_at == other.created_at
    }
}

impl Eq for TaskQueueEntry {}

impl PartialOrd for TaskQueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for TaskQueueEntry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Higher priority first, then earlier created_at
        match other.priority.cmp(&self.priority) {
            std::cmp::Ordering::Equal => self.created_at.cmp(&other.created_at),
            other => other,
        }
    }
}

/// Task Queue - Priority-based task queue with preemption support
pub struct TaskQueue {
    queue: Arc<RwLock<BinaryHeap<TaskQueueEntry>>>,
    running_task: Arc<RwLock<Option<String>>>, // Currently running task ID
}

impl TaskQueue {
    pub fn new() -> Self {
        Self {
            queue: Arc::new(RwLock::new(BinaryHeap::new())),
            running_task: Arc::new(RwLock::new(None)),
        }
    }

    /// Enqueue a task
    pub async fn enqueue(&self, task_id: String, priority: i32, created_at: u64) {
        debug!(
            "enqueuing task: task_id={}, priority={}, created_at={}",
            task_id, priority, created_at
        );
        let entry = TaskQueueEntry {
            task_id: task_id.clone(),
            priority,
            created_at,
        };
        let mut queue = self.queue.write().await;
        queue.push(entry);
        debug!("task enqueued, queue size: {}", queue.len());
    }

    /// Dequeue the highest priority task
    pub async fn dequeue(&self) -> Option<String> {
        let mut queue = self.queue.write().await;
        let result = queue.pop().map(|entry| {
            debug!(
                "dequeued task: task_id={}, priority={}, remaining_queue_size={}",
                entry.task_id,
                entry.priority,
                queue.len()
            );
            entry.task_id
        });
        if result.is_none() {
            trace!("queue is empty, nothing to dequeue");
        }
        result
    }

    /// Peek at the highest priority task without removing it
    pub async fn peek(&self) -> Option<String> {
        let queue = self.queue.read().await;
        let result = queue.peek().map(|entry| {
            debug!(
                "peeked at queue: task_id={}, priority={}",
                entry.task_id, entry.priority
            );
            entry.task_id.clone()
        });
        if result.is_none() {
            trace!("queue is empty, nothing to peek");
        }
        result
    }

    /// Check if queue is empty
    pub async fn is_empty(&self) -> bool {
        let queue = self.queue.read().await;
        let is_empty = queue.is_empty();
        trace!("queue empty check: {}", is_empty);
        is_empty
    }

    /// Get the currently running task
    pub async fn get_running_task(&self) -> Option<String> {
        let running = self.running_task.read().await;
        running.clone()
    }

    /// Set the currently running task
    pub async fn set_running_task(&self, task_id: Option<String>) {
        let mut running = self.running_task.write().await;
        let old_task = running.clone();
        *running = task_id.clone();
        if let Some(ref new_task) = task_id {
            if old_task.as_ref() != Some(new_task) {
                debug!("running task changed: {:?} -> {:?}", old_task, new_task);
            }
        } else if old_task.is_some() {
            debug!("running task cleared (was: {:?})", old_task);
        }
    }

    /// Check if a higher priority task is available (for preemption)
    pub async fn should_preempt(&self, current_priority: i32) -> bool {
        let queue = self.queue.read().await;
        let result = if let Some(entry) = queue.peek() {
            let should = entry.priority > current_priority;
            debug!(
                "preemption check: current_priority={}, queue_top_priority={}, should_preempt={}",
                current_priority, entry.priority, should
            );
            should
        } else {
            trace!("preemption check: queue is empty, should_preempt=false");
            false
        };
        result
    }

    /// Remove a task from queue (e.g., when it's cancelled)
    pub async fn remove(&self, task_id: &str) -> bool {
        let mut queue = self.queue.write().await;
        let mut temp_heap = BinaryHeap::new();
        let mut found = false;

        // Rebuild heap without the removed task
        while let Some(entry) = queue.pop() {
            if entry.task_id == task_id {
                found = true;
            } else {
                temp_heap.push(entry);
            }
        }

        *queue = temp_heap;
        found
    }

    /// Get queue size
    pub async fn len(&self) -> usize {
        let queue = self.queue.read().await;
        let size = queue.len();
        trace!("queue size: {}", size);
        size
    }
}
