// SPDX-License-Identifier: MulanPSL-2.0
// Task API Message Types
//
// Task API according to robonix spec

use serde::{Deserialize, Serialize};

/// Task submit request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubmitTaskRequest {
    pub description: String, // Natural language task description
    pub params: String,      // JSON string: optional parameters
}

impl ros2_client::Message for SubmitTaskRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubmitTaskResponse {
    pub task_id: String,
}

impl ros2_client::Message for SubmitTaskResponse {}

/// Task status request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskStatusRequest {
    pub task_id: String,
}

impl ros2_client::Message for TaskStatusRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskStatusResponse {
    pub status: String, // pending / planning / running / finished
}

impl ros2_client::Message for TaskStatusResponse {}

/// Task result request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskResultRequest {
    pub task_id: String,
}

impl ros2_client::Message for TaskResultRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskResultResponse {
    pub result: String, // JSON string: task result
}

impl ros2_client::Message for TaskResultResponse {}
