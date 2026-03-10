// SPDX-License-Identifier: MulanPSL-2.0
// Task ROS IDL Message Types

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

/// Task data request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskDataRequest {
    pub task_id: String,
}

impl ros2_client::Message for TaskDataRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskDataResponse {
    pub data: String, // JSON string: complete task data
}

impl ros2_client::Message for TaskDataResponse {}

/// Cancel task request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CancelTaskRequest {
    pub task_id: String,
}

impl ros2_client::Message for CancelTaskRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CancelTaskResponse {
    pub success: bool,
}

impl ros2_client::Message for CancelTaskResponse {}
