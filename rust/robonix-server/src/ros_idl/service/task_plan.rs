// SPDX-License-Identifier: MulanPSL-2.0
// Task Plan Service ROS IDL Message Types

use crate::ros_idl::object::Dict;
use serde::{Deserialize, Serialize};

/// PlanTask service request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanTaskRequest {
    pub description: String, // Natural language task description
    pub params: Dict,        // robonix_sdk/Dict: optional parameters (e.g., object_graph)
}

impl ros2_client::Message for PlanTaskRequest {}

/// PlanTask service response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanTaskResponse {
    pub rtdl: String,      // RTDL code (JSON string)
    pub rtdl_type: String, // RTDL type (e.g., "list")
}

impl ros2_client::Message for PlanTaskResponse {}
