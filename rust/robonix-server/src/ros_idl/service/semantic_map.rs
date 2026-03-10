// SPDX-License-Identifier: MulanPSL-2.0
// Semantic Map Service ROS IDL Message Types

use crate::ros_idl::object::{Object, Time};
use serde::{Deserialize, Serialize};

/// QuerySemanticMap service request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuerySemanticMapRequest {
    pub types: Vec<String>, // Filter by object type, e.g., ["room","window","robot"]
}

impl ros2_client::Message for QuerySemanticMapRequest {}

/// QuerySemanticMap service response
/// objects field receives Object[] from ROS2 service definition (robonix_sdk/Object[])
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuerySemanticMapResponse {
    pub objects: Vec<Object>, // robonix_sdk/Object[] - Array of Object messages
    pub stamp: Time,          // builtin_interfaces/Time - Timestamp
}

impl ros2_client::Message for QuerySemanticMapResponse {}
