// SPDX-License-Identifier: MulanPSL-2.0
// Service Registry ROS IDL Message Types
//
// This module contains message types for service registration and querying,
// not the actual service implementations.

use serde::{Deserialize, Serialize};

/// Service registration request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterServiceRequest {
    pub name: String,     // Standard service name
    pub srv_type: String, // ROS2 service type (e.g., "robonix_sdk/srv/service/spatial_map/GetSpatialMap")
    pub entry: String,    // Actual ROS2 service name
    pub metadata: String, // JSON string: metadata for instance filtering
    pub provider: String, // Service provider identifier
    pub version: String,  // Implementation version (e.g., "1.0.0", "1.0.0-alpha")
    /// Node (CLI client) that registered this capability. Empty for backward compat.
    #[serde(default)]
    pub node_id: String,
}

impl ros2_client::Message for RegisterServiceRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterServiceResponse {
    pub ok: bool,
}

impl ros2_client::Message for RegisterServiceResponse {}

/// Service query request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryServiceRequest {
    pub name: String,
    pub filter: String, // JSON string: filter by metadata. Empty string means no filter
}

impl ros2_client::Message for QueryServiceRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServiceInstance {
    pub provider: String,
    pub version: String,
    pub entry: String,
    pub metadata: String, // JSON string: metadata for instance filtering
    /// Node that registered this capability. Empty if unknown.
    #[serde(default)]
    pub node_id: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryServiceResponse {
    pub instances: Vec<ServiceInstance>,
}

impl ros2_client::Message for QueryServiceResponse {}
