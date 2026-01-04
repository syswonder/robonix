// SPDX-License-Identifier: MulanPSL-2.0
// Primitive ROS IDL Message Types

use serde::{Deserialize, Serialize};

/// Primitive registration request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterPrimitiveRequest {
    pub name: String,          // Standard primitive name
    pub input_schema: String,  // JSON string: {"argname0":"/topic0", ...}
    pub output_schema: String, // JSON string: {"argname1":"/topic1", ...}
    pub metadata: String,      // JSON string: metadata for instance filtering
    pub provider: String,      // Primitive provider identifier
    pub version: String,       // Implementation version (e.g., "1.0.0", "1.0.0-alpha")
}

impl ros2_client::Message for RegisterPrimitiveRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterPrimitiveResponse {
    pub ok: bool,
}

impl ros2_client::Message for RegisterPrimitiveResponse {}

/// Primitive query request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryPrimitiveRequest {
    pub name: String,
    pub filter: String, // JSON string: filter by metadata (e.g., {"resolution":">=720p"}, {"index":0}). Empty string means no filter
}

impl ros2_client::Message for QueryPrimitiveRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrimitiveInstance {
    pub provider: String,
    pub version: String,
    pub input_schema: String,  // JSON string: {"argname0":"/topic0", ...}
    pub output_schema: String, // JSON string: {"argname1":"/topic1", ...}
    pub metadata: String,      // JSON string: metadata for instance filtering
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryPrimitiveResponse {
    pub instances: Vec<PrimitiveInstance>,
}

impl ros2_client::Message for QueryPrimitiveResponse {}
