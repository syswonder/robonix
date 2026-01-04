// SPDX-License-Identifier: MulanPSL-2.0
// Test ROS IDL Message Types

use serde::{Deserialize, Serialize};

/// Ping Pong service types for testing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PingPongRequest {
    pub message: String,
    pub sequence: u64,
}

impl ros2_client::Message for PingPongRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PingPongResponse {
    pub message: String,
    pub sequence: u64,
    pub timestamp: u64,
}

impl ros2_client::Message for PingPongResponse {}
