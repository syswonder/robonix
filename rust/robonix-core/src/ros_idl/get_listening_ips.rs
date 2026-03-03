// SPDX-License-Identifier: MulanPSL-2.0
// GetListeningIps ROS IDL
//
// Service for core to report all IP addresses it is listening on.
// Used by CLI daemon to discover core's network addresses via ROS2.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetListeningIpsRequest {
    /// Unused; empty request. Kept for ROS2 service request shape.
    #[serde(default)]
    pub _dummy: u8,
}

impl ros2_client::Message for GetListeningIpsRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetListeningIpsResponse {
    /// JSON array of IP address strings, e.g. ["192.168.1.10", "10.0.0.1"]
    pub ips_json: String,
}

impl ros2_client::Message for GetListeningIpsResponse {}
