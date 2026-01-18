// SPDX-License-Identifier: MulanPSL-2.0
// Standard ROS2 Message Types
//
// Wrapper types for standard ROS2 messages (std_msgs, etc.)

use serde::{Deserialize, Serialize};

/// std_msgs/msg/String message type
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct String {
    pub data: std::string::String,
}

impl ros2_client::Message for String {}
