// SPDX-License-Identifier: MulanPSL-2.0
// ROS IDL Message Types
//
// This module contains all ROS IDL message type definitions used for communication.
// These types implement ros2_client::Message and are used in ROS2 service interfaces.

pub mod object; // robonix_sdk Object message types
pub mod primitive;
pub mod service;
pub mod service_registry;
pub mod skill;
pub mod task;
pub mod test;

// Re-export service-specific IDL types for convenience
pub mod service_types {
    pub use super::service::semantic_map::*;
    pub use super::service::task_plan::*;
}
