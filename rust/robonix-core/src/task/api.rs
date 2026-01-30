// SPDX-License-Identifier: MulanPSL-2.0
// Task API Message Types
//
// Task API according to robonix spec
// Re-export ROS IDL types for convenience

pub use crate::ros_idl::task::{
    CancelTaskRequest, CancelTaskResponse, SubmitTaskRequest, SubmitTaskResponse, TaskDataRequest,
    TaskDataResponse,
};
