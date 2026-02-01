// SPDX-License-Identifier: MulanPSL-2.0
// Node Module
//
// ROS2 node creation utilities

use ros2_client::{Context, Node, NodeName, NodeOptions};
use std::sync::Arc;

/// Create two ROS2 nodes on **separate contexts**: one for core API servers (ping, register, query, task submit, etc.),
/// one for task runtime (semantic_map client, task_plan client, executor) and monitors (TF, topic, image).
/// Two contexts give two independent DDS/event loops so a long-running task (e.g. semantic map update) never blocks API requests.
/// Returns (api_node, task_node, api_context, task_context). Both contexts must be kept alive.
pub fn create_nodes() -> (Node, Node, Arc<Context>, Arc<Context>) {
    let api_context = Arc::new(Context::new().unwrap());
    let api_node = api_context
        .new_node(
            NodeName::new("/rbnx", "core_api").unwrap(),
            NodeOptions::new().enable_rosout(true),
        )
        .unwrap();
    let task_context = Arc::new(Context::new().unwrap());
    let task_node = task_context
        .new_node(
            NodeName::new("/rbnx", "core_task").unwrap(),
            NodeOptions::new().enable_rosout(true),
        )
        .unwrap();
    (api_node, task_node, api_context, task_context)
}
