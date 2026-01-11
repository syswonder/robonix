// SPDX-License-Identifier: MulanPSL-2.0
// Node Module
//
// ROS2 node creation utilities

use ros2_client::{Context, Node, NodeName, NodeOptions};
use std::sync::Arc;

/// Create a ROS2 node with its context
/// Returns both the node and the context (context must be kept alive)
pub fn create_node() -> (Node, Arc<Context>) {
    let context = Arc::new(Context::new().unwrap());
    let node = context
        .new_node(
            NodeName::new("/rbnx", "core").unwrap(),
            NodeOptions::new().enable_rosout(true),
        )
        .unwrap();
    (node, context)
}
