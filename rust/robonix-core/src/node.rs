// SPDX-License-Identifier: MulanPSL-2.0
// Node Module
//
// ROS2 node creation utilities

use ros2_client::{Context, Node, NodeName, NodeOptions};

pub fn create_node() -> Node {
    let context = Context::new().unwrap();
    context
        .new_node(
            NodeName::new("/rbnx", "core").unwrap(),
            NodeOptions::new().enable_rosout(true),
        )
        .unwrap()
}
