use crate::spec::{PrimitiveSpec, ServiceSpec};
use std::collections::HashMap;

pub fn load_primitives(primitives: &mut HashMap<String, PrimitiveSpec>) {
    PRM!(primitives, "prm::arm.move.ee", "Move end effector to target pose",
         { "pose": "geometry_msgs/msg/PoseStamped" },
         { "status": "std_msgs/msg/Bool" });

    PRM!(primitives, "prm::gripper.close", "Close gripper",
         {},  // No input parameters
         { "status": "std_msgs/msg/Bool" });

    PRM!(primitives, "prm::base.move", "Move mobile base with velocity command",
         { "cmd_vel": "geometry_msgs/msg/Twist" },
         { "odom": "nav_msgs/msg/Odometry" });
}

pub fn load_services(services: &mut HashMap<String, ServiceSpec>) {
    SRV!(
        // TODO
        services,
        "control",
        "Body control service providing motion control and safety management",
        "robonix_sdk/srv/service/control/Control"
    );
}
