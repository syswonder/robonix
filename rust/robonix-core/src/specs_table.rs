// SPDX-License-Identifier: MulanPSL-2.0
// Standard Primitive and Service Specifications Table
//
// This file defines all standard primitives and services in a compact table format.
// Note: Skills do not have specifications - they are user-defined and flexible.

use crate::spec::{PrimitiveSpec, ServiceSpec};
use log::info;
use std::collections::HashMap;

pub fn load_primitives() -> HashMap<String, PrimitiveSpec> {
    let mut primitives = HashMap::new();

    PRM!(primitives, "prm::camera.capture", "Capture RGB image from camera",
         {},  // No input parameters
         { "image": "sensor_msgs/msg/Image" });

    PRM!(primitives, "prm::arm.move.ee", "Move end effector to target pose",
         { "pose": "geometry_msgs/msg/PoseStamped" },
         { "status": "std_msgs/msg/Bool" });

    PRM!(primitives, "prm::gripper.close", "Close gripper",
         {},  // No input parameters
         { "status": "std_msgs/msg/Bool" });

    info!(
        "loaded primitives specs, number of primitives: {}",
        primitives.len()
    );
    primitives
}

pub fn load_services() -> HashMap<String, ServiceSpec> {
    let mut services = HashMap::new();

    SRV!(
        services,
        "spatial_map",
        "Spatial map service providing geometric structure information",
        "robonix_sdk/srv/service/spatial_map/GetSpatialMap"
    );

    SRV!(
        services,
        "semantic_map",
        "Semantic map service providing entity-level representation",
        "robonix_sdk/srv/service/semantic_map/QuerySemanticMap"
    );

    SRV!(
        services,
        "task_plan",
        "Task planning service converting natural language to RTDL",
        "robonix_sdk/srv/service/task_plan/PlanTask"
    );

    SRV!(
        services,
        "plan_simulate",
        "Plan simulation service for feasibility and safety checking",
        "robonix_sdk/srv/service/plan_simulate/SimulatePlan"
    );

    SRV!(
        services,
        "result_feedback",
        "Result feedback service for execution verification",
        "robonix_sdk/srv/service/result_feedback/ResultFeedback"
    );

    info!(
        "loaded services specs, number of services: {}",
        services.len()
    );
    services
}
