// SPDX-License-Identifier: MulanPSL-2.0
// Standard Primitive and Service Specifications Table
//
// This file defines all standard primitives and services in a compact table format.
// Note: Skills do not have specifications - they are user-defined and flexible.

use crate::spec::{PrimitiveSpec, ServiceSpec};
use std::collections::HashMap;

pub fn load_primitives() -> HashMap<String, PrimitiveSpec> {
    let mut primitives = HashMap::new();

    // Camera capture primitive
    PRM!(primitives, "prm::camera_capture", "Capture RGB image from camera",
         {},
         { "image": "/topic/image" });

    // Arm movement primitive
    PRM!(primitives, "prm::arm_move_ee", "Move end effector to target pose",
         { "pose": "/topic/pose" },
         { "status": "/topic/status" });

    // Gripper primitive
    PRM!(primitives, "prm::gripper.close", "Close gripper",
         {},
         { "status": "/topic/status" });

    primitives
}

pub fn load_services() -> HashMap<String, ServiceSpec> {
    let mut services = HashMap::new();

    // Standard services
    SRV!(services, "spatial_map", "Spatial map service providing geometric structure information",
         "robonix_core/srv/spatial_map/GetSpatialMap");

    SRV!(services, "semantic_map", "Semantic map service providing entity-level representation",
         "robonix_core/srv/semantic_map/QuerySemanticMap");

    SRV!(services, "task_plan", "Task planning service converting natural language to RTDL",
         "robonix_core/srv/task_plan/PlanTask");

    SRV!(services, "plan_simulate", "Plan simulation service for feasibility and safety checking",
         "robonix_core/srv/plan_simulate/SimulatePlan");

    SRV!(services, "result_feedback", "Result feedback service for execution verification",
         "robonix_core/srv/result_feedback/ResultFeedback");

    services
}
