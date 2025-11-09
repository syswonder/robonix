// SPDX-License-Identifier: MulanPSL-2.0
// Standard Capability and Skill Specifications Table
//
// This file defines all standard capabilities and skills in a compact table format.
// Format:
//   CAP!(capabilities, name, desc, INPUT: [name => type, ...], OUTPUT: [name => type, ...], CONFIG: // TODO
//   SKL!(skills, name, desc, INPUT: [name => type, ...], OUTPUT: [name => type, ...], CONFIG: // TODO

use crate::spec::{CapabilitySpec, SkillSpec};
use std::collections::HashMap;

pub fn load_capabilities() -> HashMap<String, CapabilitySpec> {
    let mut capabilities = HashMap::new();

    CAP!(capabilities, "cap::grasp.move", "Move the gripper to a target pose",
         INPUT: ["target_pose" => "geometry_msgs/msg/PoseStamped"],
         OUTPUT: ["status" => "boolean"],
         CONFIG: []);

    CAP!(capabilities, "cap::vision.capture_rgb", "Capture RGB image from camera",
         INPUT: [],
         OUTPUT: ["image" => "sensor_msgs/msg/Image"],
         CONFIG: []);

    capabilities
}

pub fn load_skills() -> HashMap<String, SkillSpec> {
    let mut skills = HashMap::new();

    SKL!(skills, "skl::pick", "Pick an object by label",
         INPUT: ["target_label" => "string"],
         OUTPUT: ["status" => "boolean"],
         CONFIG: []);

    skills
}
