// SPDX-License-Identifier: MulanPSL-2.0
// Standard Capability and Skill Specifications
//
// This module defines the formal specifications for all standard capabilities
// and skills in the Robonix system. These specifications are used to validate
// registration requests.

use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct IOParameterSpec {
    pub name: String,
    pub ros_type: String, // ROS message type
}

#[derive(Debug, Clone)]
pub struct ConfigSpec {
    pub service: String,
    pub name: String,
}

#[derive(Debug, Clone)]
pub struct CapabilitySpec {
    pub description: String,
    pub inputs: Vec<IOParameterSpec>,
    pub outputs: Vec<IOParameterSpec>,
    pub configs: Vec<ConfigSpec>,
}

#[derive(Debug, Clone)]
pub struct SkillSpec {
    pub description: String,
    pub inputs: Vec<IOParameterSpec>,
    pub outputs: Vec<IOParameterSpec>,
    pub configs: Vec<ConfigSpec>,
    pub dependencies: Vec<String>, // Required capability names
}

pub struct SpecRegistry {
    pub capabilities: HashMap<String, CapabilitySpec>,
    pub skills: HashMap<String, SkillSpec>,
}

impl SpecRegistry {
    pub fn new() -> Self {
        let mut capabilities = HashMap::new();
        let mut skills = HashMap::new();

        // Register standard capabilities
        capabilities.insert(
            "cap::grasp.move".to_string(),
            CapabilitySpec {
                description: "Move the gripper to a target pose".to_string(),
                inputs: vec![IOParameterSpec {
                    name: "target_pose".to_string(),
                    ros_type: "geometry_msgs/msg/PoseStamped".to_string(),
                }],
                outputs: vec![IOParameterSpec {
                    name: "status".to_string(),
                    ros_type: "boolean".to_string(),
                }],
                configs: vec![ConfigSpec {
                    service: "/arm/configure".to_string(),
                    name: "piper_arm_config_update".to_string(),
                }],
            },
        );

        capabilities.insert(
            "cap::vision.capture_rgb".to_string(),
            CapabilitySpec {
                description: "Capture RGB image from camera".to_string(),
                inputs: vec![],
                outputs: vec![IOParameterSpec {
                    name: "image".to_string(),
                    ros_type: "sensor_msgs/msg/Image".to_string(),
                }],
                configs: vec![],
            },
        );

        // Register standard skills
        skills.insert(
            "skl::pick".to_string(),
            SkillSpec {
                description: "Pick an object by label".to_string(),
                inputs: vec![IOParameterSpec {
                    name: "target_label".to_string(),
                    ros_type: "string".to_string(),
                }],
                outputs: vec![IOParameterSpec {
                    name: "status".to_string(),
                    ros_type: "boolean".to_string(),
                }],
                configs: vec![ConfigSpec {
                    service: "/vla/configure_io".to_string(),
                    name: "setup_topics_path".to_string(),
                }],
                dependencies: vec![
                    "cap::grasp.move".to_string(),
                    "cap::vision.capture_rgb".to_string(),
                ],
            },
        );

        Self {
            capabilities,
            skills,
        }
    }

    pub fn validate_capability(
        &self,
        std_name: &str,
        inputs: &[crate::messages::IOParameter],
        outputs: &[crate::messages::IOParameter],
        configs: &[crate::messages::ConfigService],
    ) -> Result<(), String> {
        let spec = self
            .capabilities
            .get(std_name)
            .ok_or_else(|| format!("Unknown capability: {}", std_name))?;

        // Validate inputs
        if inputs.len() != spec.inputs.len() {
            return Err(format!(
                "Input count mismatch: expected {}, got {}",
                spec.inputs.len(),
                inputs.len()
            ));
        }

        for (provided, expected) in inputs.iter().zip(spec.inputs.iter()) {
            if provided.name != expected.name {
                return Err(format!(
                    "Input name mismatch: expected '{}', got '{}'",
                    expected.name, provided.name
                ));
            }
            if provided.ros_type != expected.ros_type {
                return Err(format!(
                    "Input ROS type mismatch for '{}': expected '{}', got '{}'",
                    provided.name, expected.ros_type, provided.ros_type
                ));
            }
        }

        // Validate outputs
        if outputs.len() != spec.outputs.len() {
            return Err(format!(
                "Output count mismatch: expected {}, got {}",
                spec.outputs.len(),
                outputs.len()
            ));
        }

        for (provided, expected) in outputs.iter().zip(spec.outputs.iter()) {
            if provided.name != expected.name {
                return Err(format!(
                    "Output name mismatch: expected '{}', got '{}'",
                    expected.name, provided.name
                ));
            }
            if provided.ros_type != expected.ros_type {
                return Err(format!(
                    "Output ROS type mismatch for '{}': expected '{}', got '{}'",
                    provided.name, expected.ros_type, provided.ros_type
                ));
            }
        }

        // Validate configs
        if configs.len() != spec.configs.len() {
            return Err(format!(
                "Config count mismatch: expected {}, got {}",
                spec.configs.len(),
                configs.len()
            ));
        }

        for (provided, expected) in configs.iter().zip(spec.configs.iter()) {
            if provided.service != expected.service {
                return Err(format!(
                    "Config service mismatch: expected '{}', got '{}'",
                    expected.service, provided.service
                ));
            }
            if provided.name != expected.name {
                return Err(format!(
                    "Config name mismatch: expected '{}', got '{}'",
                    expected.name, provided.name
                ));
            }
        }

        Ok(())
    }

    pub fn validate_skill(
        &self,
        std_name: &str,
        inputs: &[crate::messages::IOParameter],
        outputs: &[crate::messages::IOParameter],
        configs: &[crate::messages::ConfigService],
        dependencies: &[String],
    ) -> Result<(), String> {
        let spec = self
            .skills
            .get(std_name)
            .ok_or_else(|| format!("Unknown skill: {}", std_name))?;

        // Validate inputs
        if inputs.len() != spec.inputs.len() {
            return Err(format!(
                "Input count mismatch: expected {}, got {}",
                spec.inputs.len(),
                inputs.len()
            ));
        }

        for (provided, expected) in inputs.iter().zip(spec.inputs.iter()) {
            if provided.name != expected.name {
                return Err(format!(
                    "Input name mismatch: expected '{}', got '{}'",
                    expected.name, provided.name
                ));
            }
            if provided.ros_type != expected.ros_type {
                return Err(format!(
                    "Input ROS type mismatch for '{}': expected '{}', got '{}'",
                    provided.name, expected.ros_type, provided.ros_type
                ));
            }
        }

        // Validate outputs
        if outputs.len() != spec.outputs.len() {
            return Err(format!(
                "Output count mismatch: expected {}, got {}",
                spec.outputs.len(),
                outputs.len()
            ));
        }

        for (provided, expected) in outputs.iter().zip(spec.outputs.iter()) {
            if provided.name != expected.name {
                return Err(format!(
                    "Output name mismatch: expected '{}', got '{}'",
                    expected.name, provided.name
                ));
            }
            if provided.ros_type != expected.ros_type {
                return Err(format!(
                    "Output ROS type mismatch for '{}': expected '{}', got '{}'",
                    provided.name, expected.ros_type, provided.ros_type
                ));
            }
        }

        // Validate configs
        if configs.len() != spec.configs.len() {
            return Err(format!(
                "Config count mismatch: expected {}, got {}",
                spec.configs.len(),
                configs.len()
            ));
        }

        for (provided, expected) in configs.iter().zip(spec.configs.iter()) {
            if provided.service != expected.service {
                return Err(format!(
                    "Config service mismatch: expected '{}', got '{}'",
                    expected.service, provided.service
                ));
            }
            if provided.name != expected.name {
                return Err(format!(
                    "Config name mismatch: expected '{}', got '{}'",
                    expected.name, provided.name
                ));
            }
        }

        // Validate dependencies
        if dependencies.len() != spec.dependencies.len() {
            return Err(format!(
                "Dependency count mismatch: expected {}, got {}",
                spec.dependencies.len(),
                dependencies.len()
            ));
        }

        // Check that all expected dependencies are present
        for expected_dep in &spec.dependencies {
            if !dependencies.contains(expected_dep) {
                return Err(format!(
                    "Missing required dependency: '{}' (required: {:?}, provided: {:?})",
                    expected_dep, spec.dependencies, dependencies
                ));
            }
        }

        // Check for unexpected dependencies
        for provided_dep in dependencies {
            if !spec.dependencies.contains(provided_dep) {
                return Err(format!(
                    "Unexpected dependency: '{}' (expected: {:?})",
                    provided_dep, spec.dependencies
                ));
            }
        }

        Ok(())
    }
}
