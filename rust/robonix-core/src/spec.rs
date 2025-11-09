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
}

pub struct SpecRegistry {
    pub capabilities: HashMap<String, CapabilitySpec>,
    pub skills: HashMap<String, SkillSpec>,
}

// Macro to define a single capability
// Format: CAP!(capabilities, name, desc, [INPUT(name, type), ...], [OUTPUT(name, type), ...], [CONFIG(service, name), ...])
#[macro_export]
macro_rules! CAP {
    ($capabilities:ident, $name:expr, $desc:expr, 
     INPUT: [$($input_name:expr => $input_type:expr),* $(,)?],
     OUTPUT: [$($output_name:expr => $output_type:expr),* $(,)?],
     CONFIG: [$($config_service:expr => $config_name:expr),* $(,)?]) => {
        $capabilities.insert(
            $name.to_string(),
            $crate::spec::CapabilitySpec {
                description: $desc.to_string(),
                inputs: vec![$(
                    $crate::spec::IOParameterSpec {
                        name: $input_name.to_string(),
                        ros_type: $input_type.to_string(),
                    }
                ),*],
                outputs: vec![$(
                    $crate::spec::IOParameterSpec {
                        name: $output_name.to_string(),
                        ros_type: $output_type.to_string(),
                    }
                ),*],
                configs: vec![$(
                    $crate::spec::ConfigSpec {
                        service: $config_service.to_string(),
                        name: $config_name.to_string(),
                    }
                ),*],
            }
        );
    };
}

// Macro to define a single skill
// Format: SKL!(skills, name, desc, [INPUT(name, type), ...], [OUTPUT(name, type), ...], [CONFIG(service, name), ...])
#[macro_export]
macro_rules! SKL {
    ($skills:ident, $name:expr, $desc:expr,
     INPUT: [$($input_name:expr => $input_type:expr),* $(,)?],
     OUTPUT: [$($output_name:expr => $output_type:expr),* $(,)?],
     CONFIG: [$($config_service:expr => $config_name:expr),* $(,)?]) => {
        $skills.insert(
            $name.to_string(),
            $crate::spec::SkillSpec {
                description: $desc.to_string(),
                inputs: vec![$(
                    $crate::spec::IOParameterSpec {
                        name: $input_name.to_string(),
                        ros_type: $input_type.to_string(),
                    }
                ),*],
                outputs: vec![$(
                    $crate::spec::IOParameterSpec {
                        name: $output_name.to_string(),
                        ros_type: $output_type.to_string(),
                    }
                ),*],
                configs: vec![$(
                    $crate::spec::ConfigSpec {
                        service: $config_service.to_string(),
                        name: $config_name.to_string(),
                    }
                ),*],
            }
        );
    };
}

impl SpecRegistry {
    pub fn new() -> Self {
        // Load specifications from the table
        let capabilities = crate::specs_table::load_capabilities();
        let skills = crate::specs_table::load_skills();

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

        Ok(())
    }
}
