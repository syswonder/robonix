// SPDX-License-Identifier: MulanPSL-2.0
// Standard Primitive and Service Specifications
//
// This module defines the formal specifications for all standard primitives
// and services in the EAIOS system. These specifications are used to validate
// registration requests.
//
// Note: Skills do not have specifications - they are user-defined and flexible.

use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct PrimitiveSpec {
    pub description: String,
    // Input parameters: label -> ROS message type (e.g., {"pose": "geometry_msgs/msg/PoseStamped"})
    pub input_params: HashMap<String, String>,
    // Output parameters: label -> ROS message type (e.g., {"image": "sensor_msgs/msg/Image"})
    pub output_params: HashMap<String, String>,
}

#[derive(Debug, Clone)]
pub struct ServiceSpec {
    pub description: String,
    pub srv_type: String, // Expected ROS2 service type (e.g., "robonix_sdk/srv/service/spatial_map/GetSpatialMap")
}

pub struct SpecRegistry {
    pub primitives: HashMap<String, PrimitiveSpec>,
    pub services: HashMap<String, ServiceSpec>,
}

// Macro to define a single primitive
// Format: PRM!(primitives, name, desc, input_params, output_params)
// input_params/output_params: HashMap of label -> ROS message type
// Example: PRM!(primitives, "prm::camera.rgb", "Capture RGB image", {}, {"image": "sensor_msgs/msg/Image"})
#[macro_export]
macro_rules! PRM {
    ($primitives:ident, $name:expr, $desc:expr, $input_params:tt, $output_params:tt) => {{
        use std::collections::HashMap;
        let mut input_map = HashMap::new();
        let mut output_map = HashMap::new();

        // Parse input parameters: {label => ros_type, ...}
        let input_json = serde_json::json!($input_params);
        if let Some(input_obj) = input_json.as_object() {
            for (label, ros_type) in input_obj {
                if let Some(ros_type_str) = ros_type.as_str() {
                    input_map.insert(label.clone(), ros_type_str.to_string());
                }
            }
        }

        // Parse output parameters: {label => ros_type, ...}
        let output_json = serde_json::json!($output_params);
        if let Some(output_obj) = output_json.as_object() {
            for (label, ros_type) in output_obj {
                if let Some(ros_type_str) = ros_type.as_str() {
                    output_map.insert(label.clone(), ros_type_str.to_string());
                }
            }
        }

        $primitives.insert(
            $name.to_string(),
            $crate::spec::PrimitiveSpec {
                description: $desc.to_string(),
                input_params: input_map,
                output_params: output_map,
            },
        );
    }};
}

// Macro to define a single service
// Format: SRV!(services, name, desc, srv_type)
#[macro_export]
macro_rules! SRV {
    ($services:ident, $name:expr, $desc:expr, $srv_type:expr) => {
        $services.insert(
            $name.to_string(),
            $crate::spec::ServiceSpec {
                description: $desc.to_string(),
                srv_type: $srv_type.to_string(),
            },
        );
    };
}

impl SpecRegistry {
    pub fn new() -> Self {
        let mut primitives = HashMap::new();
        let mut services = HashMap::new();

        // Load perception specs (primitives + services)
        crate::perception::specs::load_primitives(&mut primitives);
        crate::perception::specs::load_services(&mut services);

        // Load cognition specs (primitives + services)
        crate::cognition::specs::load_primitives(&mut primitives);
        crate::cognition::specs::load_services(&mut services);

        // Load action specs (primitives + services)
        crate::action::specs::load_primitives(&mut primitives);
        crate::action::specs::load_services(&mut services);

        Self {
            primitives,
            services,
        }
    }

    /// Validate primitive registration against spec
    ///
    /// Validates that:
    /// 1. Primitive name exists in spec
    /// 2. All required input parameter labels from spec are present in the provided input_schema
    /// 3. All required output parameter labels from spec are present in the provided output_schema
    ///
    /// Note: The provided schemas contain label -> topic mappings, while spec contains label -> ROS type mappings.
    /// We only validate that the labels match, not the topics (topics are user-defined).
    ///
    /// TODO: Validate ROS message types match between spec and provided schema
    pub fn validate_primitive(
        &self,
        std_name: &str,
        input_schema: &serde_json::Value,
        output_schema: &serde_json::Value,
    ) -> Result<(), String> {
        // Check if primitive name exists in spec
        let spec = self
            .primitives
            .get(std_name)
            .ok_or_else(|| format!("Unknown primitive: {}", std_name))?;

        // Validate input schema: check that all required parameter labels from spec are present
        if let Some(provided_obj) = input_schema.as_object() {
            for label in spec.input_params.keys() {
                if !provided_obj.contains_key(label) {
                    return Err(format!(
                        "Missing required input parameter label in schema: '{}'",
                        label
                    ));
                }
            }
        } else if !spec.input_params.is_empty() {
            return Err("Input schema must be a JSON object".to_string());
        }

        // Validate output schema: check that all required parameter labels from spec are present
        if let Some(provided_obj) = output_schema.as_object() {
            for label in spec.output_params.keys() {
                if !provided_obj.contains_key(label) {
                    return Err(format!(
                        "Missing required output parameter label in schema: '{}'",
                        label
                    ));
                }
            }
        } else if !spec.output_params.is_empty() {
            return Err("Output schema must be a JSON object".to_string());
        }

        // TODO: Validate ROS message types match between spec and provided schema
        // For now, we only check that labels are present, not that types match

        Ok(())
    }

    /// Validate service registration against spec
    pub fn validate_service(&self, std_name: &str, srv_type: &str) -> Result<(), String> {
        let spec = self
            .services
            .get(std_name)
            .ok_or_else(|| format!("Unknown service: {}", std_name))?;

        if srv_type != spec.srv_type {
            return Err(format!(
                "Service type mismatch for '{}': expected '{}', got '{}'",
                std_name, spec.srv_type, srv_type
            ));
        }

        Ok(())
    }
}
