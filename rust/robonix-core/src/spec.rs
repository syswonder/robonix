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
pub struct PrimitiveSpec {
    pub description: String,
    pub input_schema: serde_json::Value,  // Expected input schema format
    pub output_schema: serde_json::Value, // Expected output schema format
}

#[derive(Debug, Clone)]
pub struct ServiceSpec {
    pub description: String,
    pub srv_type: String,  // Expected ROS2 service type (e.g., "robonix_core/srv/GetSpatialMap")
}

pub struct SpecRegistry {
    pub primitives: HashMap<String, PrimitiveSpec>,
    pub services: HashMap<String, ServiceSpec>,
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

// Macro to define a single primitive
// Format: PRM!(primitives, name, desc, input_schema_json, output_schema_json)
#[macro_export]
macro_rules! PRM {
    ($primitives:ident, $name:expr, $desc:expr, $input_schema:tt, $output_schema:tt) => {
        $primitives.insert(
            $name.to_string(),
            $crate::spec::PrimitiveSpec {
                description: $desc.to_string(),
                input_schema: serde_json::json!($input_schema),
                output_schema: serde_json::json!($output_schema),
            }
        );
    };
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
            }
        );
    };
}

impl SpecRegistry {
    pub fn new() -> Self {
        // Load specifications from the table
        let primitives = crate::specs_table::load_primitives();
        let services = crate::specs_table::load_services();

        Self {
            primitives,
            services,
        }
    }

    /// Validate primitive registration against spec
    pub fn validate_primitive(
        &self,
        std_name: &str,
        input_schema: &serde_json::Value,
        output_schema: &serde_json::Value,
    ) -> Result<(), String> {
        let spec = self
            .primitives
            .get(std_name)
            .ok_or_else(|| format!("Unknown primitive: {}", std_name))?;

        // Validate input schema structure
        // Check that all required keys from spec are present
        if let Some(spec_obj) = spec.input_schema.as_object() {
            if let Some(provided_obj) = input_schema.as_object() {
                for (key, _) in spec_obj {
                    if !provided_obj.contains_key(key) {
                        return Err(format!(
                            "Missing required input parameter in schema: '{}'",
                            key
                        ));
                    }
                }
            } else {
                return Err("Input schema must be a JSON object".to_string());
            }
        }

        // Validate output schema structure
        if let Some(spec_obj) = spec.output_schema.as_object() {
            if let Some(provided_obj) = output_schema.as_object() {
                for (key, _) in spec_obj {
                    if !provided_obj.contains_key(key) {
                        return Err(format!(
                            "Missing required output parameter in schema: '{}'",
                            key
                        ));
                    }
                }
            } else {
                return Err("Output schema must be a JSON object".to_string());
            }
        }

        Ok(())
    }

    /// Validate service registration against spec
    pub fn validate_service(
        &self,
        std_name: &str,
        srv_type: &str,
    ) -> Result<(), String> {
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
