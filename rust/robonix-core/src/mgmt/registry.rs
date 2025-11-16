// SPDX-License-Identifier: MulanPSL-2.0
// Registry Module
//
// This module handles capability and skill registration and querying.

use crate::messages::{
    Capability, ConfigService, IOParameter, QueryCapSklRequest, QueryCapSklResponse,
    RegisterCapSklRequest, RegisterCapSklResponse, Skill,
};
use crate::spec::SpecRegistry;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{error, info, warn};

/// Registry for capabilities and skills
pub struct Registry {
    capabilities: Arc<RwLock<HashMap<String, Capability>>>,
    skills: Arc<RwLock<HashMap<String, Skill>>>,
}

impl Registry {
    pub fn new() -> Self {
        Self {
            capabilities: Arc::new(RwLock::new(HashMap::new())),
            skills: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    fn parse_io_parameters(
        names: &[String],
        ros_types: &[String],
        channels: &[String],
    ) -> Result<Vec<IOParameter>, String> {
        if names.len() != ros_types.len() || names.len() != channels.len() {
            return Err(format!(
                "IO parameter arrays length mismatch: names={}, types={}, channels={}",
                names.len(),
                ros_types.len(),
                channels.len()
            ));
        }

        let mut params = Vec::new();
        for (i, name) in names.iter().enumerate() {
            params.push(IOParameter {
                name: name.clone(),
                ros_type: ros_types[i].clone(),
                channel: channels[i].clone(),
            });
        }
        Ok(params)
    }

    fn parse_config_services(
        services: &[String],
        names: &[String],
    ) -> Result<Vec<ConfigService>, String> {
        if services.len() != names.len() {
            return Err(format!(
                "Config service arrays length mismatch: services={}, names={}",
                services.len(),
                names.len()
            ));
        }

        let mut configs = Vec::new();
        for (i, service) in services.iter().enumerate() {
            configs.push(ConfigService {
                service: service.clone(),
                name: names[i].clone(),
            });
        }
        Ok(configs)
    }

    /// Register a capability or skill
    pub async fn register(
        &self,
        req: RegisterCapSklRequest,
        spec_registry: &Arc<SpecRegistry>,
    ) -> RegisterCapSklResponse {
        // Normalize impl_id: if empty, use "default"
        let impl_id = if req.impl_id.is_empty() {
            "default".to_string()
        } else {
            req.impl_id.clone()
        };

        // Create composite key: std_name::impl_id
        let key = format!("{}::{}", req.std_name, impl_id);

        // Parse IO parameters
        let inputs = match Self::parse_io_parameters(
            &req.input_names,
            &req.input_ros_types,
            &req.input_channels,
        ) {
            Ok(params) => params,
            Err(e) => {
                error!("Failed to parse input parameters: {}", e);
                return RegisterCapSklResponse {
                    success: false,
                    error_message: format!("Failed to parse input parameters: {}", e),
                };
            }
        };

        let outputs = match Self::parse_io_parameters(
            &req.output_names,
            &req.output_ros_types,
            &req.output_channels,
        ) {
            Ok(params) => params,
            Err(e) => {
                error!("Failed to parse output parameters: {}", e);
                return RegisterCapSklResponse {
                    success: false,
                    error_message: format!("Failed to parse output parameters: {}", e),
                };
            }
        };

        // Parse config services
        let configs = match Self::parse_config_services(&req.config_services, &req.config_names) {
            Ok(cfg) => cfg,
            Err(e) => {
                error!("Failed to parse config services: {}", e);
                return RegisterCapSklResponse {
                    success: false,
                    error_message: format!("Failed to parse config services: {}", e),
                };
            }
        };

        match req.package_type.as_str() {
            "cap" => {
                // Validate against spec
                match spec_registry.validate_capability(
                    &req.std_name,
                    &inputs,
                    &outputs,
                    &configs,
                ) {
                    Ok(()) => {
                        // Validation passed
                    }
                    Err(e) => {
                        warn!(
                            capability = %req.std_name,
                            impl_id = %impl_id,
                            error = %e,
                            "Capability validation failed"
                        );
                        return RegisterCapSklResponse {
                            success: false,
                            error_message: format!("Validation failed: {}", e),
                        };
                    }
                }

                let cap = Capability {
                    package_name: req.package_name.clone(),
                    std_name: req.std_name.clone(),
                    impl_id: impl_id.clone(),
                    description: req.description.clone(),
                    code_path: req.code_path.clone(),
                    inputs,
                    outputs,
                    configs,
                };

                let mut caps = self.capabilities.write().await;
                caps.insert(key.clone(), cap);

                info!(
                    capability = %req.std_name,
                    impl_id = %impl_id,
                    package = %req.package_name,
                    code_path = %req.code_path,
                    "Registered capability"
                );

                RegisterCapSklResponse {
                    success: true,
                    error_message: String::new(),
                }
            }
            "skl" => {
                // Validate against spec
                match spec_registry.validate_skill(&req.std_name, &inputs, &outputs, &configs) {
                    Ok(()) => {
                        // Validation passed
                    }
                    Err(e) => {
                        warn!(
                            skill = %req.std_name,
                            impl_id = %impl_id,
                            error = %e,
                            "Skill validation failed"
                        );
                        return RegisterCapSklResponse {
                            success: false,
                            error_message: format!("Validation failed: {}", e),
                        };
                    }
                }

                let skl = Skill {
                    package_name: req.package_name.clone(),
                    std_name: req.std_name.clone(),
                    impl_id: impl_id.clone(),
                    description: req.description.clone(),
                    code_path: req.code_path.clone(),
                    inputs,
                    outputs,
                    configs,
                };

                let mut skls = self.skills.write().await;
                skls.insert(key.clone(), skl);

                info!(
                    skill = %req.std_name,
                    impl_id = %impl_id,
                    package = %req.package_name,
                    code_path = %req.code_path,
                    "Registered skill"
                );

                RegisterCapSklResponse {
                    success: true,
                    error_message: String::new(),
                }
            }
            _ => {
                error!(
                    package_type = %req.package_type,
                    "Unknown package type"
                );
                RegisterCapSklResponse {
                    success: false,
                    error_message: format!("Unknown package type: {}", req.package_type),
                }
            }
        }
    }

    /// Query a capability or skill
    pub async fn query(&self, req: QueryCapSklRequest) -> QueryCapSklResponse {
        // Normalize impl_id: if empty, use "default" for lookup, but we'll search all if not found
        let impl_id = if req.impl_id.is_empty() {
            "default".to_string()
        } else {
            req.impl_id.clone()
        };

        // Try to find in capabilities first
        {
            let caps = self.capabilities.read().await;
            
            // If impl_id is specified, try exact match first
            if !req.impl_id.is_empty() {
                let key = format!("{}::{}", req.std_name, impl_id);
                if let Some(cap) = caps.get(&key) {
                    return Self::build_query_response(cap, &req.std_name, &impl_id, &caps);
                }
            } else {
                // If impl_id is empty, find first match and return all impl_ids
                let mut matching_caps: Vec<_> = caps
                    .iter()
                    .filter(|(key, _)| key.starts_with(&format!("{}::", req.std_name)))
                    .collect();
                
                if !matching_caps.is_empty() {
                    // Sort by key for consistent ordering
                    matching_caps.sort_by_key(|(k, _)| *k);
                    let (_, cap) = matching_caps[0];
                    return Self::build_query_response(cap, &req.std_name, &cap.impl_id, &caps);
                }
            }
        }

        // Try to find in skills
        {
            let skls = self.skills.read().await;
            
            // If impl_id is specified, try exact match first
            if !req.impl_id.is_empty() {
                let key = format!("{}::{}", req.std_name, impl_id);
                if let Some(skl) = skls.get(&key) {
                    return Self::build_query_response(skl, &req.std_name, &impl_id, &skls);
                }
            } else {
                // If impl_id is empty, find first match and return all impl_ids
                let mut matching_skls: Vec<_> = skls
                    .iter()
                    .filter(|(key, _)| key.starts_with(&format!("{}::", req.std_name)))
                    .collect();
                
                if !matching_skls.is_empty() {
                    // Sort by key for consistent ordering
                    matching_skls.sort_by_key(|(k, _)| *k);
                    let (_, skl) = matching_skls[0];
                    return Self::build_query_response(skl, &req.std_name, &skl.impl_id, &skls);
                }
            }
        }

        // Not found
        warn!(
            std_name = %req.std_name,
            impl_id = %req.impl_id,
            "Query not found"
        );

        QueryCapSklResponse {
            success: false,
            error_message: format!("Capability or skill '{}' (impl_id: '{}') not found", req.std_name, if req.impl_id.is_empty() { "any" } else { &req.impl_id }),
            impl_id: String::new(),
            impl_ids: Vec::new(),
            input_channels: Vec::new(),
            output_channels: Vec::new(),
            input_names: Vec::new(),
            output_names: Vec::new(),
            input_types: Vec::new(),
            output_types: Vec::new(),
        }
    }

    fn build_query_response<T>(
        item: &T,
        std_name: &str,
        impl_id: &str,
        all_items: &HashMap<String, T>,
    ) -> QueryCapSklResponse
    where
        T: HasIO,
    {
        let mut input_channels = Vec::new();
        let mut output_channels = Vec::new();
        let mut input_names = Vec::new();
        let mut output_names = Vec::new();
        let mut input_types = Vec::new();
        let mut output_types = Vec::new();

        for input in item.get_inputs() {
            input_channels.push(input.channel.clone());
            input_names.push(input.name.clone());
            input_types.push(input.ros_type.clone());
        }

        for output in item.get_outputs() {
            output_channels.push(output.channel.clone());
            output_names.push(output.name.clone());
            output_types.push(output.ros_type.clone());
        }

        // Collect all impl_ids for this std_name
        let mut all_impl_ids = Vec::new();
        for key in all_items.keys() {
            if key.starts_with(&format!("{}::", std_name)) {
                if let Some(id) = key.split("::").nth(1) {
                    all_impl_ids.push(id.to_string());
                }
            }
        }

        QueryCapSklResponse {
            success: true,
            error_message: String::new(),
            impl_id: impl_id.to_string(),
            impl_ids: all_impl_ids,
            input_channels,
            output_channels,
            input_names,
            output_names,
            input_types,
            output_types,
        }
    }

    /// Get all registered capabilities (returns std_name::impl_id format)
    pub async fn get_capabilities(&self) -> Vec<String> {
        let caps = self.capabilities.read().await;
        caps.keys().cloned().collect()
    }

    /// Get all registered skills (returns std_name::impl_id format)
    pub async fn get_skills(&self) -> Vec<String> {
        let skls = self.skills.read().await;
        skls.keys().cloned().collect()
    }

    /// Get a skill by name and impl_id
    pub async fn get_skill(&self, skill_name: &str, impl_id: Option<&str>) -> Option<Skill> {
        let skls = self.skills.read().await;
        let impl_id = impl_id.unwrap_or("default");
        let key = format!("{}::{}", skill_name, impl_id);
        skls.get(&key).cloned()
    }
}

// Helper trait for building query responses
trait HasIO {
    fn get_inputs(&self) -> &[crate::messages::IOParameter];
    fn get_outputs(&self) -> &[crate::messages::IOParameter];
}

impl HasIO for Capability {
    fn get_inputs(&self) -> &[crate::messages::IOParameter] {
        &self.inputs
    }

    fn get_outputs(&self) -> &[crate::messages::IOParameter] {
        &self.outputs
    }
}

impl HasIO for Skill {
    fn get_inputs(&self) -> &[crate::messages::IOParameter] {
        &self.inputs
    }

    fn get_outputs(&self) -> &[crate::messages::IOParameter] {
        &self.outputs
    }
}

