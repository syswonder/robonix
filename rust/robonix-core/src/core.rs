use crate::messages::{
    Capability, ConfigService, IOParameter, QueryRequest, QueryResponse, RegisterRequest, RegisterResponse, Skill,
};
use crate::spec::SpecRegistry;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{error, info, warn};

// Robonix core state
pub struct RobonixCore {
    capabilities: Arc<RwLock<HashMap<String, Capability>>>,
    skills: Arc<RwLock<HashMap<String, Skill>>>,
    spec_registry: Arc<SpecRegistry>,
}

impl RobonixCore {
    pub fn new() -> Self {
        Self {
            capabilities: Arc::new(RwLock::new(HashMap::new())),
            skills: Arc::new(RwLock::new(HashMap::new())),
            spec_registry: Arc::new(SpecRegistry::new()),
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

    pub async fn register(&self, req: RegisterRequest) -> RegisterResponse {
        // Parse IO parameters
        let inputs = match Self::parse_io_parameters(
            &req.input_names,
            &req.input_ros_types,
            &req.input_channels,
        ) {
            Ok(params) => params,
            Err(e) => {
                error!("Failed to parse input parameters: {}", e);
                return RegisterResponse {
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
                return RegisterResponse {
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
                return RegisterResponse {
                    success: false,
                    error_message: format!("Failed to parse config services: {}", e),
                };
            }
        };

        match req.package_type.as_str() {
            "cap" => {
                // Validate against spec
                match self.spec_registry.validate_capability(
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
                            error = %e,
                            "Capability validation failed"
                        );
                        return RegisterResponse {
                            success: false,
                            error_message: format!("Validation failed: {}", e),
                        };
                    }
                }

                let cap = Capability {
                    package_name: req.package_name.clone(),
                    std_name: req.std_name.clone(),
                    description: req.description.clone(),
                    code_path: req.code_path.clone(),
                    inputs,
                    outputs,
                    configs,
                };

                let mut caps = self.capabilities.write().await;
                caps.insert(req.std_name.clone(), cap);

                info!(
                    capability = %req.std_name,
                    package = %req.package_name,
                    code_path = %req.code_path,
                    "Registered capability"
                );

                RegisterResponse {
                    success: true,
                    error_message: String::new(),
                }
            }
            "skl" => {
                // Validate against spec
                match self.spec_registry.validate_skill(
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
                            skill = %req.std_name,
                            error = %e,
                            "Skill validation failed"
                        );
                        return RegisterResponse {
                            success: false,
                            error_message: format!("Validation failed: {}", e),
                        };
                    }
                }

                let skl = Skill {
                    package_name: req.package_name.clone(),
                    std_name: req.std_name.clone(),
                    description: req.description.clone(),
                    code_path: req.code_path.clone(),
                    inputs,
                    outputs,
                    configs,
                };

                let mut skls = self.skills.write().await;
                skls.insert(req.std_name.clone(), skl);

                info!(
                    skill = %req.std_name,
                    package = %req.package_name,
                    code_path = %req.code_path,
                    "Registered skill"
                );

                RegisterResponse {
                    success: true,
                    error_message: String::new(),
                }
            }
            _ => {
                error!(
                    package_type = %req.package_type,
                    "Unknown package type"
                );
                RegisterResponse {
                    success: false,
                    error_message: format!("Unknown package type: {}", req.package_type),
                }
            }
        }
    }

    pub async fn get_capabilities(&self) -> Vec<String> {
        let caps = self.capabilities.read().await;
        caps.keys().cloned().collect()
    }

    pub async fn get_skills(&self) -> Vec<String> {
        let skls = self.skills.read().await;
        skls.keys().cloned().collect()
    }

    pub async fn query(&self, req: QueryRequest) -> QueryResponse {
        // Try to find in capabilities first
        {
            let caps = self.capabilities.read().await;
            if let Some(cap) = caps.get(&req.std_name) {
                let mut input_channels = Vec::new();
                let mut output_channels = Vec::new();
                let mut input_names = Vec::new();
                let mut output_names = Vec::new();
                let mut input_types = Vec::new();
                let mut output_types = Vec::new();

                for input in &cap.inputs {
                    input_channels.push(input.channel.clone());
                    input_names.push(input.name.clone());
                    input_types.push(input.ros_type.clone());
                }

                for output in &cap.outputs {
                    output_channels.push(output.channel.clone());
                    output_names.push(output.name.clone());
                    output_types.push(output.ros_type.clone());
                }

                info!(
                    std_name = %req.std_name,
                    "Query capability found"
                );

                return QueryResponse {
                    success: true,
                    error_message: String::new(),
                    input_channels,
                    output_channels,
                    input_names,
                    output_names,
                    input_types,
                    output_types,
                };
            }
        }

        // Try to find in skills
        {
            let skls = self.skills.read().await;
            if let Some(skl) = skls.get(&req.std_name) {
                let mut input_channels = Vec::new();
                let mut output_channels = Vec::new();
                let mut input_names = Vec::new();
                let mut output_names = Vec::new();
                let mut input_types = Vec::new();
                let mut output_types = Vec::new();

                for input in &skl.inputs {
                    input_channels.push(input.channel.clone());
                    input_names.push(input.name.clone());
                    input_types.push(input.ros_type.clone());
                }

                for output in &skl.outputs {
                    output_channels.push(output.channel.clone());
                    output_names.push(output.name.clone());
                    output_types.push(output.ros_type.clone());
                }

                info!(
                    std_name = %req.std_name,
                    "Query skill found"
                );

                return QueryResponse {
                    success: true,
                    error_message: String::new(),
                    input_channels,
                    output_channels,
                    input_names,
                    output_names,
                    input_types,
                    output_types,
                };
            }
        }

        // Not found
        warn!(
            std_name = %req.std_name,
            "Query not found"
        );

        QueryResponse {
            success: false,
            error_message: format!("Capability or skill '{}' not found", req.std_name),
            input_channels: Vec::new(),
            output_channels: Vec::new(),
            input_names: Vec::new(),
            output_names: Vec::new(),
            input_types: Vec::new(),
            output_types: Vec::new(),
        }
    }
}
