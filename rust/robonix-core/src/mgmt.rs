// SPDX-License-Identifier: MulanPSL-2.0
// Management Module
//
// This module manages capabilities, skills, and AI model registrations.

use crate::messages::{
    Capability, ConfigService, IOParameter, ModelType, QueryCapSklRequest, QueryCapSklResponse, 
    RegisterCapSklRequest, RegisterCapSklResponse, Skill,
};
use crate::spec::SpecRegistry;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{error, info, warn};

// Model registration data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Model {
    pub model_id: String,
    pub model_name: String,
    pub model_type: ModelType,  // LLM or VLM
    pub provider: String,       // e.g., "openai", "anthropic", "local"
    pub api_endpoint: String,   // API endpoint URL
    pub api_key: Option<String>, // Optional API key
    pub description: String,
    pub capabilities: Vec<String>, // Supported capabilities
}

// Model registration request/response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterModelRequest {
    pub model_id: String,
    pub model_name: String,
    pub model_type: ModelType, // LLM or VLM
    pub provider: String,
    pub api_endpoint: String,
    pub api_key: Option<String>,
    pub description: String,
    pub capabilities: Vec<String>,
}
impl ros2_client::Message for RegisterModelRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterModelResponse {
    pub success: bool,
    pub error_message: String,
}
impl ros2_client::Message for RegisterModelResponse {}

// Query model request/response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryModelRequest {
    pub model_id: Option<String>,   // If None, query all models
    pub model_type: Option<ModelType>, // Filter by model type (LLM or VLM)
    pub capability: Option<String>,  // Filter by capability
}
impl ros2_client::Message for QueryModelRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryModelResponse {
    pub success: bool,
    pub error_message: String,
    pub models: Vec<Model>,
}
impl ros2_client::Message for QueryModelResponse {}

// Backward compatibility aliases
pub type LLMModel = Model;
pub type RegisterLLMRequest = RegisterModelRequest;
pub type RegisterLLMResponse = RegisterModelResponse;
pub type QueryLLMRequest = QueryModelRequest;
pub type QueryLLMResponse = QueryModelResponse;

/// Management module for capabilities, skills, and AI models
pub struct ManagementModule {
    capabilities: Arc<RwLock<HashMap<String, Capability>>>,
    skills: Arc<RwLock<HashMap<String, Skill>>>,
    models: Arc<RwLock<HashMap<String, Model>>>,
    spec_registry: Arc<SpecRegistry>,
}

impl ManagementModule {
    pub fn new() -> Self {
        Self {
            capabilities: Arc::new(RwLock::new(HashMap::new())),
            skills: Arc::new(RwLock::new(HashMap::new())),
            models: Arc::new(RwLock::new(HashMap::new())),
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

    /// Register a capability or skill
    pub async fn register(&self, req: RegisterCapSklRequest) -> RegisterCapSklResponse {
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
                        return RegisterCapSklResponse {
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

                RegisterCapSklResponse {
                    success: true,
                    error_message: String::new(),
                }
            }
            "skl" => {
                // Validate against spec
                match self
                    .spec_registry
                    .validate_skill(&req.std_name, &inputs, &outputs, &configs)
                {
                    Ok(()) => {
                        // Validation passed
                    }
                    Err(e) => {
                        warn!(
                            skill = %req.std_name,
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

                return QueryCapSklResponse {
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

                return QueryCapSklResponse {
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

        QueryCapSklResponse {
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

    /// Get all registered capabilities
    pub async fn get_capabilities(&self) -> Vec<String> {
        let caps = self.capabilities.read().await;
        caps.keys().cloned().collect()
    }

    /// Get all registered skills
    pub async fn get_skills(&self) -> Vec<String> {
        let skls = self.skills.read().await;
        skls.keys().cloned().collect()
    }

    /// Get a skill by name
    pub async fn get_skill(&self, skill_name: &str) -> Option<Skill> {
        let skls = self.skills.read().await;
        skls.get(skill_name).cloned()
    }

    /// Register an AI model
    pub async fn register_model(&self, req: RegisterModelRequest) -> RegisterModelResponse {
        let model = Model {
            model_id: req.model_id.clone(),
            model_name: req.model_name.clone(),
            model_type: req.model_type.clone(),
            provider: req.provider.clone(),
            api_endpoint: req.api_endpoint.clone(),
            api_key: req.api_key.clone(),
            description: req.description.clone(),
            capabilities: req.capabilities.clone(),
        };

        let mut models = self.models.write().await;
        models.insert(req.model_id.clone(), model);

        info!(
            model_id = %req.model_id,
            model_name = %req.model_name,
            model_type = ?req.model_type,
            provider = %req.provider,
            "Registered model"
        );

        RegisterModelResponse {
            success: true,
            error_message: String::new(),
        }
    }

    /// Query AI models
    pub async fn query_model(&self, req: QueryModelRequest) -> QueryModelResponse {
        let models = self.models.read().await;

        let mut result = Vec::new();

        if let Some(model_id) = &req.model_id {
            // Query specific model
            if let Some(model) = models.get(model_id) {
                // Apply additional filters
                let mut include = true;
                if let Some(ref model_type) = req.model_type {
                    if model.model_type != *model_type {
                        include = false;
                    }
                }
                if include {
                    if let Some(ref capability) = req.capability {
                        if model.capabilities.contains(capability) {
                            result.push(model.clone());
                        }
                    } else {
                        result.push(model.clone());
                    }
                }
            }
        } else {
            // Filter all models
            for model in models.values() {
                let mut include = true;

                // Filter by model type
                if let Some(ref model_type) = req.model_type {
                    if model.model_type != *model_type {
                        include = false;
                    }
                }

                // Filter by capability
                if include {
                    if let Some(ref capability) = req.capability {
                        if !model.capabilities.contains(capability) {
                            include = false;
                        }
                    }
                }

                if include {
                    result.push(model.clone());
                }
            }
        }

        QueryModelResponse {
            success: true,
            error_message: String::new(),
            models: result,
        }
    }

    /// Get a model by ID
    pub async fn get_model(&self, model_id: &str) -> Option<Model> {
        let models = self.models.read().await;
        models.get(model_id).cloned()
    }

    // Backward compatibility methods
    pub async fn register_llm(&self, req: RegisterLLMRequest) -> RegisterLLMResponse {
        self.register_model(req).await
    }

    pub async fn query_llm(&self, req: QueryLLMRequest) -> QueryLLMResponse {
        self.query_model(req).await
    }

    pub async fn get_llm_model(&self, model_id: &str) -> Option<LLMModel> {
        self.get_model(model_id).await
    }
}

