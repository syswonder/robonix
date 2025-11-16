// SPDX-License-Identifier: MulanPSL-2.0
// Model Management Module
//
// This module handles AI model (LLM/VLM) registration and querying.

use crate::messages::ModelType;
use ros2_client::Message;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::info;

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
impl Message for RegisterModelRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterModelResponse {
    pub success: bool,
    pub error_message: String,
}
impl Message for RegisterModelResponse {}

// Query model request/response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryModelRequest {
    pub model_id: Option<String>,   // If None, query all models
    pub model_type: Option<ModelType>, // Filter by model type (LLM or VLM)
    pub capability: Option<String>,  // Filter by capability
}
impl Message for QueryModelRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryModelResponse {
    pub success: bool,
    pub error_message: String,
    pub models: Vec<Model>,
}
impl Message for QueryModelResponse {}


/// Register an AI model
pub async fn register_model(
    models: &Arc<RwLock<HashMap<String, Model>>>,
    req: RegisterModelRequest,
) -> RegisterModelResponse {
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

    let mut models_map = models.write().await;
    models_map.insert(req.model_id.clone(), model);

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
pub async fn query_model(
    models: &Arc<RwLock<HashMap<String, Model>>>,
    req: QueryModelRequest,
) -> QueryModelResponse {
    let models_map = models.read().await;

    let mut result = Vec::new();

    if let Some(model_id) = &req.model_id {
        // Query specific model
        if let Some(model) = models_map.get(model_id) {
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
        for model in models_map.values() {
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
pub async fn get_model(
    models: &Arc<RwLock<HashMap<String, Model>>>,
    model_id: &str,
) -> Option<Model> {
    let models_map = models.read().await;
    models_map.get(model_id).cloned()
}

