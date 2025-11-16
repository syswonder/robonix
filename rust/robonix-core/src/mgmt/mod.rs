// SPDX-License-Identifier: MulanPSL-2.0
// Management Module
//
// This module manages capabilities, skills, and AI model registrations.

pub mod model;
pub mod registry;

use crate::messages::Skill;
use crate::spec::SpecRegistry;
use registry::Registry;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Management module for capabilities, skills, and AI models
pub struct ManagementModule {
    registry: Arc<Registry>,
    models: Arc<RwLock<std::collections::HashMap<String, model::Model>>>,
    spec_registry: Arc<SpecRegistry>,
}

impl ManagementModule {
    pub fn new() -> Self {
        Self {
            registry: Arc::new(Registry::new()),
            models: Arc::new(RwLock::new(std::collections::HashMap::new())),
            spec_registry: Arc::new(SpecRegistry::new()),
        }
    }

    pub fn get_registry(&self) -> Arc<Registry> {
        self.registry.clone()
    }

    pub fn get_spec_registry(&self) -> Arc<SpecRegistry> {
        self.spec_registry.clone()
    }

    pub async fn register(&self, req: crate::messages::RegisterCapSklRequest) -> crate::messages::RegisterCapSklResponse {
        self.registry.register(req, &self.spec_registry).await
    }

    pub async fn query(&self, req: crate::messages::QueryCapSklRequest) -> crate::messages::QueryCapSklResponse {
        self.registry.query(req).await
    }

    pub async fn register_model(&self, req: model::RegisterModelRequest) -> model::RegisterModelResponse {
        model::register_model(&self.models, req).await
    }

    pub async fn query_model(&self, req: model::QueryModelRequest) -> model::QueryModelResponse {
        model::query_model(&self.models, req).await
    }

    pub async fn get_model(&self, model_id: &str) -> Option<model::Model> {
        model::get_model(&self.models, model_id).await
    }

    pub async fn get_capabilities(&self) -> Vec<String> {
        self.registry.get_capabilities().await
    }

    pub async fn get_skills(&self) -> Vec<String> {
        self.registry.get_skills().await
    }

    pub async fn get_skill(&self, skill_name: &str, impl_id: Option<&str>) -> Option<Skill> {
        self.registry.get_skill(skill_name, impl_id).await
    }

    pub async fn ping(&self, req: crate::messages::PingRequest) -> crate::messages::PingResponse {
        use std::time::{SystemTime, UNIX_EPOCH};
        
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
        
        crate::messages::PingResponse {
            success: true,
            sequence: req.sequence,
            timestamp,
        }
    }
}

// Re-export model types
pub use model::{Model, QueryModelRequest, QueryModelResponse, RegisterModelRequest, RegisterModelResponse};

