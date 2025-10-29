use crate::messages::{Capability, RegisterRequest, RegisterResponse, Skill};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{error, info};

// Robonix core state
pub struct RobonixCore {
    capabilities: Arc<RwLock<HashMap<String, Capability>>>,
    skills: Arc<RwLock<HashMap<String, Skill>>>,
}

impl RobonixCore {
    pub fn new() -> Self {
        Self {
            capabilities: Arc::new(RwLock::new(HashMap::new())),
            skills: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    pub async fn register(&self, req: RegisterRequest) -> RegisterResponse {
        match req.provider_type.as_str() {
            "cap" => {
                let cap = Capability {
                    provider_name: req.provider_name.clone(),
                    std_name: req.std_name.clone(),
                    description: req.description.clone(),
                    input_topics: req.input_topics.clone(),
                    output_topics: req.output_topics.clone(),
                };

                let mut caps = self.capabilities.write().await;
                caps.insert(req.std_name.clone(), cap);

                info!(
                    capability = %req.std_name,
                    provider = %req.provider_name,
                    "Registered capability"
                );

                RegisterResponse { success: true }
            }
            "skl" => {
                let skl = Skill {
                    provider_name: req.provider_name.clone(),
                    std_name: req.std_name.clone(),
                    description: req.description.clone(),
                    input_topics: req.input_topics.clone(),
                    output_topics: req.output_topics.clone(),
                };

                let mut skls = self.skills.write().await;
                skls.insert(req.std_name.clone(), skl);

                info!(
                    skill = %req.std_name,
                    provider = %req.provider_name,
                    "Registered skill"
                );

                RegisterResponse { success: true }
            }
            _ => {
                error!(
                    provider_type = %req.provider_type,
                    "Unknown provider type"
                );
                RegisterResponse { success: false }
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
}
