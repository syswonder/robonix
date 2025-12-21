// SPDX-License-Identifier: MulanPSL-2.0
// Skill Library Module
//
// Skill Library stores reusable skills that can be called at runtime.
// Skills themselves do not have scheduling capabilities, but are triggered
// by the Task Manager in a controlled manner after scheduling and resource allocation.
//
// Note: Skills do not have specifications - they are user-defined and flexible.

pub mod skill;

use skill::SkillRegistry;
use std::sync::Arc;

/// Skill Library - Stores and manages reusable skills
pub struct SkillLibrary {
    registry: Arc<SkillRegistry>,
}

impl SkillLibrary {
    pub fn new() -> Self {
        Self {
            registry: Arc::new(SkillRegistry::new()),
        }
    }

    pub fn get_registry(&self) -> Arc<SkillRegistry> {
        self.registry.clone()
    }

    pub async fn register_skill(
        &self,
        req: skill::RegisterSkillRequest,
    ) -> skill::RegisterSkillResponse {
        self.registry.register(req).await
    }

    pub async fn query_skill(&self, req: skill::QuerySkillRequest) -> skill::QuerySkillResponse {
        self.registry.query(req).await
    }

    pub async fn get_skill(&self, skill_id: &str) -> Option<skill::SkillInstance> {
        self.registry.get_skill_by_id(skill_id).await
    }

    pub async fn get_skills(&self) -> Vec<String> {
        self.registry.get_all_skill_names().await
    }
}
