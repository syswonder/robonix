// SPDX-License-Identifier: MulanPSL-2.0
// Core Module
//
// Core coordination module for robonix-core

use crate::primitive::PrimitiveRegistry;
use crate::service::ServiceRegistry;
use crate::skill_library::SkillLibrary;
use crate::spec::SpecRegistry;
use crate::task_manager::TaskManager;
use ros2_client::Node;
use std::sync::Arc;
use tokio::sync::Mutex;

// robonix Core - coordinates all modules according to robonix architecture
pub struct RobonixCore {
    // robonix core components
    task_manager: Arc<TaskManager>,
    skill_library: Arc<SkillLibrary>,
    service_registry: Arc<ServiceRegistry>,
    primitive_registry: Arc<PrimitiveRegistry>,
}

impl RobonixCore {
    pub fn new(node: Arc<Mutex<Node>>) -> Self {
        // Create shared spec registry (used by both primitive and service registries)
        let spec_registry = Arc::new(SpecRegistry::new());

        // Create robonix core components
        let skill_library = Arc::new(SkillLibrary::new());
        let service_registry = Arc::new(ServiceRegistry::new_with_spec(spec_registry.clone()));
        let primitive_registry = Arc::new(PrimitiveRegistry::new_with_spec(spec_registry));

        let task_manager = TaskManager::new(
            skill_library.clone(),
            service_registry.clone(),
            primitive_registry.clone(),
            node,
        );

        Self {
            task_manager,
            skill_library,
            service_registry,
            primitive_registry,
        }
    }

    // robonix core component accessors
    pub fn get_task_manager(&self) -> Arc<TaskManager> {
        self.task_manager.clone()
    }

    pub fn get_skill_library(&self) -> Arc<SkillLibrary> {
        self.skill_library.clone()
    }

    pub fn get_service_registry(&self) -> Arc<ServiceRegistry> {
        self.service_registry.clone()
    }

    pub fn get_primitive_registry(&self) -> Arc<PrimitiveRegistry> {
        self.primitive_registry.clone()
    }
}
