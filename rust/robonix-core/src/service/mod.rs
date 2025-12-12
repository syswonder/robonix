// SPDX-License-Identifier: MulanPSL-2.0
// Service Module
//
// Services provide runtime capabilities such as perception, planning, deduction, and control.
// Services are algorithm and model carriers that run in managed environments.

pub mod service;

use crate::spec::SpecRegistry;
use service::ServiceRegistry as ServiceRegistryImpl;
use std::sync::Arc;

/// Service Registry - Manages service registration and querying
pub struct ServiceRegistry {
    registry: Arc<ServiceRegistryImpl>,
    spec_registry: Arc<SpecRegistry>,
}

impl ServiceRegistry {
    pub fn new() -> Self {
        Self {
            registry: Arc::new(ServiceRegistryImpl::new()),
            spec_registry: Arc::new(SpecRegistry::new()),
        }
    }

    pub fn get_registry(&self) -> Arc<ServiceRegistryImpl> {
        self.registry.clone()
    }

    pub async fn register_service(&self, req: service::RegisterServiceRequest) -> service::RegisterServiceResponse {
        self.registry.register(req, &self.spec_registry).await
    }

    pub async fn query_service(&self, req: service::QueryServiceRequest) -> service::QueryServiceResponse {
        self.registry.query(req).await
    }
}

