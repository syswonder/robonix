// SPDX-License-Identifier: MulanPSL-2.0
// Primitive Abstraction Layer Module
//
// The Primitive Abstraction Layer provides standardized, controllable hardware capability mapping,
// unified management of actuator and sensor access, avoiding direct binding of upper-layer tasks to underlying hardware.

pub mod primitive;

use crate::spec::SpecRegistry;
use primitive::PrimitiveRegistry as PrimitiveRegistryImpl;
use std::sync::Arc;

/// Primitive Registry - Manages primitive registration and querying
pub struct PrimitiveRegistry {
    registry: Arc<PrimitiveRegistryImpl>,
    spec_registry: Arc<SpecRegistry>,
}

impl PrimitiveRegistry {
    pub fn new() -> Self {
        Self {
            registry: Arc::new(PrimitiveRegistryImpl::new()),
            spec_registry: Arc::new(SpecRegistry::new()),
        }
    }

    pub fn get_registry(&self) -> Arc<PrimitiveRegistryImpl> {
        self.registry.clone()
    }

    pub async fn register_primitive(&self, req: primitive::RegisterPrimitiveRequest) -> primitive::RegisterPrimitiveResponse {
        self.registry.register(req, &self.spec_registry).await
    }

    pub async fn query_primitive(&self, req: primitive::QueryPrimitiveRequest) -> primitive::QueryPrimitiveResponse {
        self.registry.query(req).await
    }
}

