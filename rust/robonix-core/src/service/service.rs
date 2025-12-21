// SPDX-License-Identifier: MulanPSL-2.0
// Service Registry Module
//
// Handles service registration and querying according to robonix spec.
// Services encapsulate algorithm capabilities such as perception, planning, evaluation, and verification.

use crate::spec::SpecRegistry;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{info, warn};

/// Service registration request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterServiceRequest {
    pub name: String,     // Standard service name
    pub srv_type: String, // ROS2 service type (e.g., "robonix_sdk/srv/service/spatial_map/GetSpatialMap")
    pub entry: String,    // Actual ROS2 service name
    pub metadata: String, // JSON string: metadata for instance filtering
    pub provider: String, // Service provider identifier
    pub version: String,  // Implementation version (e.g., "1.0.0", "1.0.0-alpha")
}

impl ros2_client::Message for RegisterServiceRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterServiceResponse {
    pub ok: bool,
}

impl ros2_client::Message for RegisterServiceResponse {}

/// Service query request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryServiceRequest {
    pub name: String,
    pub filter: String, // JSON string: filter by metadata. Empty string means no filter
}

impl ros2_client::Message for QueryServiceRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServiceInstance {
    pub provider: String,
    pub version: String,
    pub entry: String,
    pub metadata: serde_json::Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryServiceResponse {
    pub instances: Vec<ServiceInstance>,
}

impl ros2_client::Message for QueryServiceResponse {}

#[derive(Debug, Clone)]
struct ServiceEntry {
    name: String,
    srv_type: String,
    entry: String,
    metadata: serde_json::Value,
    provider: String,
    version: String,
}

/// Service Registry - Manages service registration and querying
pub struct ServiceRegistry {
    services: Arc<RwLock<HashMap<String, ServiceEntry>>>,
}

impl ServiceRegistry {
    pub fn new() -> Self {
        Self {
            services: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Register a service
    pub async fn register(
        &self,
        req: RegisterServiceRequest,
        spec_registry: &Arc<SpecRegistry>,
    ) -> RegisterServiceResponse {
        // Validate against spec
        match spec_registry.validate_service(&req.name, &req.srv_type) {
            Ok(()) => {
                // Validation passed
            }
            Err(e) => {
                warn!(
                    service_name = %req.name,
                    provider = %req.provider,
                    error = %e,
                    "service validation failed"
                );
                return RegisterServiceResponse { ok: false };
            }
        }

        // Key includes name, provider, and version to distinguish different implementations
        let key = format!("{}::{}::{}", req.name, req.provider, req.version);

        // Parse metadata JSON string
        let metadata: serde_json::Value = match serde_json::from_str(&req.metadata) {
            Ok(v) => v,
            Err(e) => {
                warn!(
                    service_name = %req.name,
                    provider = %req.provider,
                    error = %e,
                    "failed to parse metadata json"
                );
                return RegisterServiceResponse { ok: false };
            }
        };

        let entry = ServiceEntry {
            name: req.name.clone(),
            srv_type: req.srv_type.clone(),
            entry: req.entry.clone(),
            metadata,
            provider: req.provider.clone(),
            version: req.version.clone(),
        };

        let mut services = self.services.write().await;
        services.insert(key, entry);

        info!(
            service_name = %req.name,
            provider = %req.provider,
            entry = %req.entry,
            srv_type = %req.srv_type,
            "registered service"
        );

        RegisterServiceResponse { ok: true }
    }

    /// Query services
    pub async fn query(&self, req: QueryServiceRequest) -> QueryServiceResponse {
        let services = self.services.read().await;
        let mut instances = Vec::new();

        for entry in services.values() {
            // Match by name
            if entry.name != req.name {
                continue;
            }

            // Apply filter if provided
            if !req.filter.is_empty() {
                let filter_value: serde_json::Value = match serde_json::from_str(&req.filter) {
                    Ok(v) => v,
                    Err(_) => continue, // Skip if filter is invalid JSON
                };
                if !self.matches_filter(&entry.metadata, &filter_value) {
                    continue;
                }
            }

            instances.push(ServiceInstance {
                provider: entry.provider.clone(),
                version: entry.version.clone(),
                entry: entry.entry.clone(),
                metadata: entry.metadata.clone(),
            });
        }

        QueryServiceResponse { instances }
    }

    /// Check if metadata matches filter
    fn matches_filter(&self, metadata: &serde_json::Value, filter: &serde_json::Value) -> bool {
        if let (Some(meta_obj), Some(filter_obj)) = (metadata.as_object(), filter.as_object()) {
            for (key, filter_value) in filter_obj {
                if let Some(meta_value) = meta_obj.get(key) {
                    // Simple equality check for now
                    // Could extend to support >=, <=, etc. for numeric values
                    if meta_value != filter_value {
                        return false;
                    }
                } else {
                    return false;
                }
            }
            true
        } else {
            false
        }
    }
}
