// SPDX-License-Identifier: MulanPSL-2.0
// Service Module
//
// Services provide runtime capabilities such as perception, planning, deduction, and control.
// Services are algorithm and model carriers that run in managed environments.

use crate::ros_idl::service_registry::{
    QueryServiceRequest, QueryServiceResponse, RegisterServiceRequest, RegisterServiceResponse,
    ServiceInstance,
};
use crate::spec::SpecRegistry;
use log::{info, warn};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

#[derive(Debug, Clone)]
struct ServiceEntry {
    name: String,
    #[allow(dead_code)] // Used for validation and future queries
    srv_type: String,
    entry: String,
    metadata: serde_json::Value,
    provider: String,
    version: String,
}

/// Service Registry - Manages service registration and querying
pub struct ServiceRegistry {
    services: Arc<RwLock<HashMap<String, ServiceEntry>>>,
    spec_registry: Arc<SpecRegistry>,
}

impl ServiceRegistry {
    pub fn new() -> Self {
        Self::new_with_spec(Arc::new(SpecRegistry::new()))
    }

    pub fn new_with_spec(spec_registry: Arc<SpecRegistry>) -> Self {
        Self {
            services: Arc::new(RwLock::new(HashMap::new())),
            spec_registry,
        }
    }

    pub async fn register_service(&self, req: RegisterServiceRequest) -> RegisterServiceResponse {
        // Validate against spec
        match self
            .spec_registry
            .validate_service(&req.name, &req.srv_type)
        {
            Ok(()) => {
                // Validation passed
            }
            Err(e) => {
                warn!(
                    "service validation failed: service_name={}, provider={}, error={}",
                    req.name, req.provider, e
                );
                return RegisterServiceResponse { ok: false };
            }
        }

        // Key includes name, provider, and version to distinguish different implementations
        let key = format!("{}::{}::{}", req.name, req.provider, req.version);

        // Parse metadata JSON string
        let mut metadata: serde_json::Value = match serde_json::from_str(&req.metadata) {
            Ok(v) => v,
            Err(e) => {
                warn!(
                    "failed to parse metadata json: service_name={}, provider={}, error={}",
                    req.name, req.provider, e
                );
                return RegisterServiceResponse { ok: false };
            }
        };

        // Update or set status field in metadata
        // If service already exists, update status to "started" (from start command)
        // If new service, set status to "registered" (from register command)
        let mut services = self.services.write().await;
        let status = if services.contains_key(&key) {
            // Service already registered, this is a state sync from start command
            info!(
                "updating service status to 'started': service_name={}, provider={}, version={}",
                req.name, req.provider, req.version
            );
            "started"
        } else {
            // New service registration from register command
            info!(
                "registering new service with status 'registered': service_name={}, provider={}, version={}",
                req.name, req.provider, req.version
            );
            "registered"
        };

        // Set status in metadata
        if let Some(meta_obj) = metadata.as_object_mut() {
            meta_obj.insert(
                "status".to_string(),
                serde_json::Value::String(status.to_string()),
            );
        } else {
            // If metadata is not an object, create a new object with status
            let mut new_meta = serde_json::Map::new();
            new_meta.insert(
                "status".to_string(),
                serde_json::Value::String(status.to_string()),
            );
            // Try to merge existing metadata if it's an object
            if let Some(existing_obj) = metadata.as_object() {
                for (k, v) in existing_obj {
                    new_meta.insert(k.clone(), v.clone());
                }
            }
            metadata = serde_json::Value::Object(new_meta);
        }

        let entry = ServiceEntry {
            name: req.name.clone(),
            srv_type: req.srv_type.clone(),
            entry: req.entry.clone(),
            metadata,
            provider: req.provider.clone(),
            version: req.version.clone(),
        };

        services.insert(key, entry);

        info!(
            "registered service: service_name={}, provider={}, entry={}, srv_type={}",
            req.name, req.provider, req.entry, req.srv_type
        );

        RegisterServiceResponse { ok: true }
    }

    pub async fn query_service(&self, req: QueryServiceRequest) -> QueryServiceResponse {
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

    /// Query service and return only instances with status="started"
    /// This is used internally by core to ensure only started services are used
    pub async fn query_started_service(&self, name: &str) -> QueryServiceResponse {
        let query_req = QueryServiceRequest {
            name: name.to_string(),
            filter: "{}".to_string(),
        };
        let resp = self.query_service(query_req).await;

        // Filter to only return instances with status="started"
        let started_instances: Vec<ServiceInstance> = resp
            .instances
            .into_iter()
            .filter(|inst| {
                let status = inst
                    .metadata
                    .get("status")
                    .and_then(|v| v.as_str())
                    .unwrap_or("registered");
                status == "started"
            })
            .collect();

        QueryServiceResponse {
            instances: started_instances,
        }
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
