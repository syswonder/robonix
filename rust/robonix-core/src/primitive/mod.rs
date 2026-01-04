// SPDX-License-Identifier: MulanPSL-2.0
// Primitive Abstraction Layer Module
//
// The Primitive Abstraction Layer provides standardized, controllable hardware capability mapping,
// unified management of actuator and sensor access, avoiding direct binding of upper-layer tasks to underlying hardware.

use crate::ros_idl::primitive::{
    PrimitiveInstance, QueryPrimitiveRequest, QueryPrimitiveResponse, RegisterPrimitiveRequest,
    RegisterPrimitiveResponse,
};
use crate::spec::SpecRegistry;
use log::{info, warn};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

#[derive(Debug, Clone)]
struct PrimitiveEntry {
    name: String,
    input_schema: String,  // JSON string: stored as string internally
    output_schema: String, // JSON string: stored as string internally
    metadata: String,      // JSON string: stored as string internally
    provider: String,
    version: String,
}

/// Primitive Registry - Manages primitive registration and querying
pub struct PrimitiveRegistry {
    primitives: Arc<RwLock<HashMap<String, PrimitiveEntry>>>,
    spec_registry: Arc<SpecRegistry>,
}

impl PrimitiveRegistry {
    pub fn new() -> Self {
        Self::new_with_spec(Arc::new(SpecRegistry::new()))
    }

    pub fn new_with_spec(spec_registry: Arc<SpecRegistry>) -> Self {
        Self {
            primitives: Arc::new(RwLock::new(HashMap::new())),
            spec_registry,
        }
    }

    pub async fn register_primitive(
        &self,
        req: RegisterPrimitiveRequest,
    ) -> RegisterPrimitiveResponse {
        // Validate JSON format and parse for spec validation
        let input_schema: serde_json::Value = match serde_json::from_str(&req.input_schema) {
            Ok(v) => v,
            Err(e) => {
                warn!(
                    "failed to parse input_schema json: primitive_name={}, provider={}, error={}",
                    req.name, req.provider, e
                );
                return RegisterPrimitiveResponse { ok: false };
            }
        };
        let output_schema: serde_json::Value = match serde_json::from_str(&req.output_schema) {
            Ok(v) => v,
            Err(e) => {
                warn!(
                    "failed to parse output_schema json: primitive_name={}, provider={}, error={}",
                    req.name, req.provider, e
                );
                return RegisterPrimitiveResponse { ok: false };
            }
        };
        // Validate metadata JSON format
        if serde_json::from_str::<serde_json::Value>(&req.metadata).is_err() {
            warn!(
                "failed to parse metadata json: primitive_name={}, provider={}",
                req.name, req.provider
            );
            return RegisterPrimitiveResponse { ok: false };
        }

        // Validate against spec
        match self
            .spec_registry
            .validate_primitive(&req.name, &input_schema, &output_schema)
        {
            Ok(()) => {
                // Validation passed
            }
            Err(e) => {
                warn!(
                    "primitive validation failed: primitive_name={}, provider={}, error={}",
                    req.name, req.provider, e
                );
                return RegisterPrimitiveResponse { ok: false };
            }
        }

        // Key includes name, provider, and version to distinguish different implementations
        let key = format!("{}::{}::{}", req.name, req.provider, req.version);

        // Store as JSON strings internally
        let entry = PrimitiveEntry {
            name: req.name.clone(),
            input_schema: req.input_schema.clone(),
            output_schema: req.output_schema.clone(),
            metadata: req.metadata.clone(),
            provider: req.provider.clone(),
            version: req.version.clone(),
        };

        let mut primitives = self.primitives.write().await;
        primitives.insert(key, entry);

        info!(
            "registered primitive: primitive_name={}, provider={}",
            req.name, req.provider
        );

        RegisterPrimitiveResponse { ok: true }
    }

    pub async fn query_primitive(&self, req: QueryPrimitiveRequest) -> QueryPrimitiveResponse {
        let primitives = self.primitives.read().await;
        let mut instances = Vec::new();

        for entry in primitives.values() {
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
                // Parse metadata for filtering
                let metadata_value: serde_json::Value = match serde_json::from_str(&entry.metadata)
                {
                    Ok(v) => v,
                    Err(_) => continue, // Skip if metadata is invalid JSON
                };
                if !self.matches_filter(&metadata_value, &filter_value) {
                    continue;
                }
            }

            instances.push(PrimitiveInstance {
                provider: entry.provider.clone(),
                version: entry.version.clone(),
                input_schema: entry.input_schema.clone(),
                output_schema: entry.output_schema.clone(),
                metadata: entry.metadata.clone(),
            });
        }

        QueryPrimitiveResponse { instances }
    }

    /// Check if metadata matches filter
    /// Supports simple equality and comparison operators (>=, <=, >, <) for numeric values
    fn matches_filter(&self, metadata: &serde_json::Value, filter: &serde_json::Value) -> bool {
        if let (Some(meta_obj), Some(filter_obj)) = (metadata.as_object(), filter.as_object()) {
            for (key, filter_value) in filter_obj {
                if let Some(meta_value) = meta_obj.get(key) {
                    // Check if filter_value is a string with comparison operator
                    if let Some(filter_str) = filter_value.as_str() {
                        if filter_str.starts_with(">=") {
                            if let (Some(meta_num), Some(filter_num)) =
                                (meta_value.as_f64(), filter_str[2..].parse::<f64>().ok())
                            {
                                if meta_num < filter_num {
                                    return false;
                                }
                            } else {
                                return false;
                            }
                        } else if filter_str.starts_with("<=") {
                            if let (Some(meta_num), Some(filter_num)) =
                                (meta_value.as_f64(), filter_str[2..].parse::<f64>().ok())
                            {
                                if meta_num > filter_num {
                                    return false;
                                }
                            } else {
                                return false;
                            }
                        } else if filter_str.starts_with(">") {
                            if let (Some(meta_num), Some(filter_num)) =
                                (meta_value.as_f64(), filter_str[1..].parse::<f64>().ok())
                            {
                                if meta_num <= filter_num {
                                    return false;
                                }
                            } else {
                                return false;
                            }
                        } else if filter_str.starts_with("<") {
                            if let (Some(meta_num), Some(filter_num)) =
                                (meta_value.as_f64(), filter_str[1..].parse::<f64>().ok())
                            {
                                if meta_num >= filter_num {
                                    return false;
                                }
                            } else {
                                return false;
                            }
                        } else {
                            // Simple equality
                            if meta_value != filter_value {
                                return false;
                            }
                        }
                    } else {
                        // Simple equality
                        if meta_value != filter_value {
                            return false;
                        }
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
