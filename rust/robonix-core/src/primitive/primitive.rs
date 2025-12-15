// SPDX-License-Identifier: MulanPSL-2.0
// Primitive Registry Module
//
// Handles primitive registration and querying according to robonix spec.
// Primitives provide standardized hardware capability mapping.

use crate::spec::SpecRegistry;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{error, info, warn};

/// Primitive registration request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterPrimitiveRequest {
    pub name: String,        // Standard primitive name
    pub input_schema: String,  // JSON string: {"argname0":"/topic0", ...}
    pub output_schema: String, // JSON string: {"argname1":"/topic1", ...}
    pub metadata: String,      // JSON string: metadata for instance filtering
    pub provider: String,      // Primitive provider identifier
    pub version: String,       // Implementation version (e.g., "1.0.0", "1.0.0-alpha")
}

impl ros2_client::Message for RegisterPrimitiveRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterPrimitiveResponse {
    pub ok: bool,
}

impl ros2_client::Message for RegisterPrimitiveResponse {}

/// Primitive query request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryPrimitiveRequest {
    pub name: String,
    pub filter: String, // JSON string: filter by metadata (e.g., {"resolution":">=720p"}, {"index":0}). Empty string means no filter
}

impl ros2_client::Message for QueryPrimitiveRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrimitiveInstance {
    pub provider: String,
    pub version: String,
    pub input_schema: serde_json::Value,
    pub output_schema: serde_json::Value,
    pub metadata: serde_json::Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryPrimitiveResponse {
    pub instances: Vec<PrimitiveInstance>,
}

impl ros2_client::Message for QueryPrimitiveResponse {}

#[derive(Debug, Clone)]
struct PrimitiveEntry {
    name: String,
    input_schema: serde_json::Value,
    output_schema: serde_json::Value,
    metadata: serde_json::Value,
    provider: String,
    version: String,
}

/// Primitive Registry - Manages primitive registration and querying
pub struct PrimitiveRegistry {
    primitives: Arc<RwLock<HashMap<String, PrimitiveEntry>>>,
}

impl PrimitiveRegistry {
    pub fn new() -> Self {
        Self {
            primitives: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Register a primitive
    pub async fn register(
        &self,
        req: RegisterPrimitiveRequest,
        spec_registry: &Arc<SpecRegistry>,
    ) -> RegisterPrimitiveResponse {
        // Parse JSON strings
        let input_schema: serde_json::Value = match serde_json::from_str(&req.input_schema) {
            Ok(v) => v,
            Err(e) => {
                warn!(
                    primitive_name = %req.name,
                    provider = %req.provider,
                    error = %e,
                    "failed to parse input_schema json"
                );
                return RegisterPrimitiveResponse { ok: false };
            }
        };
        let output_schema: serde_json::Value = match serde_json::from_str(&req.output_schema) {
            Ok(v) => v,
            Err(e) => {
                warn!(
                    primitive_name = %req.name,
                    provider = %req.provider,
                    error = %e,
                    "failed to parse output_schema json"
                );
                return RegisterPrimitiveResponse { ok: false };
            }
        };
        let metadata: serde_json::Value = match serde_json::from_str(&req.metadata) {
            Ok(v) => v,
            Err(e) => {
                warn!(
                    primitive_name = %req.name,
                    provider = %req.provider,
                    error = %e,
                    "failed to parse metadata json"
                );
                return RegisterPrimitiveResponse { ok: false };
            }
        };

        // Validate against spec
        match spec_registry.validate_primitive(
            &req.name,
            &input_schema,
            &output_schema,
        ) {
            Ok(()) => {
                // Validation passed
            }
            Err(e) => {
                warn!(
                    primitive_name = %req.name,
                    provider = %req.provider,
                    error = %e,
                    "primitive validation failed"
                );
                return RegisterPrimitiveResponse { ok: false };
            }
        }

        // Key includes name, provider, and version to distinguish different implementations
        let key = format!("{}::{}::{}", req.name, req.provider, req.version);

        let entry = PrimitiveEntry {
            name: req.name.clone(),
            input_schema,
            output_schema,
            metadata,
            provider: req.provider.clone(),
            version: req.version.clone(),
        };

        let mut primitives = self.primitives.write().await;
        primitives.insert(key, entry);

        info!(
            primitive_name = %req.name,
            provider = %req.provider,
            "registered primitive"
        );

        RegisterPrimitiveResponse { ok: true }
    }

    /// Query primitives
    pub async fn query(&self, req: QueryPrimitiveRequest) -> QueryPrimitiveResponse {
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
                if !self.matches_filter(&entry.metadata, &filter_value) {
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
                                (meta_value.as_f64(), filter_str[2..].parse::<f64>().ok()) {
                                if meta_num < filter_num {
                                    return false;
                                }
                            } else {
                                return false;
                            }
                        } else if filter_str.starts_with("<=") {
                            if let (Some(meta_num), Some(filter_num)) = 
                                (meta_value.as_f64(), filter_str[2..].parse::<f64>().ok()) {
                                if meta_num > filter_num {
                                    return false;
                                }
                            } else {
                                return false;
                            }
                        } else if filter_str.starts_with(">") {
                            if let (Some(meta_num), Some(filter_num)) = 
                                (meta_value.as_f64(), filter_str[1..].parse::<f64>().ok()) {
                                if meta_num <= filter_num {
                                    return false;
                                }
                            } else {
                                return false;
                            }
                        } else if filter_str.starts_with("<") {
                            if let (Some(meta_num), Some(filter_num)) = 
                                (meta_value.as_f64(), filter_str[1..].parse::<f64>().ok()) {
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

