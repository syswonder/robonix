// SPDX-License-Identifier: MulanPSL-2.0
// Skill Registry Module
//
// Handles skill registration and querying according to robonix spec.

use crate::ros_idl::skill::{
    QuerySkillRequest, QuerySkillResponse, RegisterSkillRequest, RegisterSkillResponse,
    SkillInstance,
};
use log::{info, warn};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Skill Registry - Manages skill registration and querying
pub struct SkillRegistry {
    skills: Arc<RwLock<HashMap<String, SkillEntry>>>,
    skill_id_counter: Arc<std::sync::atomic::AtomicU64>,
}

#[derive(Debug, Clone)]
struct SkillEntry {
    skill_id: String,
    name: String,
    r#type: String, // Skill type: "basic" | "rtdl"
    start_topic: String,
    status_topic: String,
    entry: String,                 // Basic skill entry (if type="basic")
    skill_dir: String,             // Skill directory path (if type="rtdl")
    main_rtdl: String,             // Main RTDL file name (if type="rtdl")
    start_args: serde_json::Value, // JSON: structured data for internal use
    status: serde_json::Value,     // JSON: structured data for internal use
    metadata: serde_json::Value,   // JSON: structured data for internal use
    provider: String,
    version: String,
}

impl SkillEntry {
    /// Convert to ROS2 message format (SkillInstance)
    fn to_skill_instance(&self) -> SkillInstance {
        SkillInstance {
            skill_id: self.skill_id.clone(),
            name: self.name.clone(),
            provider: self.provider.clone(),
            version: self.version.clone(),
            r#type: self.r#type.clone(),
            start_topic: self.start_topic.clone(),
            status_topic: self.status_topic.clone(),
            entry: self.entry.clone(),
            skill_dir: self.skill_dir.clone(),
            main_rtdl: self.main_rtdl.clone(),
            start_args: serde_json::to_string(&self.start_args)
                .unwrap_or_else(|_| "{}".to_string()),
            status: serde_json::to_string(&self.status).unwrap_or_else(|_| "{}".to_string()),
            metadata: serde_json::to_string(&self.metadata).unwrap_or_else(|_| "{}".to_string()),
        }
    }
}

impl SkillRegistry {
    pub fn new() -> Self {
        Self {
            skills: Arc::new(RwLock::new(HashMap::new())),
            skill_id_counter: Arc::new(std::sync::atomic::AtomicU64::new(0)),
        }
    }

    /// Register a skill
    /// Note: Skills do not have specifications - they are user-defined and flexible
    pub async fn register(&self, req: RegisterSkillRequest) -> RegisterSkillResponse {
        // Parse and validate JSON format
        let start_args: serde_json::Value = match serde_json::from_str(&req.start_args) {
            Ok(v) => v,
            Err(e) => {
                warn!(
                    "failed to parse start_args json: skill_name={}, provider={}, error={}",
                    req.name, req.provider, e
                );
                return RegisterSkillResponse {
                    ok: false,
                    skill_id: String::new(),
                };
            }
        };
        let status: serde_json::Value = match serde_json::from_str(&req.status) {
            Ok(v) => v,
            Err(e) => {
                warn!(
                    "failed to parse status json: skill_name={}, provider={}, error={}",
                    req.name, req.provider, e
                );
                return RegisterSkillResponse {
                    ok: false,
                    skill_id: String::new(),
                };
            }
        };
        let metadata: serde_json::Value = match serde_json::from_str(&req.metadata) {
            Ok(v) => v,
            Err(e) => {
                warn!(
                    "failed to parse metadata json: skill_name={}, provider={}, error={}",
                    req.name, req.provider, e
                );
                return RegisterSkillResponse {
                    ok: false,
                    skill_id: String::new(),
                };
            }
        };

        // Validate skill type
        if req.r#type != "basic" && req.r#type != "rtdl" {
            warn!(
                "invalid skill type, must be 'basic' or 'rtdl': skill_name={}, skill_type={}",
                req.name, req.r#type
            );
            return RegisterSkillResponse {
                ok: false,
                skill_id: String::new(),
            };
        }

        // Validate required fields based on type
        if req.r#type == "basic" && req.entry.is_empty() {
            warn!(
                "basic skill must provide entry field: skill_name={}",
                req.name
            );
            return RegisterSkillResponse {
                ok: false,
                skill_id: String::new(),
            };
        }
        if req.r#type == "rtdl" && (req.skill_dir.is_empty() || req.main_rtdl.is_empty()) {
            warn!(
                "rtdl skill must provide skill_dir and main_rtdl fields: skill_name={}",
                req.name
            );
            return RegisterSkillResponse {
                ok: false,
                skill_id: String::new(),
            };
        }

        // Generate unique skill_id (similar to primitive, no automatic prefix)
        let counter = self
            .skill_id_counter
            .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        // Use name directly, replace special characters for ID
        let skill_id = format!(
            "{}_{}",
            req.name.replace("::", "_").replace("-", "_"),
            counter
        );

        // Store with structured JSON internally
        let entry = SkillEntry {
            skill_id: skill_id.clone(),
            name: req.name.clone(),
            r#type: req.r#type.clone(),
            start_topic: req.start_topic.clone(),
            status_topic: req.status_topic.clone(),
            entry: req.entry.clone(),
            skill_dir: req.skill_dir.clone(),
            main_rtdl: req.main_rtdl.clone(),
            start_args,
            status,
            metadata,
            provider: req.provider.clone(),
            version: req.version.clone(),
        };

        let mut skills = self.skills.write().await;
        skills.insert(skill_id.clone(), entry);

        info!(
            "registered skill: skill_id={}, skill_name={}, provider={}",
            skill_id, req.name, req.provider
        );

        RegisterSkillResponse { ok: true, skill_id }
    }

    /// Query skills
    pub async fn query(&self, req: QuerySkillRequest) -> QuerySkillResponse {
        let skills = self.skills.read().await;
        let mut instances = Vec::new();

        for entry in skills.values() {
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

                // Check if filter contains "type" field (special handling for skill type)
                if let Some(filter_obj) = filter_value.as_object() {
                    if let Some(type_value) = filter_obj.get("type") {
                        if let Some(type_str) = type_value.as_str() {
                            if entry.r#type != type_str {
                                continue;
                            }
                        }
                    }
                }

                // Apply metadata filter (excluding "type" which is handled separately)
                let mut metadata_filter = filter_value.clone();
                if let Some(filter_obj) = metadata_filter.as_object_mut() {
                    filter_obj.remove("type");
                }

                // Use structured metadata directly (no need to parse)
                if !metadata_filter.is_null()
                    && !self.matches_filter(&entry.metadata, &metadata_filter)
                {
                    continue;
                }
            }

            // Convert to ROS2 message format (serialize JSON fields to strings)
            instances.push(entry.to_skill_instance());
        }

        QuerySkillResponse { instances }
    }

    /// Get skill by ID (returns SkillInstance)
    pub async fn get_skill_by_id(&self, skill_id: &str) -> Option<SkillInstance> {
        let skills = self.skills.read().await;
        skills.get(skill_id).map(|entry| entry.to_skill_instance())
    }

    /// Get all skill names
    pub async fn get_all_skill_names(&self) -> Vec<String> {
        let skills = self.skills.read().await;
        let mut names = std::collections::HashSet::new();
        for entry in skills.values() {
            names.insert(entry.name.clone());
        }
        names.into_iter().collect()
    }

    /// Get all registered skills (for web UI)
    pub async fn get_all_skills(&self) -> Vec<(String, SkillInstance)> {
        let skills = self.skills.read().await;
        skills
            .iter()
            .map(|(skill_id, entry)| (skill_id.clone(), entry.to_skill_instance()))
            .collect()
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
