// SPDX-License-Identifier: MulanPSL-2.0
// Skill Registry Module
//
// Handles skill registration and querying according to robonix spec.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{error, info, warn};

/// Skill registration request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterSkillRequest {
    pub name: String,        // Skill name
    pub r#type: String,      // Skill type: "basic" | "rtdl"
    pub start_topic: String, // Skill start topic
    pub status_topic: String, // Status feedback topic
    pub entry: String,       // Basic skill entry (required if type="basic")
    pub skill_dir: String,   // Skill directory path (required if type="rtdl")
    pub main_rtdl: String,    // Main RTDL file name (required if type="rtdl")
    pub start_args: String, // JSON string: input parameter schema
    pub status: String,     // JSON string: status feedback schema
    pub metadata: String,   // JSON string: metadata for instance filtering
    pub provider: String,   // Skill provider identifier
    pub version: String,    // Skill version
}

impl ros2_client::Message for RegisterSkillRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterSkillResponse {
    pub ok: bool,
    pub skill_id: String,
}

impl ros2_client::Message for RegisterSkillResponse {}

/// Skill query request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuerySkillRequest {
    pub name: String,
    pub filter: String, // JSON string: filter by metadata. Empty string means no filter
}

impl ros2_client::Message for QuerySkillRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SkillInstance {
    pub skill_id: String,
    pub provider: String,
    pub version: String,
    pub r#type: String,      // Skill type: "basic" | "rtdl"
    pub start_topic: String,
    pub status_topic: String,
    pub entry: String,       // Basic skill entry (if type="basic")
    pub skill_dir: String,   // Skill directory path (if type="rtdl")
    pub main_rtdl: String,    // Main RTDL file name (if type="rtdl")
    pub start_args: serde_json::Value,
    pub status: serde_json::Value,
    pub metadata: serde_json::Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuerySkillResponse {
    pub instances: Vec<SkillInstance>,
}

impl ros2_client::Message for QuerySkillResponse {}

/// Skill Registry - Manages skill registration and querying
pub struct SkillRegistry {
    skills: Arc<RwLock<HashMap<String, SkillEntry>>>,
    skill_id_counter: Arc<std::sync::atomic::AtomicU64>,
}

#[derive(Debug, Clone)]
struct SkillEntry {
    skill_id: String,
    name: String,
    r#type: String,      // Skill type: "basic" | "rtdl"
    start_topic: String,
    status_topic: String,
    entry: String,       // Basic skill entry (if type="basic")
    skill_dir: String,   // Skill directory path (if type="rtdl")
    main_rtdl: String,    // Main RTDL file name (if type="rtdl")
    start_args: serde_json::Value,
    status: serde_json::Value,
    metadata: serde_json::Value,
    provider: String,
    version: String,
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
    pub async fn register(
        &self,
        req: RegisterSkillRequest,
    ) -> RegisterSkillResponse {
        // Parse JSON strings
        let start_args: serde_json::Value = match serde_json::from_str(&req.start_args) {
            Ok(v) => v,
            Err(e) => {
                warn!(
                    skill_name = %req.name,
                    provider = %req.provider,
                    error = %e,
                    "failed to parse start_args json"
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
                    skill_name = %req.name,
                    provider = %req.provider,
                    error = %e,
                    "failed to parse status json"
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
                    skill_name = %req.name,
                    provider = %req.provider,
                    error = %e,
                    "failed to parse metadata json"
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
                skill_name = %req.name,
                skill_type = %req.r#type,
                "invalid skill type, must be 'basic' or 'rtdl'"
            );
            return RegisterSkillResponse {
                ok: false,
                skill_id: String::new(),
            };
        }

        // Validate required fields based on type
        if req.r#type == "basic" && req.entry.is_empty() {
            warn!(
                skill_name = %req.name,
                "basic skill must provide entry field"
            );
            return RegisterSkillResponse {
                ok: false,
                skill_id: String::new(),
            };
        }
        if req.r#type == "rtdl" && (req.skill_dir.is_empty() || req.main_rtdl.is_empty()) {
            warn!(
                skill_name = %req.name,
                "rtdl skill must provide skill_dir and main_rtdl fields"
            );
            return RegisterSkillResponse {
                ok: false,
                skill_id: String::new(),
            };
        }

        // Generate unique skill_id
        let counter = self.skill_id_counter.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        let skill_id = format!("skl_{}_{}", req.name.replace("::", "_"), counter);

        // Basic validation: skill name should start with 'skl::'
        if !req.name.starts_with("skl::") {
            warn!(skill_name = %req.name, "skill name should start with 'skl::'");
        }

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
            skill_id = %skill_id,
            skill_name = %req.name,
            provider = %req.provider,
            "registered skill"
        );

        RegisterSkillResponse {
            ok: true,
            skill_id,
        }
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
                
                if !metadata_filter.is_null() && !self.matches_filter(&entry.metadata, &metadata_filter) {
                    continue;
                }
            }

            instances.push(SkillInstance {
                skill_id: entry.skill_id.clone(),
                provider: entry.provider.clone(),
                version: entry.version.clone(),
                r#type: entry.r#type.clone(),
                start_topic: entry.start_topic.clone(),
                status_topic: entry.status_topic.clone(),
                entry: entry.entry.clone(),
                skill_dir: entry.skill_dir.clone(),
                main_rtdl: entry.main_rtdl.clone(),
                start_args: entry.start_args.clone(),
                status: entry.status.clone(),
                metadata: entry.metadata.clone(),
            });
        }

        QuerySkillResponse { instances }
    }

    /// Get skill by ID (returns SkillInstance)
    pub async fn get_skill_by_id(&self, skill_id: &str) -> Option<SkillInstance> {
        let skills = self.skills.read().await;
        if let Some(entry) = skills.get(skill_id) {
            Some(SkillInstance {
                skill_id: entry.skill_id.clone(),
                provider: entry.provider.clone(),
                version: entry.version.clone(),
                r#type: entry.r#type.clone(),
                start_topic: entry.start_topic.clone(),
                status_topic: entry.status_topic.clone(),
                entry: entry.entry.clone(),
                skill_dir: entry.skill_dir.clone(),
                main_rtdl: entry.main_rtdl.clone(),
                start_args: entry.start_args.clone(),
                status: entry.status.clone(),
                metadata: entry.metadata.clone(),
            })
        } else {
            None
        }
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

