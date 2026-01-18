// SPDX-License-Identifier: MulanPSL-2.0
// Skill ROS IDL Message Types

use serde::{Deserialize, Serialize};

/// Skill registration request (robonix spec)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterSkillRequest {
    pub name: String,         // Skill name
    pub r#type: String,       // Skill type: "basic" | "rtdl"
    pub start_topic: String,  // Skill start topic
    pub status_topic: String, // Status feedback topic
    pub entry: String,        // Basic skill entry (required if type="basic")
    pub skill_dir: String,    // Skill directory path (required if type="rtdl")
    pub main_rtdl: String,    // Main RTDL file name (required if type="rtdl")
    pub start_args: String,   // JSON string: input parameter schema
    pub status: String,       // JSON string: status feedback schema
    pub metadata: String,     // JSON string: metadata for instance filtering
    pub provider: String,     // Skill provider identifier
    pub version: String,      // Skill version
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
    pub name: String, // Standard skill name (e.g., "skl::wandering")
    pub provider: String,
    pub version: String,
    pub r#type: String, // Skill type: "basic" | "rtdl"
    pub start_topic: String,
    pub status_topic: String,
    pub entry: String,      // Basic skill entry (if type="basic")
    pub skill_dir: String,  // Skill directory path (if type="rtdl")
    pub main_rtdl: String,  // Main RTDL file name (if type="rtdl")
    pub start_args: String, // JSON string: input parameter schema
    pub status: String,     // JSON string: status feedback schema
    pub metadata: String,   // JSON string: metadata for instance filtering
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuerySkillResponse {
    pub instances: Vec<SkillInstance>,
}

impl ros2_client::Message for QuerySkillResponse {}
