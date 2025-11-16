// SPDX-License-Identifier: MulanPSL-2.0
// DSL Generation Module
//
// This module generates DSL code from natural language using AI models.

use crate::mgmt::ManagementModule;
use crate::perception::PerceptionModule;
use serde_json::json;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{error, info};

/// DSL Generator
pub struct DSLGenerator {
    mgmt: Arc<RwLock<Option<Arc<ManagementModule>>>>,
    perception: Arc<RwLock<Option<Arc<PerceptionModule>>>>,
}

impl DSLGenerator {
    pub fn new() -> Self {
        Self {
            mgmt: Arc::new(RwLock::new(None)),
            perception: Arc::new(RwLock::new(None)),
        }
    }

    pub fn set_mgmt(&self, mgmt: Arc<ManagementModule>) {
        let mut mgmt_guard = self.mgmt.blocking_write();
        *mgmt_guard = Some(mgmt);
    }

    pub fn set_perception(&self, perception: Arc<PerceptionModule>) {
        let mut perception_guard = self.perception.blocking_write();
        *perception_guard = Some(perception);
    }

    /// Generate DSL code from natural language using a model
    pub async fn generate_dsl(&self, natural_language: &str) -> Result<String, String> {
        // Get available models
        let mgmt_guard = self.mgmt.read().await;
        let mgmt = mgmt_guard.as_ref().ok_or("Management module not set")?;
        
        // Query for LLM models with planning capability
        let query_req = crate::mgmt::QueryModelRequest {
            model_id: None,
            model_type: Some(crate::messages::ModelType::LLM),
            capability: Some("planning".to_string()),
        };
        
        let query_resp = mgmt.query_model(query_req).await;
        if !query_resp.success || query_resp.models.is_empty() {
            return Err("No suitable model found for planning".to_string());
        }

        // Use the first available model
        let model = &query_resp.models[0];
        info!(model_id = %model.model_id, "Using model for DSL generation");

        // Get semantic map context
        let semantic_map_context = {
            let perception_guard = self.perception.read().await;
            if let Some(perception) = perception_guard.as_ref() {
                let perception_clone = perception.clone();
                drop(perception_guard);
                self.get_semantic_map_context(&perception_clone).await
            } else {
                String::new()
            }
        };

        // Get available skills
        let skills_context = self.get_skills_context(mgmt).await;

        // Build prompt
        let prompt = self.build_prompt(natural_language, &semantic_map_context, &skills_context);

        // Call model API (simplified - in production, use actual HTTP client)
        let dsl_code = self.call_model_api(model, &prompt).await?;

        info!("Generated DSL code");
        Ok(dsl_code)
    }

    async fn get_semantic_map_context(&self, perception: &Arc<PerceptionModule>) -> String {
        let semantic_map = perception.get_semantic_map();
        let map = semantic_map.read().await;
        let entities = map.get_all_entities();
        
        if entities.is_empty() {
            return "Semantic map is empty.".to_string();
        }

        let mut context = "Semantic Map (Entities and their capabilities):\n".to_string();
        for entity in entities {
            context.push_str(&format!("- {} (id: {})\n", entity.label, entity.id));
            
            // Show registered skills (for robots and controllable entities)
            if !entity.registered_skills.is_empty() {
                context.push_str(&format!("  Skills: {}\n", entity.registered_skills.join(", ")));
            }
            
            // Show registered capabilities
            if !entity.registered_capabilities.is_empty() {
                context.push_str(&format!("  Capabilities: {}\n", entity.registered_capabilities.join(", ")));
            }
            
            // Show spatial relations
            if !entity.relations.is_empty() {
                context.push_str("  Relations:\n");
                for rel in &entity.relations {
                    context.push_str(&format!("    - {} -> {}\n", 
                        format!("{:?}", rel.relation_type), rel.target_entity_id));
                }
            }
        }
        context
    }

    async fn get_skills_context(&self, mgmt: &Arc<ManagementModule>) -> String {
        // Get all registered skills from management module
        // This provides a global view of all available skills
        let skills = mgmt.get_skills().await;
        
        if skills.is_empty() {
            return "No skills registered in system.".to_string();
        }

        let mut context = "All Registered Skills in System:\n".to_string();
        for skill_key in skills {
            context.push_str(&format!("- {}\n", skill_key));
        }
        context
    }

    fn build_prompt(&self, natural_language: &str, semantic_map: &str, skills: &str) -> String {
        let data_types = self.get_data_types_info();
        
        format!(
            r#"You are a robot task planner. Given a natural language task description, generate DSL code to execute it.

DSL Format (simple list, one instruction per line):
skill_name(param1=value1, param2=value2)
another_skill(param3=value3)

Each line is a skill call that will be executed sequentially. 
Skills are called on entities (robots) that have those skills registered.
When referencing entities, use their entity id or label from the semantic map.

{data_types}

{semantic_map}

{skills}

Task: {natural_language}

Generate DSL code to accomplish this task. Only output the DSL code, one instruction per line, no explanations.
Use skills that are available on the entities in the semantic map.
When specifying parameter values, use appropriate data types as described above."#
        )
    }

    fn get_data_types_info(&self) -> String {
        r#"Data Types Available:

Robonix Custom Message Types (robonix_core package):
- Point3D: {{x: float64, y: float64, z: float64}} - 3D point coordinates
- BoundingBox: {{scale_x: float64, scale_y: float64, scale_z: float64, yaw: float64}} - 3D bounding box
- Entity: {{id: string, label: string, relations: Relation[], registered_skills: string[], registered_capabilities: string[], frame_mapping: FrameMapping}} - Semantic entity
- Relation: {{relation_type: RelationType, target_entity_id: string}} - Entity relationship
- SpatialMapEntry: {{frame_id: string, timestamp: uint64, source_skill: string, height: uint32, width: uint32, point_data: bytes, ...}} - Point cloud data

Standard ROS2 Message Types (commonly used):
- geometry_msgs/msg/PoseStamped: {{header: Header, pose: Pose}} - Position and orientation with frame
- geometry_msgs/msg/Pose: {{position: Point3D, orientation: Quaternion}} - Position and orientation
- geometry_msgs/msg/Point: {{x: float64, y: float64, z: float64}} - 3D point
- geometry_msgs/msg/Quaternion: {{x: float64, y: float64, z: float64, w: float64}} - Orientation quaternion
- geometry_msgs/msg/Twist: {{linear: Vector3, angular: Vector3}} - Velocity
- sensor_msgs/msg/Image: {{header: Header, height: uint32, width: uint32, encoding: string, data: uint8[]}} - Image data
- sensor_msgs/msg/PointCloud2: {{header: Header, height: uint32, width: uint32, fields: PointField[], data: uint8[]}} - Point cloud
- std_msgs/msg/String: {{data: string}} - String message
- std_msgs/msg/Bool: {{data: bool}} - Boolean message
- std_msgs/msg/Int32: {{data: int32}} - Integer message
- std_msgs/msg/Float64: {{data: float64}} - Float message

Parameter Value Format in DSL:
- For simple types: use literal values (e.g., param1="value", param2=123, param3=45.6)
- For entity references: use entity id or label (e.g., entity_id="robot_01")
- For complex types: use JSON-like format or structured string representation
- For Point3D: {{"x": 1.0, "y": 2.0, "z": 3.0}} or as string "1.0,2.0,3.0"
- For Pose: use structured format or coordinate string
- For images/pointclouds: typically passed as topic names or file paths"#.to_string()
    }

    async fn call_model_api(&self, model: &crate::mgmt::Model, prompt: &str) -> Result<String, String> {
        // Clone data for use in blocking context
        let model_clone = model.clone();
        let prompt_clone = prompt.to_string();
        
        // Use blocking client in unblock to avoid Tokio runtime requirement
        smol::unblock(move || {
            // Build OpenAI-compatible API request using blocking client
            let client = reqwest::blocking::Client::new();
            
            // Determine the API endpoint - if it ends with /chat/completions, use as-is, otherwise append it
            let endpoint = if model_clone.api_endpoint.ends_with("/chat/completions") {
                model_clone.api_endpoint.clone()
            } else {
                format!("{}/chat/completions", model_clone.api_endpoint.trim_end_matches('/'))
            };
            
            // Build request body in OpenAI format
            let request_body = json!({
                "model": model_clone.model_name,
                "messages": [
                    {
                        "role": "system",
                        "content": "You are a robot task planner. Generate DSL code based on the task description."
                    },
                    {
                        "role": "user",
                        "content": prompt_clone
                    }
                ],
                "temperature": 0.7,
                "max_tokens": 2000
            });
            
            info!(
                model_id = %model_clone.model_id,
                endpoint = %endpoint,
                "Calling model API"
            );
            
            // Build request with authorization header
            let mut request = client
                .post(&endpoint)
                .json(&request_body)
                .header("Content-Type", "application/json");
            
            // Add API key if provided
            if let Some(api_key) = &model_clone.api_key {
                if !api_key.is_empty() {
                    request = request.header("Authorization", format!("Bearer {}", api_key));
                }
                else {
                    error!(
                        model_id = %model_clone.model_id,
                        "API key is empty"
                    );
                    return Err("API key is empty".to_string());
                }
            }
            
            // Send request (blocking)
            let response = request
                .send()
                .map_err(|e| format!("Failed to send request to model API: {}", e))?;
            
            // Check response status
            if !response.status().is_success() {
                let status = response.status();
                let error_text = response.text().unwrap_or_else(|_| "Unknown error".to_string());
                error!(
                    model_id = %model_clone.model_id,
                    status = %status,
                    error = %error_text,
                    "Model API request failed"
                );
                return Err(format!("Model API returned error {}: {}", status, error_text));
            }
            
            // Parse response (blocking)
            let response_json: serde_json::Value = response
                .json()
                .map_err(|e| format!("Failed to parse model API response: {}", e))?;
            
            // Extract content from OpenAI format response
            let dsl_code = response_json
                .get("choices")
                .and_then(|choices| choices.as_array())
                .and_then(|arr| arr.get(0))
                .and_then(|choice| choice.get("message"))
                .and_then(|msg| msg.get("content"))
                .and_then(|content| content.as_str())
                .ok_or_else(|| {
                    error!(
                        model_id = %model_clone.model_id,
                        response = ?response_json,
                        "Unexpected response format from model API"
                    );
                    "Failed to extract DSL code from model response".to_string()
                })?;
            
            // Clean up the response - remove markdown code blocks if present
            let dsl_code = dsl_code.trim();
            let dsl_code = if dsl_code.starts_with("```") {
                // Remove markdown code blocks
                dsl_code
                    .trim_start_matches("```")
                    .trim_start_matches("dsl")
                    .trim_start_matches("```")
                    .trim_end_matches("```")
                    .trim()
            } else {
                dsl_code
            };
            
            info!(
                model_id = %model_clone.model_id,
                dsl_length = dsl_code.len(),
                "Successfully generated DSL from model"
            );
            
            Ok(dsl_code.to_string())
        })
        .await
    }
}

