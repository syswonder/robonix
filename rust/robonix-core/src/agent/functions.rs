// SPDX-License-Identifier: MulanPSL-2.0
// Function Registry Module
//
// Registry for system functions that can be called by the agent

use crate::core::RobonixCore;
use anyhow::{Context, Result};
use serde_json::{Value, json};
use std::sync::Arc;

pub struct FunctionRegistry {
    core: Arc<RobonixCore>,
}

impl FunctionRegistry {
    pub fn new(core: Arc<RobonixCore>) -> Self {
        Self { core }
    }

    pub fn get_function_schemas(&self) -> Vec<Value> {
        vec![
            json!({
                "type": "function",
                "function": {
                    "name": "query_nearest_objects",
                    "description": "Query the nearest objects from the semantic map relative to the robot. Returns a list of objects sorted by distance.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "count": {
                                "type": "integer",
                                "description": "Number of nearest objects to return (default: 5)",
                                "default": 5
                            },
                            "types": {
                                "type": "array",
                                "items": {"type": "string"},
                                "description": "Filter objects by type (optional). Examples: ['table', 'chair', 'box']"
                            }
                        }
                    }
                }
            }),
            json!({
                "type": "function",
                "function": {
                    "name": "infer_environment",
                    "description": "Infer the current environment/room type based on objects in the semantic map. Returns a description of the environment.",
                    "parameters": {
                        "type": "object",
                        "properties": {}
                    }
                }
            }),
            json!({
                "type": "function",
                "function": {
                    "name": "submit_task",
                    "description": "Submit a task to the system. The task will be planned and executed by the task manager.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "description": {
                                "type": "string",
                                "description": "Natural language description of the task to execute"
                            }
                        },
                        "required": ["description"]
                    }
                }
            }),
            json!({
                "type": "function",
                "function": {
                    "name": "query_skills",
                    "description": "Query all registered skills in the system. Returns a list of available skills with their metadata.",
                    "parameters": {
                        "type": "object",
                        "properties": {}
                    }
                }
            }),
            json!({
                "type": "function",
                "function": {
                    "name": "query_services",
                    "description": "Query all registered services in the system. Returns a list of available services with their metadata.",
                    "parameters": {
                        "type": "object",
                        "properties": {}
                    }
                }
            }),
            json!({
                "type": "function",
                "function": {
                    "name": "query_primitives",
                    "description": "Query all registered primitives in the system. Returns a list of available primitives with their metadata.",
                    "parameters": {
                        "type": "object",
                        "properties": {}
                    }
                }
            }),
            json!({
                "type": "function",
                "function": {
                    "name": "get_system_status",
                    "description": "Get the current system status including active tasks, registered skills, services, and primitives.",
                    "parameters": {
                        "type": "object",
                        "properties": {}
                    }
                }
            }),
        ]
    }

    pub async fn call_function(&self, name: &str, arguments: Value) -> Result<Value> {
        match name {
            "query_nearest_objects" => self.query_nearest_objects(arguments).await,
            "infer_environment" => self.infer_environment().await,
            "submit_task" => self.submit_task(arguments).await,
            "query_skills" => self.query_skills().await,
            "query_services" => self.query_services().await,
            "query_primitives" => self.query_primitives().await,
            "get_system_status" => self.get_system_status().await,
            _ => anyhow::bail!("Unknown function: {}", name),
        }
    }

    async fn query_nearest_objects(&self, arguments: Value) -> Result<Value> {
        let count = arguments.get("count").and_then(|c| c.as_u64()).unwrap_or(5) as usize;

        let types_filter: Vec<String> = arguments
            .get("types")
            .and_then(|t| t.as_array())
            .map(|arr| {
                arr.iter()
                    .filter_map(|v| v.as_str().map(|s| s.to_string()))
                    .collect()
            })
            .unwrap_or_default();

        // Get semantic map from cache
        let task_manager = self.core.get_task_manager();
        let cache = task_manager.get_semantic_map_cache();
        let cache_guard = cache.lock().await;
        let objects = cache_guard
            .as_object()
            .and_then(|obj| obj.get("objects"))
            .and_then(|o| o.as_array())
            .cloned()
            .unwrap_or_default();
        drop(cache_guard);

        // Calculate distances (simplified: assume robot is at origin for now)
        // In a real implementation, we'd get robot pose from TF tree
        let mut objects_with_distance: Vec<(Value, f64)> = objects
            .into_iter()
            .filter_map(|obj| {
                // Filter by type if specified
                if !types_filter.is_empty() {
                    let obj_type = obj.get("type").and_then(|t| t.as_str()).unwrap_or("");
                    if !types_filter.contains(&obj_type.to_string()) {
                        return None;
                    }
                }

                // Calculate distance from origin (robot position)
                // Objects have frame_mapping with center coordinates
                if let Some(frame_mappings) = obj.get("frame_mapping").and_then(|f| f.as_array()) {
                    for mapping in frame_mappings {
                        if let Some(frame_id) = mapping.get("frame_id").and_then(|f| f.as_str()) {
                            if frame_id == "map" || frame_id == "base_link" {
                                if let Some(center) = mapping.get("center") {
                                    let x = center.get("x").and_then(|x| x.as_f64()).unwrap_or(0.0);
                                    let y = center.get("y").and_then(|y| y.as_f64()).unwrap_or(0.0);
                                    let z = center.get("z").and_then(|z| z.as_f64()).unwrap_or(0.0);
                                    let distance = (x * x + y * y + z * z).sqrt();
                                    return Some((obj.clone(), distance));
                                }
                            }
                        }
                    }
                }
                None
            })
            .collect();

        // Sort by distance
        objects_with_distance
            .sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        // Take nearest N objects
        let nearest: Vec<Value> = objects_with_distance
            .into_iter()
            .take(count)
            .map(|(obj, distance)| {
                let mut result = obj.clone();
                result
                    .as_object_mut()
                    .unwrap()
                    .insert("distance".to_string(), json!(distance));
                result
            })
            .collect();

        Ok(json!({
            "objects": nearest,
            "count": nearest.len()
        }))
    }

    async fn infer_environment(&self) -> Result<Value> {
        // Get semantic map
        let task_manager = self.core.get_task_manager();
        let cache = task_manager.get_semantic_map_cache();
        let cache_guard = cache.lock().await;
        let objects = cache_guard
            .as_object()
            .and_then(|obj| obj.get("objects"))
            .and_then(|o| o.as_array())
            .cloned()
            .unwrap_or_default();
        drop(cache_guard);

        // Simple heuristic-based environment inference
        let mut object_types: Vec<String> = objects
            .iter()
            .filter_map(|obj| {
                obj.get("type")
                    .and_then(|t| t.as_str())
                    .map(|s| s.to_string())
            })
            .collect();

        object_types.sort();
        object_types.dedup();

        let environment = if object_types
            .iter()
            .any(|t| t.contains("table") || t.contains("desk"))
        {
            if object_types.iter().any(|t| t.contains("chair")) {
                "office or workspace"
            } else {
                "workspace"
            }
        } else if object_types
            .iter()
            .any(|t| t.contains("box") || t.contains("container"))
        {
            "storage area"
        } else if object_types.is_empty() {
            "empty space"
        } else {
            "unknown environment"
        };

        Ok(json!({
            "environment": environment,
            "object_types": object_types,
            "object_count": objects.len()
        }))
    }

    async fn submit_task(&self, arguments: Value) -> Result<Value> {
        let description = arguments
            .get("description")
            .and_then(|d| d.as_str())
            .context("Missing task description")?;

        let task_manager = self.core.get_task_manager();
        let request = crate::task::api::SubmitTaskRequest {
            description: description.to_string(),
            params: "{}".to_string(),
        };
        let response = task_manager.submit_task(request).await;

        Ok(json!({
            "task_id": response.task_id,
            "status": "submitted",
            "description": description
        }))
    }

    async fn query_skills(&self) -> Result<Value> {
        let skill_library = self.core.get_skill_library();
        let registry = skill_library.get_registry();
        let skills = registry.get_all_skills().await;

        let skills_json: Vec<Value> = skills
            .into_iter()
            .map(|(skill_id, instance)| {
                json!({
                    "skill_id": skill_id,
                    "name": instance.name,
                    "provider": instance.provider,
                    "version": instance.version,
                    "type": instance.r#type,
                })
            })
            .collect();

        Ok(json!({
            "skills": skills_json,
            "count": skills_json.len()
        }))
    }

    async fn query_services(&self) -> Result<Value> {
        let service_registry = self.core.get_service_registry();
        let services = service_registry.get_all_services().await;

        let services_json: Vec<Value> = services
            .into_iter()
            .map(|(key, instance)| {
                json!({
                    "key": key,
                    "provider": instance.provider,
                    "version": instance.version,
                })
            })
            .collect();

        Ok(json!({
            "services": services_json,
            "count": services_json.len()
        }))
    }

    async fn query_primitives(&self) -> Result<Value> {
        let primitive_registry = self.core.get_primitive_registry();
        let primitives = primitive_registry.get_all_primitives().await;

        let primitives_json: Vec<Value> = primitives
            .into_iter()
            .map(|(key, instance)| {
                json!({
                    "key": key,
                    "name": key.split('$').next().unwrap_or(&key),
                    "provider": instance.provider,
                    "version": instance.version,
                })
            })
            .collect();

        Ok(json!({
            "primitives": primitives_json,
            "count": primitives_json.len()
        }))
    }

    async fn get_system_status(&self) -> Result<Value> {
        let task_manager = self.core.get_task_manager();
        let skill_library = self.core.get_skill_library();
        let service_registry = self.core.get_service_registry();
        let primitive_registry = self.core.get_primitive_registry();

        let task_store = task_manager.get_task_store();
        let all_tasks = task_store.get_all_tasks().await;
        let active_tasks = all_tasks
            .iter()
            .filter(|task| {
                !matches!(
                    task.state,
                    crate::task::task::TaskState::Finished
                        | crate::task::task::TaskState::Failed
                        | crate::task::task::TaskState::Cancelled
                )
            })
            .count();

        let skills = skill_library.get_registry().get_all_skills().await;
        let services = service_registry.get_all_services().await;
        let primitives = primitive_registry.get_all_primitives().await;

        Ok(json!({
            "active_tasks": active_tasks,
            "total_tasks": all_tasks.len(),
            "registered_skills": skills.len(),
            "registered_services": services.len(),
            "registered_primitives": primitives.len(),
        }))
    }
}
