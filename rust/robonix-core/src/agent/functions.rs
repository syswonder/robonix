// SPDX-License-Identifier: MulanPSL-2.0
// Function Registry Module
//
// Registry for system functions that can be called by the agent

use crate::agent::llm::{AgentConfig, LLMClient};
use crate::core::RobonixCore;
use crate::perception::image_monitor::ImageMonitor;
use anyhow::{Context, Result};
use base64::Engine;
use base64::engine::general_purpose::STANDARD as BASE64_STANDARD;
use serde_json::{Value, json};
use std::sync::Arc;

pub struct FunctionRegistry {
    core: Arc<RobonixCore>,
    image_monitor: Arc<ImageMonitor>,
    agent_config: AgentConfig,
}

impl FunctionRegistry {
    pub fn new(
        core: Arc<RobonixCore>,
        image_monitor: Arc<ImageMonitor>,
        agent_config: AgentConfig,
    ) -> Self {
        Self {
            core,
            image_monitor,
            agent_config,
        }
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
            json!({
                "type": "function",
                "function": {
                    "name": "list_tasks",
                    "description": "Get a list of all submitted tasks. Returns task IDs, descriptions, states, and basic information.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "state_filter": {
                                "type": "string",
                                "description": "Optional filter by task state: 'active' (pending/planning/running/suspended), 'finished', 'failed', 'cancelled', or 'all' (default: 'all')",
                                "enum": ["all", "active", "finished", "failed", "cancelled"]
                            },
                            "limit": {
                                "type": "integer",
                                "description": "Maximum number of tasks to return (default: 50, max: 100)"
                            }
                        }
                    }
                }
            }),
            json!({
                "type": "function",
                "function": {
                    "name": "get_task_details",
                    "description": "Get detailed information about a specific task, including RTDL understanding, execution status, results, and error messages.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "task_id": {
                                "type": "string",
                                "description": "The task ID to get details for"
                            }
                        },
                        "required": ["task_id"]
                    }
                }
            }),
            json!({
                "type": "function",
                "function": {
                    "name": "describe_robot_vision",
                    "description": "Describe what the robot currently sees from its RGB cameras. Use when the user asks about the robot's view, e.g. 'tell me what the robot sees', 'describe the current scene', '告诉我机器人目前看到的画面', 'what does the robot see?'. Uses the latest RGB images from the image monitor and a vision-language model to answer.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "question": {
                                "type": "string",
                                "description": "The user's question about the robot's view. Default: 'Describe what you see in these images from the robot\'s cameras.'"
                            }
                        }
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
            "list_tasks" => self.list_tasks(arguments).await,
            "get_task_details" => self.get_task_details(arguments).await,
            "describe_robot_vision" => self.describe_robot_vision(arguments).await,
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
        // Cache is directly an array of objects, not an object with "objects" field
        let task_manager = self.core.get_task_manager();
        let cache = task_manager.get_semantic_map_cache();
        let cache_guard = cache.lock().await;
        let objects = cache_guard.as_array().cloned().unwrap_or_default();
        drop(cache_guard);

        // Calculate distances (simplified: assume robot is at origin for now)
        // In a real implementation, we'd get robot pose from TF tree
        let mut objects_with_distance: Vec<(Value, Option<f64>)> = objects
            .into_iter()
            .filter_map(|obj| {
                // Filter by type if specified
                // Object has "label" field (not "type"), but we check both for compatibility
                if !types_filter.is_empty() {
                    let obj_type = obj
                        .get("type")
                        .and_then(|t| t.as_str())
                        .or_else(|| obj.get("label").and_then(|l| l.as_str()))
                        .unwrap_or("");
                    // Case-insensitive comparison
                    let obj_type_lower = obj_type.to_lowercase();
                    if !types_filter
                        .iter()
                        .any(|filter| filter.to_lowercase() == obj_type_lower)
                    {
                        return None;
                    }
                }

                // Calculate distance from origin (robot position)
                // Objects have frame_mapping with center coordinates
                // Prefer "map" frame over "base_link" for more accurate distance calculation
                let mut distance_opt: Option<f64> = None;
                if let Some(frame_mappings) = obj.get("frame_mapping").and_then(|f| f.as_array()) {
                    // First pass: look for "map" frame (preferred)
                    for mapping in frame_mappings.iter() {
                        if let Some(frame_id) = mapping.get("frame_id").and_then(|f| f.as_str()) {
                            if frame_id == "map" {
                                if let Some(center) = mapping.get("center") {
                                    let x = center.get("x").and_then(|x| x.as_f64()).unwrap_or(0.0);
                                    let y = center.get("y").and_then(|y| y.as_f64()).unwrap_or(0.0);
                                    let z = center.get("z").and_then(|z| z.as_f64()).unwrap_or(0.0);
                                    let distance = (x * x + y * y + z * z).sqrt();
                                    distance_opt = Some(distance);
                                    break; // Found map frame, use it
                                }
                            }
                        }
                    }
                    // Second pass: if no map frame found, try base_link
                    if distance_opt.is_none() {
                        for mapping in frame_mappings.iter() {
                            if let Some(frame_id) = mapping.get("frame_id").and_then(|f| f.as_str())
                            {
                                if frame_id == "base_link" {
                                    if let Some(center) = mapping.get("center") {
                                        let x =
                                            center.get("x").and_then(|x| x.as_f64()).unwrap_or(0.0);
                                        let y =
                                            center.get("y").and_then(|y| y.as_f64()).unwrap_or(0.0);
                                        let z =
                                            center.get("z").and_then(|z| z.as_f64()).unwrap_or(0.0);
                                        let distance = (x * x + y * y + z * z).sqrt();
                                        distance_opt = Some(distance);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
                // Return object even if no distance available (distance will be None)
                Some((obj.clone(), distance_opt))
            })
            .collect();

        // Sort by distance: objects with distance first, then by distance value
        // Objects without distance go to the end
        objects_with_distance.sort_by(|a, b| match (a.1, b.1) {
            (Some(da), Some(db)) => da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal),
            (Some(_), None) => std::cmp::Ordering::Less,
            (None, Some(_)) => std::cmp::Ordering::Greater,
            (None, None) => std::cmp::Ordering::Equal,
        });

        // Take nearest N objects
        let nearest: Vec<Value> = objects_with_distance
            .into_iter()
            .take(count)
            .map(|(obj, distance_opt)| {
                let mut result = obj.clone();
                if let Some(distance) = distance_opt {
                    result
                        .as_object_mut()
                        .unwrap()
                        .insert("distance".to_string(), json!(distance));
                } else {
                    // Mark objects without distance information
                    result
                        .as_object_mut()
                        .unwrap()
                        .insert("distance".to_string(), json!(null));
                }
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
        // Cache is directly an array of objects, not an object with "objects" field
        let task_manager = self.core.get_task_manager();
        let cache = task_manager.get_semantic_map_cache();
        let cache_guard = cache.lock().await;
        let objects = cache_guard.as_array().cloned().unwrap_or_default();
        drop(cache_guard);

        // Simple heuristic-based environment inference
        // Object has "label" field (not "type"), but we check both for compatibility
        let mut object_types: Vec<String> = objects
            .iter()
            .filter_map(|obj| {
                obj.get("type")
                    .and_then(|t| t.as_str())
                    .or_else(|| obj.get("label").and_then(|l| l.as_str()))
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

    async fn list_tasks(&self, arguments: Value) -> Result<Value> {
        let task_manager = self.core.get_task_manager();
        let task_store = task_manager.get_task_store();
        let all_tasks = task_store.get_all_tasks().await;

        // Parse filter
        let state_filter = arguments
            .get("state_filter")
            .and_then(|s| s.as_str())
            .unwrap_or("all");

        let limit = arguments
            .get("limit")
            .and_then(|l| l.as_u64())
            .map(|l| l.min(100) as usize)
            .unwrap_or(50);

        // Filter tasks by state
        let filtered_tasks: Vec<_> = all_tasks
            .into_iter()
            .filter(|task| {
                match state_filter {
                    "active" => {
                        matches!(
                            task.state,
                            crate::task::task::TaskState::Pending
                                | crate::task::task::TaskState::Planning
                                | crate::task::task::TaskState::Running
                                | crate::task::task::TaskState::Suspended
                        )
                    }
                    "finished" => matches!(task.state, crate::task::task::TaskState::Finished),
                    "failed" => matches!(task.state, crate::task::task::TaskState::Failed),
                    "cancelled" => matches!(task.state, crate::task::task::TaskState::Cancelled),
                    _ => true, // "all"
                }
            })
            .take(limit)
            .collect();

        // Sort by created_at (newest first)
        let mut sorted_tasks = filtered_tasks;
        sorted_tasks.sort_by(|a, b| b.created_at.cmp(&a.created_at));

        let tasks_json: Vec<Value> = sorted_tasks
            .into_iter()
            .map(|task| {
                json!({
                    "task_id": task.task_id,
                    "description": task.description,
                    "state": format!("{:?}", task.state),
                    "priority": task.context.priority,
                    "created_at": task.created_at,
                    "updated_at": task.updated_at,
                    "has_rtdl": task.context.rtdl.is_some(),
                    "has_result": task.result.is_some(),
                    "has_error": task.error_message.is_some(),
                })
            })
            .collect();

        Ok(json!({
            "tasks": tasks_json,
            "count": tasks_json.len(),
            "filter": state_filter
        }))
    }

    async fn get_task_details(&self, arguments: Value) -> Result<Value> {
        let task_id = arguments
            .get("task_id")
            .and_then(|id| id.as_str())
            .context("Missing task_id parameter")?;

        let task_manager = self.core.get_task_manager();
        let task_store = task_manager.get_task_store();

        if let Some(task) = task_store.get_task(task_id).await {
            // Build comprehensive task details
            let mut details = json!({
                "task_id": task.task_id,
                "description": task.description,
                "params": task.params,
                "state": format!("{:?}", task.state),
                "priority": task.context.priority,
                "created_at": task.created_at,
                "updated_at": task.updated_at,
                "retry_count": task.context.retry_count,
                "rtdl_instruction_pointer": task.context.rtdl_instruction_pointer,
            });

            // RTDL information
            if let Some(ref rtdl) = task.context.rtdl {
                details["rtdl"] = json!(rtdl);
            }
            if let Some(ref rtdl_type) = task.context.rtdl_type {
                details["rtdl_type"] = json!(rtdl_type);
            }

            // Execution status
            details["execution_status"] = json!({
                "state": format!("{:?}", task.state),
                "current_instruction_index": task.context.rtdl_instruction_pointer,
                "retry_count": task.context.retry_count,
            });

            if let Some(ref exception) = task.context.last_exception {
                details["last_exception"] = json!(exception);
            }

            // Task results
            if let Some(ref result) = task.result {
                details["result"] = result.clone();
            }

            if let Some(ref error_msg) = task.error_message {
                details["error_message"] = json!(error_msg);
            }

            // Object graph information
            details["object_graph"] = task.context.object_graph.clone();
            details["object_graph_updated_at"] = json!(task.context.object_graph_updated_at);
            if let Some(arr) = task.context.object_graph.as_array() {
                details["object_graph_count"] = json!(arr.len());
            } else {
                details["object_graph_count"] = json!(0);
            }

            Ok(details)
        } else {
            anyhow::bail!("Task not found: {}", task_id)
        }
    }

    async fn describe_robot_vision(&self, arguments: Value) -> Result<Value> {
        let question = arguments
            .get("question")
            .and_then(|q| q.as_str())
            .unwrap_or("Describe what you see in these images from the robot's cameras.")
            .to_string();

        let paths = self.image_monitor.get_rgb_image_paths().await;
        if paths.is_empty() {
            return Ok(json!({
                "answer": "No RGB camera images are currently available. The image monitor has not received any RGB image topics yet, or no topics with RGB encoding (rgb8/bgr8/rgba8/bgra8) are subscribed.",
                "image_count": 0
            }));
        }

        let mut image_base64_urls = Vec::with_capacity(paths.len());
        for path in &paths {
            let data = tokio::fs::read(path)
                .await
                .context("Failed to read image file")?;
            let b64 = BASE64_STANDARD.encode(&data);
            image_base64_urls.push(format!("data:image/jpeg;base64,{}", b64));
        }

        let client = LLMClient::new(self.agent_config.clone());
        let answer = client
            .chat_vision(image_base64_urls, &question)
            .await
            .context("VLM call failed")?;

        Ok(json!({
            "answer": answer,
            "image_count": paths.len()
        }))
    }
}
