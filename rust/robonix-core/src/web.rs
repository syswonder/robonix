// SPDX-License-Identifier: MulanPSL-2.0
// Web Module
//
// Modern web-based management interface for robonix-core

use rocket::State;
use rocket::response::content::RawHtml;
use rocket::serde::{Deserialize, Serialize, json::Json};
use std::collections::VecDeque;
use std::sync::Arc;
use tokio::sync::Mutex;

use crate::agent::{Agent, AgentConfig as LLMAgentConfig, AgentRequest, AgentResponse};
use crate::config::CoreConfig;
use crate::core::RobonixCore;
use crate::perception::image_monitor::ImageMonitor;
use crate::perception::tf_monitor::{TfMonitor, TfTreeResponse};
use crate::perception::topic_monitor::{TopicMonitor, TopicsResponse};
use crate::task::task::TaskState;

use log::{trace, warn};
#[derive(Clone, Serialize, Deserialize)]
pub struct LogEntry {
    pub timestamp: String,
    pub level: String,
    pub message: String,
}

pub struct LogBuffer {
    logs: Arc<Mutex<VecDeque<LogEntry>>>,
    max_size: usize,
}

impl LogBuffer {
    pub fn new(max_size: usize) -> Self {
        Self {
            logs: Arc::new(Mutex::new(VecDeque::with_capacity(max_size))),
            max_size,
        }
    }

    pub fn add_log(&self, entry: LogEntry) {
        if let Ok(mut logs) = self.logs.try_lock() {
            logs.push_back(entry);
            if logs.len() > self.max_size {
                logs.pop_front();
            }
        }
    }

    pub async fn get_logs(&self, limit: usize) -> Vec<LogEntry> {
        let logs = self.logs.lock().await;
        logs.iter().rev().take(limit).rev().cloned().collect()
    }
}

#[derive(Clone)]
pub struct WebState {
    pub core: Arc<RobonixCore>,
    pub node: Arc<Mutex<ros2_client::Node>>,
    pub tf_monitor: Arc<TfMonitor>,
    pub topic_monitor: Arc<TopicMonitor>,
    pub log_buffer: Arc<LogBuffer>,
    pub image_monitor: Arc<ImageMonitor>,
    pub agent: Arc<Mutex<Agent>>,
    pub web_dir: std::path::PathBuf,
}

#[derive(Serialize, Deserialize)]
pub struct SystemStatus {
    pub core_initialized: bool,
    pub active_tasks: u32,
    pub registered_skills: u32,
    pub registered_services: u32,
    pub registered_primitives: u32,
}

pub fn create_web_state(
    core: Arc<RobonixCore>,
    node: Arc<Mutex<ros2_client::Node>>,
    tf_monitor: Arc<TfMonitor>,
    topic_monitor: Arc<TopicMonitor>,
    log_buffer: Arc<LogBuffer>,
    image_monitor: Arc<ImageMonitor>,
    agent: Arc<Mutex<Agent>>,
    web_dir: std::path::PathBuf,
) -> WebState {
    WebState {
        core,
        node,
        tf_monitor,
        topic_monitor,
        log_buffer,
        image_monitor,
        agent,
        web_dir,
    }
}

#[rocket::get("/")]
pub async fn index(state: &State<WebState>) -> Result<RawHtml<String>, rocket::http::Status> {
    let index_path = state.web_dir.join("index.html");
    match tokio::fs::read_to_string(&index_path).await {
        Ok(content) => Ok(RawHtml(content)),
        Err(e) => {
            warn!("Failed to read index.html from {:?}: {}", index_path, e);
            Err(rocket::http::Status::InternalServerError)
        }
    }
}

#[rocket::get("/api/status")]
pub async fn status_handler(state: &State<WebState>) -> Json<SystemStatus> {
    // Get status from core components
    let task_manager = state.core.get_task_manager();
    let skill_library = state.core.get_skill_library();
    let service_registry = state.core.get_service_registry();
    let primitive_registry = state.core.get_primitive_registry();

    // Get all tasks and count active ones (not finished, failed, or cancelled)
    let task_store = task_manager.get_task_store();
    let all_tasks = task_store.get_all_tasks().await;
    let active_tasks = all_tasks
        .iter()
        .filter(|task| {
            !matches!(
                task.state,
                TaskState::Finished | TaskState::Failed | TaskState::Cancelled
            )
        })
        .count() as u32;

    // Get counts from registries
    let skills = skill_library.get_registry().get_all_skills().await;
    let registered_skills = skills.len() as u32;

    let services = service_registry.get_all_services().await;
    let registered_services = services.len() as u32;

    let primitives = primitive_registry.get_all_primitives().await;
    let registered_primitives = primitives.len() as u32;

    let status = SystemStatus {
        core_initialized: true,
        active_tasks,
        registered_skills,
        registered_services,
        registered_primitives,
    };

    Json(status)
}

#[rocket::get("/api/tf-tree")]
pub async fn tf_tree_handler(state: &State<WebState>) -> Json<TfTreeResponse> {
    // Get tree from monitor (automatically updated by subscriptions)
    let tree = state.tf_monitor.get_tree().await;

    Json(tree)
}

#[rocket::get("/api/topics")]
pub async fn topics_handler(state: &State<WebState>) -> Json<TopicsResponse> {
    // Get topics from monitor
    let topics = state.topic_monitor.get_topics().await;

    Json(topics)
}

#[rocket::get("/api/tasks")]
pub async fn tasks_handler(state: &State<WebState>) -> Json<serde_json::Value> {
    let task_manager = state.core.get_task_manager();
    let task_store = task_manager.get_task_store();
    let tasks = task_store.get_all_tasks().await;

    let tasks_json: Vec<serde_json::Value> = tasks
        .into_iter()
        .map(|task| {
            serde_json::json!({
                "task_id": task.task_id,
                "description": task.description,
                "state": format!("{:?}", task.state),
                "priority": task.context.priority,
                "rtdl": task.context.rtdl,
                "rtdl_type": task.context.rtdl_type,
                "rtdl_instruction_pointer": task.context.rtdl_instruction_pointer,
                "created_at": task.created_at,
                "updated_at": task.updated_at,
                "result": task.result,
                "error_message": task.error_message,
            })
        })
        .collect();

    Json(serde_json::json!({
        "tasks": tasks_json
    }))
}

#[rocket::get("/api/skills")]
pub async fn skills_handler(state: &State<WebState>) -> Json<serde_json::Value> {
    let skill_library = state.core.get_skill_library();
    let registry = skill_library.get_registry();
    let skills = registry.get_all_skills().await;

    let skills_json: Vec<serde_json::Value> = skills
        .into_iter()
        .map(|(skill_id, instance)| {
            serde_json::json!({
                "skill_id": skill_id,
                "name": instance.name,
                "provider": instance.provider,
                "version": instance.version,
                "type": instance.r#type,
                "start_topic": instance.start_topic,
                "status_topic": instance.status_topic,
                "entry": instance.entry,
                "skill_dir": instance.skill_dir,
                "main_rtdl": instance.main_rtdl,
                "metadata": instance.metadata,
            })
        })
        .collect();

    Json(serde_json::json!({
        "skills": skills_json
    }))
}

#[rocket::get("/api/services")]
pub async fn services_handler(state: &State<WebState>) -> Json<serde_json::Value> {
    let service_registry = state.core.get_service_registry();
    let services = service_registry.get_all_services().await;

    let services_json: Vec<serde_json::Value> = services
        .into_iter()
        .map(|(key, instance)| {
            // Extract name from key (format: "name$provider$version")
            // Service names typically don't contain "::", but we handle it anyway
            let key_parts: Vec<&str> = key.split('$').collect();
            let name = if key_parts.len() >= 3 {
                // Key has at least 3 parts: name (may contain ::), provider, version
                key_parts[..key_parts.len() - 2].join("$")
            } else {
                key_parts
                    .first()
                    .map(|s| s.to_string())
                    .unwrap_or_else(|| key.clone())
            };
            serde_json::json!({
                "key": key,
                "name": name,
                "provider": instance.provider,
                "version": instance.version,
                "entry": instance.entry,
                "metadata": instance.metadata,
            })
        })
        .collect();

    Json(serde_json::json!({
        "services": services_json
    }))
}

#[rocket::get("/api/primitives")]
pub async fn primitives_handler(state: &State<WebState>) -> Json<serde_json::Value> {
    let primitive_registry = state.core.get_primitive_registry();
    let primitives = primitive_registry.get_all_primitives().await;

    let primitives_json: Vec<serde_json::Value> = primitives
        .into_iter()
        .map(|(key, instance)| {
            // Extract name from key (format: "name$provider$version")
            // Primitive names can contain "::" (e.g., "prm::camera.capture")
            // Key uses "$" as separator, so we split by "$" and take everything except last two parts
            let key_parts: Vec<&str> = key.split('$').collect();
            let name = if key_parts.len() >= 3 {
                // Key has at least 3 parts: name (may contain ::), provider, version
                // Join all parts except the last two (provider and version)
                key_parts[..key_parts.len() - 2].join("$")
            } else {
                // Fallback: just take first part
                key_parts
                    .first()
                    .map(|s| s.to_string())
                    .unwrap_or_else(|| key.clone())
            };
            serde_json::json!({
                "key": key,
                "name": name,
                "provider": instance.provider,
                "version": instance.version,
                "input_schema": instance.input_schema,
                "output_schema": instance.output_schema,
                "metadata": instance.metadata,
            })
        })
        .collect();

    Json(serde_json::json!({
        "primitives": primitives_json
    }))
}

#[rocket::get("/api/logs?<limit>")]
pub async fn logs_handler(state: &State<WebState>, limit: Option<usize>) -> Json<Vec<LogEntry>> {
    let limit = limit.unwrap_or(100);
    let logs = state.log_buffer.get_logs(limit).await;
    Json(logs)
}

#[rocket::get("/api/semantic-map")]
pub async fn semantic_map_handler(state: &State<WebState>) -> Json<serde_json::Value> {
    let task_manager = state.core.get_task_manager();
    let cache = task_manager.get_semantic_map_cache();
    let cache_guard = cache.lock().await;
    let object_graph = cache_guard.clone();
    drop(cache_guard);

    Json(serde_json::json!({
        "objects": object_graph
    }))
}

#[rocket::get("/api/image-topics")]
pub async fn image_topics_handler(state: &State<WebState>) -> Json<serde_json::Value> {
    let image_topics = state.image_monitor.get_image_topics().await;
    let topics_json: Vec<serde_json::Value> = image_topics
        .into_iter()
        .map(|info| {
            let last_update_secs = info.last_update.and_then(|t| {
                t.duration_since(std::time::UNIX_EPOCH)
                    .ok()
                    .map(|d| d.as_secs())
            });
            let image_paths: Vec<serde_json::Value> = info
                .image_paths
                .into_iter()
                .map(|img| {
                    serde_json::json!({
                        "path": img.path,
                        "timestamp": img.timestamp,
                    })
                })
                .collect();
            serde_json::json!({
                "topic_name": info.topic_name,
                "message_type": info.message_type,
                "last_update": last_update_secs,
                "image_paths": image_paths,
            })
        })
        .collect();

    Json(serde_json::json!({
        "image_topics": topics_json
    }))
}

#[rocket::get("/api/images/<filename>")]
pub async fn image_handler(
    state: &State<WebState>,
    filename: String,
) -> Result<rocket::fs::NamedFile, rocket::http::Status> {
    let storage_dir = state.image_monitor.get_storage_dir();
    let image_path = storage_dir.join(&filename);

    trace!(
        "Image handler: looking for file '{}' in storage dir {:?}, full path: {:?}",
        filename, storage_dir, image_path
    );

    // Check if file exists
    if !image_path.exists() {
        warn!("Image file does not exist: {:?}", image_path);
        // List some files in the directory for debugging
        if let Ok(entries) = std::fs::read_dir(storage_dir) {
            let sample_files: Vec<String> = entries
                .filter_map(|e| e.ok())
                .take(5)
                .filter_map(|e| e.file_name().into_string().ok())
                .collect();
            trace!("Sample files in storage dir: {:?}", sample_files);
        }
        return Err(rocket::http::Status::NotFound);
    }

    match rocket::fs::NamedFile::open(&image_path).await {
        Ok(file) => {
            trace!("Image file found and opened: {:?}", image_path);
            Ok(file)
        }
        Err(e) => {
            warn!("Image file error opening {:?}: {:?}", image_path, e);
            Err(rocket::http::Status::NotFound)
        }
    }
}

#[rocket::get("/settings")]
pub async fn settings_page(
    state: &State<WebState>,
) -> Result<RawHtml<String>, rocket::http::Status> {
    let settings_path = state.web_dir.join("settings.html");
    match tokio::fs::read_to_string(&settings_path).await {
        Ok(content) => Ok(RawHtml(content)),
        Err(e) => {
            warn!(
                "Failed to read settings.html from {:?}: {}",
                settings_path, e
            );
            Err(rocket::http::Status::InternalServerError)
        }
    }
}

#[rocket::get("/api/config")]
pub async fn get_config_handler() -> Json<serde_json::Value> {
    match CoreConfig::load() {
        Ok(config) => Json(serde_json::json!({
            "agent": {
                "llm_provider": config.agent.llm_provider,
                "api_key": config.agent.api_key.as_ref().map(|_| "***").unwrap_or_default(),
                "api_base": config.agent.api_base,
                "model": config.agent.model,
                "temperature": config.agent.temperature,
            }
        })),
        Err(e) => {
            warn!("Failed to load config: {}", e);
            Json(serde_json::json!({
                "error": format!("Failed to load config: {}", e)
            }))
        }
    }
}

#[derive(Deserialize)]
pub struct UpdateConfigRequest {
    pub agent: Option<AgentConfigUpdate>,
}

#[derive(Deserialize)]
pub struct AgentConfigUpdate {
    pub llm_provider: Option<String>,
    pub api_key: Option<String>,
    pub api_base: Option<String>,
    pub model: Option<String>,
    pub temperature: Option<f64>,
}

#[rocket::post("/api/config", data = "<request>")]
pub async fn update_config_handler(
    state: &State<WebState>,
    request: Json<UpdateConfigRequest>,
) -> Json<serde_json::Value> {
    match CoreConfig::load() {
        Ok(mut config) => {
            if let Some(agent_update) = &request.agent {
                if let Some(llm_provider) = &agent_update.llm_provider {
                    config.agent.llm_provider = llm_provider.clone();
                }
                if let Some(api_key) = &agent_update.api_key {
                    // Only update if not masked
                    if api_key != "***" {
                        config.agent.api_key = Some(api_key.clone());
                    }
                }
                if let Some(api_base) = &agent_update.api_base {
                    config.agent.api_base = api_base.clone();
                }
                if let Some(model) = &agent_update.model {
                    config.agent.model = model.clone();
                }
                if let Some(temperature) = agent_update.temperature {
                    config.agent.temperature = temperature;
                }

                // Save config
                if let Err(e) = config.save() {
                    return Json(serde_json::json!({
                        "error": format!("Failed to save config: {}", e)
                    }));
                }

                // Recreate agent with new config
                let new_agent_config = LLMAgentConfig {
                    llm_provider: config.agent.llm_provider.clone(),
                    api_key: config.agent.api_key.clone(),
                    api_base: config.agent.api_base.clone(),
                    model: config.agent.model.clone(),
                    temperature: config.agent.temperature,
                };
                let new_agent = Agent::new(state.core.clone(), new_agent_config);
                *state.agent.lock().await = new_agent;

                Json(serde_json::json!({
                    "status": "success",
                    "message": "Config updated successfully"
                }))
            } else {
                Json(serde_json::json!({
                    "error": "No agent config provided"
                }))
            }
        }
        Err(e) => Json(serde_json::json!({
            "error": format!("Failed to load config: {}", e)
        })),
    }
}

#[rocket::post("/api/agent/chat", data = "<request>")]
pub async fn agent_chat_handler(
    state: &State<WebState>,
    request: Json<AgentRequest>,
) -> Result<Json<AgentResponse>, rocket::http::Status> {
    let mut agent = state.agent.lock().await;
    match agent.chat(request.into_inner()).await {
        Ok(response) => Ok(Json(response)),
        Err(e) => {
            warn!("Agent chat error: {}", e);
            Err(rocket::http::Status::InternalServerError)
        }
    }
}
