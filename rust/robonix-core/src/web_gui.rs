// SPDX-License-Identifier: MulanPSL-2.0
// Web GUI Module
//
// Modern web-based management interface for robonix-core

use rocket::State;
use rocket::response::content::RawHtml;
use rocket::serde::{Deserialize, Serialize, json::Json};
use std::collections::VecDeque;
use std::sync::Arc;
use tokio::sync::Mutex;

use crate::core::RobonixCore;
use crate::perception::image_monitor::ImageMonitor;
use crate::perception::tf_monitor::{TfMonitor, TfTreeResponse};
use crate::perception::topic_monitor::{TopicMonitor, TopicsResponse};

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
pub struct WebGuiState {
    pub core: Arc<RobonixCore>,
    pub node: Arc<Mutex<ros2_client::Node>>,
    pub tf_monitor: Arc<TfMonitor>,
    pub topic_monitor: Arc<TopicMonitor>,
    pub log_buffer: Arc<LogBuffer>,
    pub image_monitor: Arc<ImageMonitor>,
}

#[derive(Serialize, Deserialize)]
pub struct SystemStatus {
    pub core_initialized: bool,
    pub active_tasks: u32,
    pub registered_skills: u32,
    pub registered_services: u32,
    pub registered_primitives: u32,
}

pub fn create_web_gui_state(
    core: Arc<RobonixCore>,
    node: Arc<Mutex<ros2_client::Node>>,
    tf_monitor: Arc<TfMonitor>,
    topic_monitor: Arc<TopicMonitor>,
    log_buffer: Arc<LogBuffer>,
    image_monitor: Arc<ImageMonitor>,
) -> WebGuiState {
    WebGuiState {
        core,
        node,
        tf_monitor,
        topic_monitor,
        log_buffer,
        image_monitor,
    }
}

#[rocket::get("/")]
pub fn index() -> RawHtml<&'static str> {
    RawHtml(include_str!("../web_gui/index.html"))
}

#[rocket::get("/api/status")]
pub fn status_handler(state: &State<WebGuiState>) -> Json<SystemStatus> {
    // Get status from core components
    let _task_manager = state.core.get_task_manager();
    let _skill_library = state.core.get_skill_library();
    let _service_registry = state.core.get_service_registry();
    let _primitive_registry = state.core.get_primitive_registry();

    let status = SystemStatus {
        core_initialized: true,
        active_tasks: 0,
        registered_skills: 0,
        registered_services: 0,
        registered_primitives: 0,
    };

    Json(status)
}

#[rocket::get("/api/tf-tree")]
pub async fn tf_tree_handler(state: &State<WebGuiState>) -> Json<TfTreeResponse> {
    // Get tree from monitor (automatically updated by subscriptions)
    let tree = state.tf_monitor.get_tree().await;

    Json(tree)
}

#[rocket::get("/api/topics")]
pub async fn topics_handler(state: &State<WebGuiState>) -> Json<TopicsResponse> {
    // Get topics from monitor
    let topics = state.topic_monitor.get_topics().await;

    Json(topics)
}

#[rocket::get("/api/tasks")]
pub async fn tasks_handler(state: &State<WebGuiState>) -> Json<serde_json::Value> {
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
pub async fn skills_handler(state: &State<WebGuiState>) -> Json<serde_json::Value> {
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
pub async fn services_handler(state: &State<WebGuiState>) -> Json<serde_json::Value> {
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
pub async fn primitives_handler(state: &State<WebGuiState>) -> Json<serde_json::Value> {
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
pub async fn logs_handler(state: &State<WebGuiState>, limit: Option<usize>) -> Json<Vec<LogEntry>> {
    let limit = limit.unwrap_or(100);
    let logs = state.log_buffer.get_logs(limit).await;
    Json(logs)
}

#[rocket::get("/api/semantic-map")]
pub async fn semantic_map_handler(state: &State<WebGuiState>) -> Json<serde_json::Value> {
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
pub async fn image_topics_handler(state: &State<WebGuiState>) -> Json<serde_json::Value> {
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
    state: &State<WebGuiState>,
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
