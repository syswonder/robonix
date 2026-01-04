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
use crate::tf_monitor::{TfMonitor, TfTreeResponse};

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
    pub log_buffer: Arc<LogBuffer>,
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
    log_buffer: Arc<LogBuffer>,
) -> WebGuiState {
    WebGuiState {
        core,
        node,
        tf_monitor,
        log_buffer,
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

#[rocket::get("/api/tasks")]
pub fn tasks_handler() -> Json<serde_json::Value> {
    Json(serde_json::json!({
        "tasks": []
    }))
}

#[rocket::get("/api/skills")]
pub fn skills_handler() -> Json<serde_json::Value> {
    Json(serde_json::json!({
        "skills": []
    }))
}

#[rocket::get("/api/services")]
pub fn services_handler() -> Json<serde_json::Value> {
    Json(serde_json::json!({
        "services": []
    }))
}

#[rocket::get("/api/primitives")]
pub fn primitives_handler() -> Json<serde_json::Value> {
    Json(serde_json::json!({
        "primitives": []
    }))
}

#[rocket::get("/api/logs?<limit>")]
pub async fn logs_handler(state: &State<WebGuiState>, limit: Option<usize>) -> Json<Vec<LogEntry>> {
    let limit = limit.unwrap_or(100);
    let logs = state.log_buffer.get_logs(limit).await;
    Json(logs)
}
