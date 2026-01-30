// SPDX-License-Identifier: MulanPSL-2.0
// Web Module
//
// Modern web-based management interface for robonix-core

use rocket::State;
use rocket::data::ByteUnit;
use rocket::response::content::RawHtml;
use rocket::serde::{Deserialize, Serialize, json::Json};
use serde::{Deserialize as SerdeDeserialize, Serialize as SerdeSerialize};
use std::collections::VecDeque;
use std::sync::Arc;
use tokio::sync::Mutex;

use crate::agent::{Agent, AgentConfig as LLMAgentConfig, AgentRequest, AgentResponse};
use crate::config::CoreConfig;
use crate::core::RobonixCore;
use crate::perception::image_monitor::ImageMonitor;
use crate::perception::tf_monitor::{TfMonitor, TfTreeResponse};
use crate::perception::topic_monitor::{TopicMonitor, TopicsResponse};
use crate::speech::{SttService, TtsService};
use crate::task::task::TaskState;

use log::{info, trace, warn};
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
    pub tts_service: Arc<TtsService>,
    pub stt_service: Arc<SttService>,
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
    tts_service: Arc<TtsService>,
    stt_service: Arc<SttService>,
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
        tts_service,
        stt_service,
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
                "params": task.params,
            })
        })
        .collect();

    Json(serde_json::json!({
        "tasks": tasks_json
    }))
}

#[rocket::post("/api/tasks/<task_id>/cancel")]
pub async fn task_cancel_handler(
    state: &State<WebState>,
    task_id: &str,
) -> Json<serde_json::Value> {
    use crate::task::api::CancelTaskRequest;

    let task_manager = state.core.get_task_manager();
    let req = CancelTaskRequest {
        task_id: task_id.to_string(),
    };
    let resp = task_manager.cancel_task(req).await;
    Json(serde_json::json!({ "success": resp.success }))
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
            // Primitive names can contain "::" (e.g., "prm::camera.rgb")
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
                "api_key": config.agent.api_key.unwrap_or_default(),
                "api_base": config.agent.api_base,
                "model": config.agent.model,
                "temperature": config.agent.temperature,
            },
            "speech": {
                "access_token": config.speech.access_token,
                "appkey": config.speech.appkey,
                "region": config.speech.region,
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
    pub speech: Option<SpeechConfigUpdate>,
}

#[derive(Deserialize)]
pub struct SpeechConfigUpdate {
    pub access_token: Option<String>,
    pub appkey: Option<String>,
    pub region: Option<String>,
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
            let mut config_changed = false;

            // Update agent config if provided
            if let Some(agent_update) = &request.agent {
                if let Some(llm_provider) = &agent_update.llm_provider {
                    config.agent.llm_provider = llm_provider.clone();
                    config_changed = true;
                }
                if let Some(api_key) = &agent_update.api_key {
                    // Only update if not masked
                    if api_key != "***" {
                        config.agent.api_key = Some(api_key.clone());
                        config_changed = true;
                    }
                }
                if let Some(api_base) = &agent_update.api_base {
                    config.agent.api_base = api_base.clone();
                    config_changed = true;
                }
                if let Some(model) = &agent_update.model {
                    config.agent.model = model.clone();
                    config_changed = true;
                }
                if let Some(temperature) = agent_update.temperature {
                    config.agent.temperature = temperature;
                    config_changed = true;
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
            }

            // Update speech config if provided
            if let Some(speech_update) = &request.speech {
                if let Some(access_token) = &speech_update.access_token {
                    if access_token != "***" && !access_token.is_empty() {
                        config.speech.access_token = access_token.clone();
                        config_changed = true;
                    }
                }
                if let Some(appkey) = &speech_update.appkey {
                    if appkey != "***" && !appkey.is_empty() {
                        config.speech.appkey = appkey.clone();
                        config_changed = true;
                    }
                }
                if let Some(region) = &speech_update.region {
                    config.speech.region = region.clone();
                    config_changed = true;
                }

                // Recreate TTS and STT services with new config
                use crate::speech::{SttService, TtsService};
                let new_tts_service = Arc::new(TtsService::new(config.speech.clone()));
                let new_stt_service = Arc::new(SttService::new(config.speech.clone()));
                // Note: We can't directly replace Arc in WebState, but we can update the inner values
                // For now, the services will be recreated on next server restart
                // The config is saved below, so it will be loaded on restart
            }

            // Save config if anything changed
            if config_changed {
                if let Err(e) = config.save() {
                    return Json(serde_json::json!({
                        "error": format!("Failed to save config: {}", e)
                    }));
                }
            }

            if request.agent.is_some() || request.speech.is_some() {
                Json(serde_json::json!({
                    "status": "success",
                    "message": "Config updated successfully",
                    "restart_required": true,
                    "restart_message": "Please restart robonix-core for the changes to take effect."
                }))
            } else {
                Json(serde_json::json!({
                    "error": "No config provided"
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

#[rocket::post("/api/agent/reset")]
pub async fn agent_reset_handler(
    state: &State<WebState>,
) -> Result<Json<serde_json::Value>, rocket::http::Status> {
    let mut agent = state.agent.lock().await;
    agent.clear_history();
    Ok(Json(serde_json::json!({
        "status": "success",
        "message": "Agent history cleared"
    })))
}

#[derive(SerdeSerialize, SerdeDeserialize)]
pub struct TtsRequest {
    pub text: String,
    #[serde(default = "default_format")]
    pub format: String,
    #[serde(default = "default_voice")]
    pub voice: String,
}

fn default_format() -> String {
    "wav".to_string()
}

fn default_voice() -> String {
    "zhishuo".to_string()
}

#[rocket::post("/api/tts", data = "<request>")]
pub async fn tts_handler(
    state: &State<WebState>,
    request: Json<TtsRequest>,
) -> Result<(rocket::http::ContentType, Vec<u8>), rocket::http::Status> {
    let text = request.text.clone();
    if text.is_empty() {
        return Err(rocket::http::Status::BadRequest);
    }

    match state
        .tts_service
        .synthesize(&text, Some(&request.format), Some(&request.voice))
        .await
    {
        Ok(audio_data) => {
            // Save TTS audio to debug directory
            if let Ok(debug_base_dir) = std::env::current_dir().map(|cwd| cwd.join("debug")) {
                let debug_dir = debug_base_dir.join("tts");
                if std::fs::create_dir_all(&debug_dir).is_ok() {
                    use std::io::Write;
                    use std::time::{SystemTime, UNIX_EPOCH};

                    let timestamp = SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_secs();

                    // Save audio file
                    let audio_filename = format!("synthesis_{}.{}", timestamp, request.format);
                    let audio_path = debug_dir.join(&audio_filename);

                    if let Err(e) = std::fs::File::create(&audio_path)
                        .and_then(|mut f| f.write_all(&audio_data))
                    {
                        warn!("Failed to save TTS audio to {:?}: {}", audio_path, e);
                    } else {
                        info!(
                            "Saved TTS audio to: {:?} ({} bytes)",
                            audio_path,
                            audio_data.len()
                        );
                    }

                    // Save request info
                    let info_filename = format!("synthesis_{}_info.txt", timestamp);
                    let info_path = debug_dir.join(&info_filename);
                    let info_content = format!(
                        "TTS Synthesis Info\n\
                        ==================\n\
                        Timestamp: {}\n\
                        Text: {}\n\
                        Format: {}\n\
                        Voice: {}\n\
                        Audio Size: {} bytes\n",
                        timestamp,
                        text,
                        request.format,
                        request.voice,
                        audio_data.len()
                    );

                    if let Err(e) = std::fs::write(&info_path, info_content) {
                        warn!("Failed to save TTS info to {:?}: {}", info_path, e);
                    } else {
                        info!("Saved TTS request info to: {:?}", info_path);
                    }
                }
            }

            let content_type = match request.format.as_str() {
                "mp3" => rocket::http::ContentType::new("audio", "mpeg"),
                "wav" => rocket::http::ContentType::new("audio", "wav"),
                _ => rocket::http::ContentType::new("audio", "pcm"),
            };
            Ok((content_type, audio_data))
        }
        Err(e) => {
            warn!("TTS synthesis error: {}", e);
            Err(rocket::http::Status::InternalServerError)
        }
    }
}

#[rocket::post("/api/stt", data = "<audio_data>")]
pub async fn stt_handler(
    state: &State<WebState>,
    audio_data: rocket::data::Data<'_>,
) -> Result<Json<serde_json::Value>, rocket::http::Status> {
    use log::info;
    use std::io::Write;
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    // Read audio data (limit to 2MB for audio files)
    let audio_bytes = audio_data
        .open(ByteUnit::Megabyte(2))
        .into_bytes()
        .await
        .map_err(|e| {
            warn!("Failed to read audio data: {}", e);
            rocket::http::Status::BadRequest
        })?;

    let audio_size = audio_bytes.len();
    info!("STT request received: audio_size={} bytes", audio_size);

    if audio_size == 0 {
        warn!("STT request with empty audio data");
        return Err(rocket::http::Status::BadRequest);
    }

    // Save audio file to debug directory (organized by function)
    let debug_base_dir = match std::env::current_dir() {
        Ok(cwd) => cwd.join("debug"),
        Err(e) => {
            warn!("Failed to get current directory: {}, using /tmp", e);
            PathBuf::from("/tmp/robonix-debug")
        }
    };
    let debug_dir = debug_base_dir.join("stt");

    if let Err(e) = std::fs::create_dir_all(&debug_dir) {
        warn!("Failed to create debug directory {:?}: {}", debug_dir, e);
    } else {
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();

        // Save original audio file
        let original_ext = if audio_bytes.len() >= 4
            && audio_bytes[0] == 0x1A
            && audio_bytes[1] == 0x45
            && audio_bytes[2] == 0xDF
            && audio_bytes[3] == 0xA3
        {
            "webm"
        } else if audio_bytes.len() >= 4 && &audio_bytes[0..4] == b"RIFF" {
            "wav"
        } else if audio_bytes.len() >= 4 && &audio_bytes[0..4] == b"OggS" {
            "ogg"
        } else {
            "unknown"
        };

        let original_filename = format!("recording_{}_original.{}", timestamp, original_ext);
        let original_path = debug_dir.join(&original_filename);

        match std::fs::File::create(&original_path).and_then(|mut f| f.write_all(&audio_bytes)) {
            Ok(_) => {
                info!(
                    "Saved original audio recording to: {:?} ({} bytes, format: {})",
                    original_path, audio_size, original_ext
                );
            }
            Err(e) => {
                warn!(
                    "Failed to save original audio file to {:?}: {}",
                    original_path, e
                );
            }
        }
    }

    // Detect audio format from file header
    let (format, detected_sample_rate) = detect_audio_format(&audio_bytes);

    // For WAV files, try to read actual sample rate from header
    let sample_rate = if format == "wav" && audio_bytes.len() >= 28 {
        // WAV file: sample rate is at offset 24 (little-endian u32)
        let sr = u32::from_le_bytes([
            audio_bytes[24],
            audio_bytes[25],
            audio_bytes[26],
            audio_bytes[27],
        ]);
        info!("Read sample rate from WAV header: {} Hz", sr);
        sr
    } else {
        detected_sample_rate
    };

    info!(
        "Detected audio format: format={}, sample_rate={} Hz, audio_size={} bytes",
        format, sample_rate, audio_size
    );

    // Save the audio that will be sent to STT service (converted WAV if applicable)
    let stt_audio_bytes = &audio_bytes;
    let stt_format = format.clone();

    // Save converted/sent audio to debug directory
    if let Ok(debug_base_dir) = std::env::current_dir().map(|cwd| cwd.join("debug")) {
        let debug_dir = debug_base_dir.join("stt");
        if std::fs::create_dir_all(&debug_dir).is_ok() {
            let timestamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs();

            let sent_filename = format!("recording_{}_sent_to_stt.{}", timestamp, stt_format);
            let sent_path = debug_dir.join(&sent_filename);

            if let Err(e) =
                std::fs::File::create(&sent_path).and_then(|mut f| f.write_all(stt_audio_bytes))
            {
                warn!("Failed to save sent audio to {:?}: {}", sent_path, e);
            } else {
                info!(
                    "Saved audio sent to STT service: {:?} ({} bytes)",
                    sent_path,
                    stt_audio_bytes.len()
                );
            }
        }
    }

    match state
        .stt_service
        .recognize(stt_audio_bytes, Some(&stt_format), Some(sample_rate))
        .await
    {
        Ok(text) => {
            info!("STT recognition successful: result_length={}", text.len());

            // Save STT request info and result to debug directory
            if let Ok(debug_base_dir) = std::env::current_dir().map(|cwd| cwd.join("debug")) {
                let debug_dir = debug_base_dir.join("stt");
                if std::fs::create_dir_all(&debug_dir).is_ok() {
                    let timestamp = SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_secs();

                    // Save STT request info
                    let info_filename = format!("request_{}_info.txt", timestamp);
                    let info_path = debug_dir.join(&info_filename);
                    let info_content = format!(
                        "STT Request Info\n\
                        ================\n\
                        Timestamp: {}\n\
                        Audio Size: {} bytes\n\
                        Format: {}\n\
                        Sample Rate: {} Hz\n\
                        Result Length: {} chars\n\
                        Result: {}\n",
                        timestamp,
                        audio_size,
                        stt_format,
                        sample_rate,
                        text.len(),
                        text
                    );

                    if let Err(e) = std::fs::write(&info_path, info_content) {
                        warn!("Failed to save STT info to {:?}: {}", info_path, e);
                    } else {
                        info!("Saved STT request info to: {:?}", info_path);
                    }
                }
            }

            Ok(Json(serde_json::json!({
                "status": "success",
                "result": text
            })))
        }
        Err(e) => {
            warn!("STT recognition error: {}", e);
            Err(rocket::http::Status::InternalServerError)
        }
    }
}

/// Detect audio format from file header
fn detect_audio_format(audio_data: &[u8]) -> (String, u32) {
    // Check for WebM format (starts with 0x1A 0x45 0xDF 0xA3)
    if audio_data.len() >= 4
        && audio_data[0] == 0x1A
        && audio_data[1] == 0x45
        && audio_data[2] == 0xDF
        && audio_data[3] == 0xA3
    {
        // WebM typically contains Opus audio, but Aliyun needs OGG-encapsulated Opus
        // Try opus format, but it might not work since webm != ogg
        info!("Detected WebM format, trying opus format");
        return ("opus".to_string(), 16000);
    }

    // Check for WAV format (starts with "RIFF")
    if audio_data.len() >= 4 && &audio_data[0..4] == b"RIFF" {
        info!("Detected WAV format");
        return ("wav".to_string(), 16000);
    }

    // Check for OGG format (starts with "OggS")
    if audio_data.len() >= 4 && &audio_data[0..4] == b"OggS" {
        info!("Detected OGG format");
        return ("opus".to_string(), 16000);
    }

    // Default to wav if unknown
    warn!("Unknown audio format, defaulting to wav");
    ("wav".to_string(), 16000)
}
