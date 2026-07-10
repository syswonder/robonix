// SPDX-License-Identifier: MulanPSL-2.0
// robonix-liaison — unified user-facing entry layer.
//
// Design
// ──────
// All input modalities ultimately produce the same `Task` and receive the
// same `PilotEvent` stream. The Liaison gRPC facade is contract
// `robonix/system/liaison`, which exposes:
//
//   * SubmitTask(Task) → stream PilotEvent
//       For text / API / pre-built tasks. Liaison normalises `Task.user_id`
//       (defaulting to `local:<os_user>` when empty) and forwards to Pilot.
//
//   * StartVoiceSession(...) → stream VoiceEvent
//       For push-to-talk voice. Liaison drives mic → ASR → voiceprint →
//       Pilot → optional TTS → speaker, wrapping every stage as a
//       VoiceEvent. See `voice.rs`.
//
// Liaison itself does NOT contain ASR / TTS / voiceprint logic. Each is an
// Atlas-registered system capability (robonix/service/speech/asr,
// robonix/service/speech/tts, robonix/service/voiceprint/identify) discovered
// and called via gRPC — same pattern as the VLM service in Pilot.

mod access;
mod handsfree;
mod pb;
mod voice;

use access::{AccessControlConfig, AccessDecision};
use anyhow::{Context, Result, anyhow};
use clap::Parser;
use pb::contracts::{
    robonix_system_liaison_handsfree_set_enabled_server::{
        RobonixSystemLiaisonHandsfreeSetEnabled, RobonixSystemLiaisonHandsfreeSetEnabledServer,
    },
    robonix_system_liaison_handsfree_status_server::{
        RobonixSystemLiaisonHandsfreeStatus, RobonixSystemLiaisonHandsfreeStatusServer,
    },
    robonix_system_liaison_submit_server::{
        RobonixSystemLiaisonSubmit, RobonixSystemLiaisonSubmitServer,
    },
    robonix_system_liaison_voice_server::{
        RobonixSystemLiaisonVoice, RobonixSystemLiaisonVoiceServer,
    },
    robonix_system_pilot_client::RobonixSystemPilotClient,
};
use pb::liaison::{
    GetHandsfreeStatusRequest, GetHandsfreeStatusResponse, SetHandsfreeRequest,
    SetHandsfreeResponse, StartVoiceSessionRequest, VoiceEvent,
};
use pb::pilot::rtdl_node_state::RtdlNodeStateEnum;
use pb::pilot::{CapabilityCall, PilotEvent, Plan, Task};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{debug, info, warn};
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{Mutex, mpsc};
use tokio_stream::{Stream, StreamExt, wrappers::ReceiverStream};
use tonic::{Request, Response, Status};
use uuid::Uuid;

const LIAISON_PROVIDER_ID: &str = "liaison";
const LIAISON_NAMESPACE: &str = "robonix/system/liaison";
const LIAISON_SUBMIT_CONTRACT: &str = "robonix/system/liaison/submit";
const LIAISON_VOICE_CONTRACT: &str = "robonix/system/liaison/voice";
const LIAISON_HANDSFREE_SET_CONTRACT: &str = "robonix/system/liaison/handsfree/set_enabled";
const LIAISON_HANDSFREE_STATUS_CONTRACT: &str = "robonix/system/liaison/handsfree/status";
const LIAISON_SUBMIT_TOML: &str = "capabilities/system/liaison/submit.v1.toml";
const LIAISON_VOICE_TOML: &str = "capabilities/system/liaison/voice.v1.toml";
const LIAISON_HANDSFREE_SET_TOML: &str =
    "capabilities/system/liaison/handsfree/set_enabled.v1.toml";
const LIAISON_HANDSFREE_STATUS_TOML: &str = "capabilities/system/liaison/handsfree/status.v1.toml";

/// `lib/system/pilot/msg/Task.msg` source: TEXT=0 AUDIO=1
const INTENT_SOURCE_TEXT: u32 = 0;
const INTENT_SOURCE_AUDIO: u32 = 1;

/// `lib/system/pilot/msg/PilotEvent.msg` event_kind.
const EVT_TEXT_CHUNK: u32 = 0;
const EVT_TASK_GRAPH: u32 = 1;
const EVT_BATCH_RESULT: u32 = 2;
const EVT_STATUS: u32 = 3;
const EVT_FINAL_TEXT: u32 = 4;

// ── LiaisonPipeline ─────────────────────────────────────────────────────────
//
// Forward a `Task` to Pilot (`RobonixSystemPilot.SubmitTask`), return the event
// channel. Opens a new gRPC channel per call so liaison can start before
// Pilot and survive Pilot restarts without restarting itself. Pilot endpoint
// is re-resolved through Atlas on every call (so a Pilot restart on a new
// port is picked up automatically).

pub struct LiaisonPipeline {
    pilot_endpoint_default: String,
    atlas: Arc<Mutex<AtlasClient>>,
    access: Arc<AccessControlConfig>,
}

impl LiaisonPipeline {
    /// Create a Liaison pipeline with a shared access-control policy. Every
    /// text/API task is normalized first, then authorized before any Pilot
    /// channel is opened.
    pub fn new(
        pilot_endpoint_default: impl Into<String>,
        atlas: Arc<Mutex<AtlasClient>>,
        access: Arc<AccessControlConfig>,
    ) -> Self {
        Self {
            pilot_endpoint_default: pilot_endpoint_default.into(),
            atlas,
            access,
        }
    }

    /// Forward `task` to Pilot. Returns a channel that yields `PilotEvent`s.
    pub async fn handle_intent(
        &self,
        mut task: Task,
    ) -> Result<mpsc::Receiver<Result<PilotEvent, Status>>> {
        ensure_user_id(&mut task);
        let user_id = task_user_id(&task);
        match self.access.authorize_user(&user_id) {
            AccessDecision::Allow {
                user_id,
                method,
                reason,
                ..
            } => {
                info!("[liaison/access] text allow user={user_id} via {method:?}: {reason}");
            }
            AccessDecision::Deny {
                user_id, reason, ..
            } => {
                warn!("[liaison/access] text deny user={user_id}: {reason}");
                return Err(anyhow!("access denied for user '{user_id}': {reason}"));
            }
        }
        let (tx, rx) = mpsc::channel(64);

        let pilot_ep = match resolve_pilot_endpoint(&self.atlas).await {
            Some(ep) => ep,
            None => self.pilot_endpoint_default.clone(),
        };

        let mut client = RobonixSystemPilotClient::connect(pilot_ep.clone())
            .await
            .with_context(|| format!("connect Pilot at {pilot_ep}"))?;
        let response = client
            .submit_task(Request::new(task))
            .await
            .with_context(|| format!("Pilot SubmitTask at {pilot_ep}"))?;
        let mut grpc = response.into_inner();
        tokio::spawn(async move {
            while let Some(item) = grpc.next().await {
                if tx
                    .send(item.map_err(|e| Status::internal(e.to_string())))
                    .await
                    .is_err()
                {
                    break;
                }
            }
        });
        Ok(rx)
    }
}

async fn resolve_pilot_endpoint(atlas: &Arc<Mutex<AtlasClient>>) -> Option<String> {
    let mut atlas = atlas.lock().await;
    let transport = atlas_pb::Transport::Grpc;
    let providers = atlas
        .query_capabilities("", "robonix/system/pilot", transport)
        .await
        .ok()?;
    let provider = providers.iter().find(|r| {
        r.capabilities
            .iter()
            .any(|i| i.contract_id == "robonix/system/pilot" && i.transport == transport as i32)
    })?;
    let (_channel_id, endpoint, _params) = atlas
        .connect_capability(
            LIAISON_PROVIDER_ID,
            &provider.id,
            "robonix/system/pilot",
            transport,
        )
        .await
        .ok()?;
    if endpoint.is_empty() {
        None
    } else {
        Some(localhost_to_ipv4(&endpoint))
    }
}

fn localhost_to_ipv4(ep: &str) -> String {
    let with_scheme = if ep.starts_with("http") {
        ep.to_string()
    } else {
        format!("http://{ep}")
    };
    with_scheme.replace("localhost", "127.0.0.1")
}

/// Default the user_id (stored inside `context_json.user_id` since Task itself
/// has no `user_id` field yet) to `local:<os_user>` and tag the modality so
/// Pilot can tell text vs voice without inspecting `source`.
fn ensure_user_id(task: &mut Task) {
    let mut ctx: serde_json::Value = if task.context_json.trim().is_empty() {
        serde_json::json!({})
    } else {
        serde_json::from_str(&task.context_json).unwrap_or_else(|_| serde_json::json!({}))
    };
    if let Some(obj) = ctx.as_object_mut() {
        let has_user = obj
            .get("user_id")
            .and_then(|v| v.as_str())
            .is_some_and(|s| !s.is_empty());
        if !has_user {
            obj.insert(
                "user_id".to_string(),
                serde_json::json!(format!("local:{}", whoami::username())),
            );
        }
        let modality = match task.source {
            INTENT_SOURCE_TEXT => "text",
            INTENT_SOURCE_AUDIO => "audio",
            _ => "unknown",
        };
        obj.entry("modality").or_insert(serde_json::json!(modality));
    }
    task.context_json = ctx.to_string();
}

fn task_user_id(task: &Task) -> String {
    serde_json::from_str::<serde_json::Value>(&task.context_json)
        .ok()
        .and_then(|ctx| {
            ctx.get("user_id")
                .and_then(|v| v.as_str())
                .map(ToString::to_string)
        })
        .unwrap_or_default()
}

// ── SystemLiaison gRPC impl ─────────────────────────────────────────────────

struct LiaisonServiceImpl {
    pipeline: Arc<LiaisonPipeline>,
    atlas: Arc<Mutex<AtlasClient>>,
    pilot_endpoint_default: String,
    access: Arc<AccessControlConfig>,
    handsfree: Arc<handsfree::HandsfreeController>,
}

#[tonic::async_trait]
impl RobonixSystemLiaisonSubmit for LiaisonServiceImpl {
    type SubmitTaskStream = ReceiverStream<Result<PilotEvent, Status>>;

    async fn submit_task(
        &self,
        request: Request<Task>,
    ) -> Result<Response<Self::SubmitTaskStream>, Status> {
        let task = request.into_inner();
        let rx = self
            .pipeline
            .handle_intent(task)
            .await
            .map_err(|e| Status::unavailable(format!("Pilot unreachable: {e:#}")))?;
        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

#[tonic::async_trait]
impl RobonixSystemLiaisonVoice for LiaisonServiceImpl {
    type StartVoiceSessionStream =
        Pin<Box<dyn Stream<Item = Result<VoiceEvent, Status>> + Send + 'static>>;

    async fn start_voice_session(
        &self,
        request: Request<StartVoiceSessionRequest>,
    ) -> Result<Response<Self::StartVoiceSessionStream>, Status> {
        let req = request.into_inner();
        let stream = voice::start_voice_session(
            req,
            Arc::clone(&self.atlas),
            self.pilot_endpoint_default.clone(),
            Arc::clone(&self.access),
        )
        .await?;
        let boxed: Self::StartVoiceSessionStream = Box::pin(stream);
        Ok(Response::new(boxed))
    }
}

#[tonic::async_trait]
impl RobonixSystemLiaisonHandsfreeSetEnabled for LiaisonServiceImpl {
    async fn set_handsfree(
        &self,
        request: Request<SetHandsfreeRequest>,
    ) -> Result<Response<SetHandsfreeResponse>, Status> {
        let request = request.into_inner();
        let status = self
            .handsfree
            .set_enabled(
                request.enabled,
                request.mic_provider_id,
                request.speaker_provider_id,
            )
            .await
            .map_err(|error| Status::invalid_argument(error.to_string()))?;
        Ok(Response::new(SetHandsfreeResponse {
            ok: true,
            enabled: status.enabled,
            state: status.state,
            detail: String::new(),
        }))
    }
}

#[tonic::async_trait]
impl RobonixSystemLiaisonHandsfreeStatus for LiaisonServiceImpl {
    async fn get_handsfree_status(
        &self,
        _request: Request<GetHandsfreeStatusRequest>,
    ) -> Result<Response<GetHandsfreeStatusResponse>, Status> {
        Ok(Response::new(self.handsfree.snapshot().await))
    }
}

// ── Stdin text loop (headless fallback) ─────────────────────────────────────

async fn drain_session_end(pipeline: &LiaisonPipeline, session_id: &str) {
    let task = Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: INTENT_SOURCE_TEXT,
        text: String::new(),
        audio_data: vec![],
        context_json: r#"{"session_end":true}"#.to_string(),
        timestamp_ms: now_ms(),
    };
    match pipeline.handle_intent(task).await {
        Ok(rx) => {
            let mut stream = ReceiverStream::new(rx);
            while stream.next().await.is_some() {}
        }
        Err(e) => debug!("[liaison/text] session_end: {e:#}"),
    }
}

async fn run_text_loop(pipeline: Arc<LiaisonPipeline>) -> Result<()> {
    use std::io::{self, Write};

    let session_id = Uuid::new_v4().to_string();
    println!("[liaison/text] session {session_id}");
    println!("[liaison/text] type a message and press Enter  (Ctrl-D to exit)");

    let stdin = io::stdin();
    loop {
        print!("> ");
        io::stdout().flush()?;

        let mut line = String::new();
        if stdin.read_line(&mut line)? == 0 {
            drain_session_end(&pipeline, &session_id).await;
            break;
        }
        let text = line.trim().to_string();
        if text.is_empty() {
            continue;
        }

        let task = Task {
            task_id: Uuid::new_v4().to_string(),
            session_id: session_id.clone(),
            source: INTENT_SOURCE_TEXT,
            text,
            audio_data: vec![],
            context_json: String::new(),
            timestamp_ms: now_ms(),
        };

        match pipeline.handle_intent(task).await {
            Err(e) => eprintln!("[liaison/text] error: {e:#}"),
            Ok(rx) => {
                let mut stream = ReceiverStream::new(rx);
                let mut printing = false;
                while let Some(item) = stream.next().await {
                    let ev = match item {
                        Ok(e) => e,
                        Err(e) => {
                            eprintln!("[liaison/text] stream error: {e}");
                            break;
                        }
                    };
                    match ev.event_kind {
                        EVT_TEXT_CHUNK => {
                            if !printing {
                                print!("\nPilot: ");
                                printing = true;
                            }
                            print!("{}", ev.text_chunk);
                            io::stdout().flush()?;
                        }
                        EVT_FINAL_TEXT => {
                            let t = ev.final_text.clone();
                            if printing {
                                println!();
                            } else {
                                println!("\nPilot: {t}");
                            }
                            printing = false;
                        }
                        EVT_TASK_GRAPH => {
                            if let Some(ref p) = ev.plan {
                                if printing {
                                    println!();
                                    printing = false;
                                }
                                println!(
                                    "[round {}] dispatching {} call(s)…",
                                    p.round,
                                    plan_calls(p).len()
                                );
                                for c in plan_calls(p) {
                                    println!("  · {}", c.contract_id);
                                }
                            }
                        }
                        EVT_BATCH_RESULT => {
                            if let Some(ref r) = ev.batch_result {
                                let ok = r
                                    .results
                                    .iter()
                                    .filter(|x| x.state == RtdlNodeStateEnum::Succeeded as u32)
                                    .count();
                                println!(
                                    "[round {}] results: {ok} ok, {} failed",
                                    r.round,
                                    r.results.len() - ok
                                );
                            }
                        }
                        EVT_STATUS => {}
                        _ => {}
                    }
                }
                println!();
            }
        }
    }
    println!("\nBye.");
    Ok(())
}

// ── main ────────────────────────────────────────────────────────────────────

#[derive(Parser, Debug)]
#[command(
    name = "robonix-liaison",
    about = "Unified user-facing entry — text + voice → Pilot"
)]
struct Args {
    /// gRPC listen address (host:port). Defaults to 127.0.0.1:50081 or
    /// $ROBONIX_LIAISON_PORT (port-only).
    #[arg(long)]
    listen: Option<String>,

    /// Atlas endpoint. Defaults to $ROBONIX_ATLAS_ENDPOINT, $ROBONIX_ATLAS,
    /// then 127.0.0.1:50051.
    #[arg(long)]
    atlas: Option<String>,

    /// Pilot fallback endpoint when Atlas can't yet resolve RobonixSystemPilot.
    /// Defaults to $ROBONIX_PILOT_ENDPOINT, then 127.0.0.1:50071.
    #[arg(long = "pilot-endpoint")]
    pilot_endpoint: Option<String>,

    /// Log level for this component (`debug`/`info`/`warn`/`error`). Sets the
    /// scribe log-file floor; falls back to `SCRIBE_FILE_LEVEL` / `info`.
    /// Normally arrives inside `--config-json`, not as a standalone flag.
    #[arg(long)]
    log: Option<String>,

    /// The component's `system.liaison` manifest block, serialized to JSON by
    /// rbnx and passed as one arg (`--config-json '{...}'`). Parsed by the binary
    /// itself; `robonix_scribe::init_from_config` reads the `log` key from it so
    /// the manifest's per-component level reaches the log.
    #[arg(long = "config-json")]
    config_json: Option<String>,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    // Apply the manifest's per-component `log:` level (delivered inside
    // --config-json) to scribe's file sink before the first log line.
    robonix_scribe::init_from_config("liaison", args.config_json.as_deref());
    info!("robonix-liaison starting");

    let atlas_endpoint = args.atlas.clone().unwrap_or_else(|| {
        env_first(
            &["ROBONIX_ATLAS_ENDPOINT", "ROBONIX_ATLAS"],
            "127.0.0.1:50051",
        )
    });
    let atlas_http = if atlas_endpoint.starts_with("http") {
        atlas_endpoint.clone()
    } else {
        format!("http://{atlas_endpoint}")
    };

    let pilot_endpoint = args
        .pilot_endpoint
        .clone()
        .unwrap_or_else(|| env_first(&["ROBONIX_PILOT_ENDPOINT"], "127.0.0.1:50071"));
    let pilot_http = {
        let raw = if pilot_endpoint.starts_with("http") {
            pilot_endpoint.clone()
        } else {
            format!("http://{pilot_endpoint}")
        };
        raw.replace("localhost", "127.0.0.1")
    };

    // --listen accepts host:port. If the manifest passes just a port (or
    // the user sets ROBONIX_LIAISON_PORT), bind 0.0.0.0:<port>.
    let listen_addr: std::net::SocketAddr = if let Some(spec) = args.listen.clone() {
        if spec.contains(':') {
            spec.parse()
                .with_context(|| format!("invalid --listen '{spec}'"))?
        } else {
            format!("0.0.0.0:{spec}").parse()?
        }
    } else {
        let port: u16 = std::env::var("ROBONIX_LIAISON_PORT")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(50081);
        format!("0.0.0.0:{port}").parse()?
    };
    let listen_port = listen_addr.port();
    let advertised = format!("127.0.0.1:{listen_port}");

    info!("connecting to atlas at {atlas_http}");
    let mut atlas = AtlasClient::connect_with_retry(&atlas_http, 10, Duration::from_secs(2))
        .await
        .context("connect to atlas")?;

    atlas
        .register_service(LIAISON_PROVIDER_ID, LIAISON_NAMESPACE, "")
        .await
        .context("register liaison capability")?;
    atlas
        .declare_capability(
            LIAISON_PROVIDER_ID,
            LIAISON_SUBMIT_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                LIAISON_SUBMIT_TOML,
                "robonix.contracts.RobonixSystemLiaisonSubmit",
                "/robonix.contracts.RobonixSystemLiaisonSubmit/SubmitTask",
            ),
        )
        .await
        .context("declare liaison submit gRPC capability")?;
    atlas
        .declare_capability(
            LIAISON_PROVIDER_ID,
            LIAISON_VOICE_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                LIAISON_VOICE_TOML,
                "robonix.contracts.RobonixSystemLiaisonVoice",
                "/robonix.contracts.RobonixSystemLiaisonVoice/StartVoiceSession",
            ),
        )
        .await
        .context("declare liaison voice gRPC capability")?;
    atlas
        .declare_capability(
            LIAISON_PROVIDER_ID,
            LIAISON_HANDSFREE_SET_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                LIAISON_HANDSFREE_SET_TOML,
                "robonix.contracts.RobonixSystemLiaisonHandsfreeSetEnabled",
                "/robonix.contracts.RobonixSystemLiaisonHandsfreeSetEnabled/SetHandsfree",
            ),
        )
        .await
        .context("declare liaison hands-free set gRPC capability")?;
    atlas
        .declare_capability(
            LIAISON_PROVIDER_ID,
            LIAISON_HANDSFREE_STATUS_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                LIAISON_HANDSFREE_STATUS_TOML,
                "robonix.contracts.RobonixSystemLiaisonHandsfreeStatus",
                "/robonix.contracts.RobonixSystemLiaisonHandsfreeStatus/GetHandsfreeStatus",
            ),
        )
        .await
        .context("declare liaison hands-free status gRPC capability")?;
    // Liaison has no Driver(CMD_INIT/CMD_ACTIVATE) handshake — it's a Rust binary
    // that's fully ready as soon as the gRPC server is listening. Push the
    // state explicitly so `rbnx caps` shows ACTIVE instead of stopping at
    // the legacy-fallback INACTIVE that atlas infers from the first declare.
    if let Err(e) = atlas
        .set_lifecycle_state(
            LIAISON_PROVIDER_ID,
            atlas_pb::LifecycleState::StateActive,
            "",
        )
        .await
    {
        warn!("SetLifecycleState(ACTIVE) on {LIAISON_PROVIDER_ID} failed: {e:#}");
    }
    info!("registered as '{LIAISON_PROVIDER_ID}', SystemLiaison gRPC on :{listen_port}");
    info!("robonix-liaison ready on :{listen_port}  (pilot_default={pilot_http})");

    {
        let mut hb = atlas.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await;
            loop {
                tick.tick().await;
                if let Err(e) = hb.heartbeat(LIAISON_PROVIDER_ID).await {
                    warn!("heartbeat failed: {e:#}");
                }
            }
        });
    }

    let atlas = Arc::new(Mutex::new(atlas));
    let access = Arc::new(AccessControlConfig::from_env());
    info!(
        "access gate enabled={} allowed_users={} voice_threshold={:.2}",
        access.enabled,
        access.allowed_users.len(),
        access.voice_threshold
    );
    let pipeline = Arc::new(LiaisonPipeline::new(
        pilot_http.clone(),
        Arc::clone(&atlas),
        Arc::clone(&access),
    ));
    let handsfree_config = args
        .config_json
        .as_deref()
        .map(serde_json::from_str::<handsfree::HandsfreeConfig>)
        .transpose()
        .context("parse liaison hands-free config")?
        .unwrap_or_default();
    let handsfree = handsfree::HandsfreeController::new(
        handsfree_config,
        Arc::clone(&atlas),
        pilot_http.clone(),
        Arc::clone(&access),
    );
    handsfree.spawn();

    let source = std::env::var("ROBONIX_LIAISON_SOURCE").unwrap_or_default();
    let text_handle: Option<tokio::task::JoinHandle<Result<()>>> = if source == "text" {
        info!("activating stdin text loop (headless mode)");
        Some(tokio::spawn(run_text_loop(Arc::clone(&pipeline))))
    } else {
        None
    };

    let svc = Arc::new(LiaisonServiceImpl {
        pipeline,
        atlas: Arc::clone(&atlas),
        pilot_endpoint_default: pilot_http,
        access,
        handsfree,
    });
    let server = tonic::transport::Server::builder()
        .add_service(RobonixSystemLiaisonSubmitServer::from_arc(Arc::clone(&svc)))
        .add_service(RobonixSystemLiaisonVoiceServer::from_arc(Arc::clone(&svc)))
        .add_service(RobonixSystemLiaisonHandsfreeSetEnabledServer::from_arc(
            Arc::clone(&svc),
        ))
        .add_service(RobonixSystemLiaisonHandsfreeStatusServer::from_arc(svc))
        .serve(listen_addr);

    if let Some(handle) = text_handle {
        tokio::select! {
            res = server => { res?; }
            res = handle => { res??; }
        }
    } else {
        server.await?;
    }

    Ok(())
}

fn env_first(vars: &[&str], default: &str) -> String {
    for v in vars {
        if let Ok(val) = std::env::var(v)
            && !val.is_empty()
        {
            return val;
        }
    }
    default.to_string()
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

fn plan_calls(plan: &Plan) -> Vec<&CapabilityCall> {
    plan.nodes
        .iter()
        .filter_map(|node| node.call.as_ref())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ensure_user_id_fills_default_for_text() {
        let mut t = Task {
            task_id: "t".into(),
            session_id: "s".into(),
            source: 0,
            text: "hi".into(),
            audio_data: vec![],
            context_json: String::new(),
            timestamp_ms: 0,
        };
        ensure_user_id(&mut t);
        let v: serde_json::Value = serde_json::from_str(&t.context_json).unwrap();
        let user_id = v["user_id"].as_str().unwrap();
        assert!(user_id.starts_with("local:"));
        assert_eq!(v["modality"], "text");
    }

    #[test]
    fn ensure_user_id_preserves_caller_value() {
        let mut t = Task {
            task_id: "t".into(),
            session_id: "s".into(),
            source: 1,
            text: "hi".into(),
            audio_data: vec![],
            context_json: r#"{"foo":"bar","user_id":"voice:alice"}"#.into(),
            timestamp_ms: 0,
        };
        ensure_user_id(&mut t);
        let v: serde_json::Value = serde_json::from_str(&t.context_json).unwrap();
        assert_eq!(v["modality"], "audio");
        assert_eq!(v["user_id"], "voice:alice");
        assert_eq!(v["foo"], "bar");
    }

    #[test]
    fn task_user_id_reads_context_field() {
        let t = Task {
            task_id: "t".into(),
            session_id: "s".into(),
            source: 0,
            text: "hi".into(),
            audio_data: vec![],
            context_json: r#"{"user_id":"local:alice"}"#.into(),
            timestamp_ms: 0,
        };
        assert_eq!(task_user_id(&t), "local:alice");
    }
}
