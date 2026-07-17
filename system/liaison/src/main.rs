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
    robonix_lifecycle_driver_client::RobonixLifecycleDriverClient,
    robonix_lifecycle_driver_server::{RobonixLifecycleDriver, RobonixLifecycleDriverServer},
    robonix_system_liaison_handsfree_events_server::{
        RobonixSystemLiaisonHandsfreeEvents, RobonixSystemLiaisonHandsfreeEventsServer,
    },
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
    SetHandsfreeResponse, StartVoiceSessionRequest, VoiceEvent, WatchHandsfreeEventsRequest,
};
use pb::lifecycle::{DriverRequest, DriverResponse};
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
const LIAISON_HANDSFREE_EVENTS_CONTRACT: &str = "robonix/system/liaison/handsfree/events";
const LIAISON_SUBMIT_TOML: &str = "capabilities/system/liaison/submit.v1.toml";
const LIAISON_VOICE_TOML: &str = "capabilities/system/liaison/voice.v1.toml";
const LIAISON_HANDSFREE_SET_TOML: &str =
    "capabilities/system/liaison/handsfree/set_enabled.v1.toml";
const LIAISON_HANDSFREE_STATUS_TOML: &str = "capabilities/system/liaison/handsfree/status.v1.toml";
const LIAISON_HANDSFREE_EVENTS_TOML: &str = "capabilities/system/liaison/handsfree/events.v1.toml";
const SHARED_DRIVER_CONTRACT: &str = "robonix/lifecycle/driver";
const CMD_INIT: u32 = 0;
const CMD_ACTIVATE: u32 = 1;
const CMD_DEACTIVATE: u32 = 2;
const CMD_SHUTDOWN: u32 = 3;

#[derive(Clone)]
struct SystemLifecycleDriver {
    atlas: AtlasClient,
    provider_id: String,
    shutdown_tx: tokio::sync::watch::Sender<bool>,
}

impl SystemLifecycleDriver {
    fn new(atlas: AtlasClient, provider_id: String) -> Self {
        let (shutdown_tx, _) = tokio::sync::watch::channel(false);
        Self {
            atlas,
            provider_id,
            shutdown_tx,
        }
    }

    /// Apply one no-op lifecycle callback and publish its authoritative state
    /// to Atlas. Unknown commands and rejected Atlas transitions are errors.
    async fn transition(&self, command: u32) -> Result<&'static str> {
        let (state, label) = lifecycle_target(command)
            .ok_or_else(|| anyhow!("unknown lifecycle command code {command}"))?;
        let mut atlas = self.atlas.clone();
        atlas
            .set_lifecycle_state(&self.provider_id, state, "")
            .await
            .with_context(|| format!("publish lifecycle state for '{}'", self.provider_id))?;
        if command == CMD_SHUTDOWN {
            self.shutdown_tx.send_replace(true);
        }
        Ok(label)
    }

    fn subscribe_shutdown(&self) -> tokio::sync::watch::Receiver<bool> {
        self.shutdown_tx.subscribe()
    }
}

#[tonic::async_trait]
impl RobonixLifecycleDriver for SystemLifecycleDriver {
    /// Serve the shared Driver RPC and report callback/transition failures in
    /// the stable DriverResponse envelope used by launchers.
    async fn driver(
        &self,
        request: Request<DriverRequest>,
    ) -> std::result::Result<Response<DriverResponse>, Status> {
        let response = match self.transition(request.into_inner().command).await {
            Ok(state) => DriverResponse {
                ok: true,
                state: state.to_string(),
                error: String::new(),
            },
            Err(error) => DriverResponse {
                ok: false,
                state: "error".to_string(),
                error: format!("{error:#}"),
            },
        };
        Ok(Response::new(response))
    }
}

fn lifecycle_target(command: u32) -> Option<(atlas_pb::LifecycleState, &'static str)> {
    match command {
        CMD_INIT => Some((atlas_pb::LifecycleState::StateInactive, "inactive")),
        CMD_ACTIVATE => Some((atlas_pb::LifecycleState::StateActive, "active")),
        CMD_DEACTIVATE => Some((atlas_pb::LifecycleState::StateInactive, "inactive")),
        CMD_SHUTDOWN => Some((atlas_pb::LifecycleState::StateTerminated, "terminated")),
        _ => None,
    }
}

/// Wait until Driver(CMD_SHUTDOWN) has published TERMINATED. A watch channel
/// preserves a shutdown sent before this particular waiter starts polling.
async fn wait_for_driver_shutdown(mut shutdown: tokio::sync::watch::Receiver<bool>) {
    if *shutdown.borrow() {
        return;
    }
    while shutdown.changed().await.is_ok() {
        if *shutdown.borrow() {
            return;
        }
    }
}

/// Return a loopback endpoint for an unspecified bind address so the process
/// can self-dial without changing the endpoint it advertises through Atlas.
fn startup_driver_endpoint(listen_addr: std::net::SocketAddr) -> String {
    let ip = match listen_addr.ip() {
        std::net::IpAddr::V4(ip) if ip.is_unspecified() => {
            std::net::IpAddr::V4(std::net::Ipv4Addr::LOCALHOST)
        }
        std::net::IpAddr::V6(ip) if ip.is_unspecified() => {
            std::net::IpAddr::V6(std::net::Ipv6Addr::LOCALHOST)
        }
        ip => ip,
    };
    std::net::SocketAddr::new(ip, listen_addr.port()).to_string()
}

/// Connect to this process's just-spawned Driver endpoint. Retries only the
/// bounded bind/startup window; a connected endpoint is returned immediately.
async fn connect_startup_driver(
    endpoint: &str,
) -> Result<RobonixLifecycleDriverClient<tonic::transport::Channel>> {
    let endpoint = if endpoint.starts_with("http") {
        endpoint.to_string()
    } else {
        format!("http://{endpoint}")
    };
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    loop {
        match RobonixLifecycleDriverClient::connect(endpoint.clone()).await {
            Ok(client) => return Ok(client),
            Err(_) if tokio::time::Instant::now() < deadline => {
                tokio::time::sleep(Duration::from_millis(25)).await;
            }
            Err(error) => return Err(error).context("connect startup lifecycle Driver"),
        }
    }
}

/// Invoke one startup lifecycle command through the real generated Driver RPC
/// and reject an `ok=false` response as a startup failure.
async fn call_startup_driver(
    client: &mut RobonixLifecycleDriverClient<tonic::transport::Channel>,
    command: u32,
) -> Result<String> {
    let response = client
        .driver(DriverRequest {
            command,
            config_json: "{}".to_string(),
        })
        .await
        .context("call startup lifecycle Driver")?
        .into_inner();
    if !response.ok {
        anyhow::bail!("startup lifecycle Driver failed: {}", response.error);
    }
    Ok(response.state)
}

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
        self.handsfree.suspend_capture().await;
        let stream = match voice::start_voice_session(
            req,
            Arc::clone(&self.atlas),
            self.pilot_endpoint_default.clone(),
            Arc::clone(&self.access),
        )
        .await
        {
            Ok(stream) => stream,
            Err(status) => {
                self.handsfree.resume_capture().await;
                return Err(status);
            }
        };
        let (tx, rx) = mpsc::channel(64);
        let handsfree = Arc::clone(&self.handsfree);
        tokio::spawn(async move {
            let mut stream = Box::pin(stream);
            let mut capture_suspended = true;
            while let Some(item) = stream.next().await {
                if capture_suspended
                    && item
                        .as_ref()
                        .is_ok_and(|event| event.event_kind == voice::KIND_ASR_FINAL)
                {
                    handsfree.resume_capture().await;
                    capture_suspended = false;
                }
                if tx.send(item).await.is_err() {
                    break;
                }
            }
            if capture_suspended {
                handsfree.resume_capture().await;
            }
        });
        let boxed: Self::StartVoiceSessionStream = Box::pin(ReceiverStream::new(rx));
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

#[tonic::async_trait]
impl RobonixSystemLiaisonHandsfreeEvents for LiaisonServiceImpl {
    type WatchHandsfreeEventsStream = ReceiverStream<Result<VoiceEvent, Status>>;

    async fn watch_handsfree_events(
        &self,
        _request: Request<WatchHandsfreeEventsRequest>,
    ) -> Result<Response<Self::WatchHandsfreeEventsStream>, Status> {
        Ok(Response::new(self.handsfree.subscribe_events()))
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
            SHARED_DRIVER_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/lifecycle/driver.v1.toml",
                "robonix.contracts.RobonixLifecycleDriver",
                "/robonix.contracts.RobonixLifecycleDriver/Driver",
            ),
        )
        .await
        .context("declare liaison shared lifecycle Driver")?;
    let lifecycle = SystemLifecycleDriver::new(atlas.clone(), LIAISON_PROVIDER_ID.to_string());
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
    atlas
        .declare_capability(
            LIAISON_PROVIDER_ID,
            LIAISON_HANDSFREE_EVENTS_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                LIAISON_HANDSFREE_EVENTS_TOML,
                "robonix.contracts.RobonixSystemLiaisonHandsfreeEvents",
                "/robonix.contracts.RobonixSystemLiaisonHandsfreeEvents/WatchHandsfreeEvents",
            ),
        )
        .await
        .context("declare liaison hands-free events gRPC capability")?;
    let mut heartbeat_atlas = atlas.clone();
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

    let svc = Arc::new(LiaisonServiceImpl {
        pipeline: Arc::clone(&pipeline),
        atlas: Arc::clone(&atlas),
        pilot_endpoint_default: pilot_http.clone(),
        access,
        handsfree,
    });
    let server_shutdown = lifecycle.subscribe_shutdown();
    let server_lifecycle = lifecycle.clone();
    let mut server_task = tokio::spawn(async move {
        tonic::transport::Server::builder()
            .add_service(RobonixLifecycleDriverServer::new(server_lifecycle))
            .add_service(RobonixSystemLiaisonSubmitServer::from_arc(Arc::clone(&svc)))
            .add_service(RobonixSystemLiaisonVoiceServer::from_arc(Arc::clone(&svc)))
            .add_service(RobonixSystemLiaisonHandsfreeSetEnabledServer::from_arc(
                Arc::clone(&svc),
            ))
            .add_service(RobonixSystemLiaisonHandsfreeStatusServer::from_arc(
                Arc::clone(&svc),
            ))
            .add_service(RobonixSystemLiaisonHandsfreeEventsServer::from_arc(svc))
            .serve_with_shutdown(listen_addr, wait_for_driver_shutdown(server_shutdown))
            .await
    });
    let startup_endpoint = startup_driver_endpoint(listen_addr);
    let mut startup_driver = tokio::select! {
        client = connect_startup_driver(&startup_endpoint) => client?,
        result = &mut server_task => {
            result.context("join Liaison gRPC server")?
                .context("Liaison gRPC server failed before readiness")?;
            anyhow::bail!("Liaison gRPC server stopped before readiness");
        }
    };
    call_startup_driver(&mut startup_driver, CMD_INIT)
        .await
        .context("initialize liaison lifecycle")?;
    call_startup_driver(&mut startup_driver, CMD_ACTIVATE)
        .await
        .context("activate liaison lifecycle")?;
    drop(startup_driver);

    let shutdown = lifecycle.subscribe_shutdown();
    tokio::spawn(async move {
        let mut tick = tokio::time::interval(Duration::from_secs(20));
        tick.tick().await;
        let shutdown = wait_for_driver_shutdown(shutdown);
        tokio::pin!(shutdown);
        loop {
            tokio::select! {
                _ = &mut shutdown => break,
                _ = tick.tick() => {
                    if let Err(e) = heartbeat_atlas.heartbeat(LIAISON_PROVIDER_ID).await {
                        warn!("heartbeat failed: {e:#}");
                    }
                }
            }
        }
    });
    info!("registered as '{LIAISON_PROVIDER_ID}', SystemLiaison gRPC on :{listen_port}");
    info!("robonix-liaison ready on :{listen_port}  (pilot_default={pilot_http})");

    let source = std::env::var("ROBONIX_LIAISON_SOURCE").unwrap_or_default();
    let text_handle: Option<tokio::task::JoinHandle<Result<()>>> = if source == "text" {
        info!("activating stdin text loop (headless mode)");
        Some(tokio::spawn(run_text_loop(Arc::clone(&pipeline))))
    } else {
        None
    };

    if let Some(handle) = text_handle {
        tokio::select! {
            res = &mut server_task => {
                res.context("join Liaison gRPC server")?
                    .context("Liaison gRPC server failed")?;
            }
            res = handle => {
                res??;
                server_task.abort();
            }
        }
    } else {
        server_task
            .await
            .context("join Liaison gRPC server")?
            .context("Liaison gRPC server failed")?;
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
    use pb::contracts::robonix_lifecycle_driver_client::RobonixLifecycleDriverClient;
    use robonix_atlas::service::{AtlasRegistry, serve_atlas};
    use std::net::SocketAddr;

    fn reserve_address() -> SocketAddr {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("reserve port");
        listener.local_addr().expect("reserved address")
    }

    #[test]
    fn startup_dial_rewrites_unspecified_addresses() {
        assert_eq!(
            startup_driver_endpoint("0.0.0.0:51001".parse().unwrap()),
            "127.0.0.1:51001"
        );
        assert_eq!(
            startup_driver_endpoint("[::]:51001".parse().unwrap()),
            "[::1]:51001"
        );
    }

    /// Dial a just-spawned Driver server, retrying only the short startup race.
    async fn connect_driver(
        endpoint: String,
    ) -> RobonixLifecycleDriverClient<tonic::transport::Channel> {
        let mut last_error = None;
        for _ in 0..50 {
            match RobonixLifecycleDriverClient::connect(endpoint.clone()).await {
                Ok(client) => return client,
                Err(error) => last_error = Some(error),
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        panic!(
            "connect Driver: {:#}",
            last_error.expect("connection error")
        );
    }

    /// Exercise the generated Driver client against the real tonic endpoint
    /// and verify that both RPCs publish their lifecycle states to Atlas.
    #[tokio::test]
    async fn shared_driver_rpc_is_callable() {
        let atlas_addr = reserve_address();
        let registry = Arc::new(AtlasRegistry::default());
        let atlas_server = tokio::spawn(serve_atlas(Arc::clone(&registry), atlas_addr));
        let mut atlas = AtlasClient::connect_with_retry(
            format!("http://{atlas_addr}"),
            50,
            Duration::from_millis(10),
        )
        .await
        .expect("connect test Atlas");
        let provider_id = "liaison-driver-rpc-test";
        atlas
            .register_service(provider_id, LIAISON_NAMESPACE, "")
            .await
            .expect("register test Provider");

        let driver_addr = reserve_address();
        atlas
            .declare_capability(
                provider_id,
                SHARED_DRIVER_CONTRACT,
                atlas_pb::Transport::Grpc,
                &driver_addr.to_string(),
                atlas_client::grpc_params(
                    "capabilities/lifecycle/driver.v1.toml",
                    "robonix.contracts.RobonixLifecycleDriver",
                    "/robonix.contracts.RobonixLifecycleDriver/Driver",
                ),
            )
            .await
            .expect("declare shared Driver");
        let lifecycle = SystemLifecycleDriver::new(atlas.clone(), provider_id.to_string());
        let server_shutdown = lifecycle.subscribe_shutdown();
        let driver_server = tokio::spawn(
            tonic::transport::Server::builder()
                .add_service(RobonixLifecycleDriverServer::new(lifecycle))
                .serve_with_shutdown(driver_addr, wait_for_driver_shutdown(server_shutdown)),
        );
        let mut client = connect_driver(format!("http://{driver_addr}")).await;

        let init = client
            .driver(DriverRequest {
                command: CMD_INIT,
                config_json: "{}".to_string(),
            })
            .await
            .expect("Driver INIT RPC")
            .into_inner();
        assert!(init.ok, "{}", init.error);
        assert_eq!(init.state, "inactive");
        let activate = client
            .driver(DriverRequest {
                command: CMD_ACTIVATE,
                config_json: String::new(),
            })
            .await
            .expect("Driver ACTIVATE RPC")
            .into_inner();
        assert!(activate.ok, "{}", activate.error);
        assert_eq!(activate.state, "active");

        let shutdown = client
            .driver(DriverRequest {
                command: CMD_SHUTDOWN,
                config_json: String::new(),
            })
            .await
            .expect("Driver SHUTDOWN RPC")
            .into_inner();
        assert!(shutdown.ok, "{}", shutdown.error);
        assert_eq!(shutdown.state, "terminated");
        tokio::time::timeout(Duration::from_secs(2), driver_server)
            .await
            .expect("Driver server did not stop after SHUTDOWN")
            .expect("join Driver server")
            .expect("graceful Driver server shutdown");

        let provider = atlas
            .query(
                atlas_pb::Kind::Service,
                provider_id,
                "",
                "",
                atlas_pb::Transport::Unspecified,
            )
            .await
            .expect("query test Provider")
            .pop()
            .expect("test Provider");
        assert_eq!(
            provider.state,
            atlas_pb::LifecycleState::StateTerminated as i32
        );

        atlas_server.abort();
    }

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
