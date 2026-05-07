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
// Atlas-registered system capability (robonix/system/speech/asr,
// robonix/system/speech/tts, robonix/system/speech/voiceprint) discovered
// and called via gRPC — same pattern as the VLM service in Pilot.

mod pb;
mod voice;

use anyhow::{Context, Result};
use clap::Parser;
use pb::contracts::{
    system_liaison_server::{SystemLiaison, SystemLiaisonServer},
    system_pilot_client::SystemPilotClient,
};
use pb::liaison::{StartVoiceSessionRequest, VoiceEvent};
use pb::pilot::{PilotEvent, Task};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{Mutex, mpsc};
use tokio_stream::{Stream, StreamExt, wrappers::ReceiverStream};
use tonic::{Request, Response, Status};
use uuid::Uuid;

const LIAISON_CAPABILITY_ID: &str = "com.robonix.system.liaison";
const LIAISON_NAMESPACE: &str = "robonix/system/liaison";
const LIAISON_CONTRACT_ID: &str = "robonix/system/liaison";
const LIAISON_CAP_TOML: &str = "capabilities/system/liaison.v1.toml";

/// `lib/system/pilot/msg/Task.msg` source: TEXT=0 AUDIO=1 API=2.
const INTENT_SOURCE_TEXT: u32 = 0;

/// `lib/system/pilot/msg/PilotEvent.msg` event_kind.
const EVT_TEXT_CHUNK: u32 = 0;
const EVT_TASK_GRAPH: u32 = 1;
const EVT_BATCH_RESULT: u32 = 2;
const EVT_STATUS: u32 = 3;
const EVT_FINAL_TEXT: u32 = 4;

// ── LiaisonPipeline ─────────────────────────────────────────────────────────
//
// Forward a `Task` to Pilot (`SystemPilot.SubmitTask`), return the event
// channel. Opens a new gRPC channel per call so liaison can start before
// Pilot and survive Pilot restarts without restarting itself. Pilot endpoint
// is re-resolved through Atlas on every call (so a Pilot restart on a new
// port is picked up automatically).

pub struct LiaisonPipeline {
    pilot_endpoint_default: String,
    atlas: Arc<Mutex<AtlasClient>>,
}

impl LiaisonPipeline {
    pub fn new(pilot_endpoint_default: impl Into<String>, atlas: Arc<Mutex<AtlasClient>>) -> Self {
        Self {
            pilot_endpoint_default: pilot_endpoint_default.into(),
            atlas,
        }
    }

    /// Forward `task` to Pilot. Returns a channel that yields `PilotEvent`s.
    pub async fn handle_intent(
        &self,
        mut task: Task,
    ) -> Result<mpsc::Receiver<Result<PilotEvent, Status>>> {
        ensure_user_id(&mut task);
        let (tx, rx) = mpsc::channel(64);

        let pilot_ep = match resolve_pilot_endpoint(&self.atlas).await {
            Some(ep) => ep,
            None => self.pilot_endpoint_default.clone(),
        };

        let mut client = SystemPilotClient::connect(pilot_ep.clone())
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
    let records = atlas
        .query_capabilities("", "robonix/system/pilot", transport)
        .await
        .ok()?;
    let cap = records.iter().find(|r| {
        r.interfaces
            .iter()
            .any(|i| i.contract_id == "robonix/system/pilot" && i.transport == transport as i32)
    })?;
    let (_channel_id, endpoint, _params) = atlas
        .connect_capability(
            LIAISON_CAPABILITY_ID,
            &cap.capability_id,
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
            1 => "voice",
            2 => "api",
            _ => "unknown",
        };
        obj.entry("modality").or_insert(serde_json::json!(modality));
    }
    task.context_json = ctx.to_string();
}

// ── SystemLiaison gRPC impl ─────────────────────────────────────────────────

struct LiaisonServiceImpl {
    pipeline: Arc<LiaisonPipeline>,
    atlas: Arc<Mutex<AtlasClient>>,
    pilot_endpoint_default: String,
}

#[tonic::async_trait]
impl SystemLiaison for LiaisonServiceImpl {
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
        )
        .await?;
        let boxed: Self::StartVoiceSessionStream = Box::pin(stream);
        Ok(Response::new(boxed))
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
        Err(e) => log::debug!("[liaison/text] session_end: {e:#}"),
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
                                    p.calls.len()
                                );
                                for c in &p.calls {
                                    println!("  · {}", c.contract_id);
                                }
                            }
                        }
                        EVT_BATCH_RESULT => {
                            if let Some(ref r) = ev.batch_result {
                                let ok = r.results.iter().filter(|x| x.success).count();
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

    /// Pilot fallback endpoint when Atlas can't yet resolve SystemPilot.
    /// Defaults to $ROBONIX_PILOT_ENDPOINT, then 127.0.0.1:50071.
    #[arg(long = "pilot-endpoint")]
    pilot_endpoint: Option<String>,

    /// Log filter (env_logger format). Defaults to $RUST_LOG, then "robonix_liaison=info".
    #[arg(long)]
    log: Option<String>,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let log_filter = args
        .log
        .clone()
        .or_else(|| std::env::var("RUST_LOG").ok())
        .unwrap_or_else(|| "robonix_liaison=info".to_string());
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(log_filter)).init();

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

    log::info!("connecting to atlas at {atlas_http}");
    let mut atlas = AtlasClient::connect_with_retry(&atlas_http, 10, Duration::from_secs(2))
        .await
        .context("connect to atlas")?;

    atlas
        .register_capability(LIAISON_CAPABILITY_ID, LIAISON_NAMESPACE, "")
        .await
        .context("register liaison capability")?;
    atlas
        .declare_interface(
            LIAISON_CAPABILITY_ID,
            LIAISON_CONTRACT_ID,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                LIAISON_CAP_TOML,
                "robonix.contracts.SystemLiaison",
                "/robonix.contracts.SystemLiaison/SubmitTask",
            ),
        )
        .await
        .context("declare liaison gRPC interface")?;
    // Liaison has no Driver(CMD_INIT/CMD_UP) handshake — it's a Rust binary
    // that's fully ready as soon as the gRPC server is listening. Push the
    // state explicitly so `rbnx caps` shows ONLINE instead of stopping at the
    // legacy-fallback INITIALIZED that atlas infers from the first declare.
    if let Err(e) = atlas
        .set_capability_state(LIAISON_CAPABILITY_ID, atlas_pb::CapabilityState::StateOnline, "")
        .await
    {
        log::warn!("SetCapabilityState(ONLINE) on {LIAISON_CAPABILITY_ID} failed: {e:#}");
    }
    log::info!("registered as '{LIAISON_CAPABILITY_ID}', SystemLiaison gRPC on :{listen_port}");
    eprintln!("robonix-liaison ready on :{listen_port}  (pilot_default={pilot_http})");

    {
        let mut hb = atlas.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await;
            loop {
                tick.tick().await;
                if let Err(e) = hb.heartbeat(LIAISON_CAPABILITY_ID).await {
                    log::warn!("heartbeat failed: {e:#}");
                }
            }
        });
    }

    let atlas = Arc::new(Mutex::new(atlas));
    let pipeline = Arc::new(LiaisonPipeline::new(pilot_http.clone(), Arc::clone(&atlas)));

    let source = std::env::var("ROBONIX_LIAISON_SOURCE").unwrap_or_default();
    let text_handle: Option<tokio::task::JoinHandle<Result<()>>> = if source == "text" {
        log::info!("activating stdin text loop (headless mode)");
        Some(tokio::spawn(run_text_loop(Arc::clone(&pipeline))))
    } else {
        None
    };

    let svc = LiaisonServiceImpl {
        pipeline,
        atlas: Arc::clone(&atlas),
        pilot_endpoint_default: pilot_http,
    };
    let server = tonic::transport::Server::builder()
        .add_service(SystemLiaisonServer::new(svc))
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
        assert_eq!(v["modality"], "voice");
        assert_eq!(v["user_id"], "voice:alice");
        assert_eq!(v["foo"], "bar");
    }
}
