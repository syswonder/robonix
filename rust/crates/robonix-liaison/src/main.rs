// SPDX-License-Identifier: MulanPSL-2.0
// robonix-liaison — unified user-facing input layer
//
// ── Design ───────────────────────────────────────────────────────────────────
//
//  All input modalities ultimately produce the same `Task` and receive the
//  same `PilotEvent` stream.  The liaison gRPC facade is contract `SrvLiaison`
//  (`robonix_contracts.proto`), which now exposes two RPCs:
//
//    * Stream(Task) → stream PilotEvent
//        For text / API / pre-built tasks. Liaison normalises `Task.user_id`
//        (defaulting to "local:<os_user>" when empty) and forwards to Pilot.
//
//    * StartVoiceSession(...) → stream VoiceEvent
//        For push-to-talk voice. Liaison itself drives mic → ASR → voiceprint
//        → Pilot → optional TTS → speaker, wrapping every stage as a
//        VoiceEvent. See `voice.rs`.
//
//  Key principle: liaison itself does NOT contain ASR/TTS logic.  ASR, TTS
//  and voiceprint are Atlas-registered system nodes (robonix/srv/speech/asr,
//  robonix/srv/speech/tts, robonix/srv/speech/voiceprint) discovered and
//  called via gRPC — same pattern as the VLM service in Pilot.

mod voice;

use robonix_interfaces::{contracts, liaison, pilot};

use anyhow::Result;
use contracts::srv_liaison_server::{SrvLiaison, SrvLiaisonServer};
use contracts::srv_pilot_client::SrvPilotClient;
use liaison::{StartVoiceSessionRequest, VoiceEvent};
use pilot::{PilotEvent, Task};
use robonix_sdk::RobonixClient;
use std::pin::Pin;
use std::sync::Arc;
use tokio::sync::{Mutex, mpsc};
use tokio_stream::{Stream, StreamExt, wrappers::ReceiverStream};
use tonic::{Request, Response, Status};
use uuid::Uuid;

const LIAISON_NODE_ID: &str = "com.robonix.runtime.liaison";

/// `lib/pilot/msg/Task.msg` source: TEXT=0 AUDIO=1 API=2
const INTENT_SOURCE_TEXT: u32 = 0;
/// `lib/pilot/msg/PilotEvent.msg` event_kind
const EVT_TEXT_CHUNK: u32 = 0;
const EVT_TASK_GRAPH: u32 = 1;
const EVT_BATCH_RESULT: u32 = 2;
const EVT_STATUS: u32 = 3;
const EVT_FINAL_TEXT: u32 = 4;

// ── LiaisonPipeline ───────────────────────────────────────────────────────────
//
// Forward an `Task` to Pilot (`SrvPilot.Stream`), return the event channel.
// Opens a new gRPC channel per call so liaison can start before Pilot and
// survive Pilot restarts without restarting itself.

pub struct LiaisonPipeline {
    pilot_endpoint: String,
}

impl LiaisonPipeline {
    pub fn new(pilot_endpoint: impl Into<String>) -> Self {
        Self {
            pilot_endpoint: pilot_endpoint.into(),
        }
    }

    pub fn pilot_endpoint(&self) -> &str {
        &self.pilot_endpoint
    }

    /// Forward `task` to Pilot. Returns a channel that yields `PilotEvent`s.
    /// A background task pumps the underlying gRPC stream into the channel.
    /// If Pilot is unreachable, emits a fallback "成功接收信息" response so the
    /// TUI remains functional during testing without a live Pilot.
    pub async fn handle_intent(
        &self,
        mut task: Task,
    ) -> Result<mpsc::Receiver<Result<PilotEvent, Status>>> {
        ensure_user_id(&mut task);
        let (tx, rx) = mpsc::channel(64);
        let pilot_ep = self.pilot_endpoint.clone();

        let session_id = task.session_id.clone();
        let connect_result = SrvPilotClient::connect(pilot_ep.clone()).await;
        match connect_result {
            Ok(mut client) => match client.stream(Request::new(task)).await {
                Ok(response) => {
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
                }
                Err(e) => {
                    log::warn!("Pilot Stream RPC failed: {e}, using fallback");
                    let sid = session_id.clone();
                    tokio::spawn(async move {
                        send_fallback_pilot_events(tx, &sid).await;
                    });
                }
            },
            Err(e) => {
                log::warn!("Pilot unreachable at {pilot_ep}: {e}, using fallback");
                tokio::spawn(async move {
                    send_fallback_pilot_events(tx, &session_id).await;
                });
            }
        }
        Ok(rx)
    }
}

async fn send_fallback_pilot_events(
    tx: mpsc::Sender<Result<PilotEvent, Status>>,
    session_id: &str,
) {
    let fallback = "成功接收信息";
    let _ = tx
        .send(Ok(PilotEvent {
            event_kind: EVT_TEXT_CHUNK,
            session_id: session_id.to_string(),
            text_chunk: fallback.to_string(),
            task_graph: None,
            batch_result: None,
            status: None,
            final_text: String::new(),
        }))
        .await;
    let _ = tx
        .send(Ok(PilotEvent {
            event_kind: EVT_FINAL_TEXT,
            session_id: session_id.to_string(),
            text_chunk: String::new(),
            task_graph: None,
            batch_result: None,
            status: None,
            final_text: fallback.to_string(),
        }))
        .await;
}

/// Default `Task.user_id` to `local:<os_user>` (and mirror the value into
/// `context_json`) so Pilot always sees a populated user identity. Existing
/// values are left untouched.
fn ensure_user_id(task: &mut Task) {
    if task.user_id.is_empty() {
        task.user_id = format!("local:{}", whoami::username());
    }
    let mut ctx: serde_json::Value = if task.context_json.trim().is_empty() {
        serde_json::json!({})
    } else {
        serde_json::from_str(&task.context_json).unwrap_or_else(|_| serde_json::json!({}))
    };
    if let Some(obj) = ctx.as_object_mut() {
        obj.entry("user_id")
            .or_insert(serde_json::json!(task.user_id));
        // Mark the modality so Pilot can tell text vs voice without inspecting `source`.
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

// ── SrvLiaison gRPC impl ─────────────────────────────────────────────────────
//
// All clients — rbnx chat, mobile app, GUI, the SpeechBridge — use this
// contract service. Interrupt / turn-cancel is expressed as an `Task` with
// `context_json` `{"abort_turn":true}` end-to-end (Pilot handles it); there
// is no separate liaison RPC.

struct LiaisonServiceImpl {
    pipeline: Arc<LiaisonPipeline>,
    sdk: Arc<Mutex<RobonixClient>>,
}

#[tonic::async_trait]
impl SrvLiaison for LiaisonServiceImpl {
    type StreamStream = ReceiverStream<Result<PilotEvent, Status>>;

    async fn stream(&self, request: Request<Task>) -> Result<Response<Self::StreamStream>, Status> {
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
        let pilot_ep = self.pipeline.pilot_endpoint().to_string();
        let stream =
            voice::start_voice_session(req, Arc::clone(&self.sdk), pilot_ep).await?;
        let boxed: Self::StartVoiceSessionStream = Box::pin(stream);
        Ok(Response::new(boxed))
    }
}

// ── Stdin text loop ─────────────────────────────────────────────────────────
//
// Convenience fallback for headless / pipe usage.
// Activated by ROBONIX_LIAISON_SOURCE=text.
// `rbnx chat` (gRPC TUI) is the preferred interactive interface.

async fn drain_session_end(pipeline: &LiaisonPipeline, session_id: &str) {
    let task = Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: INTENT_SOURCE_TEXT,
        text: String::new(),
        audio_data: vec![],
        context_json: r#"{"session_end":true}"#.to_string(),
        timestamp_ms: now_ms(),
        user_id: String::new(),
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
            user_id: String::new(),
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
                            if let Some(ref g) = ev.task_graph {
                                if printing {
                                    println!();
                                    printing = false;
                                }
                                println!(
                                    "[round {}] dispatching {} call(s)…",
                                    g.round,
                                    g.calls.len()
                                );
                                for c in &g.calls {
                                    println!("  · {}", c.tool_name);
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

// ── main ──────────────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("robonix_liaison=info"),
    )
    .init();

    let atlas_endpoint = resolve_endpoint(
        &["ROBONIX_ATLAS_ENDPOINT", "ROBONIX_ATLAS"],
        "127.0.0.1:50051",
    );
    let atlas_http = if atlas_endpoint.starts_with("http") {
        atlas_endpoint.clone()
    } else {
        format!("http://{atlas_endpoint}")
    };

    let pilot_endpoint = resolve_endpoint(&["ROBONIX_PILOT_ENDPOINT"], "127.0.0.1:50071");
    let pilot_http = {
        let raw = if pilot_endpoint.starts_with("http") {
            pilot_endpoint.clone()
        } else {
            format!("http://{pilot_endpoint}")
        };
        raw.replace("localhost", "127.0.0.1")
    };

    let listen_port: u16 = std::env::var("ROBONIX_LIAISON_PORT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(50081);
    let listen_addr: std::net::SocketAddr = format!("0.0.0.0:{listen_port}").parse()?;
    let advertised = format!("127.0.0.1:{listen_port}");

    log::info!("connecting to Atlas at {atlas_http}");
    let mut sdk =
        RobonixClient::connect_with_retry(&atlas_http, 10, std::time::Duration::from_secs(2))
            .await?;
    sdk.register_node(LIAISON_NODE_ID, "robonix/srv/liaison", "service", "")
        .await?;
    sdk.declare_interface_full(
        LIAISON_NODE_ID,
        "liaison",
        vec!["grpc".to_string()],
        serde_json::json!({ "endpoint": advertised }).to_string(),
        listen_port as u32,
        "robonix/srv/liaison",
    )
    .await?;
    log::info!("registered as '{LIAISON_NODE_ID}', SrvLiaison gRPC on :{listen_port}");
    eprintln!("robonix-liaison ready on :{listen_port}  (pilot={pilot_http})");

    let pipeline = Arc::new(LiaisonPipeline::new(pilot_http));
    // Voice orchestration discovers ASR / mic / voiceprint / TTS / speaker via Atlas.
    let sdk = Arc::new(Mutex::new(sdk));

    let source = std::env::var("ROBONIX_LIAISON_SOURCE").unwrap_or_default();
    let text_handle: Option<tokio::task::JoinHandle<Result<()>>> = if source == "text" {
        log::info!("activating stdin text loop (headless mode)");
        Some(tokio::spawn(run_text_loop(Arc::clone(&pipeline))))
    } else {
        None
    };

    let svc = LiaisonServiceImpl {
        pipeline,
        sdk: Arc::clone(&sdk),
    };
    let server = tonic::transport::Server::builder()
        .add_service(SrvLiaisonServer::new(svc))
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

// ── helpers ───────────────────────────────────────────────────────────────────

fn resolve_endpoint(vars: &[&str], default: &str) -> String {
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
    use robonix_interfaces::pilot::Task;

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
            user_id: String::new(),
        };
        ensure_user_id(&mut t);
        assert!(t.user_id.starts_with("local:"));
        let v: serde_json::Value = serde_json::from_str(&t.context_json).unwrap();
        assert_eq!(v["modality"], "text");
        assert_eq!(v["user_id"], t.user_id);
    }

    #[test]
    fn ensure_user_id_preserves_caller_value() {
        let mut t = Task {
            task_id: "t".into(),
            session_id: "s".into(),
            source: 1,
            text: "hi".into(),
            audio_data: vec![],
            context_json: r#"{"foo":"bar"}"#.into(),
            timestamp_ms: 0,
            user_id: "voice:alice".into(),
        };
        ensure_user_id(&mut t);
        assert_eq!(t.user_id, "voice:alice");
        let v: serde_json::Value = serde_json::from_str(&t.context_json).unwrap();
        assert_eq!(v["modality"], "voice");
        assert_eq!(v["user_id"], "voice:alice");
        assert_eq!(v["foo"], "bar");
    }
}
