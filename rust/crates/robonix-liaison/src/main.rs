// SPDX-License-Identifier: MulanPSL-2.0
// robonix-liaison — unified user-facing input layer
//
// ── Design ───────────────────────────────────────────────────────────────────
//
//  All input modalities ultimately produce the same `Intent` and receive the
//  same `PilotEvent` stream.  The liaison gRPC facade is contract `SysRuntimeLiaison`
//  (`robonix_contracts.proto`); speech is the only modality that needs pre/post processing:
//
//
//  Key principle: liaison itself does NOT contain ASR/TTS logic.  ASR and TTS
//  are Atlas-registered system nodes (robonix/sys/speech/asr,
//  robonix/sys/speech/tts) discovered and called via gRPC — the same pattern
//  as the VLM service in Pilot.  The `Recorder` and `Speaker` traits abstract
//  platform audio I/O (ALSA, PulseAudio, CoreAudio …) from the service calls.
//
// ── Runtime topology ─────────────────────────────────────────────────────────
//
//   Always started:  SysRuntimeLiaison gRPC server on ROBONIX_LIAISON_PORT
//   Optional module: SpeechBridge   (set ROBONIX_LIAISON_SPEECH=1)
//   Fallback stdin:  text loop      (set ROBONIX_LIAISON_SOURCE=text,
//                                    useful for headless / pipe mode)

use robonix_interfaces::{contracts, pilot};

use anyhow::Result;
use contracts::sys_runtime_liaison_server::{SysRuntimeLiaison, SysRuntimeLiaisonServer};
use contracts::sys_runtime_pilot_client::SysRuntimePilotClient;
use pilot::{Intent, PilotEvent};
use robonix_sdk::RobonixClient;
use std::sync::Arc;
use tokio::sync::mpsc;
use tokio_stream::{StreamExt, wrappers::ReceiverStream};
use tonic::{Request, Response, Status};
use uuid::Uuid;

const LIAISON_NODE_ID: &str = "com.robonix.runtime.liaison";

/// `lib/pilot/msg/ Intent.msg` source: TEXT=0 AUDIO=1 API=2
const INTENT_SOURCE_TEXT: u32 = 0;
/// `lib/pilot/msg/PilotEvent.msg` event_kind
const EVT_TEXT_CHUNK: u32 = 0;
const EVT_TASK_GRAPH: u32 = 1;
const EVT_BATCH_RESULT: u32 = 2;
const EVT_STATUS: u32 = 3;
const EVT_FINAL_TEXT: u32 = 4;

// ── LiaisonPipeline ───────────────────────────────────────────────────────────
//
// Core: forward an Intent to Pilot (`SysRuntimePilot.Stream`), return the event channel.
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

    /// Forward `intent` to Pilot.  Returns a channel that yields `PilotEvent`s.
    /// A background task pumps the underlying gRPC stream into the channel.
    pub async fn handle_intent(
        &self,
        intent: Intent,
    ) -> Result<mpsc::Receiver<Result<PilotEvent, Status>>> {
        let mut client = SysRuntimePilotClient::connect(self.pilot_endpoint.clone()).await?;
        let mut grpc = client.stream(Request::new(intent)).await?.into_inner();
        let (tx, rx) = mpsc::channel(64);
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

// ── SysRuntimeLiaison gRPC impl ─────────────────────────────────────────────
//
// All clients — rbnx chat, mobile app, GUI, and the SpeechBridge below — use
// this contract service.  There is no special-cased input path per modality.
// Interrupt / turn-cancel is expressed as an `Intent` with `context_json` `{"abort_turn":true}`
// end-to-end (Pilot handles it); there is no separate liaison RPC.

struct LiaisonServiceImpl {
    pipeline: Arc<LiaisonPipeline>,
}

#[tonic::async_trait]
impl SysRuntimeLiaison for LiaisonServiceImpl {
    type StreamStream = ReceiverStream<Result<PilotEvent, Status>>;

    async fn stream(
        &self,
        request: Request<Intent>,
    ) -> Result<Response<Self::StreamStream>, Status> {
        let intent = request.into_inner();
        let rx = self
            .pipeline
            .handle_intent(intent)
            .await
            .map_err(|e| Status::unavailable(format!("Pilot unreachable: {e:#}")))?;
        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

// ── Stdin text loop ─────────────────────────────────────────────────────────
//
// Convenience fallback for headless / pipe usage.
// Activated by ROBONIX_LIAISON_SOURCE=text.
// `rbnx chat` (gRPC TUI) is the preferred interactive interface.

async fn drain_session_end(pipeline: &LiaisonPipeline, session_id: &str) {
    let intent = Intent {
        intent_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: INTENT_SOURCE_TEXT,
        text: String::new(),
        audio_data: vec![],
        context_json: r#"{"session_end":true}"#.to_string(),
        timestamp_ms: now_ms(),
    };
    match pipeline.handle_intent(intent).await {
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

        let intent = Intent {
            intent_id: Uuid::new_v4().to_string(),
            session_id: session_id.clone(),
            source: INTENT_SOURCE_TEXT,
            text,
            audio_data: vec![],
            context_json: String::new(),
            timestamp_ms: now_ms(),
        };

        match pipeline.handle_intent(intent).await {
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

    // Register with Atlas.
    log::info!("connecting to Atlas at {atlas_http}");
    let mut sdk =
        RobonixClient::connect_with_retry(&atlas_http, 10, std::time::Duration::from_secs(2))
            .await?;
    sdk.register_node(
        LIAISON_NODE_ID,
        "robonix/sys/runtime/liaison",
        "service",
        "",
    )
    .await?;
    sdk.declare_interface_full(
        LIAISON_NODE_ID,
        "liaison",
        vec!["grpc".to_string()],
        serde_json::json!({ "endpoint": advertised }).to_string(),
        listen_port as u32,
        "robonix/sys/runtime/liaison",
    )
    .await?;
    log::info!("registered as '{LIAISON_NODE_ID}', SysRuntimeLiaison gRPC on :{listen_port}");
    eprintln!("robonix-liaison ready on :{listen_port}  (pilot={pilot_http})");

    let pipeline = Arc::new(LiaisonPipeline::new(pilot_http));

    // ── Optional: SpeechBridge ────────────────────────────────────────────────
    // TODO(speech-owner): uncomment once SpeechBridge is implemented.
    //
    // if std::env::var("ROBONIX_LIAISON_SPEECH").as_deref() == Ok("1") {
    //     let bridge = SpeechBridge { sdk: Arc::new(Mutex::new(sdk)), pipeline: Arc::clone(&pipeline) };
    //     tokio::spawn(bridge.run());
    // }

    // ── Optional: stdin text loop (headless / pipe fallback) ──────────────────
    let source = std::env::var("ROBONIX_LIAISON_SOURCE").unwrap_or_default();
    let text_handle: Option<tokio::task::JoinHandle<Result<()>>> = if source == "text" {
        log::info!("activating stdin text loop (headless mode)");
        Some(tokio::spawn(run_text_loop(Arc::clone(&pipeline))))
    } else {
        None
    };

    // ── gRPC server (always running) ──────────────────────────────────────────
    let svc = LiaisonServiceImpl { pipeline };
    let server = tonic::transport::Server::builder()
        .add_service(SysRuntimeLiaisonServer::new(svc))
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
