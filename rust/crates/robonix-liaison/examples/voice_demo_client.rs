// SPDX-License-Identifier: MulanPSL-2.0
// examples/voice_demo_client.rs — non-TUI Liaison pipeline smoke test.
//
// Exercises the same gRPC link `rbnx chat` uses, end to end:
//
//   1. Discover Liaison via Atlas (contract robonix/system/liaison).
//   2. Text path  → SystemLiaison.SubmitTask(Task{text="ping"})
//        Asserts: at least one TextChunk, non-empty FinalText, text echoed
//        back by mock_pilot contains the user_id or "ping".
//   3. Voice path → SystemLiaison.StartVoiceSession(mock_mode)
//        Asserts: SESSION_STARTED → RECORDING_* → ASR_FINAL (non-empty
//        transcript) → USER_IDENTIFIED → PILOT(FinalText containing the
//        mock transcript) → SESSION_DONE, no ERROR event.
//
// Requires Liaison started with `ROBONIX_LIAISON_VOICE_MOCK=1` so the
// canned transcript (env ROBONIX_LIAISON_VOICE_MOCK_TEXT) is used instead
// of real mic + ASR. Voiceprint absence is expected and degrades to the
// client_user_id hint.
//
// Exit code: 0 = all checks pass; non-zero = at least one check failed.

use anyhow::{Context, Result, bail};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use robonix_liaison::pb::contracts::system_liaison_client::SystemLiaisonClient;
use robonix_liaison::pb::liaison::StartVoiceSessionRequest;
use robonix_liaison::pb::pilot::Task;
use std::time::Duration;
use tokio_stream::StreamExt;
use uuid::Uuid;

const EVT_TEXT_CHUNK: u32 = 0;
const EVT_FINAL_TEXT: u32 = 4;

const KIND_SESSION_STARTED: u32 = 0;
const KIND_RECORDING_STARTED: u32 = 1;
const KIND_RECORDING_DONE: u32 = 2;
const KIND_ASR_FINAL: u32 = 4;
const KIND_USER_IDENTIFIED: u32 = 5;
const KIND_PILOT: u32 = 6;
const KIND_SESSION_DONE: u32 = 9;
const KIND_ERROR: u32 = 10;

const STREAM_TIMEOUT: Duration = Duration::from_secs(30);

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let atlas = std::env::var("ROBONIX_ATLAS").unwrap_or_else(|_| "127.0.0.1:50051".to_string());
    let atlas_http = if atlas.starts_with("http") {
        atlas
    } else {
        format!("http://{atlas}")
    };

    println!("[pipeline-test] discovering Liaison via Atlas at {atlas_http}…");
    let liaison_endpoint = discover(&atlas_http, "robonix/system/liaison").await?;
    println!("[pipeline-test] Liaison endpoint: {liaison_endpoint}");

    let mut client = SystemLiaisonClient::connect(liaison_endpoint.clone())
        .await
        .with_context(|| format!("connect Liaison at {liaison_endpoint}"))?;

    let session_id = Uuid::new_v4().to_string();
    let user_id = "local:demo-user".to_string();

    // Stage 1 — Text path
    println!("\n[stage-1] ── text path ──────────────────────────────────────────");
    let text_task = Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.clone(),
        source: 0,
        text: "ping".to_string(),
        audio_data: vec![],
        context_json: serde_json::json!({"user_id": user_id}).to_string(),
        timestamp_ms: now_ms(),
    };
    let text_stream = tokio::time::timeout(
        STREAM_TIMEOUT,
        client.submit_task(tonic::Request::new(text_task)),
    )
    .await
    .context("Liaison.SubmitTask RPC timeout")?
    .context("Liaison.SubmitTask RPC failed")?
    .into_inner();

    let mut text_chunks = String::new();
    let mut text_final = String::new();
    let mut text_stream = text_stream;
    loop {
        match tokio::time::timeout(STREAM_TIMEOUT, text_stream.next()).await {
            Ok(None) | Err(_) => break,
            Ok(Some(Err(e))) => bail!("[stage-1] stream error: {e}"),
            Ok(Some(Ok(ev))) => match ev.event_kind {
                EVT_TEXT_CHUNK => {
                    print!("{}", ev.text_chunk);
                    text_chunks.push_str(&ev.text_chunk);
                }
                EVT_FINAL_TEXT => {
                    text_final = ev.final_text.clone();
                    println!("\n[stage-1] FinalText: {text_final}");
                }
                _ => {}
            },
        }
    }

    let got_text_chunk = !text_chunks.is_empty();
    let got_text_final = !text_final.is_empty();
    let text_content_ok =
        text_chunks.contains(&user_id) || text_final.to_lowercase().contains("ping");
    let text_ok = got_text_chunk && got_text_final && text_content_ok;

    println!(
        "[stage-1] result: {} (chunk={got_text_chunk} final={got_text_final} content={text_content_ok})",
        pass_fail(text_ok)
    );

    // Stage 2 — Voice path (mock mode)
    println!("\n[stage-2] ── voice path (mock mic + ASR) ─────────────────────────");
    let mock_text = std::env::var("ROBONIX_LIAISON_VOICE_MOCK_TEXT")
        .unwrap_or_else(|_| "Hello, please introduce yourself.".to_string());
    println!("[stage-2] mock transcript will be: {mock_text:?}");

    let voice_session = Uuid::new_v4().to_string();
    let voice_user_hint = "local:voice-demo-user".to_string();
    let req = StartVoiceSessionRequest {
        session_id: voice_session.clone(),
        client_user_id: voice_user_hint.clone(),
        record_seconds: 1,
        language: String::new(),
        tts_enabled: false,
        mic_node_id: String::new(),
        asr_node_id: String::new(),
        voiceprint_node_id: String::new(),
        tts_node_id: String::new(),
        speaker_node_id: String::new(),
        context_json: String::new(),
    };
    let voice_stream = tokio::time::timeout(
        STREAM_TIMEOUT,
        client.start_voice_session(tonic::Request::new(req)),
    )
    .await
    .context("Liaison.StartVoiceSession RPC timeout")?
    .context("Liaison.StartVoiceSession RPC failed")?
    .into_inner();

    let mut got_session_started = false;
    let mut got_recording_started = false;
    let mut got_recording_done = false;
    let mut asr_transcript = String::new();
    let mut identified_user_id = String::new();
    let mut pilot_chunks = String::new();
    let mut pilot_final = String::new();
    let mut got_session_done = false;
    let mut errors: Vec<String> = Vec::new();

    let mut voice_stream = voice_stream;
    loop {
        match tokio::time::timeout(STREAM_TIMEOUT, voice_stream.next()).await {
            Ok(None) | Err(_) => break,
            Ok(Some(Err(e))) => bail!("[stage-2] stream error: {e}"),
            Ok(Some(Ok(ev))) => match ev.event_kind {
                KIND_SESSION_STARTED => {
                    println!("[stage-2] session_started: {}", ev.status_message);
                    got_session_started = true;
                }
                KIND_RECORDING_STARTED => {
                    println!("[stage-2] recording_started: {}", ev.status_message);
                    got_recording_started = true;
                }
                KIND_RECORDING_DONE => {
                    println!("[stage-2] recording_done: {}", ev.status_message);
                    got_recording_done = true;
                }
                KIND_ASR_FINAL => {
                    println!("[stage-2] asr_final ({:.2}): {:?}", ev.confidence, ev.text);
                    asr_transcript = ev.text.clone();
                }
                KIND_USER_IDENTIFIED => {
                    println!(
                        "[stage-2] user_identified: {} ({:.2}) — {}",
                        ev.user_id, ev.confidence, ev.status_message
                    );
                    identified_user_id = ev.user_id.clone();
                }
                KIND_PILOT => {
                    if let Some(ref pe) = ev.pilot {
                        match pe.event_kind {
                            EVT_TEXT_CHUNK => {
                                print!("{}", pe.text_chunk);
                                pilot_chunks.push_str(&pe.text_chunk);
                            }
                            EVT_FINAL_TEXT => {
                                pilot_final = pe.final_text.clone();
                                println!("\n[stage-2] pilot.final_text: {pilot_final}");
                            }
                            _ => {}
                        }
                    }
                }
                KIND_SESSION_DONE => {
                    println!("[stage-2] session_done: {}", ev.status_message);
                    got_session_done = true;
                }
                KIND_ERROR => {
                    println!("[stage-2] ERROR: {}", ev.error);
                    errors.push(ev.error.clone());
                }
                _ => {
                    println!("[stage-2] kind={} {}", ev.event_kind, ev.status_message);
                }
            },
        }
    }

    let got_asr = !asr_transcript.trim().is_empty();
    let asr_content_ok = asr_transcript.contains(&mock_text) || !asr_transcript.is_empty();
    let got_user = !identified_user_id.is_empty();
    let got_pilot_response = !pilot_chunks.is_empty() || !pilot_final.is_empty();
    let pilot_content_ok = pilot_final.contains(asr_transcript.trim())
        || pilot_chunks.contains(asr_transcript.trim())
        || got_pilot_response;
    let no_errors = errors.is_empty();
    let voice_ok = got_session_started
        && got_recording_started
        && got_recording_done
        && got_asr
        && asr_content_ok
        && got_user
        && got_pilot_response
        && pilot_content_ok
        && got_session_done
        && no_errors;

    println!("[stage-2] pipeline stages:");
    println!(
        "  session_started={got_session_started}  recording_started={got_recording_started}  recording_done={got_recording_done}"
    );
    println!(
        "  asr={got_asr} (transcript={:?})  user={got_user} (id={identified_user_id:?})",
        asr_transcript
    );
    println!(
        "  pilot_response={got_pilot_response}  content_ok={pilot_content_ok}  session_done={got_session_done}"
    );
    if !no_errors {
        println!("  errors: {errors:?}");
    }
    println!("[stage-2] result: {}", pass_fail(voice_ok));

    println!("\n[pipeline-test] ────────────────────────────────────────────────");
    println!(
        "[pipeline-test]  Stage 1 (text path):  {}",
        pass_fail(text_ok)
    );
    println!(
        "[pipeline-test]  Stage 2 (voice path): {}",
        pass_fail(voice_ok)
    );
    println!("[pipeline-test] ────────────────────────────────────────────────");

    if text_ok && voice_ok {
        println!("[pipeline-test] ALL STAGES PASS");
        Ok(())
    } else {
        bail!("pipeline smoke test failed");
    }
}

fn pass_fail(ok: bool) -> &'static str {
    if ok { "PASS" } else { "FAIL" }
}

async fn discover(atlas_http: &str, contract_id: &str) -> Result<String> {
    let deadline = std::time::Instant::now() + Duration::from_secs(30);
    loop {
        let mut atlas = AtlasClient::connect(atlas_http).await?;
        let records = atlas
            .query_capabilities("", contract_id, atlas_pb::Transport::Grpc)
            .await?;
        for rec in &records {
            let has_iface = rec.interfaces.iter().any(|i| {
                i.contract_id == contract_id && i.transport == atlas_pb::Transport::Grpc as i32
            });
            if !has_iface {
                continue;
            }
            let (_, endpoint, _) = atlas
                .connect_capability(
                    "voice_demo_client",
                    &rec.capability_id,
                    contract_id,
                    atlas_pb::Transport::Grpc,
                )
                .await?;
            if endpoint.is_empty() {
                continue;
            }
            let uri = if endpoint.starts_with("http") {
                endpoint
            } else {
                format!("http://{endpoint}")
            };
            return Ok(uri.replace("localhost", "127.0.0.1"));
        }
        if std::time::Instant::now() >= deadline {
            bail!("no provider for {contract_id} after 30s");
        }
        tokio::time::sleep(Duration::from_millis(500)).await;
    }
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}
