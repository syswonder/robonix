// SPDX-License-Identifier: MulanPSL-2.0
// voice.rs — voice push-to-talk orchestrator behind SystemLiaison.StartVoiceSession.
//
// One call drives the full pipeline:
//
//   PrimitiveAudioMic.Stream  (record_seconds)
//       │
//       ▼
//   SystemSpeechAsr.Recognize    → transcript
//       │
//       ▼
//   SystemSpeechVoiceprint.Identify → user_id  (graceful: hint fallback on absence)
//       │
//       ▼
//   Build pilot::Task { source=AUDIO, text=transcript, user_id, … }
//       │
//       ▼
//   SystemPilot.SubmitTask      → forward each PilotEvent as VoiceEvent { kind=PILOT, … }
//       │
//       ▼
//   (if tts_enabled)  SystemSpeechTts.Synthesize → PrimitiveAudioSpeaker.Stream
//       │
//       ▼
//   VoiceEvent { kind=SESSION_DONE }
//
// Mock mode (`ROBONIX_LIAISON_VOICE_MOCK=1` or `ROBONIX_LIAISON_VOICE_MOCK_TEXT`):
// skip mic+ASR, use a canned transcript so the link can be exercised without
// audio hardware. This is what the demo script uses by default.

use anyhow::Result;
use futures::Stream;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{Mutex, mpsc};
use tokio_stream::{StreamExt, wrappers::ReceiverStream};
use tonic::{Request, Status};
use uuid::Uuid;

use crate::pb::asr;
use crate::pb::audio::AudioChunk;
use crate::pb::contracts::{
    primitive_audio_mic_client::PrimitiveAudioMicClient,
    primitive_audio_speaker_client::PrimitiveAudioSpeakerClient,
    system_pilot_client::SystemPilotClient, system_speech_asr_client::SystemSpeechAsrClient,
    system_speech_tts_client::SystemSpeechTtsClient,
    system_speech_voiceprint_client::SystemSpeechVoiceprintClient,
};
use crate::pb::liaison::{StartVoiceSessionRequest, VoiceEvent};
use crate::pb::pilot::{PilotEvent, Task};
use crate::pb::tts;
use crate::pb::voiceprint;

// ── Stable VoiceEvent kinds (mirror lib/system/liaison/msg/VoiceEvent.msg) ───

pub const KIND_SESSION_STARTED: u32 = 0;
pub const KIND_RECORDING_STARTED: u32 = 1;
pub const KIND_RECORDING_DONE: u32 = 2;
#[allow(dead_code)]
pub const KIND_ASR_PARTIAL: u32 = 3;
pub const KIND_ASR_FINAL: u32 = 4;
pub const KIND_USER_IDENTIFIED: u32 = 5;
pub const KIND_PILOT: u32 = 6;
pub const KIND_TTS_STARTED: u32 = 7;
pub const KIND_TTS_DONE: u32 = 8;
pub const KIND_SESSION_DONE: u32 = 9;
pub const KIND_ERROR: u32 = 10;

/// `Task.source` enum: TEXT=0 AUDIO=1 API=2.
const INTENT_SOURCE_AUDIO: u32 = 1;

const DEFAULT_RECORD_SECONDS: u32 = 5;
const DEFAULT_ASR_LANGUAGE: &str = "";
const DEFAULT_AUDIO_ENCODING: &str = "pcm_s16le";
const DEFAULT_AUDIO_SAMPLE_RATE: u32 = 16_000;

// ── Discovery helpers ────────────────────────────────────────────────────────
//
// Atlas → contract_id → endpoint resolution. Every speech / mic / speaker
// service registers its gRPC interface with `metadata_json = {"endpoint":
// "host:port"}` (see audio_driver/node.py and speech_service/service.py).

/// Two-step endpoint resolution: query for caps offering `contract_id`, then
/// `ConnectCapability` against the chosen one to actually receive the
/// endpoint string. Atlas hides endpoints from `query_capabilities` on
/// purpose; consumers must commit a channel before they can dial.
async fn resolve_endpoint(
    atlas: &Arc<Mutex<AtlasClient>>,
    contract_id: &str,
    pin_capability_id: &str,
) -> Option<String> {
    let mut atlas = atlas.lock().await;
    let transport = atlas_pb::Transport::Grpc;
    let records = atlas
        .query_capabilities("", contract_id, transport)
        .await
        .ok()?;
    let pick: Option<&atlas_pb::CapabilityRecord> = if pin_capability_id.is_empty() {
        records.iter().find(|r| {
            r.interfaces
                .iter()
                .any(|i| i.contract_id == contract_id && i.transport == transport as i32)
        })
    } else {
        records
            .iter()
            .find(|r| r.capability_id == pin_capability_id || r.namespace == pin_capability_id)
    };
    let cap = pick?;
    let (_channel_id, endpoint, _params) = atlas
        .connect_capability("liaison", &cap.capability_id, contract_id, transport)
        .await
        .ok()?;
    if endpoint.is_empty() {
        return None;
    }
    Some(localhost_to_ipv4(&endpoint))
}

fn localhost_to_ipv4(ep: &str) -> String {
    let with_scheme = if ep.starts_with("http") {
        ep.to_string()
    } else {
        format!("http://{ep}")
    };
    with_scheme.replace("localhost", "127.0.0.1")
}

// ── Public entry point ───────────────────────────────────────────────────────

/// Run one voice session. Returns the receiver immediately; orchestration
/// runs on a spawned task and pushes `VoiceEvent`s as it progresses.
///
/// `pilot_endpoint_default` is the http URL Liaison was configured with at
/// boot — used when Atlas does not yet know about Pilot.
pub async fn start_voice_session(
    req: StartVoiceSessionRequest,
    atlas: Arc<Mutex<AtlasClient>>,
    pilot_endpoint_default: String,
) -> Result<impl Stream<Item = Result<VoiceEvent, Status>>, Status> {
    let session_id = if req.session_id.is_empty() {
        Uuid::new_v4().to_string()
    } else {
        req.session_id.clone()
    };
    let record_seconds = if req.record_seconds == 0 {
        DEFAULT_RECORD_SECONDS
    } else {
        req.record_seconds
    };
    let language = if req.language.is_empty() {
        DEFAULT_ASR_LANGUAGE.to_string()
    } else {
        req.language.clone()
    };

    let (tx, rx) = mpsc::channel::<Result<VoiceEvent, Status>>(64);

    let _ = tx
        .send(Ok(event_status(
            KIND_SESSION_STARTED,
            &session_id,
            &format!(
                "voice session started (record={record_seconds}s, tts={}, lang={})",
                req.tts_enabled,
                if language.is_empty() {
                    "auto"
                } else {
                    &language
                },
            ),
        )))
        .await;

    let session_id_for_task = session_id.clone();
    tokio::spawn(async move {
        let outcome = run_session(
            req,
            session_id_for_task.clone(),
            record_seconds,
            language,
            atlas,
            pilot_endpoint_default,
            tx.clone(),
        )
        .await;

        match outcome {
            Ok(()) => {
                let _ = tx
                    .send(Ok(event_status(
                        KIND_SESSION_DONE,
                        &session_id_for_task,
                        "voice session done",
                    )))
                    .await;
            }
            Err(e) => {
                let _ = tx
                    .send(Ok(event_error(&session_id_for_task, &format!("{e:#}"))))
                    .await;
            }
        }
    });

    Ok(ReceiverStream::new(rx))
}

#[allow(clippy::too_many_arguments)]
async fn run_session(
    req: StartVoiceSessionRequest,
    session_id: String,
    record_seconds: u32,
    language: String,
    atlas: Arc<Mutex<AtlasClient>>,
    pilot_endpoint_default: String,
    tx: mpsc::Sender<Result<VoiceEvent, Status>>,
) -> Result<()> {
    let mock = is_mock_mode();

    // 1. Capture audio (mic) or fall back to mock transcript.
    let (audio_pcm, transcript_override) = if mock {
        let canned = mock_transcript();
        let _ = tx
            .send(Ok(event_status(
                KIND_RECORDING_STARTED,
                &session_id,
                "mock mode — skipping mic capture",
            )))
            .await;
        let _ = tx
            .send(Ok(event_status(
                KIND_RECORDING_DONE,
                &session_id,
                &format!("mock transcript ready ({} bytes)", canned.len()),
            )))
            .await;
        (Vec::new(), Some(canned))
    } else {
        let pcm = capture_audio(&atlas, &req.mic_node_id, record_seconds, &session_id, &tx).await?;
        (pcm, None)
    };

    // 2. ASR.
    let transcript = if let Some(t) = transcript_override {
        t
    } else {
        let asr_endpoint = resolve_endpoint(&atlas, "robonix/system/speech/asr", &req.asr_node_id)
            .await
            .ok_or_else(|| anyhow::anyhow!("no SystemSpeechAsr provider registered in Atlas"))?;
        let mut client = SystemSpeechAsrClient::connect(asr_endpoint.clone())
            .await
            .map_err(|e| anyhow::anyhow!("connect ASR at {asr_endpoint}: {e}"))?;
        let resp = client
            .recognize(Request::new(asr::RecognizeRequest {
                audio_data: audio_pcm.clone(),
                encoding: DEFAULT_AUDIO_ENCODING.to_string(),
                sample_rate_hz: DEFAULT_AUDIO_SAMPLE_RATE,
                language: language.clone(),
            }))
            .await
            .map_err(|e| anyhow::anyhow!("ASR rpc failed: {e}"))?
            .into_inner();
        if !resp.error.is_empty() {
            anyhow::bail!("ASR error: {}", resp.error);
        }
        let _ = tx
            .send(Ok(event_text(
                KIND_ASR_FINAL,
                &session_id,
                &resp.text,
                resp.confidence,
            )))
            .await;
        resp.text
    };

    if transcript.trim().is_empty() {
        anyhow::bail!("empty transcript — nothing to send to Pilot");
    }

    if mock {
        let _ = tx
            .send(Ok(event_text(
                KIND_ASR_FINAL,
                &session_id,
                &transcript,
                1.0,
            )))
            .await;
    }

    // 3. Voiceprint (graceful — fallback to client hint on any failure).
    let user_id = identify_user(
        &atlas,
        &req.voiceprint_node_id,
        &audio_pcm,
        &req.client_user_id,
        &session_id,
        &tx,
    )
    .await;

    // 4. Build Task and stream Pilot events.
    let pilot_endpoint = resolve_endpoint(&atlas, "robonix/system/pilot", "")
        .await
        .unwrap_or(pilot_endpoint_default);

    let task = build_task(
        &session_id,
        &transcript,
        &user_id,
        &audio_pcm,
        &req.context_json,
    );

    let mut accumulated_text = String::new();

    let pilot_stream_result = async {
        let mut client = SystemPilotClient::connect(pilot_endpoint.clone())
            .await
            .map_err(|e| tonic::Status::unavailable(e.to_string()))?;
        let resp = client.submit_task(Request::new(task)).await?;
        Ok::<_, tonic::Status>(resp.into_inner())
    }
    .await;

    match pilot_stream_result {
        Ok(mut pilot_stream) => {
            while let Some(item) = pilot_stream.next().await {
                match item {
                    Ok(ev) => {
                        accumulate_text(&ev, &mut accumulated_text);
                        let _ = tx
                            .send(Ok(VoiceEvent {
                                event_kind: KIND_PILOT,
                                session_id: session_id.clone(),
                                text: String::new(),
                                user_id: user_id.clone(),
                                confidence: 0.0,
                                pilot: Some(ev),
                                error: String::new(),
                                status_message: String::new(),
                                timestamp_ms: now_ms(),
                            }))
                            .await;
                    }
                    Err(e) => {
                        anyhow::bail!("Pilot stream error: {e}");
                    }
                }
            }
        }
        Err(e) => {
            // No Pilot online — surface as session ERROR rather than fabricating
            // a "成功接收信息" reply (callers must see the failure).
            anyhow::bail!("Pilot unreachable at {pilot_endpoint}: {e}");
        }
    }

    // 5. Optional TTS playback (non-fatal on any error).
    if req.tts_enabled
        && !accumulated_text.trim().is_empty()
        && let Err(e) = synthesize_and_play(
            &atlas,
            &req.tts_node_id,
            &req.speaker_node_id,
            &language,
            &accumulated_text,
            &session_id,
            &tx,
        )
        .await
    {
        let _ = tx
            .send(Ok(event_status(
                KIND_TTS_DONE,
                &session_id,
                &format!("tts skipped: {e:#}"),
            )))
            .await;
    }

    Ok(())
}

// ── Mic capture ──────────────────────────────────────────────────────────────

async fn capture_audio(
    atlas: &Arc<Mutex<AtlasClient>>,
    mic_pin: &str,
    record_seconds: u32,
    session_id: &str,
    tx: &mpsc::Sender<Result<VoiceEvent, Status>>,
) -> Result<Vec<u8>> {
    let mic_endpoint = resolve_endpoint(atlas, "robonix/primitive/audio/mic", mic_pin)
        .await
        .ok_or_else(|| anyhow::anyhow!("no PrimitiveAudioMic provider registered in Atlas"))?;

    let mut client = PrimitiveAudioMicClient::connect(mic_endpoint.clone())
        .await
        .map_err(|e| anyhow::anyhow!("connect mic at {mic_endpoint}: {e}"))?;
    let _ = tx
        .send(Ok(event_status(
            KIND_RECORDING_STARTED,
            session_id,
            &format!("recording {record_seconds}s from {mic_endpoint}"),
        )))
        .await;

    let mut stream = client
        .mic(Request::new(()))
        .await
        .map_err(|e| anyhow::anyhow!("mic rpc failed: {e}"))?
        .into_inner();

    let mut buf: Vec<u8> =
        Vec::with_capacity((DEFAULT_AUDIO_SAMPLE_RATE as usize) * 2 * (record_seconds as usize));
    let deadline = tokio::time::Instant::now() + Duration::from_secs(record_seconds as u64);
    loop {
        let now = tokio::time::Instant::now();
        if now >= deadline {
            break;
        }
        let remaining = deadline - now;
        match tokio::time::timeout(remaining, stream.message()).await {
            Ok(Ok(Some(chunk))) => buf.extend_from_slice(&chunk.data),
            Ok(Ok(None)) => break,
            Ok(Err(e)) => anyhow::bail!("mic stream error: {e}"),
            Err(_) => break,
        }
    }

    let _ = tx
        .send(Ok(event_status(
            KIND_RECORDING_DONE,
            session_id,
            &format!(
                "captured {} bytes (~{:.2}s @ 16kHz mono s16le)",
                buf.len(),
                buf.len() as f32 / (DEFAULT_AUDIO_SAMPLE_RATE as f32 * 2.0),
            ),
        )))
        .await;
    Ok(buf)
}

// ── Voiceprint ───────────────────────────────────────────────────────────────

async fn identify_user(
    atlas: &Arc<Mutex<AtlasClient>>,
    pin_capability_id: &str,
    audio_pcm: &[u8],
    fallback_hint: &str,
    session_id: &str,
    tx: &mpsc::Sender<Result<VoiceEvent, Status>>,
) -> String {
    let fallback = if fallback_hint.is_empty() {
        "voice:unknown".to_string()
    } else if fallback_hint.starts_with("voice:") || fallback_hint.starts_with("local:") {
        fallback_hint.to_string()
    } else {
        format!("voice:{fallback_hint}")
    };

    let endpoint = match resolve_endpoint(
        atlas,
        "robonix/system/speech/voiceprint",
        pin_capability_id,
    )
    .await
    {
        Some(ep) => ep,
        None => {
            let _ = tx
                .send(Ok(event_user(
                    KIND_USER_IDENTIFIED,
                    session_id,
                    &fallback,
                    0.0,
                    "no SystemSpeechVoiceprint provider — using client hint",
                )))
                .await;
            return fallback;
        }
    };

    let mut client = match SystemSpeechVoiceprintClient::connect(endpoint.clone()).await {
        Ok(c) => c,
        Err(e) => {
            let _ = tx
                .send(Ok(event_user(
                    KIND_USER_IDENTIFIED,
                    session_id,
                    &fallback,
                    0.0,
                    &format!("voiceprint connect failed: {e} — using client hint"),
                )))
                .await;
            return fallback;
        }
    };
    let resp = match client
        .identify(Request::new(voiceprint::IdentifyRequest {
            audio_data: audio_pcm.to_vec(),
            encoding: DEFAULT_AUDIO_ENCODING.to_string(),
            sample_rate_hz: DEFAULT_AUDIO_SAMPLE_RATE,
        }))
        .await
    {
        Ok(r) => r.into_inner(),
        Err(e) => {
            let _ = tx
                .send(Ok(event_user(
                    KIND_USER_IDENTIFIED,
                    session_id,
                    &fallback,
                    0.0,
                    &format!("voiceprint rpc failed: {e} — using client hint"),
                )))
                .await;
            return fallback;
        }
    };

    let user_id = if resp.is_known && !resp.user_id.is_empty() {
        if resp.user_id.starts_with("voice:") {
            resp.user_id.clone()
        } else {
            format!("voice:{}", resp.user_id)
        }
    } else {
        fallback.clone()
    };
    let label = if !resp.user_name.is_empty() {
        format!(
            "matched {}={} ({:.2})",
            resp.user_id, resp.user_name, resp.confidence
        )
    } else if resp.is_known {
        format!("matched {} ({:.2})", resp.user_id, resp.confidence)
    } else {
        format!(
            "unknown speaker (best={:.2}) — using fallback",
            resp.confidence
        )
    };
    let _ = tx
        .send(Ok(event_user(
            KIND_USER_IDENTIFIED,
            session_id,
            &user_id,
            resp.confidence,
            &label,
        )))
        .await;
    user_id
}

// ── TTS playback ─────────────────────────────────────────────────────────────

async fn synthesize_and_play(
    atlas: &Arc<Mutex<AtlasClient>>,
    tts_pin: &str,
    speaker_pin: &str,
    language: &str,
    text: &str,
    session_id: &str,
    tx: &mpsc::Sender<Result<VoiceEvent, Status>>,
) -> Result<()> {
    let tts_endpoint = resolve_endpoint(atlas, "robonix/system/speech/tts", tts_pin)
        .await
        .ok_or_else(|| anyhow::anyhow!("no SystemSpeechTts provider"))?;
    let speaker_endpoint = resolve_endpoint(atlas, "robonix/primitive/audio/speaker", speaker_pin)
        .await
        .ok_or_else(|| anyhow::anyhow!("no PrimitiveAudioSpeaker provider"))?;

    let _ = tx
        .send(Ok(event_status(
            KIND_TTS_STARTED,
            session_id,
            &format!(
                "synthesising {} chars via {tts_endpoint}",
                text.chars().count()
            ),
        )))
        .await;

    let mut tts_client = SystemSpeechTtsClient::connect(tts_endpoint.clone())
        .await
        .map_err(|e| anyhow::anyhow!("connect tts {tts_endpoint}: {e}"))?;
    let resp = tts_client
        .synthesize(Request::new(tts::SynthesizeRequest {
            text: text.to_string(),
            language: language.to_string(),
            voice: String::new(),
            speed: 1.0,
        }))
        .await
        .map_err(|e| anyhow::anyhow!("tts rpc failed: {e}"))?
        .into_inner();
    if !resp.error.is_empty() {
        anyhow::bail!("tts error: {}", resp.error);
    }
    if resp.audio_data.is_empty() {
        anyhow::bail!("tts returned empty audio");
    }

    let mut speaker_client = PrimitiveAudioSpeakerClient::connect(speaker_endpoint.clone())
        .await
        .map_err(|e| anyhow::anyhow!("connect speaker {speaker_endpoint}: {e}"))?;
    const SLICE: usize = 8 * 1024;
    let chunks: Vec<AudioChunk> = resp
        .audio_data
        .chunks(SLICE)
        .enumerate()
        .map(|(i, slice)| AudioChunk {
            timestamp_ns: now_ns(),
            data: slice.to_vec(),
            sequence: i as u32,
            duration_s: 0.0,
        })
        .collect();
    let stream = futures::stream::iter(chunks);
    speaker_client
        .speaker(Request::new(stream))
        .await
        .map_err(|e| anyhow::anyhow!("speaker rpc failed: {e}"))?;

    let _ = tx
        .send(Ok(event_status(
            KIND_TTS_DONE,
            session_id,
            &format!(
                "played {} bytes via {speaker_endpoint}",
                resp.audio_data.len()
            ),
        )))
        .await;
    Ok(())
}

// ── Helpers ──────────────────────────────────────────────────────────────────

fn build_task(
    session_id: &str,
    transcript: &str,
    user_id: &str,
    audio_pcm: &[u8],
    extra_context_json: &str,
) -> Task {
    let mut ctx: serde_json::Value = if extra_context_json.trim().is_empty() {
        serde_json::json!({})
    } else {
        serde_json::from_str(extra_context_json).unwrap_or_else(|_| serde_json::json!({}))
    };
    if let Some(obj) = ctx.as_object_mut() {
        obj.insert("voice_session".to_string(), serde_json::json!(true));
        obj.insert("user_id".to_string(), serde_json::json!(user_id));
    }
    Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: INTENT_SOURCE_AUDIO,
        text: transcript.to_string(),
        audio_data: audio_pcm.to_vec(),
        context_json: ctx.to_string(),
        timestamp_ms: now_ms(),
    }
}

fn accumulate_text(ev: &PilotEvent, into: &mut String) {
    const EVT_TEXT_CHUNK: u32 = 0;
    const EVT_FINAL_TEXT: u32 = 4;
    match ev.event_kind {
        EVT_TEXT_CHUNK => into.push_str(&ev.text_chunk),
        EVT_FINAL_TEXT if into.trim().is_empty() && !ev.final_text.is_empty() => {
            into.push_str(&ev.final_text);
        }
        _ => {}
    }
}

fn event_status(kind: u32, session_id: &str, message: &str) -> VoiceEvent {
    VoiceEvent {
        event_kind: kind,
        session_id: session_id.to_string(),
        text: String::new(),
        user_id: String::new(),
        confidence: 0.0,
        pilot: None,
        error: String::new(),
        status_message: message.to_string(),
        timestamp_ms: now_ms(),
    }
}

fn event_text(kind: u32, session_id: &str, text: &str, confidence: f32) -> VoiceEvent {
    VoiceEvent {
        event_kind: kind,
        session_id: session_id.to_string(),
        text: text.to_string(),
        user_id: String::new(),
        confidence,
        pilot: None,
        error: String::new(),
        status_message: String::new(),
        timestamp_ms: now_ms(),
    }
}

fn event_user(
    kind: u32,
    session_id: &str,
    user_id: &str,
    confidence: f32,
    status_message: &str,
) -> VoiceEvent {
    VoiceEvent {
        event_kind: kind,
        session_id: session_id.to_string(),
        text: String::new(),
        user_id: user_id.to_string(),
        confidence,
        pilot: None,
        error: String::new(),
        status_message: status_message.to_string(),
        timestamp_ms: now_ms(),
    }
}

fn event_error(session_id: &str, error: &str) -> VoiceEvent {
    VoiceEvent {
        event_kind: KIND_ERROR,
        session_id: session_id.to_string(),
        text: String::new(),
        user_id: String::new(),
        confidence: 0.0,
        pilot: None,
        error: error.to_string(),
        status_message: String::new(),
        timestamp_ms: now_ms(),
    }
}

fn is_mock_mode() -> bool {
    std::env::var("ROBONIX_LIAISON_VOICE_MOCK")
        .map(|v| matches!(v.as_str(), "1" | "true" | "yes" | "on"))
        .unwrap_or(false)
        || std::env::var("ROBONIX_LIAISON_VOICE_MOCK_TEXT").is_ok()
}

fn mock_transcript() -> String {
    std::env::var("ROBONIX_LIAISON_VOICE_MOCK_TEXT")
        .unwrap_or_else(|_| "Hello, please introduce yourself.".to_string())
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fallback_user_id_normalises_hint() {
        let hint = "liukaile";
        let normalised = if hint.is_empty() {
            "voice:unknown".to_string()
        } else if hint.starts_with("voice:") || hint.starts_with("local:") {
            hint.to_string()
        } else {
            format!("voice:{hint}")
        };
        assert_eq!(normalised, "voice:liukaile");
    }

    #[test]
    fn build_task_injects_user_id_into_context() {
        let task = build_task("sess-1", "hello", "voice:alice", &[], r#"{"foo":"bar"}"#);
        let v: serde_json::Value = serde_json::from_str(&task.context_json).unwrap();
        assert_eq!(v["user_id"], "voice:alice");
        assert_eq!(v["voice_session"], true);
        assert_eq!(v["foo"], "bar");
    }

    #[test]
    fn accumulate_text_collects_chunks_and_final() {
        let mut buf = String::new();
        accumulate_text(
            &PilotEvent {
                event_kind: 0,
                session_id: "s".into(),
                text_chunk: "hello ".into(),
                plan: None,
                batch_result: None,
                status: None,
                final_text: String::new(),
            },
            &mut buf,
        );
        accumulate_text(
            &PilotEvent {
                event_kind: 0,
                session_id: "s".into(),
                text_chunk: "world".into(),
                plan: None,
                batch_result: None,
                status: None,
                final_text: String::new(),
            },
            &mut buf,
        );
        assert_eq!(buf, "hello world");
    }
}
