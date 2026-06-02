// SPDX-License-Identifier: MulanPSL-2.0
// voice.rs — voice push-to-talk orchestrator behind SystemLiaison.StartVoiceSession.
//
// One call drives the full pipeline:
//
//   RobonixPrimitiveAudioMic.Stream  (record_seconds)
//       │
//       ▼
//   Tencent Cloud ASR WebSocket API   → transcript
//       │
//       ▼
//   RobonixSystemSpeechVoiceprint.Identify → user_id  (graceful: hint fallback on absence)
//       │
//       ▼
//   Build pilot::Task { source=AUDIO, text=transcript, user_id, … }
//       │
//       ▼
//   RobonixSystemPilot.SubmitTask      → forward each PilotEvent as VoiceEvent { kind=PILOT, … }
//       │
//       ▼
//   (if tts_enabled)  RobonixSystemSpeechTts.Synthesize → RobonixPrimitiveAudioSpeaker.Stream
//       │
//       ▼
//   VoiceEvent { kind=SESSION_DONE }
//
// Mock mode (`ROBONIX_LIAISON_VOICE_MOCK=1` or `ROBONIX_LIAISON_VOICE_MOCK_TEXT`):
// skip mic+ASR, use a canned transcript so the link can be exercised without
// audio hardware. This is what the demo script uses by default.

use anyhow::Result;
use base64::Engine as _;
use futures::{SinkExt, Stream};
use hmac::{Hmac, Mac};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{Mutex, mpsc};
use tokio_stream::{StreamExt, wrappers::ReceiverStream};
use tokio_tungstenite::{MaybeTlsStream, WebSocketStream, tungstenite::Message};
use tonic::{Request, Status};
use uuid::Uuid;

use crate::pb::audio::AudioChunk;
use crate::pb::contracts::{
    robonix_primitive_audio_mic_client::RobonixPrimitiveAudioMicClient,
    robonix_primitive_audio_speaker_client::RobonixPrimitiveAudioSpeakerClient,
    robonix_system_pilot_client::RobonixSystemPilotClient,
    robonix_system_speech_tts_client::RobonixSystemSpeechTtsClient,
    robonix_system_speech_voiceprint_client::RobonixSystemSpeechVoiceprintClient,
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

/// Hard ceiling on a single voice turn. Real end-of-turn comes from
/// the silence-VAD in the mic pump (see VAD_* below); this just keeps
/// a stuck mic from monopolizing the session forever.
const DEFAULT_RECORD_SECONDS: u32 = 30;

/// Silence-VAD parameters. After we've heard a chunk above
/// `VAD_SPEECH_RMS`, count consecutive sub-threshold chunks; once the
/// silence lasts `VAD_END_SILENCE_SECS`, treat the utterance as done
/// and stop collecting mic chunks before calling the ASR API.
/// Tuned for 16 kHz mono s16le speech: speech RMS is typically 1k–8k,
/// background noise sits around 50–300, so 500 is a comfortable gap.
const VAD_SPEECH_RMS: f32 = 500.0;
const VAD_END_SILENCE_SECS: f32 = 1.2;
const DEFAULT_ASR_LANGUAGE: &str = "";
const DEFAULT_AUDIO_ENCODING: &str = "pcm_s16le";
const DEFAULT_AUDIO_SAMPLE_RATE: u32 = 16_000;
const TENCENT_ASR_DEFAULT_HOST: &str = "asr.cloud.tencent.com";
const TENCENT_ASR_DEFAULT_PATH: &str = "/asr/v2";
const TENCENT_ASR_DEFAULT_ENGINE: &str = "16k_zh";
const TENCENT_ASR_CHUNK_BYTES: usize = 6400;
const TENCENT_ASR_CHUNK_INTERVAL_MS: u64 = 200;
const TENCENT_ASR_END_TAG: &str = r#"{"type":"end"}"#;

// ── Discovery helpers ────────────────────────────────────────────────────────
//
// Atlas → contract_id → endpoint resolution. Every speech / mic / speaker
// service registers its gRPC capability with `metadata_json = {"endpoint":
// "host:port"}` (see audio_driver/node.py and speech_service/service.py).

/// Two-step endpoint resolution: query for providers offering `contract_id`, then
/// `ConnectCapability` against the chosen one to actually receive the
/// endpoint string. Atlas hides endpoints from `query_capabilities` on
/// purpose; consumers must commit a channel before they can dial.
async fn resolve_endpoint(
    atlas: &Arc<Mutex<AtlasClient>>,
    contract_id: &str,
    pin_provider_id: &str,
) -> Option<String> {
    let mut atlas = atlas.lock().await;
    let transport = atlas_pb::Transport::Grpc;
    let providers = atlas
        .query_capabilities("", contract_id, transport)
        .await
        .ok()?;

    // Auto-pick path: any provider providing this contract over the right transport.
    let auto_pick = || -> Option<&atlas_pb::CapabilityProvider> {
        providers.iter().find(|r| {
            r.capabilities
                .iter()
                .any(|i| i.contract_id == contract_id && i.transport == transport as i32)
        })
    };

    let pick: Option<&atlas_pb::CapabilityProvider> = if pin_provider_id.is_empty() {
        auto_pick()
    } else {
        // Try the pinned provider first; if it isn't in atlas anymore (stale chat
        // config / pin pointed at a provider that's not in this deploy), fall back
        // to auto-pick rather than failing hard. The pin is a hint, not a
        // hard requirement.
        match providers
            .iter()
            .find(|r| r.id == pin_provider_id || r.namespace == pin_provider_id)
        {
            Some(provider) => Some(provider),
            None => {
                log::warn!(
                    "[voice] pinned provider '{pin_provider_id}' for {contract_id} not in atlas; \
                     falling back to auto-pick. Available providers: {:?}",
                    providers.iter().map(|r| r.id.as_str()).collect::<Vec<_>>()
                );
                auto_pick()
            }
        }
    };

    let provider = pick?;
    let (_channel_id, endpoint, _params) = atlas
        .connect_capability("liaison", &provider.id, contract_id, transport)
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
                "voice session started (vad, record≤{record_seconds}s, tts={}, lang={})",
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
    // `record_seconds` is now a hard-stop ceiling on streaming capture
    // (local silence-VAD ends the turn under most conditions; this just
    // protects against a sensor that never goes silent). 0 / unset →
    // 30 s default. ASR recognition now goes through the Tencent Cloud ASR API
    // after mic capture finishes.
    let max_seconds: u32 = if record_seconds == 0 {
        30
    } else {
        record_seconds.max(5)
    };

    let (audio_pcm, transcript) = if mock {
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
        let _ = tx
            .send(Ok(event_text(KIND_ASR_FINAL, &session_id, &canned, 1.0)))
            .await;
        (Vec::new(), canned)
    } else {
        // Mic capture still comes from the audio primitive. ASR now calls
        // Tencent Cloud ASR over WebSocket directly instead of discovering a
        // Robonix speech/asr_stream gRPC provider.
        capture_and_transcribe_with_cloud_asr(
            &atlas,
            &req.mic_node_id,
            max_seconds,
            &session_id,
            &tx,
        )
        .await?
    };

    if transcript.trim().is_empty() {
        anyhow::bail!("empty transcript — nothing to send to Pilot");
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
        let mut client = RobonixSystemPilotClient::connect(pilot_endpoint.clone())
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

// ── Mic capture + Tencent Cloud ASR API ─────────────────────────────────────
//
// Open the mic primitive, collect one utterance with the local silence-VAD,
// then send the PCM to Tencent Cloud ASR over WebSocket. Tencent expects
// 16 kHz mono s16le audio in 6400-byte chunks roughly every 200 ms.

async fn capture_and_transcribe_with_cloud_asr(
    atlas: &Arc<Mutex<AtlasClient>>,
    mic_pin: &str,
    max_seconds: u32,
    session_id: &str,
    tx: &mpsc::Sender<Result<VoiceEvent, Status>>,
) -> Result<(Vec<u8>, String)> {
    let audio_pcm = capture_audio_pcm(atlas, mic_pin, max_seconds, session_id, tx).await?;
    let _ = tx
        .send(Ok(event_status(
            KIND_RECORDING_DONE,
            session_id,
            &format!(
                "captured {} bytes (~{:.2}s @ 16kHz mono s16le); sending to Tencent ASR API",
                audio_pcm.len(),
                audio_pcm.len() as f32 / (DEFAULT_AUDIO_SAMPLE_RATE as f32 * 2.0),
            ),
        )))
        .await;

    let transcript = cloud_asr_transcribe_pcm(&audio_pcm).await?;
    if !transcript.is_empty() {
        let _ = tx
            .send(Ok(event_text(KIND_ASR_FINAL, session_id, &transcript, 1.0)))
            .await;
    }
    Ok((audio_pcm, transcript))
}

async fn capture_audio_pcm(
    atlas: &Arc<Mutex<AtlasClient>>,
    mic_pin: &str,
    max_seconds: u32,
    session_id: &str,
    tx: &mpsc::Sender<Result<VoiceEvent, Status>>,
) -> Result<Vec<u8>> {
    let mic_endpoint = resolve_endpoint(atlas, "robonix/primitive/audio/mic", mic_pin)
        .await
        .ok_or_else(|| {
            anyhow::anyhow!("no RobonixPrimitiveAudioMic provider registered in Atlas")
        })?;
    let mut mic_client = RobonixPrimitiveAudioMicClient::connect(mic_endpoint.clone())
        .await
        .map_err(|e| anyhow::anyhow!("connect mic at {mic_endpoint}: {e}"))?;

    let _ = tx
        .send(Ok(event_status(
            KIND_RECORDING_STARTED,
            session_id,
            &format!("recording mic {mic_endpoint} for Tencent ASR API"),
        )))
        .await;

    let mut mic_stream = mic_client
        .mic(Request::new(()))
        .await
        .map_err(|e| anyhow::anyhow!("mic rpc failed: {e}"))?
        .into_inner();

    let deadline = tokio::time::Instant::now() + Duration::from_secs(max_seconds as u64);
    let mut pcm_buf =
        Vec::<u8>::with_capacity((DEFAULT_AUDIO_SAMPLE_RATE as usize) * 2 * (max_seconds as usize));
    let mut has_spoken = false;
    let mut silence_secs: f32 = 0.0;

    loop {
        if tokio::time::Instant::now() >= deadline {
            break;
        }
        let remaining = deadline - tokio::time::Instant::now();
        match tokio::time::timeout(remaining, mic_stream.message()).await {
            Ok(Ok(Some(chunk))) => {
                let dur = chunk.duration_s.max(0.0);
                let rms = pcm_rms_s16le(&chunk.data);
                if rms >= VAD_SPEECH_RMS {
                    has_spoken = true;
                    silence_secs = 0.0;
                } else if has_spoken {
                    silence_secs += dur;
                }
                pcm_buf.extend_from_slice(&chunk.data);
                if has_spoken && silence_secs >= VAD_END_SILENCE_SECS {
                    break;
                }
            }
            Ok(Ok(None)) => break,
            Ok(Err(e)) => anyhow::bail!("mic stream error: {e}"),
            Err(_) => break,
        }
    }

    if pcm_buf.is_empty() {
        anyhow::bail!("mic returned empty audio");
    }
    Ok(pcm_buf)
}

async fn cloud_asr_transcribe_pcm(audio_pcm: &[u8]) -> Result<String> {
    if audio_pcm.is_empty() {
        anyhow::bail!("empty audio for Tencent ASR");
    }

    let config = TencentAsrConfig::from_env()?;
    let url = config.signed_url()?;
    let (mut ws, _resp) = tokio_tungstenite::connect_async(&url)
        .await
        .map_err(|e| anyhow::anyhow!("connect Tencent ASR websocket: {e}"))?;

    let mut results = TencentAsrResults::default();
    let mut send_finished = false;
    wait_for_tencent_asr_handshake(&mut ws, &mut results).await?;

    for chunk in audio_pcm.chunks(TENCENT_ASR_CHUNK_BYTES) {
        ws.send(Message::Binary(chunk.to_vec().into()))
            .await
            .map_err(|e| anyhow::anyhow!("send Tencent ASR audio chunk: {e}"))?;
        drain_tencent_asr_messages(&mut ws, &mut results).await?;
        tokio::time::sleep(Duration::from_millis(TENCENT_ASR_CHUNK_INTERVAL_MS)).await;
    }
    ws.send(Message::Text(TENCENT_ASR_END_TAG.into()))
        .await
        .map_err(|e| anyhow::anyhow!("send Tencent ASR end tag: {e}"))?;

    let final_deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    while tokio::time::Instant::now() < final_deadline {
        match tokio::time::timeout(Duration::from_millis(500), ws.next()).await {
            Ok(Some(Ok(msg))) => {
                handle_tencent_asr_message(msg, &mut results, &mut send_finished)?;
                if send_finished {
                    break;
                }
            }
            Ok(Some(Err(e))) => anyhow::bail!("receive Tencent ASR message: {e}"),
            Ok(None) => break,
            Err(_) if !results.is_empty() => break,
            Err(_) => {}
        }
    }
    let _ = ws.close(None).await;

    Ok(results.transcript())
}

async fn wait_for_tencent_asr_handshake(
    ws: &mut WebSocketStream<MaybeTlsStream<tokio::net::TcpStream>>,
    results: &mut TencentAsrResults,
) -> Result<()> {
    match tokio::time::timeout(Duration::from_secs(5), ws.next()).await {
        Ok(Some(Ok(msg))) => {
            let mut finished = false;
            handle_tencent_asr_message(msg, results, &mut finished)?;
            Ok(())
        }
        Ok(Some(Err(e))) => anyhow::bail!("receive Tencent ASR handshake: {e}"),
        Ok(None) => anyhow::bail!("Tencent ASR websocket closed before handshake"),
        Err(_) => anyhow::bail!("Tencent ASR handshake timed out"),
    }
}

async fn drain_tencent_asr_messages(
    ws: &mut WebSocketStream<MaybeTlsStream<tokio::net::TcpStream>>,
    results: &mut TencentAsrResults,
) -> Result<()> {
    loop {
        match tokio::time::timeout(Duration::from_millis(10), ws.next()).await {
            Ok(Some(Ok(msg))) => {
                let mut finished = false;
                handle_tencent_asr_message(msg, results, &mut finished)?;
                if finished {
                    break;
                }
            }
            Ok(Some(Err(e))) => anyhow::bail!("receive Tencent ASR message: {e}"),
            Ok(None) => break,
            Err(_) => break,
        }
    }
    Ok(())
}

fn handle_tencent_asr_message(
    msg: Message,
    results: &mut TencentAsrResults,
    finished: &mut bool,
) -> Result<TencentAsrAction> {
    let text = match msg {
        Message::Text(s) => s.to_string(),
        Message::Binary(bytes) => String::from_utf8(bytes.to_vec())
            .map_err(|e| anyhow::anyhow!("Tencent ASR binary message is not utf-8: {e}"))?,
        Message::Close(_) => {
            *finished = true;
            return Ok(TencentAsrAction::Closed);
        }
        Message::Ping(_) | Message::Pong(_) | Message::Frame(_) => {
            return Ok(TencentAsrAction::Other);
        }
    };
    if text.is_empty() {
        *finished = true;
        return Ok(TencentAsrAction::Closed);
    }

    let value: serde_json::Value = serde_json::from_str(&text)
        .map_err(|e| anyhow::anyhow!("parse Tencent ASR response JSON: {e}; raw={text}"))?;
    let code = value.get("code").and_then(|v| v.as_i64()).unwrap_or(0);
    if code != 0 {
        let message = value
            .get("message")
            .and_then(|v| v.as_str())
            .unwrap_or("unknown error");
        anyhow::bail!("Tencent ASR error code {code}: {message}; raw={text}");
    }
    if let Some(result) = value.get("result") {
        results.push_result(result);
    }
    if value.get("final").and_then(|v| v.as_i64()) == Some(1) {
        *finished = true;
        Ok(TencentAsrAction::Final)
    } else if value.get("result").is_some() {
        Ok(TencentAsrAction::Result)
    } else {
        Ok(TencentAsrAction::Started)
    }
}

#[derive(PartialEq, Eq)]
enum TencentAsrAction {
    Started,
    Result,
    Final,
    Closed,
    Other,
}

#[derive(Default)]
struct TencentAsrResults {
    latest: BTreeMap<i64, String>,
    stable: BTreeMap<i64, String>,
}

impl TencentAsrResults {
    fn push_result(&mut self, result: &serde_json::Value) {
        let index = result.get("index").and_then(|v| v.as_i64()).unwrap_or(0);
        let slice_type = result
            .get("slice_type")
            .and_then(|v| v.as_i64())
            .unwrap_or(0);
        let text = result
            .get("voice_text_str")
            .and_then(|v| v.as_str())
            .unwrap_or_default()
            .to_string();
        if text.is_empty() {
            return;
        }
        self.latest.insert(index, text.clone());
        if slice_type == 2 {
            self.stable.insert(index, text);
        }
    }

    fn transcript(&self) -> String {
        let source = if self.stable.is_empty() {
            &self.latest
        } else {
            &self.stable
        };
        source.values().cloned().collect::<String>()
    }

    fn is_empty(&self) -> bool {
        self.latest.is_empty() && self.stable.is_empty()
    }
}

struct TencentAsrConfig {
    app_id: String,
    secret_id: String,
    secret_key: String,
    host: String,
    path: String,
    engine_model_type: String,
    need_vad: String,
}

impl TencentAsrConfig {
    /// Loads Tencent ASR credentials from environment so secrets do not land
    /// in the repository or boot manifest by accident.
    fn from_env() -> Result<Self> {
        let app_id = std::env::var("ROBONIX_LIAISON_TENCENT_ASR_APP_ID")
            .or_else(|_| std::env::var("TENCENT_ASR_APP_ID"))
            .or_else(|_| std::env::var("TENCENTCLOUD_APP_ID"))
            .map_err(|_| {
                anyhow::anyhow!(
                    "missing Tencent ASR app id; set ROBONIX_LIAISON_TENCENT_ASR_APP_ID or TENCENT_ASR_APP_ID"
                )
            })?;
        let secret_id = std::env::var("ROBONIX_LIAISON_TENCENT_ASR_SECRET_ID")
            .or_else(|_| std::env::var("TENCENT_ASR_SECRET_ID"))
            .or_else(|_| std::env::var("TENCENTCLOUD_SECRET_ID"))
            .map_err(|_| {
                anyhow::anyhow!(
                    "missing Tencent ASR SecretId; set ROBONIX_LIAISON_TENCENT_ASR_SECRET_ID or TENCENTCLOUD_SECRET_ID"
                )
            })?;
        let secret_key = std::env::var("ROBONIX_LIAISON_TENCENT_ASR_SECRET_KEY")
            .or_else(|_| std::env::var("TENCENT_ASR_SECRET_KEY"))
            .or_else(|_| std::env::var("TENCENTCLOUD_SECRET_KEY"))
            .map_err(|_| {
                anyhow::anyhow!(
                    "missing Tencent ASR SecretKey; set ROBONIX_LIAISON_TENCENT_ASR_SECRET_KEY or TENCENTCLOUD_SECRET_KEY"
                )
            })?;
        let host = std::env::var("ROBONIX_LIAISON_TENCENT_ASR_HOST")
            .unwrap_or_else(|_| TENCENT_ASR_DEFAULT_HOST.to_string());
        let path = std::env::var("ROBONIX_LIAISON_TENCENT_ASR_PATH")
            .unwrap_or_else(|_| TENCENT_ASR_DEFAULT_PATH.to_string());
        let engine_model_type = std::env::var("ROBONIX_LIAISON_TENCENT_ASR_ENGINE_MODEL_TYPE")
            .or_else(|_| std::env::var("TENCENT_ASR_ENGINE_MODEL_TYPE"))
            .unwrap_or_else(|_| TENCENT_ASR_DEFAULT_ENGINE.to_string());
        let need_vad = std::env::var("ROBONIX_LIAISON_TENCENT_ASR_NEED_VAD")
            .unwrap_or_else(|_| "1".to_string());
        Ok(Self {
            app_id,
            secret_id,
            secret_key,
            host,
            path,
            engine_model_type,
            need_vad,
        })
    }

    fn signed_url(&self) -> Result<String> {
        type HmacSha1 = Hmac<sha1::Sha1>;
        let timestamp = now_secs();
        let expired = timestamp + 600;
        let nonce = (timestamp % 10_000_000_000).to_string();
        let mut params = BTreeMap::<String, String>::new();
        params.insert("engine_model_type".into(), self.engine_model_type.clone());
        params.insert("expired".into(), expired.to_string());
        params.insert("filter_dirty".into(), "1".into());
        params.insert("filter_modal".into(), "1".into());
        params.insert("filter_punc".into(), "0".into());
        params.insert("needvad".into(), self.need_vad.clone());
        params.insert("nonce".into(), nonce);
        params.insert("secretid".into(), self.secret_id.clone());
        params.insert("timestamp".into(), timestamp.to_string());
        params.insert("voice_format".into(), "1".into());
        params.insert("voice_id".into(), Uuid::new_v4().to_string());

        let query = params
            .iter()
            .map(|(k, v)| format!("{k}={v}"))
            .collect::<Vec<_>>()
            .join("&");
        let path = format!("{}/{}", self.path.trim_end_matches('/'), self.app_id);
        let sign_payload = format!("{}{path}?{query}", self.host);
        let mut mac = HmacSha1::new_from_slice(self.secret_key.as_bytes())
            .map_err(|e| anyhow::anyhow!("create Tencent ASR hmac signer: {e}"))?;
        mac.update(sign_payload.as_bytes());
        let signature =
            base64::engine::general_purpose::STANDARD.encode(mac.finalize().into_bytes());
        let encoded_query = params
            .iter()
            .map(|(k, v)| format!("{}={}", urlencoding::encode(k), urlencoding::encode(v)))
            .collect::<Vec<_>>()
            .join("&");
        Ok(format!(
            "wss://{}{path}?{encoded_query}&signature={}",
            self.host,
            urlencoding::encode(&signature)
        ))
    }
}

fn now_secs() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs()
}

// ── Voiceprint ───────────────────────────────────────────────────────────────

async fn identify_user(
    atlas: &Arc<Mutex<AtlasClient>>,
    pin_provider_id: &str,
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

    let endpoint =
        match resolve_endpoint(atlas, "robonix/system/speech/voiceprint", pin_provider_id).await {
            Some(ep) => ep,
            None => {
                let _ = tx
                    .send(Ok(event_user(
                        KIND_USER_IDENTIFIED,
                        session_id,
                        &fallback,
                        0.0,
                        "no RobonixSystemSpeechVoiceprint provider — using client hint",
                    )))
                    .await;
                return fallback;
            }
        };

    let mut client = match RobonixSystemSpeechVoiceprintClient::connect(endpoint.clone()).await {
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
        .ok_or_else(|| anyhow::anyhow!("no RobonixSystemSpeechTts provider"))?;
    let speaker_endpoint = resolve_endpoint(atlas, "robonix/primitive/audio/speaker", speaker_pin)
        .await
        .ok_or_else(|| anyhow::anyhow!("no RobonixPrimitiveAudioSpeaker provider"))?;

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

    let mut tts_client = RobonixSystemSpeechTtsClient::connect(tts_endpoint.clone())
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

    let mut speaker_client = RobonixPrimitiveAudioSpeakerClient::connect(speaker_endpoint.clone())
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
        // Pilot reads `modality` to decide brevity / no-markdown rules
        // for voice-mode replies (planner.rs:181). Don't rename without
        // updating pilot in lockstep.
        obj.insert("modality".to_string(), serde_json::json!("voice"));
        obj.insert("voice_session".to_string(), serde_json::json!(true));
        obj.insert("user_id".to_string(), serde_json::json!(user_id));
    }
    Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: crate::INTENT_SOURCE_AUDIO,
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

/// RMS amplitude of a 16-bit little-endian PCM buffer. Used by the
/// silence-VAD in the mic pump. Returns 0.0 on empty / odd-length input.
fn pcm_rms_s16le(data: &[u8]) -> f32 {
    let n = data.len() / 2;
    if n == 0 {
        return 0.0;
    }
    let sum_sq: f64 = data
        .chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]) as f64)
        .map(|s| s * s)
        .sum();
    (sum_sq / n as f64).sqrt() as f32
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

    #[test]
    fn tencent_asr_results_prefers_stable_slices() {
        let mut results = TencentAsrResults::default();
        results.push_result(&serde_json::json!({
            "slice_type": 0,
            "index": 0,
            "voice_text_str": "你"
        }));
        results.push_result(&serde_json::json!({
            "slice_type": 2,
            "index": 0,
            "voice_text_str": "你好"
        }));
        results.push_result(&serde_json::json!({
            "slice_type": 2,
            "index": 1,
            "voice_text_str": "世界"
        }));

        assert_eq!(results.transcript(), "你好世界");
    }

    #[test]
    fn tencent_asr_signed_url_has_required_query_params() {
        let config = TencentAsrConfig {
            app_id: "1250000000".into(),
            secret_id: "AKIDexample".into(),
            secret_key: "secret".into(),
            host: TENCENT_ASR_DEFAULT_HOST.into(),
            path: TENCENT_ASR_DEFAULT_PATH.into(),
            engine_model_type: TENCENT_ASR_DEFAULT_ENGINE.into(),
            need_vad: "1".into(),
        };

        let url = config.signed_url().unwrap();
        assert!(url.starts_with("wss://asr.cloud.tencent.com/asr/v2/1250000000?"));
        assert!(url.contains("engine_model_type=16k_zh"));
        assert!(url.contains("secretid=AKIDexample"));
        assert!(url.contains("voice_format=1"));
        assert!(url.contains("signature="));
    }

    #[tokio::test]
    #[ignore]
    async fn tencent_asr_live_connects_with_env_credentials() {
        let mut pcm = Vec::with_capacity(DEFAULT_AUDIO_SAMPLE_RATE as usize * 2);
        for i in 0..DEFAULT_AUDIO_SAMPLE_RATE {
            let phase = i as f32 * 440.0 * std::f32::consts::TAU / DEFAULT_AUDIO_SAMPLE_RATE as f32;
            let sample = (phase.sin() * 1200.0) as i16;
            pcm.extend_from_slice(&sample.to_le_bytes());
        }

        let _ = cloud_asr_transcribe_pcm(&pcm).await.unwrap();
    }
}
