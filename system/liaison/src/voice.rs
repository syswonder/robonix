// SPDX-License-Identifier: MulanPSL-2.0
// voice.rs — voice push-to-talk orchestrator behind SystemLiaison.StartVoiceSession.
//
// One call drives the full pipeline:
//
//   RobonixPrimitiveAudioMic.Stream  (record_seconds / VAD)
//       │
//       ▼
//   RobonixServiceVoiceprintIdentify.Identify → user_id  (graceful: hint fallback on absence)
//       │
//       ├─ access denied → stop before ASR / Pilot / TTS
//       ▼
//   RobonixServiceSpeechAsr.Recognize  → transcript
//       │
//       ▼
//   Build pilot::Task { source=AUDIO, text=transcript, user_id, … }
//       │
//       ▼
//   RobonixSystemPilot.SubmitTask      → forward each PilotEvent as VoiceEvent { kind=PILOT, … }
//       │
//       ▼
//   (if tts_enabled)  RobonixServiceSpeechTts.Synthesize → RobonixPrimitiveAudioSpeaker.Stream
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

use crate::pb::audio::AudioChunk;
use crate::pb::contracts::{
    robonix_primitive_audio_mic_client::RobonixPrimitiveAudioMicClient,
    robonix_primitive_audio_speaker_client::RobonixPrimitiveAudioSpeakerClient,
    robonix_service_speech_asr_client::RobonixServiceSpeechAsrClient,
    robonix_service_speech_tts_client::RobonixServiceSpeechTtsClient,
    robonix_service_voiceprint_identify_client::RobonixServiceVoiceprintIdentifyClient,
    robonix_system_pilot_client::RobonixSystemPilotClient,
};
use crate::pb::liaison::{StartVoiceSessionRequest, VoiceEvent};
use crate::pb::pilot::{PilotEvent, Task};
use crate::pb::tts;
use crate::pb::voiceprint;
use crate::{access, access::AccessControlConfig};

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
/// and close the ASR request stream so the server flushes its final.
/// Tuned for 16 kHz mono s16le speech: speech RMS is typically 1k–8k,
/// background noise sits around 50–300, so 500 is a comfortable gap.
const VAD_SPEECH_RMS: f32 = 500.0;
const VAD_END_SILENCE_SECS: f32 = 1.2;
const DEFAULT_ASR_LANGUAGE: &str = "";
const DEFAULT_AUDIO_ENCODING: &str = "pcm_s16le";
const DEFAULT_AUDIO_SAMPLE_RATE: u32 = 16_000;

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
    access: Arc<AccessControlConfig>,
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
            access,
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
    access: Arc<AccessControlConfig>,
    tx: mpsc::Sender<Result<VoiceEvent, Status>>,
) -> Result<()> {
    let mock = is_mock_mode();
    // `record_seconds` is now a hard-stop ceiling on streaming capture
    // (FunASR's VAD ends the turn under most conditions; this just
    // protects against a sensor that never goes silent). 0 / unset →
    // 30 s default. Whisper's old "record-then-recognize" pattern is
    // gone; voice is recorded once, then sent to one-shot ASR after access
    // control. This is more stable for cloud ASR providers than keeping a
    // second streaming session open after the mic capture has already ended.
    let max_seconds: u32 = if record_seconds == 0 {
        30
    } else {
        record_seconds.max(5)
    };

    let audio_pcm = if mock {
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
        Vec::new()
    } else {
        capture_voice_for_access(&atlas, &req.mic_node_id, max_seconds, &session_id, &tx).await?
    };

    let hint_decision = access.authorize_voice(&req.client_user_id, None);
    let decision = if matches!(hint_decision, access::AccessDecision::Allow { .. }) {
        hint_decision
    } else {
        let identity = identify_user(
            &atlas,
            &req.voiceprint_node_id,
            &audio_pcm,
            &req.client_user_id,
            &session_id,
            &tx,
        )
        .await;
        access.authorize_voice(&req.client_user_id, identity.response.as_ref())
    };
    let (user_id, access_context) = match decision {
        access::AccessDecision::Allow {
            user_id,
            method,
            confidence,
            reason,
        } => {
            log::info!(
                "[liaison/access] voice allow user={user_id} via {method:?} confidence={confidence:.2}: {reason}"
            );
            let _ = tx
                .send(Ok(event_user(
                    KIND_USER_IDENTIFIED,
                    &session_id,
                    &user_id,
                    confidence,
                    &format!("access allowed: {reason}"),
                )))
                .await;
            (
                user_id,
                AccessContext {
                    method: method.as_str().to_string(),
                    confidence,
                    reason,
                },
            )
        }
        access::AccessDecision::Deny {
            user_id,
            confidence,
            reason,
        } => {
            log::warn!(
                "[liaison/access] voice deny user={user_id} confidence={confidence:.2}: {reason}"
            );
            anyhow::bail!("access denied for voice user '{user_id}': {reason}");
        }
    };

    let transcript = if mock {
        let canned = mock_transcript();
        let _ = tx
            .send(Ok(event_text(KIND_ASR_FINAL, &session_id, &canned, 1.0)))
            .await;
        canned
    } else {
        recognize_recorded_audio(
            &atlas,
            &req.asr_node_id,
            &language,
            &audio_pcm,
            &session_id,
            &tx,
        )
        .await?
    };

    if transcript.trim().is_empty() {
        anyhow::bail!("empty transcript — nothing to send to Pilot");
    }

    // 4. Build Task and stream Pilot events.
    let pilot_endpoint = resolve_endpoint(&atlas, "robonix/system/pilot", "")
        .await
        .unwrap_or(pilot_endpoint_default);

    let task = build_task(
        &session_id,
        &transcript,
        &user_id,
        &access_context,
        &audio_pcm,
        &req.context_json,
    );

    let mut accumulated_text = String::new();
    let mut tts_pending_text = String::new();

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
                        let before_len = accumulated_text.len();
                        accumulate_text(&ev, &mut accumulated_text);
                        if req.tts_enabled && accumulated_text.len() > before_len {
                            tts_pending_text.push_str(&accumulated_text[before_len..]);
                            let sentences = drain_complete_sentences(&mut tts_pending_text);
                            for sentence in sentences {
                                if let Err(e) = synthesize_and_play(
                                    &atlas,
                                    &req.tts_node_id,
                                    &req.speaker_node_id,
                                    &language,
                                    &sentence,
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
                            }
                        }
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
            // a "message received successfully" reply (callers must see the failure).
            anyhow::bail!("Pilot unreachable at {pilot_endpoint}: {e}");
        }
    }

    // 5. Optional TTS playback (non-fatal on any error).
    if req.tts_enabled
        && !tts_pending_text.trim().is_empty()
        && let Err(e) = synthesize_and_play(
            &atlas,
            &req.tts_node_id,
            &req.speaker_node_id,
            &language,
            tts_pending_text.trim(),
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

// ── Mic capture + gated ASR ─────────────────────────────────────────────────
//
// Capture happens before ASR so access-denied voices never reach the speech
// service. Once the access gate allows the speaker, the captured PCM is sent
// to `robonix/service/speech/asr_stream` in the same AudioChunk shape the live
// mic pump used previously.

#[allow(clippy::too_many_arguments)]
async fn capture_voice_for_access(
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
            &format!("capturing mic {mic_endpoint} for access check"),
        )))
        .await;

    let mut mic_stream = mic_client
        .mic(Request::new(()))
        .await
        .map_err(|e| anyhow::anyhow!("mic rpc failed: {e}"))?
        .into_inner();

    let mut pcm =
        Vec::<u8>::with_capacity((DEFAULT_AUDIO_SAMPLE_RATE as usize) * 2 * (max_seconds as usize));
    let deadline = tokio::time::Instant::now() + Duration::from_secs(max_seconds as u64);
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
                pcm.extend_from_slice(&chunk.data);
                if has_spoken && silence_secs >= VAD_END_SILENCE_SECS {
                    break;
                }
            }
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
                pcm.len(),
                pcm.len() as f32 / (DEFAULT_AUDIO_SAMPLE_RATE as f32 * 2.0),
            ),
        )))
        .await;
    Ok(pcm)
}

async fn recognize_recorded_audio(
    atlas: &Arc<Mutex<AtlasClient>>,
    asr_pin: &str,
    language: &str,
    audio_pcm: &[u8],
    session_id: &str,
    tx: &mpsc::Sender<Result<VoiceEvent, Status>>,
) -> Result<String> {
    let asr_endpoint = resolve_endpoint(atlas, "robonix/service/speech/asr", asr_pin)
        .await
        .ok_or_else(|| {
            anyhow::anyhow!("no RobonixServiceSpeechAsr provider registered in Atlas")
        })?;

    let mut asr_client = RobonixServiceSpeechAsrClient::connect(asr_endpoint.clone())
        .await
        .map_err(|e| anyhow::anyhow!("connect asr at {asr_endpoint}: {e}"))?;

    let _ = tx
        .send(Ok(event_status(
            KIND_ASR_PARTIAL,
            session_id,
            &format!("access allowed; sending captured audio to asr {asr_endpoint}"),
        )))
        .await;

    let resp = asr_client
        .recognize(Request::new(crate::pb::asr::RecognizeRequest {
            audio_data: audio_pcm.to_vec(),
            encoding: DEFAULT_AUDIO_ENCODING.to_string(),
            sample_rate_hz: DEFAULT_AUDIO_SAMPLE_RATE,
            language: language.to_string(),
        }))
        .await
        .map_err(|e| anyhow::anyhow!("asr rpc failed: {e}"))?
        .into_inner();

    if !resp.error.is_empty() {
        anyhow::bail!("asr error: {}", resp.error);
    }

    let transcript = resp.text.trim().to_string();
    if !transcript.is_empty() {
        let _ = tx
            .send(Ok(event_text(
                KIND_ASR_FINAL,
                session_id,
                &transcript,
                resp.confidence,
            )))
            .await;
    }
    Ok(transcript)
}

// ── Voiceprint ───────────────────────────────────────────────────────────────

/// Result of the optional voiceprint RPC. `None` means the provider was absent
/// or failed; when the gate is enabled, access control treats that as no voice
/// match rather than granting access.
struct VoiceIdentity {
    response: Option<voiceprint::IdentifyResponse>,
}

struct AccessContext {
    method: String,
    confidence: f32,
    reason: String,
}

async fn identify_user(
    atlas: &Arc<Mutex<AtlasClient>>,
    pin_provider_id: &str,
    audio_pcm: &[u8],
    fallback_hint: &str,
    session_id: &str,
    tx: &mpsc::Sender<Result<VoiceEvent, Status>>,
) -> VoiceIdentity {
    let fallback = if fallback_hint.is_empty() {
        "voice:unknown".to_string()
    } else if fallback_hint.starts_with("voice:") || fallback_hint.starts_with("local:") {
        fallback_hint.to_string()
    } else {
        format!("voice:{fallback_hint}")
    };

    let endpoint = match resolve_endpoint(
        atlas,
        "robonix/service/voiceprint/identify",
        pin_provider_id,
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
                    "no RobonixServiceVoiceprintIdentify provider — using client hint",
                )))
                .await;
            return VoiceIdentity { response: None };
        }
    };

    let mut client = match RobonixServiceVoiceprintIdentifyClient::connect(endpoint.clone()).await {
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
            return VoiceIdentity { response: None };
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
            return VoiceIdentity { response: None };
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
    let label = if resp.is_known && !resp.user_name.is_empty() {
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
    VoiceIdentity {
        response: Some(resp),
    }
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
    let tts_endpoint = resolve_endpoint(atlas, "robonix/service/speech/tts", tts_pin)
        .await
        .ok_or_else(|| anyhow::anyhow!("no RobonixServiceSpeechTts provider"))?;
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

    let mut tts_client = RobonixServiceSpeechTtsClient::connect(tts_endpoint.clone())
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
    access: &AccessContext,
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
        obj.insert(
            "access".to_string(),
            serde_json::json!({
                "allowed": true,
                "method": access.method,
                "confidence": access.confidence,
                "reason": access.reason,
            }),
        );
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

/// Fold Pilot streaming output into a single text buffer for optional TTS.
/// Chunk events append directly; final-text events are appended when they
/// are not already present so multi-round Pilot replies are all spoken.
fn accumulate_text(ev: &PilotEvent, into: &mut String) {
    const EVT_TEXT_CHUNK: u32 = 0;
    const EVT_FINAL_TEXT: u32 = 4;
    match ev.event_kind {
        EVT_TEXT_CHUNK => into.push_str(&ev.text_chunk),
        EVT_FINAL_TEXT if !ev.final_text.is_empty() => {
            if into.trim().is_empty() {
                into.push_str(&ev.final_text);
            } else if !into.contains(&ev.final_text) {
                if !into.ends_with('\n') {
                    into.push('\n');
                }
                into.push_str(&ev.final_text);
            }
        }
        _ => {}
    }
}

/// Move completed sentences out of `pending` for low-latency TTS playback.
/// Leaves an incomplete trailing phrase in place so it can be extended by
/// later Pilot chunks or flushed at the end of the turn.
fn drain_complete_sentences(pending: &mut String) -> Vec<String> {
    let mut cut = 0;
    for (idx, ch) in pending.char_indices() {
        if matches!(ch, '。' | '！' | '？' | '!' | '?' | '；' | ';' | '\n') {
            cut = idx + ch.len_utf8();
        }
    }
    if cut == 0 {
        return Vec::new();
    }

    let completed = pending[..cut].to_string();
    let rest = pending[cut..].to_string();
    *pending = rest;

    completed
        .split_inclusive(['。', '！', '？', '!', '?', '；', ';', '\n'])
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(ToString::to_string)
        .collect()
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
        let task = build_task(
            "sess-1",
            "hello",
            "voice:alice",
            &AccessContext {
                method: "voiceprint".to_string(),
                confidence: 0.9,
                reason: "matched".to_string(),
            },
            &[],
            r#"{"foo":"bar"}"#,
        );
        let v: serde_json::Value = serde_json::from_str(&task.context_json).unwrap();
        assert_eq!(v["user_id"], "voice:alice");
        assert_eq!(v["voice_session"], true);
        assert_eq!(v["access"]["allowed"], true);
        assert_eq!(v["access"]["method"], "voiceprint");
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
    fn drain_complete_sentences_keeps_trailing_fragment() {
        let mut pending = "第一句。第二句！还有半句".to_string();
        let sentences = drain_complete_sentences(&mut pending);

        assert_eq!(sentences, vec!["第一句。", "第二句！"]);
        assert_eq!(pending, "还有半句");
    }
}
