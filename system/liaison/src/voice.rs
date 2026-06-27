// SPDX-License-Identifier: MulanPSL-2.0
// voice.rs — voice push-to-talk orchestrator behind SystemLiaison.StartVoiceSession.
//
// One call drives the full pipeline:
//
//   RobonixPrimitiveAudioMic.Stream  (record_seconds)
//       │
//       ▼
//   SystemSpeechAsr.Recognize    → transcript
//       │
//       ▼
//   RobonixServiceVoiceprintIdentify.Identify → user_id  (graceful: hint fallback on absence)
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
    robonix_service_speech_asr_stream_client::RobonixServiceSpeechAsrStreamClient,
    robonix_service_speech_tts_client::RobonixServiceSpeechTtsClient,
    robonix_service_voiceprint_identify_client::RobonixServiceVoiceprintIdentifyClient,
    robonix_system_pilot_client::RobonixSystemPilotClient,
};
use crate::pb::liaison::{StartVoiceSessionRequest, VoiceEvent};
use crate::pb::pilot::{PilotEvent, Task};
use crate::pb::tts;
use crate::pb::voiceprint;
use robonix_scribe::{info, warn};

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
                warn!(
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
    // (FunASR's VAD ends the turn under most conditions; this just
    // protects against a sensor that never goes silent). 0 / unset →
    // 30 s default. Whisper's old "record-then-recognize" pattern is
    // gone; voice goes through `asr_stream` end-to-end.
    let max_seconds: u32 = if record_seconds == 0 {
        30
    } else {
        record_seconds.max(5)
    };

    let t_cap = std::time::Instant::now();
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
        // Mic → AsrAudioChunk pump + asr_stream.RecognizeStream both
        // run inside `stream_capture_and_recognize`. It returns once
        // FunASR emits is_final, the user hits max_seconds, or the
        // mic stream drops. Voiceprint still gets the accumulated PCM
        // for downstream identification (currently fallback-only).
        stream_capture_and_recognize(
            &atlas,
            &req.mic_node_id,
            &req.asr_node_id,
            &language,
            max_seconds,
            &session_id,
            &tx,
        )
        .await?
    };

    // [profile] record start → final transcript: covers mic-open + audio
    // transfer + streaming ASR inference. Compare against the per-chunk mic
    // timings above (transfer) and the speech service's [profile-asr] lines
    // (inference) to see which dominates.
    {
        let secs = audio_pcm.len() as f64 / (DEFAULT_AUDIO_SAMPLE_RATE as f64 * 2.0);
        info!(
            "[profile] record→transcript {} ms (audio {:.2}s, {} pcm bytes, transcript {} chars)",
            t_cap.elapsed().as_millis(),
            secs,
            audio_pcm.len(),
            transcript.chars().count()
        );
    }

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
    // Track whether we've already spoken at least one per-plan answer, so the
    // end-of-turn fallback below doesn't double-speak.
    let mut spoke_any = false;
    // [profile] pilot round-trip (submit → first event → turn done).
    let t_pilot = std::time::Instant::now();
    let mut first_pilot_logged = false;

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
                        if !first_pilot_logged {
                            info!(
                                "[profile] pilot: first event +{} ms (after transcript submit)",
                                t_pilot.elapsed().as_millis()
                            );
                            first_pilot_logged = true;
                        }
                        accumulate_text(&ev, &mut accumulated_text);
                        // Speak each plan's natural-language answer the moment it
                        // arrives (EVT_FINAL_TEXT = 4), rather than once at the end
                        // of the whole turn — so a multi-plan reply is voiced live,
                        // result by result.
                        let speak_now = req.tts_enabled
                            && ev.event_kind == 4
                            && !ev.final_text.trim().is_empty();
                        let say = if speak_now { ev.final_text.clone() } else { String::new() };
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
                        if speak_now {
                            spoke_any = true;
                            let t_tts = std::time::Instant::now();
                            let say_chars = say.chars().count();
                            if let Err(e) = synthesize_and_play(
                                &atlas,
                                &req.tts_node_id,
                                &req.speaker_node_id,
                                &language,
                                &say,
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
                            info!(
                                "[profile] tts+play +{} ms ({} chars spoken)",
                                t_tts.elapsed().as_millis(),
                                say_chars
                            );
                        }
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

    info!(
        "[profile] pilot: turn done +{} ms ({} chars of answer text)",
        t_pilot.elapsed().as_millis(),
        accumulated_text.chars().count()
    );

    // 5. Fallback TTS: if the turn produced only streamed chunks (no
    // EVT_FINAL_TEXT was spoken per-result above), speak the accumulated text
    // once at the end. Non-fatal on any error.
    if req.tts_enabled
        && !spoke_any
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

// ── Mic capture + streaming ASR ─────────────────────────────────────────────
//
// Open the mic primitive AND `robonix/service/speech/asr_stream` (FunASR
// paraformer-zh-streaming bidi) at the same time. Mic AudioChunks get
// wrapped as AsrAudioChunk and pumped up the request side; the response
// side yields RecognizeStreamEvents (PARTIAL → … → FINAL). On FINAL,
// stop both streams and return `(accumulated_pcm, final_text)`.
//
// `max_seconds` is a hard ceiling — FunASR's VAD usually ends the turn
// well before that, but we don't want to hang on a stuck mic.

#[allow(clippy::too_many_arguments)]
async fn stream_capture_and_recognize(
    atlas: &Arc<Mutex<AtlasClient>>,
    mic_pin: &str,
    asr_pin: &str,
    language: &str,
    max_seconds: u32,
    session_id: &str,
    tx: &mpsc::Sender<Result<VoiceEvent, Status>>,
) -> Result<(Vec<u8>, String)> {
    let mic_endpoint = resolve_endpoint(atlas, "robonix/primitive/audio/mic", mic_pin)
        .await
        .ok_or_else(|| {
            anyhow::anyhow!("no RobonixPrimitiveAudioMic provider registered in Atlas")
        })?;
    let asr_endpoint = resolve_endpoint(atlas, "robonix/service/speech/asr_stream", asr_pin)
        .await
        .ok_or_else(|| {
            anyhow::anyhow!("no RobonixServiceSpeechAsrStream provider registered in Atlas")
        })?;

    let mut mic_client = RobonixPrimitiveAudioMicClient::connect(mic_endpoint.clone())
        .await
        .map_err(|e| anyhow::anyhow!("connect mic at {mic_endpoint}: {e}"))?;
    let mut asr_client = RobonixServiceSpeechAsrStreamClient::connect(asr_endpoint.clone())
        .await
        .map_err(|e| anyhow::anyhow!("connect asr_stream at {asr_endpoint}: {e}"))?;

    let _ = tx
        .send(Ok(event_status(
            KIND_RECORDING_STARTED,
            session_id,
            &format!("streaming mic {mic_endpoint} → asr_stream {asr_endpoint}"),
        )))
        .await;

    let mut mic_stream = mic_client
        .mic(Request::new(()))
        .await
        .map_err(|e| anyhow::anyhow!("mic rpc failed: {e}"))?
        .into_inner();

    // Buffered hand-off: mic-pump task drops AsrAudioChunks onto the
    // mpsc; tonic forwards them up the bidi as the request stream.
    let (asr_req_tx, asr_req_rx) = mpsc::channel::<crate::pb::asr::AsrAudioChunk>(64);
    let asr_req_stream = ReceiverStream::new(asr_req_rx);

    // Accumulator the mic-pump writes into; the outer task reads it
    // back at the end so we can hand the raw PCM to identify_user.
    let pcm_buf = Arc::new(Mutex::new(Vec::<u8>::with_capacity(
        (DEFAULT_AUDIO_SAMPLE_RATE as usize) * 2 * (max_seconds as usize),
    )));
    let pcm_buf_for_pump = Arc::clone(&pcm_buf);
    let _language_owned = language.to_string();

    // Mic pump — drains the mic gRPC stream into the asr_req sender,
    // accumulates raw PCM, runs silence-VAD on each chunk, stops on
    // (whichever first):
    //   1. max_seconds hard ceiling
    //   2. mic stream EOF / error
    //   3. silence-VAD end-of-utterance (after speech then quiet)
    //   4. outer task drops asr_req_rx (server already returned FINAL)
    let pump_handle: tokio::task::JoinHandle<()> = tokio::spawn(async move {
        let deadline = tokio::time::Instant::now() + Duration::from_secs(max_seconds as u64);
        // [profile] mic transfer: t_pump=request start; first-chunk latency is
        // the audio-source startup (e.g. macOS bridge over the network); the
        // chunk count + total audio vs wall time shows transfer throughput.
        let t_pump = tokio::time::Instant::now();
        let mut n_chunks: u32 = 0u32;
        let mut audio_s: f32 = 0.0;
        let mut logged_first = false;
        let mut has_spoken = false;
        let mut silence_secs: f32 = 0.0;
        loop {
            if tokio::time::Instant::now() >= deadline {
                break;
            }
            let remaining = deadline - tokio::time::Instant::now();
            match tokio::time::timeout(remaining, mic_stream.message()).await {
                Ok(Ok(Some(chunk))) => {
                    if !logged_first {
                        info!(
                            "[profile] mic: first chunk +{} ms (audio-source startup / transfer)",
                            t_pump.elapsed().as_millis()
                        );
                        logged_first = true;
                    }
                    n_chunks += 1;
                    audio_s += chunk.duration_s.max(0.0);
                    let dur = chunk.duration_s.max(0.0);
                    let rms = pcm_rms_s16le(&chunk.data);
                    if rms >= VAD_SPEECH_RMS {
                        has_spoken = true;
                        silence_secs = 0.0;
                    } else if has_spoken {
                        silence_secs += dur;
                    }
                    pcm_buf_for_pump.lock().await.extend_from_slice(&chunk.data);
                    let asr_chunk = crate::pb::asr::AsrAudioChunk { chunk: Some(chunk) };
                    if asr_req_tx.send(asr_chunk).await.is_err() {
                        break;
                    }
                    if has_spoken && silence_secs >= VAD_END_SILENCE_SECS {
                        // Drops asr_req_tx → server sees stream EOF →
                        // FunASR emits its is_final flush → outer loop
                        // collects the final transcript and breaks.
                        break;
                    }
                }
                Ok(Ok(None)) => break,
                Ok(Err(_)) => break,
                Err(_) => break,
            }
        }
        let wall = t_pump.elapsed().as_millis();
        info!(
            "[profile] mic: pump done +{wall} ms, {n_chunks} chunks, {audio_s:.2}s audio \
             (transfer realtime-factor {:.2}; >1 means audio arrived slower than realtime)",
            (wall as f32 / 1000.0) / audio_s.max(0.001)
        );
    });

    // FunASR docs: AsrAudioChunk + ASR backend reads its own audio_config
    // defaults (16 kHz mono pcm_s16le); language hint is on the bidi
    // request metadata, not the per-chunk message. Tonic-codegen
    // doesn't surface a separate header field here, so we just rely on
    // FunASR's auto-detection — paraformer-zh-streaming is zh-only so
    // this is fine. If a multi-lingual backend is ever wired in, the
    // contract needs an extra preamble message.
    let asr_resp = asr_client
        .recognize_stream(Request::new(asr_req_stream))
        .await
        .map_err(|e| anyhow::anyhow!("asr_stream rpc failed: {e}"))?;
    let mut asr_events = asr_resp.into_inner();

    // FunASR's paraformer-zh-streaming emits *incremental* partials —
    // each event carries only the words decoded from the most recent
    // chunk window, not a cumulative transcript. The is_final flush
    // (`recognize_chunk(b"", is_final=True)`) emits the residual
    // window's text the same way (sometimes empty).
    //
    // Strategy: append every non-empty event text into `accumulated`,
    // and use the accumulator as the final transcript. The server's
    // is_final flag only tells us when to stop the loop — it does
    // NOT mean "this event's text is the whole utterance"; treating
    // it that way drops every prior partial and leaves the user with
    // just the last syllable.
    let mut accumulated = String::new();
    let mut last_confidence = 0.0_f32;
    while let Some(ev_or_err) = asr_events.next().await {
        let ev = ev_or_err.map_err(|e| anyhow::anyhow!("asr_stream recv: {e}"))?;
        if !ev.error.is_empty() {
            anyhow::bail!("asr error: {}", ev.error);
        }
        let is_final = ev.is_final || ev.event_type == 1;
        let text = ev.text.clone();
        if !text.is_empty() {
            accumulated.push_str(&text);
            last_confidence = ev.confidence;
            if !is_final {
                let _ = tx
                    .send(Ok(event_text(
                        KIND_ASR_PARTIAL,
                        session_id,
                        &text,
                        ev.confidence,
                    )))
                    .await;
            }
        }
        if is_final {
            break;
        }
    }
    let transcript = accumulated;
    if !transcript.is_empty() {
        let _ = tx
            .send(Ok(event_text(
                KIND_ASR_FINAL,
                session_id,
                &transcript,
                last_confidence,
            )))
            .await;
    }

    // Stop the mic pump (the outer task already dropped asr_req_rx by
    // returning from the `while let` loop above, which closes the
    // request stream).
    pump_handle.abort();
    let _ = pump_handle.await;

    let pcm = std::mem::take(&mut *pcm_buf.lock().await);
    let _ = tx
        .send(Ok(event_status(
            KIND_RECORDING_DONE,
            session_id,
            &format!(
                "captured {} bytes (~{:.2}s @ 16kHz mono s16le); transcript={:?}",
                pcm.len(),
                pcm.len() as f32 / (DEFAULT_AUDIO_SAMPLE_RATE as f32 * 2.0),
                transcript,
            ),
        )))
        .await;
    Ok((pcm, transcript))
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
            return fallback;
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
                node_state: None,
                task_state: None,
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
                node_state: None,
                task_state: None,
            },
            &mut buf,
        );
        assert_eq!(buf, "hello world");
    }
}
