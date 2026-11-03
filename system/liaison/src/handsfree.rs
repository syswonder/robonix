// SPDX-License-Identifier: MulanPSL-2.0
// Robot-local voice mode orchestration. This module contains no speech
// algorithm: Audio provides PCM, Speech performs KWS/ASR/TTS, Voiceprint
// identifies the speaker, and Liaison only connects those capabilities.

use anyhow::{Context, Result, anyhow};
use robonix_atlas::client::AtlasClient;
use robonix_scribe::{info, warn};
use serde::Deserialize;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::Duration;
use tokio::sync::{Mutex, Notify, broadcast, mpsc};
use tokio::task::JoinHandle;
use tokio_stream::{StreamExt, wrappers::ReceiverStream};
use tonic::Request;

use crate::access::AccessControlConfig;
use crate::pb::contracts::{
    robonix_primitive_audio_mic_client::RobonixPrimitiveAudioMicClient,
    robonix_service_speech_wake_word_client::RobonixServiceSpeechWakeWordClient,
};
use crate::pb::liaison::{GetHandsfreeStatusResponse, StartVoiceSessionRequest, VoiceEvent};
use crate::{voice, voice::KIND_ASR_FINAL};

const MIC_CONTRACT: &str = "robonix/primitive/audio/mic";
const WAKE_WORD_CONTRACT: &str = "robonix/service/speech/wake_word";

#[derive(Clone, Debug, Deserialize)]
#[serde(default)]
pub struct HandsfreeConfig {
    pub handsfree_enabled: bool,
    pub handsfree_ack_text: String,
    pub handsfree_mic_provider_id: String,
    pub handsfree_speech_provider_id: String,
    pub handsfree_voiceprint_provider_id: String,
    pub handsfree_speaker_provider_id: String,
    pub handsfree_session_id: String,
    pub handsfree_record_seconds: u32,
}

impl Default for HandsfreeConfig {
    fn default() -> Self {
        Self {
            handsfree_enabled: false,
            handsfree_ack_text: "\u{6211}\u{5728}".to_string(),
            handsfree_mic_provider_id: String::new(),
            handsfree_speech_provider_id: "speech".to_string(),
            handsfree_voiceprint_provider_id: "voiceprint".to_string(),
            handsfree_speaker_provider_id: String::new(),
            handsfree_session_id: "handsfree".to_string(),
            handsfree_record_seconds: 20,
        }
    }
}

#[derive(Default)]
struct RuntimeState {
    state: String,
    keyword: String,
    last_wake_ms: u64,
    last_transcript: String,
    last_error: String,
}

pub struct HandsfreeController {
    enabled: AtomicBool,
    suspended: AtomicUsize,
    config: Mutex<HandsfreeConfig>,
    state: Mutex<RuntimeState>,
    changed: Notify,
    atlas: Arc<Mutex<AtlasClient>>,
    pilot_endpoint_default: String,
    access: Arc<AccessControlConfig>,
    events: broadcast::Sender<VoiceEvent>,
    active_turns: Mutex<Vec<JoinHandle<()>>>,
}

impl HandsfreeController {
    pub fn new(
        config: HandsfreeConfig,
        atlas: Arc<Mutex<AtlasClient>>,
        pilot_endpoint_default: String,
        access: Arc<AccessControlConfig>,
    ) -> Arc<Self> {
        let enabled = config.handsfree_enabled
            && !config.handsfree_mic_provider_id.is_empty()
            && !config.handsfree_speaker_provider_id.is_empty();
        let (events, _) = broadcast::channel(256);
        Arc::new(Self {
            enabled: AtomicBool::new(enabled),
            suspended: AtomicUsize::new(0),
            config: Mutex::new(config),
            state: Mutex::new(RuntimeState {
                state: if enabled { "starting" } else { "disabled" }.to_string(),
                ..RuntimeState::default()
            }),
            changed: Notify::new(),
            atlas,
            pilot_endpoint_default,
            access,
            events,
            active_turns: Mutex::new(Vec::new()),
        })
    }

    pub fn spawn(self: &Arc<Self>) {
        let controller = Arc::clone(self);
        tokio::spawn(async move { controller.run().await });
    }

    pub async fn set_enabled(
        &self,
        enabled: bool,
        mic_provider_id: String,
        speaker_provider_id: String,
    ) -> Result<GetHandsfreeStatusResponse> {
        {
            let mut config = self.config.lock().await;
            if !mic_provider_id.trim().is_empty() {
                config.handsfree_mic_provider_id = mic_provider_id;
            }
            if !speaker_provider_id.trim().is_empty() {
                config.handsfree_speaker_provider_id = speaker_provider_id;
            }
            if enabled
                && (config.handsfree_mic_provider_id.trim().is_empty()
                    || config.handsfree_speaker_provider_id.trim().is_empty())
            {
                anyhow::bail!(
                    "select both an input and an output audio primitive before enabling hands-free mode"
                );
            }
        }
        self.enabled.store(enabled, Ordering::Release);
        if !enabled {
            let mut turns = self.active_turns.lock().await;
            for turn in turns.drain(..) {
                turn.abort();
            }
        }
        self.set_state(if enabled { "starting" } else { "disabled" }, None)
            .await;
        self.changed.notify_waiters();
        Ok(self.snapshot().await)
    }

    pub async fn snapshot(&self) -> GetHandsfreeStatusResponse {
        let config = self.config.lock().await.clone();
        let state = self.state.lock().await;
        GetHandsfreeStatusResponse {
            enabled: self.enabled.load(Ordering::Acquire),
            state: state.state.clone(),
            keyword: state.keyword.clone(),
            mic_provider_id: config.handsfree_mic_provider_id,
            speaker_provider_id: config.handsfree_speaker_provider_id,
            last_wake_ms: state.last_wake_ms,
            last_transcript: state.last_transcript.clone(),
            last_error: state.last_error.clone(),
        }
    }

    /// Pause only wake-word microphone ownership while an explicit voice
    /// capture is active.  Enabled state is preserved and listening resumes
    /// automatically at ASR final (or on any failed/disconnected capture).
    pub async fn suspend_capture(&self) {
        self.suspended.fetch_add(1, Ordering::AcqRel);
        self.set_state("suspended", None).await;
        self.changed.notify_waiters();
    }

    pub async fn resume_capture(&self) {
        let mut current = self.suspended.load(Ordering::Acquire);
        while current > 0 {
            match self.suspended.compare_exchange_weak(
                current,
                current - 1,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => break,
                Err(actual) => current = actual,
            }
        }
        if self.suspended.load(Ordering::Acquire) == 0 {
            self.changed.notify_waiters();
        }
    }

    /// Subscribe to the existing VoiceEvent model rather than inventing a
    /// hands-free-specific client protocol. This stream is observational and
    /// never drives microphone ownership or task execution.
    pub fn subscribe_events(&self) -> ReceiverStream<Result<VoiceEvent, tonic::Status>> {
        let mut source = self.events.subscribe();
        let (tx, rx) = mpsc::channel(64);
        tokio::spawn(async move {
            loop {
                match source.recv().await {
                    Ok(event) => {
                        if tx.send(Ok(event)).await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(skipped)) => {
                        warn!("[liaison/handsfree] event observer lagged by {skipped} event(s)");
                    }
                    Err(broadcast::error::RecvError::Closed) => break,
                }
            }
        });
        ReceiverStream::new(rx)
    }

    fn publish(&self, event: VoiceEvent) {
        // A disconnected UI must never block the robot-local interaction.
        let _ = self.events.send(event);
    }

    async fn publish_error(&self, session_id: &str, message: &str) {
        self.state.lock().await.last_error = message.to_string();
        match voice::event_error(session_id, message) {
            Ok(event) => self.publish(event),
            Err(status) => warn!("[liaison/handsfree] cannot timestamp error event: {status}"),
        }
    }

    async fn set_state(&self, value: &str, error: Option<String>) {
        let mut state = self.state.lock().await;
        state.state = value.to_string();
        if let Some(error) = error {
            state.last_error = error;
        }
    }

    async fn run(self: Arc<Self>) {
        loop {
            if !self.enabled.load(Ordering::Acquire) {
                self.set_state("disabled", None).await;
                self.changed.notified().await;
                continue;
            }
            if self.suspended.load(Ordering::Acquire) > 0 {
                self.set_state("suspended", None).await;
                self.changed.notified().await;
                continue;
            }

            self.set_state("listening", None).await;
            let wait = self.wait_for_wake();
            tokio::pin!(wait);
            let wake_result = tokio::select! {
                result = &mut wait => Some(result),
                _ = self.changed.notified() => None,
            };
            let Some(wake_result) = wake_result else {
                continue;
            };

            match wake_result {
                Ok(Some(keyword)) => {
                    let timestamp = match voice::now_ms() {
                        Ok(timestamp) => timestamp,
                        Err(error) => {
                            warn!("[liaison/handsfree] wake timestamp failed: {error}");
                            self.set_state("error", Some(error.to_string())).await;
                            tokio::time::sleep(Duration::from_secs(1)).await;
                            continue;
                        }
                    };
                    {
                        let mut state = self.state.lock().await;
                        state.state = "triggered".to_string();
                        state.keyword = keyword.clone();
                        state.last_wake_ms = timestamp;
                        state.last_error.clear();
                    }
                    info!("[liaison/handsfree] wake phrase detected: {keyword}");
                    if let Err(error) = self.run_voice_turn().await {
                        let message = format!("{error:#}");
                        warn!("[liaison/handsfree] voice turn failed: {message}");
                        let session_id = self.config.lock().await.handsfree_session_id.clone();
                        self.publish_error(&session_id, &message).await;
                        self.set_state("error", Some(message)).await;
                        tokio::time::sleep(Duration::from_secs(1)).await;
                    }
                }
                Ok(None) => {
                    self.set_state("listening", None).await;
                    tokio::time::sleep(Duration::from_millis(200)).await;
                }
                Err(error) => {
                    let message = format!("{error:#}");
                    warn!("[liaison/handsfree] wake listener failed: {message}");
                    self.set_state("error", Some(message)).await;
                    tokio::time::sleep(Duration::from_secs(1)).await;
                }
            }
        }
    }

    async fn wait_for_wake(&self) -> Result<Option<String>> {
        let config = self.config.lock().await.clone();
        let mic_endpoint =
            voice::resolve_endpoint(&self.atlas, MIC_CONTRACT, &config.handsfree_mic_provider_id)
                .await
                .ok_or_else(|| anyhow!("no mic provider '{}'", config.handsfree_mic_provider_id))?;
        let wake_endpoint = voice::resolve_endpoint(
            &self.atlas,
            WAKE_WORD_CONTRACT,
            &config.handsfree_speech_provider_id,
        )
        .await
        .ok_or_else(|| {
            anyhow!(
                "no speech wake-word provider '{}'",
                config.handsfree_speech_provider_id
            )
        })?;

        let mut mic = RobonixPrimitiveAudioMicClient::connect(mic_endpoint.clone())
            .await
            .with_context(|| format!("connect mic at {mic_endpoint}"))?;
        let mut wake = RobonixServiceSpeechWakeWordClient::connect(wake_endpoint.clone())
            .await
            .with_context(|| format!("connect wake-word service at {wake_endpoint}"))?;
        let mut mic_stream = mic.mic(Request::new(())).await?.into_inner();
        let (tx, rx) = mpsc::channel(64);
        let pump = tokio::spawn(async move {
            while let Ok(Some(chunk)) = mic_stream.message().await {
                if tx.send(chunk).await.is_err() {
                    break;
                }
            }
        });
        let response = wake
            .detect_wake_word(Request::new(ReceiverStream::new(rx)))
            .await;
        pump.abort();
        let response = response?.into_inner();
        if !response.error.is_empty() {
            return Err(anyhow!(response.error));
        }
        Ok(response.detected.then_some(response.keyword))
    }

    async fn run_voice_turn(self: &Arc<Self>) -> Result<()> {
        let config = self.config.lock().await.clone();
        let is_steer = self.has_active_turn().await;
        let ack = config.handsfree_ack_text.trim();
        // A mid-task wake phrase is itself the acknowledgement.  Starting a
        // second prompt here would queue behind the reply the user is trying
        // to interrupt and delay capture by several seconds.
        if !is_steer && !ack.is_empty() {
            self.set_state("acknowledging", None).await;
            if let Err(error) = voice::play_prompt(
                &self.atlas,
                &config.handsfree_speech_provider_id,
                &config.handsfree_speaker_provider_id,
                "",
                ack,
                &config.handsfree_session_id,
            )
            .await
            {
                let message = format!("wake acknowledgement failed: {error:#}");
                warn!("[liaison/handsfree] {message}");
                self.state.lock().await.last_error = message;
            }
        }
        self.set_state("in_voice", None).await;
        let request = StartVoiceSessionRequest {
            session_id: config.handsfree_session_id.clone(),
            client_user_id: "voice:anyone".to_string(),
            record_seconds: config.handsfree_record_seconds,
            language: String::new(),
            tts_enabled: true,
            mic_node_id: config.handsfree_mic_provider_id.clone(),
            asr_node_id: config.handsfree_speech_provider_id.clone(),
            voiceprint_node_id: config.handsfree_voiceprint_provider_id.clone(),
            tts_node_id: config.handsfree_speech_provider_id.clone(),
            speaker_node_id: config.handsfree_speaker_provider_id.clone(),
            context_json: if is_steer {
                r#"{"client":"robot-handsfree","interaction_mode":"steer","steer":true,"barge_in":true,"handsfree":true}"#.to_string()
            } else {
                r#"{"client":"robot-handsfree","interaction_mode":"auto","barge_in":false,"handsfree":true}"#.to_string()
            },
        };
        let mut stream = voice::start_voice_session(
            request,
            Arc::clone(&self.atlas),
            self.pilot_endpoint_default.clone(),
            Arc::clone(&self.access),
        )
        .await
        .map_err(|status| anyhow!(status.to_string()))?;
        while let Some(event) = stream.next().await {
            let event = event.map_err(|status| anyhow!(status.to_string()))?;
            if event.event_kind == KIND_ASR_FINAL {
                self.state.lock().await.last_transcript = event.text.clone();
            }
            self.publish(event.clone());
            if !event.error.is_empty() {
                return Err(anyhow!(event.error));
            }
            if event.event_kind == KIND_ASR_FINAL {
                // Recording ownership ends at ASR final, not when Pilot and
                // TTS eventually finish.  Continue relaying that turn in the
                // background and immediately restore wake-word listening so
                // the user can steer a long-running action.
                let controller = Arc::clone(self);
                let session_id = config.handsfree_session_id.clone();
                let handle = tokio::spawn(async move {
                    while let Some(item) = stream.next().await {
                        match item {
                            Ok(event) => {
                                let failed = !event.error.is_empty();
                                controller.publish(event.clone());
                                if failed {
                                    warn!(
                                        "[liaison/handsfree] background voice turn failed: {}",
                                        event.error
                                    );
                                    break;
                                }
                            }
                            Err(status) => {
                                controller
                                    .publish_error(&session_id, &status.to_string())
                                    .await;
                                warn!(
                                    "[liaison/handsfree] background voice stream failed: {status}"
                                );
                                break;
                            }
                        }
                    }
                });
                self.active_turns.lock().await.push(handle);
                if self.enabled.load(Ordering::Acquire) {
                    self.set_state("listening", None).await;
                }
                return Ok(());
            }
        }
        Err(anyhow!("voice session ended before ASR final"))
    }

    async fn has_active_turn(&self) -> bool {
        let mut turns = self.active_turns.lock().await;
        turns.retain(|turn| !turn.is_finished());
        !turns.is_empty()
    }
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
    use super::*;

    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../chronos/clock/tests/support/mod.rs"
    ));

    #[test]
    fn observers_survive_turn_errors_and_clock_outages() {
        run_case(
            "handsfree::tests::observers_survive_turn_errors_and_clock_outages",
            || {
                tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .unwrap()
                    .block_on(async {
                        let reserved = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
                        let address = reserved.local_addr().unwrap();
                        drop(reserved);
                        let server = tokio::spawn(robonix_atlas::service::serve_atlas(
                            Arc::new(robonix_atlas::service::AtlasRegistry::default()),
                            address,
                        ));
                        let atlas = AtlasClient::connect_with_retry(
                            address.to_string(),
                            50,
                            Duration::from_millis(10),
                        )
                        .await
                        .unwrap();
                        let controller = HandsfreeController::new(
                            HandsfreeConfig::default(),
                            Arc::new(Mutex::new(atlas)),
                            String::new(),
                            Arc::new(AccessControlConfig::from_env()),
                        );
                        let mut first = controller.subscribe_events();
                        let mut second = controller.subscribe_events();
                        controller
                            .publish_error("failed-turn", "upstream unavailable")
                            .await;
                        for observer in [&mut first, &mut second] {
                            let event =
                                tokio::time::timeout(Duration::from_secs(2), observer.next())
                                    .await
                                    .unwrap()
                                    .unwrap()
                                    .unwrap();
                            assert_eq!(event.session_id, "failed-turn");
                            assert_eq!(event.error, "upstream unavailable");
                            assert!(event.timestamp_ms > 0);
                        }
                        mode(1);
                        controller
                            .publish_error("clock-failed-turn", "original failure")
                            .await;
                        controller.set_state("listening", None).await;
                        assert_eq!(controller.snapshot().await.last_error, "original failure");
                        mode(0);
                        let recovered = VoiceEvent {
                            session_id: "next-turn".into(),
                            timestamp_ms: voice::now_ms().unwrap(),
                            ..Default::default()
                        };
                        controller.publish(recovered.clone());
                        for observer in [&mut first, &mut second] {
                            let event =
                                tokio::time::timeout(Duration::from_secs(2), observer.next())
                                    .await
                                    .unwrap()
                                    .unwrap()
                                    .unwrap();
                            assert_eq!(event, recovered);
                        }
                        server.abort();
                        let _ = server.await;
                    });
            },
        );
    }
}
