// SPDX-License-Identifier: MulanPSL-2.0
// Robot-local voice mode orchestration. This module contains no speech
// algorithm: Audio provides PCM, Speech performs KWS/ASR/TTS, Voiceprint
// identifies the speaker, and Liaison only connects those capabilities.

use anyhow::{Context, Result, anyhow};
use robonix_atlas::client::AtlasClient;
use robonix_scribe::{info, warn};
use serde::Deserialize;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tokio::sync::{Mutex, Notify, mpsc};
use tokio_stream::{StreamExt, wrappers::ReceiverStream};
use tonic::Request;

use crate::access::AccessControlConfig;
use crate::pb::contracts::{
    robonix_primitive_audio_mic_client::RobonixPrimitiveAudioMicClient,
    robonix_service_speech_wake_word_client::RobonixServiceSpeechWakeWordClient,
};
use crate::pb::liaison::{GetHandsfreeStatusResponse, StartVoiceSessionRequest};
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
            handsfree_ack_text: "我在".to_string(),
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
    config: Mutex<HandsfreeConfig>,
    state: Mutex<RuntimeState>,
    changed: Notify,
    atlas: Arc<Mutex<AtlasClient>>,
    pilot_endpoint_default: String,
    access: Arc<AccessControlConfig>,
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
        Arc::new(Self {
            enabled: AtomicBool::new(enabled),
            config: Mutex::new(config),
            state: Mutex::new(RuntimeState {
                state: if enabled { "starting" } else { "disabled" }.to_string(),
                ..RuntimeState::default()
            }),
            changed: Notify::new(),
            atlas,
            pilot_endpoint_default,
            access,
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

    async fn set_state(&self, value: &str, error: Option<String>) {
        let mut state = self.state.lock().await;
        state.state = value.to_string();
        if let Some(error) = error {
            state.last_error = error;
        } else if value == "listening" {
            // A connected microphone and a fresh wake listener mean an older
            // reconnect failure is no longer actionable in the client UI.
            state.last_error.clear();
        }
    }

    async fn run(self: Arc<Self>) {
        loop {
            if !self.enabled.load(Ordering::Acquire) {
                self.set_state("disabled", None).await;
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
                    {
                        let mut state = self.state.lock().await;
                        state.state = "triggered".to_string();
                        state.keyword = keyword.clone();
                        state.last_wake_ms = now_ms();
                        state.last_error.clear();
                    }
                    info!("[liaison/handsfree] wake phrase detected: {keyword}");
                    if let Err(error) = self.run_voice_turn().await {
                        let message = format!("{error:#}");
                        warn!("[liaison/handsfree] voice turn failed: {message}");
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

    async fn run_voice_turn(&self) -> Result<()> {
        let config = self.config.lock().await.clone();
        let ack = config.handsfree_ack_text.trim();
        if !ack.is_empty() {
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
            context_json:
                r#"{"client":"robot-handsfree","interaction_mode":"auto","handsfree":true}"#
                    .to_string(),
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
                self.state.lock().await.last_transcript = event.text;
            }
            if !event.error.is_empty() {
                return Err(anyhow!(event.error));
            }
        }
        if self.enabled.load(Ordering::Acquire) {
            self.set_state("listening", None).await;
        }
        Ok(())
    }
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}
