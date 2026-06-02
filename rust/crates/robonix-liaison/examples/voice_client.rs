// SPDX-License-Identifier: MulanPSL-2.0
// Minimal non-interactive client for RobonixSystemLiaisonVoice.
//
// Useful for smoke-testing a running Robonix stack:
//   cargo run -p robonix-liaison --example voice_client

use anyhow::Result;
use robonix_liaison::pb::contracts::robonix_system_liaison_voice_client::RobonixSystemLiaisonVoiceClient;
use robonix_liaison::pb::liaison::{StartVoiceSessionRequest, VoiceEvent};
use tokio_stream::StreamExt;
use uuid::Uuid;

const KIND_ASR_FINAL: u32 = 4;
const KIND_SESSION_DONE: u32 = 9;
const KIND_ERROR: u32 = 10;

#[tokio::main]
async fn main() -> Result<()> {
    let endpoint = std::env::var("ROBONIX_LIAISON_ENDPOINT")
        .unwrap_or_else(|_| "http://127.0.0.1:50081".to_string());
    let record_seconds = std::env::var("ROBONIX_VOICE_TEST_RECORD_SECONDS")
        .ok()
        .and_then(|v| v.parse::<u32>().ok())
        .unwrap_or(8);

    let req = StartVoiceSessionRequest {
        session_id: Uuid::new_v4().to_string(),
        client_user_id: "voice-test".to_string(),
        record_seconds,
        language: String::new(),
        tts_enabled: false,
        mic_node_id: std::env::var("ROBONIX_CHAT_MIC_NODE").unwrap_or_default(),
        asr_node_id: String::new(),
        voiceprint_node_id: String::new(),
        tts_node_id: String::new(),
        speaker_node_id: std::env::var("ROBONIX_CHAT_SPEAKER_NODE").unwrap_or_default(),
        context_json: r#"{"test":"liaison_asr"}"#.to_string(),
    };

    let mut client = RobonixSystemLiaisonVoiceClient::connect(endpoint.clone()).await?;
    let mut stream = client
        .start_voice_session(tonic::Request::new(req))
        .await?
        .into_inner();

    let mut asr_final = String::new();
    let mut saw_done = false;
    while let Some(item) = stream.next().await {
        let event = item?;
        print_event(&event);
        match event.event_kind {
            KIND_ASR_FINAL => asr_final = event.text.clone(),
            KIND_SESSION_DONE => saw_done = true,
            KIND_ERROR => anyhow::bail!("voice session error: {}", event.error),
            _ => {}
        }
    }

    if asr_final.trim().is_empty() {
        anyhow::bail!("no ASR_FINAL text received");
    }
    if !saw_done {
        anyhow::bail!("voice session ended without SESSION_DONE");
    }
    println!("ASR_FINAL={asr_final}");
    Ok(())
}

fn print_event(event: &VoiceEvent) {
    let kind = match event.event_kind {
        0 => "SESSION_STARTED",
        1 => "RECORDING_STARTED",
        2 => "RECORDING_DONE",
        3 => "ASR_PARTIAL",
        4 => "ASR_FINAL",
        5 => "USER_IDENTIFIED",
        6 => "PILOT",
        7 => "TTS_STARTED",
        8 => "TTS_DONE",
        9 => "SESSION_DONE",
        10 => "ERROR",
        _ => "UNKNOWN",
    };
    if !event.text.is_empty() {
        println!("{kind}: {}", event.text);
    } else if !event.status_message.is_empty() {
        println!("{kind}: {}", event.status_message);
    } else if !event.error.is_empty() {
        println!("{kind}: {}", event.error);
    } else if let Some(pilot) = &event.pilot {
        println!("{kind}: pilot_event kind={}", pilot.event_kind);
    } else {
        println!("{kind}");
    }
}
