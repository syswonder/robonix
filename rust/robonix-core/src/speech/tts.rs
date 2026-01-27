// SPDX-License-Identifier: MulanPSL-2.0
// Text-to-Speech Module
//
// Aliyun Intelligent Speech TTS service

use crate::config::SpeechConfig;
use anyhow::{Context, Result};
use reqwest;
use serde::{Deserialize, Serialize};

pub struct TtsService {
    config: SpeechConfig,
    client: reqwest::Client,
}

#[derive(Debug, Serialize)]
struct TtsRequest {
    appkey: String,
    text: String,
    token: String,
    format: String,
    sample_rate: u32,
    voice: String,
    volume: u32,
    speech_rate: i32,
    pitch_rate: i32,
}

#[derive(Debug, Deserialize)]
struct TtsErrorResponse {
    task_id: Option<String>,
    result: Option<String>,
    status: u32,
    message: String,
}

impl TtsService {
    pub fn new(config: SpeechConfig) -> Self {
        Self {
            config,
            client: reqwest::Client::new(),
        }
    }

    fn get_gateway_url(&self) -> String {
        let region = match self.config.region.as_str() {
            "beijing" => "beijing",
            "shenzhen" => "shenzhen",
            _ => "shanghai",
        };
        format!(
            "https://nls-gateway-cn-{}.aliyuncs.com/stream/v1/tts",
            region
        )
    }

    pub async fn synthesize(
        &self,
        text: &str,
        format: Option<&str>,
        voice: Option<&str>,
    ) -> Result<Vec<u8>> {
        let url = self.get_gateway_url();

        let format = format.unwrap_or("wav");
        let voice = voice.unwrap_or("zhishuo");

        // Use POST method
        let request_body = TtsRequest {
            appkey: self.config.appkey.clone(),
            text: text.to_string(),
            token: self.config.access_token.clone(),
            format: format.to_string(),
            sample_rate: 16000,
            voice: voice.to_string(),
            volume: 50,
            speech_rate: 0,
            pitch_rate: 0,
        };

        let response = self
            .client
            .post(&url)
            .header("Content-Type", "application/json")
            .json(&request_body)
            .send()
            .await
            .context("Failed to send TTS request")?;

        let content_type = response
            .headers()
            .get("Content-Type")
            .and_then(|h| h.to_str().ok())
            .unwrap_or("");

        if content_type.starts_with("audio/") {
            // Success - return audio data
            let audio_data = response
                .bytes()
                .await
                .context("Failed to read audio data")?;
            Ok(audio_data.to_vec())
        } else {
            // Error response
            let error: TtsErrorResponse = response
                .json()
                .await
                .context("Failed to parse error response")?;
            anyhow::bail!("TTS error (status: {}): {}", error.status, error.message);
        }
    }
}
