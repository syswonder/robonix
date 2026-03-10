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
        // Validate required fields
        if self.config.appkey.is_empty() {
            anyhow::bail!("TTS appkey is not configured");
        }
        if self.config.access_token.is_empty() {
            anyhow::bail!("TTS access_token is not configured");
        }
        if text.is_empty() {
            anyhow::bail!("TTS text cannot be empty");
        }
        if text.len() > 300 {
            log::warn!(
                "TTS text length {} exceeds 300 characters, will be truncated",
                text.len()
            );
        }

        let url = self.get_gateway_url();
        let format = format.unwrap_or("wav");
        let voice = voice.unwrap_or("zhishuo");

        // Use POST method - build request body according to RESTful API spec
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

        // Serialize to JSON - ensure proper UTF-8 encoding
        let json_body = serde_json::to_string(&request_body)
            .context("Failed to serialize TTS request to JSON")?;

        log::info!("TTS request URL: {}", url);
        log::debug!("TTS request JSON: {}", json_body);

        // Build POST request with proper headers
        let request_builder = self
            .client
            .post(&url)
            .header("Content-Type", "application/json");

        // Optionally set token in header (alternative to body)
        // According to docs, token can be in body or header X-NLS-Token
        // We're using body, but header is also supported

        let response = request_builder
            .body(json_body)
            .send()
            .await
            .context("Failed to send TTS request")?;

        let status = response.status();
        let content_type = response
            .headers()
            .get("Content-Type")
            .and_then(|h| h.to_str().ok())
            .unwrap_or("");

        log::debug!(
            "TTS response status: {}, Content-Type: {}",
            status,
            content_type
        );

        // According to docs: success when Content-Type is "audio/mpeg"
        // Failure when Content-Type is "application/json" or missing
        if content_type == "audio/mpeg" || content_type.starts_with("audio/") {
            // Success - return audio data
            let audio_data = response
                .bytes()
                .await
                .context("Failed to read audio data")?;
            log::info!("TTS synthesis succeeded: {} bytes", audio_data.len());
            Ok(audio_data.to_vec())
        } else {
            // Error response - read response body and try to parse as JSON
            let status_code = status.as_u16();
            let response_bytes = response
                .bytes()
                .await
                .context("Failed to read error response body")?;

            // Try to parse as JSON error response
            match serde_json::from_slice::<TtsErrorResponse>(&response_bytes) {
                Ok(error) => {
                    log::warn!(
                        "TTS error response: status={}, message={}",
                        error.status,
                        error.message
                    );
                    anyhow::bail!("TTS error (status: {}): {}", error.status, error.message);
                }
                Err(_) => {
                    // If JSON parsing fails, use raw response text
                    let response_text = String::from_utf8_lossy(&response_bytes);
                    log::warn!(
                        "TTS request failed: status={}, response={}",
                        status_code,
                        response_text
                    );
                    anyhow::bail!(
                        "TTS HTTP error (status: {}): {}",
                        status_code,
                        response_text
                    );
                }
            }
        }
    }
}
