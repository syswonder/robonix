// SPDX-License-Identifier: MulanPSL-2.0
// Speech-to-Text Module
//
// Aliyun Intelligent Speech STT service

use crate::config::SpeechConfig;
use anyhow::{Context, Result};
use reqwest;
use serde::Deserialize;

pub struct SttService {
    config: SpeechConfig,
    client: reqwest::Client,
}

#[derive(Debug, Deserialize)]
pub struct SttResponse {
    pub task_id: Option<String>,
    pub result: Option<String>,
    pub status: u32,
    pub message: String,
}

impl SttService {
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
            "https://nls-gateway-cn-{}.aliyuncs.com/stream/v1/asr",
            region
        )
    }

    pub async fn recognize(
        &self,
        audio_data: &[u8],
        format: Option<&str>,
        sample_rate: Option<u32>,
    ) -> Result<String> {
        use log::{info, warn};

        let url = self.get_gateway_url();
        let format = format.unwrap_or("wav");
        let sample_rate = sample_rate.unwrap_or(16000);

        info!(
            "STT recognize: url={}, format={}, sample_rate={}, audio_size={}",
            url,
            format,
            sample_rate,
            audio_data.len()
        );

        // Build query parameters
        let query_params = vec![
            ("appkey", self.config.appkey.clone()),
            ("format", format.to_string()),
            ("sample_rate", sample_rate.to_string()),
            ("enable_punctuation_prediction", "true".to_string()),
        ];

        let query_string = query_params
            .iter()
            .map(|(k, v)| format!("{}={}", k, v))
            .collect::<Vec<_>>()
            .join("&");

        let full_url = format!("{}?{}", url, query_string);
        info!("STT request URL: {}", full_url);

        let response = self
            .client
            .post(&full_url)
            .header("X-NLS-Token", &self.config.access_token)
            .header("Content-Type", "application/octet-stream")
            .header("Content-Length", audio_data.len())
            .body(audio_data.to_vec())
            .send()
            .await
            .context("Failed to send STT request")?;

        let status = response.status();
        info!("STT HTTP response status: {}", status);

        if !status.is_success() {
            let error_text = response.text().await.unwrap_or_default();
            warn!("STT HTTP error response: {}", error_text);
            anyhow::bail!("STT HTTP error ({}): {}", status, error_text);
        }

        let result: SttResponse = response
            .json()
            .await
            .context("Failed to parse STT response")?;

        info!(
            "STT response: status={}, task_id={:?}, message={}",
            result.status, result.task_id, result.message
        );

        if result.status == 20000000 {
            // Success
            let result_text = result.result.unwrap_or_default();
            info!("STT recognition result: {}", result_text);
            Ok(result_text)
        } else {
            warn!(
                "STT error: status={}, message={}, task_id={:?}",
                result.status, result.message, result.task_id
            );
            anyhow::bail!("STT error (status: {}): {}", result.status, result.message);
        }
    }
}
