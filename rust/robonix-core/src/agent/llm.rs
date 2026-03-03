// SPDX-License-Identifier: MulanPSL-2.0
// LLM Client Module
//
// LLM API client for agent interactions

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use serde_json::json;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentConfig {
    pub llm_provider: String,
    pub api_key: Option<String>,
    pub api_base: String,
    pub model: String,
    pub temperature: f64,
}

impl Default for AgentConfig {
    fn default() -> Self {
        Self {
            llm_provider: "qwen".to_string(),
            api_key: None,
            // 华北2（北京）地域，与百炼/阿里云控制台一致；新加坡用 dashscope-intl.aliyuncs.com
            api_base: "https://dashscope.aliyuncs.com/compatible-mode/v1".to_string(),
            model: "qwen3-vl-plus".to_string(),
            temperature: 0.7,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChatMessage {
    pub role: String,
    pub content: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FunctionCall {
    pub name: String,
    pub arguments: serde_json::Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LLMResponse {
    pub message: String,
    pub function_calls: Vec<FunctionCall>,
}

pub struct LLMClient {
    config: AgentConfig,
    client: reqwest::Client,
}

impl LLMClient {
    pub fn new(config: AgentConfig) -> Self {
        Self {
            config,
            client: reqwest::Client::new(),
        }
    }

    pub async fn chat(
        &self,
        messages: Vec<ChatMessage>,
        functions: Vec<serde_json::Value>,
    ) -> Result<LLMResponse> {
        match self.config.llm_provider.as_str() {
            "deepseek" => self.chat_openai_compat(messages, functions).await,
            "qwen" => self.chat_openai_compat(messages, functions).await,
            _ => anyhow::bail!("Unsupported LLM provider: {}", self.config.llm_provider),
        }
    }

    /// OpenAI-compatible chat completions (used by DeepSeek and Qwen).
    async fn chat_openai_compat(
        &self,
        messages: Vec<ChatMessage>,
        functions: Vec<serde_json::Value>,
    ) -> Result<LLMResponse> {
        let api_key = self
            .config
            .api_key
            .as_ref()
            .context("API key not configured")?;

        // If api_base already ends with /v1 (e.g. DashScope .../compatible-mode/v1), use /chat/completions; else /v1/chat/completions (e.g. DeepSeek)
        let base = self.config.api_base.trim_end_matches('/');
        let path = if base.ends_with("/v1") {
            "/chat/completions"
        } else {
            "/v1/chat/completions"
        };
        let url = format!("{}{}", base, path);

        let mut request_body = json!({
            "model": self.config.model,
            "messages": messages,
            "temperature": self.config.temperature,
            "max_tokens": 1024,
        });

        if !functions.is_empty() {
            request_body["tools"] = json!(functions);
            request_body["tool_choice"] = json!("auto");
        }

        let response = self
            .client
            .post(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .header("Content-Type", "application/json")
            .json(&request_body)
            .send()
            .await
            .context("Failed to send request to LLM API")?;

        if !response.status().is_success() {
            let status = response.status();
            let text = response.text().await.unwrap_or_default();
            anyhow::bail!("LLM API error: {} - {}", status, text);
        }

        let response_json: serde_json::Value = response
            .json()
            .await
            .context("Failed to parse LLM API response")?;

        // Parse response
        let choices = response_json
            .get("choices")
            .and_then(|c| c.as_array())
            .context("Invalid response format: missing choices")?;

        if choices.is_empty() {
            anyhow::bail!("No choices in LLM response");
        }

        let choice = &choices[0];
        let message = choice
            .get("message")
            .context("Invalid response format: missing message")?;

        // Extract text content
        let content = message
            .get("content")
            .and_then(|c| c.as_str())
            .unwrap_or("")
            .to_string();

        // Extract function calls
        let mut function_calls = Vec::new();
        if let Some(tool_calls) = message.get("tool_calls") {
            if let Some(tool_calls_array) = tool_calls.as_array() {
                for tool_call in tool_calls_array {
                    if let Some(function) = tool_call.get("function") {
                        let name = function
                            .get("name")
                            .and_then(|n| n.as_str())
                            .context("Invalid tool call: missing name")?
                            .to_string();
                        let arguments_str = function
                            .get("arguments")
                            .and_then(|a| a.as_str())
                            .unwrap_or("{}");
                        let arguments: serde_json::Value = serde_json::from_str(arguments_str)
                            .context("Failed to parse function arguments")?;
                        function_calls.push(FunctionCall { name, arguments });
                    }
                }
            }
        }

        Ok(LLMResponse {
            message: content,
            function_calls,
        })
    }

    /// Call Qwen3-VL (or compatible VLM) with images and a text question.
    /// Each element of image_base64_urls should be "data:image/jpeg;base64,...".
    pub async fn chat_vision(
        &self,
        image_base64_urls: Vec<String>,
        question: &str,
    ) -> Result<String> {
        let api_key = self
            .config
            .api_key
            .as_ref()
            .context("API key not configured for VLM")?;

        // If api_base already ends with /v1 (e.g. DashScope .../compatible-mode/v1), use /chat/completions; else /v1/chat/completions (e.g. DeepSeek)
        let base = self.config.api_base.trim_end_matches('/');
        let path = if base.ends_with("/v1") {
            "/chat/completions"
        } else {
            "/v1/chat/completions"
        };
        let url = format!("{}{}", base, path);

        // Build multimodal user content: image_url entries + text
        let mut content: Vec<serde_json::Value> = Vec::new();
        for url in &image_base64_urls {
            content.push(json!({
                "type": "image_url",
                "image_url": { "url": url }
            }));
        }
        content.push(json!({
            "type": "text",
            "text": question
        }));

        let mut request_body = json!({
            "model": "qwen3-vl-plus",
            "messages": [
                {
                    "role": "user",
                    "content": content
                }
            ],
            "temperature": self.config.temperature,
            "max_tokens": 2048,
        });

        // Qwen thinking (optional)
        if self.config.llm_provider == "qwen" {
            request_body["extra_body"] = json!({
                "enable_thinking": false,
                "thinking_budget": 8192
            });
        }

        let response = self
            .client
            .post(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .header("Content-Type", "application/json")
            .json(&request_body)
            .send()
            .await
            .context("Failed to send request to VLM API")?;

        if !response.status().is_success() {
            let status = response.status();
            let text = response.text().await.unwrap_or_default();
            anyhow::bail!("VLM API error: {} - {}", status, text);
        }

        let response_json: serde_json::Value = response
            .json()
            .await
            .context("Failed to parse VLM API response")?;

        let choices = response_json
            .get("choices")
            .and_then(|c| c.as_array())
            .context("Invalid response format: missing choices")?;

        if choices.is_empty() {
            anyhow::bail!("No choices in VLM response");
        }

        let content = choices[0]
            .get("message")
            .and_then(|m| m.get("content"))
            .and_then(|c| c.as_str())
            .unwrap_or("")
            .to_string();

        Ok(content)
    }
}
