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
            llm_provider: "deepseek".to_string(),
            api_key: None,
            api_base: "https://api.deepseek.com".to_string(),
            model: "deepseek-chat".to_string(),
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
            "deepseek" => self.chat_deepseek(messages, functions).await,
            _ => anyhow::bail!("Unsupported LLM provider: {}", self.config.llm_provider),
        }
    }

    async fn chat_deepseek(
        &self,
        messages: Vec<ChatMessage>,
        functions: Vec<serde_json::Value>,
    ) -> Result<LLMResponse> {
        let api_key = self
            .config
            .api_key
            .as_ref()
            .context("API key not configured")?;

        let url = format!("{}/v1/chat/completions", self.config.api_base);

        let mut request_body = json!({
            "model": self.config.model,
            "messages": messages,
            "temperature": self.config.temperature,
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
}
