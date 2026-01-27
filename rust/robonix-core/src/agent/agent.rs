// SPDX-License-Identifier: MulanPSL-2.0
// Agent Module
//
// Main agent implementation for natural language interaction

use crate::agent::functions::FunctionRegistry;
use crate::agent::llm::{ChatMessage, LLMClient};
use crate::core::RobonixCore;
use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::sync::Arc;

pub struct Agent {
    llm_client: LLMClient,
    function_registry: FunctionRegistry,
    conversation_history: Vec<ChatMessage>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentRequest {
    pub message: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentResponse {
    pub message: String,
    pub function_results: Vec<FunctionResult>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FunctionResult {
    pub name: String,
    pub result: Value,
}

impl Agent {
    pub fn new(core: Arc<RobonixCore>, config: crate::agent::llm::AgentConfig) -> Self {
        let llm_client = LLMClient::new(config);
        let function_registry = FunctionRegistry::new(core);

        // Initialize with system message
        let mut conversation_history = Vec::new();
        conversation_history.push(ChatMessage {
            role: "system".to_string(),
            content: "You are a helpful assistant for the Robonix robot system. You can help users query the semantic map, submit tasks, and query system capabilities. Always provide natural language responses and use function calls when appropriate.".to_string(),
        });

        Self {
            llm_client,
            function_registry,
            conversation_history,
        }
    }

    pub async fn chat(&mut self, request: AgentRequest) -> Result<AgentResponse> {
        // Add user message to history
        self.conversation_history.push(ChatMessage {
            role: "user".to_string(),
            content: request.message,
        });

        // Get function schemas
        let functions = self.function_registry.get_function_schemas();

        // Call LLM
        let llm_response = self
            .llm_client
            .chat(self.conversation_history.clone(), functions)
            .await
            .context("Failed to get LLM response")?;

        // Execute function calls
        let mut function_results = Vec::new();
        for function_call in &llm_response.function_calls {
            match self
                .function_registry
                .call_function(&function_call.name, function_call.arguments.clone())
                .await
            {
                Ok(result) => {
                    function_results.push(FunctionResult {
                        name: function_call.name.clone(),
                        result,
                    });
                }
                Err(e) => {
                    function_results.push(FunctionResult {
                        name: function_call.name.clone(),
                        result: json!({
                            "error": format!("Function execution failed: {}", e)
                        }),
                    });
                }
            }
        }

        // If there were function calls, add them to conversation and get final response
        let final_message = if !llm_response.function_calls.is_empty() {
            // Add assistant message with function calls
            let mut assistant_message = format!("{}\n\n", llm_response.message);
            for (i, function_call) in llm_response.function_calls.iter().enumerate() {
                if i < function_results.len() {
                    assistant_message.push_str(&format!(
                        "Function {} returned: {}\n",
                        function_call.name,
                        serde_json::to_string_pretty(&function_results[i].result)
                            .unwrap_or_default()
                    ));
                }
            }

            // Add to conversation
            self.conversation_history.push(ChatMessage {
                role: "assistant".to_string(),
                content: assistant_message.clone(),
            });

            // Get final response from LLM with function results
            let final_functions = self.function_registry.get_function_schemas();
            let final_response = self
                .llm_client
                .chat(self.conversation_history.clone(), final_functions)
                .await
                .context("Failed to get final LLM response")?;

            // Update conversation history
            self.conversation_history.push(ChatMessage {
                role: "assistant".to_string(),
                content: final_response.message.clone(),
            });

            final_response.message
        } else {
            // No function calls, just use the message
            self.conversation_history.push(ChatMessage {
                role: "assistant".to_string(),
                content: llm_response.message.clone(),
            });
            llm_response.message
        };

        Ok(AgentResponse {
            message: final_message,
            function_results,
        })
    }

    pub fn clear_history(&mut self) {
        self.conversation_history.clear();
        self.conversation_history.push(ChatMessage {
            role: "system".to_string(),
            content: "You are a helpful assistant for the Robonix robot system. You can help users query the semantic map, submit tasks, and query system capabilities. Always provide natural language responses and use function calls when appropriate.".to_string(),
        });
    }
}
