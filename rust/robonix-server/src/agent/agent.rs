// SPDX-License-Identifier: MulanPSL-2.0
// Agent Module
//
// Main agent implementation for natural language interaction

use crate::agent::functions::FunctionRegistry;
use crate::agent::llm::{AgentConfig, ChatMessage, LLMClient};
use crate::core::RobonixCore;
use crate::perception::image_monitor::ImageMonitor;
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
    pub function_calls: Vec<FunctionCallInfo>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FunctionCallInfo {
    pub name: String,
    pub arguments: Value,
    pub result: Value,
}

impl Agent {
    fn create_system_message() -> ChatMessage {
        ChatMessage {
            role: "system".to_string(),
            content: concat!(
                "You are a helpful and friendly assistant for the Robonix robot system. ",
                "Keep responses concise: answer directly and avoid long paragraphs or unnecessary elaboration unless the user explicitly asks for detail.\n\n",
                "You can help users in two ways:\n\n",
                "1. Robonix system operations: Help users query the semantic map, submit tasks, ",
                "query system capabilities, describe what the robot sees (using describe_robot_vision when the user asks about the robot's view or current scene), and perform robot operations using function calls when appropriate.\n\n",
                "2. General conversation: You can also engage in general conversations, answer questions, ",
                "provide introductions, explain concepts, and have friendly chats with users. ",
                "Don't limit yourself to only robot-related topics - be helpful and conversational ",
                "about any topic the user asks about.\n\n",
                "Always provide natural language responses and use function calls only when the user ",
                "needs to interact with the Robonix system. For general questions, conversations, ",
                "or introductions, respond naturally without function calls.\n\n",
                "IMPORTANT FORMATTING RULES:\n",
                "- Return only plain text without any markdown formatting (no **, ##, ```, etc.)\n",
                "- Write in complete, natural sentences and paragraphs only\n",
                "- NEVER use bullet points, numbered lists, or any list format (no \"1.\", \"2.\", \"-\", etc.)\n",
                "- NEVER use colons followed by line breaks or lists (no \"可能意味着：\\n\" or similar patterns)\n",
                "- When presenting multiple points, connect them with words like \"and\", \"also\", \"additionally\", ",
                "or write separate sentences\n",
                "- Ensure exactly one space between words and sentences - no extra spaces anywhere\n",
                "- Write as if you're having a natural conversation, not formatting a document\n",
                "- Example BAD (Chinese): \"可能意味着：\\n机器人当前所在的区域比较空旷\\n或者语义地图还没有加载\"\n",
                "- Example GOOD (Chinese): \"这可能意味着机器人当前所在的区域比较空旷，或者语义地图还没有加载。\"\n",
                "- Example BAD (English): \"This may mean:\\nThe robot is in an empty area\\nor the semantic map hasn't loaded\"\n",
                "- Example GOOD (English): \"This may mean the robot is in an empty area, or the semantic map hasn't loaded yet.\"\n",
                "- Example BAD (Chinese): \"活跃任务：0个\\n注册的基础操作：0个\"\n",
                "- Example GOOD (Chinese): \"当前系统中有0个活跃任务和0个注册的基础操作。\"\n",
                "- Example BAD (English): \"Active tasks: 0\\nRegistered primitives: 0\"\n",
                "- Example GOOD (English): \"Currently there are 0 active tasks and 0 registered primitives in the system.\""
            ).to_string(),
        }
    }

    pub fn new(
        core: Arc<RobonixCore>,
        config: AgentConfig,
        image_monitor: Arc<ImageMonitor>,
    ) -> Self {
        let llm_client = LLMClient::new(config.clone());
        let function_registry = FunctionRegistry::new(core, image_monitor, config);

        // Initialize with system message
        let mut conversation_history = Vec::new();
        conversation_history.push(Self::create_system_message());

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

        // Execute function calls and collect results
        let mut function_calls_info = Vec::new();
        for function_call in &llm_response.function_calls {
            let result = match self
                .function_registry
                .call_function(&function_call.name, function_call.arguments.clone())
                .await
            {
                Ok(result) => result,
                Err(e) => {
                    json!({
                        "error": format!("Function execution failed: {}", e)
                    })
                }
            };

            function_calls_info.push(FunctionCallInfo {
                name: function_call.name.clone(),
                arguments: function_call.arguments.clone(),
                result,
            });
        }

        // If there were function calls, add them to conversation and get final response
        let final_message = if !llm_response.function_calls.is_empty() {
            // Add assistant message with function calls
            let mut assistant_message = format!("{}\n\n", llm_response.message);
            for function_call_info in &function_calls_info {
                assistant_message.push_str(&format!(
                    "Function {} returned: {}\n",
                    function_call_info.name,
                    serde_json::to_string_pretty(&function_call_info.result).unwrap_or_default()
                ));
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
            function_calls: function_calls_info,
        })
    }

    pub fn clear_history(&mut self) {
        self.conversation_history.clear();
        self.conversation_history
            .push(Self::create_system_message());
    }
}
