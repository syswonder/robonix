// SPDX-License-Identifier: MulanPSL-2.0
// Embedded OpenAI-compatible chat-completions client.
//
// VLM is no longer a separate Robonix capability — pilot owns the LLM
// connection directly using `async-openai`. Connection params come from the
// `vlm:` block of pilot's config (see `config.rs::VlmConfig`).
//
// Public surface (consumed by `planner.rs`):
//   * `Message` / `ToolCall` / `FnCall` / `ToolDef`  — internal data types
//     used by `planner.rs` to keep history / dispatch tool calls.
//   * `VlmClient::new(&VlmConfig)`                    — constructs the client.
//   * `VlmClient::chat_stream(messages, tools)`       — returns a Stream of
//     `VlmStreamItem` (text deltas → tool call accumulations → finish).

use crate::config::VlmConfig;
use anyhow::{Context, Result};
use async_openai::config::OpenAIConfig;
use async_openai::Client;
use async_openai::types::chat::{
    ChatCompletionMessageToolCall, ChatCompletionMessageToolCalls,
    ChatCompletionRequestAssistantMessageArgs, ChatCompletionRequestMessage,
    ChatCompletionRequestMessageContentPartImage, ChatCompletionRequestMessageContentPartText,
    ChatCompletionRequestSystemMessageArgs, ChatCompletionRequestToolMessageArgs,
    ChatCompletionRequestUserMessageArgs, ChatCompletionRequestUserMessageContent,
    ChatCompletionRequestUserMessageContentPart, ChatCompletionTool, ChatCompletionTools,
    CreateChatCompletionRequestArgs, FunctionCall, FunctionObject, FunctionObjectArgs,
    ImageDetail, ImageUrl,
};
use futures_util::stream::{Stream, StreamExt};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;
use std::pin::Pin;

// ── Pilot-internal message / tool data types ───────────────────────────────

#[derive(Serialize, Deserialize, Clone)]
pub struct Message {
    pub role: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub content: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_calls: Option<Vec<ToolCall>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_call_id: Option<String>,
    /// Optional inline image (base64-encoded JPEG bytes).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub image_base64: Option<String>,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct ToolCall {
    pub id: String,
    #[serde(rename = "type")]
    pub kind: String,
    pub function: FnCall,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct FnCall {
    pub name: String,
    pub arguments: String,
}

#[derive(Serialize, Clone)]
pub struct ToolDef {
    #[serde(rename = "type")]
    kind: String,
    function: FnDef,
}

#[derive(Serialize, Clone)]
struct FnDef {
    name: String,
    description: String,
    parameters: Value,
}

impl ToolDef {
    pub fn new(name: &str, description: &str, parameters: Value) -> Self {
        Self {
            kind: "function".into(),
            function: FnDef {
                name: name.into(),
                description: description.into(),
                parameters,
            },
        }
    }
}

impl Message {
    pub fn system(content: &str) -> Self {
        Self {
            role: "system".into(),
            name: None,
            content: Some(content.into()),
            tool_calls: None,
            tool_call_id: None,
            image_base64: None,
        }
    }
    pub fn user(content: &str) -> Self {
        Self {
            role: "user".into(),
            name: None,
            content: Some(content.into()),
            tool_calls: None,
            tool_call_id: None,
            image_base64: None,
        }
    }
    pub fn user_with_image(content: &str, image_base64: String) -> Self {
        Self {
            role: "user".into(),
            name: None,
            content: Some(content.into()),
            tool_calls: None,
            tool_call_id: None,
            image_base64: Some(image_base64),
        }
    }
    pub fn assistant(content: &str) -> Self {
        Self {
            role: "assistant".into(),
            name: None,
            content: Some(content.into()),
            tool_calls: None,
            tool_call_id: None,
            image_base64: None,
        }
    }
    pub fn assistant_tool_calls(tc: Vec<ToolCall>) -> Self {
        Self {
            role: "assistant".into(),
            name: None,
            content: None,
            tool_calls: Some(tc),
            tool_call_id: None,
            image_base64: None,
        }
    }
    pub fn tool_result(id: &str, content: &str) -> Self {
        Self {
            role: "tool".into(),
            name: None,
            content: Some(content.into()),
            tool_calls: None,
            tool_call_id: Some(id.into()),
            image_base64: None,
        }
    }
}

// ── VLM client ─────────────────────────────────────────────────────────────

/// Item yielded by the chat completion stream. `planner.rs` matches on this
/// enum to drive token streaming, tool dispatch, and finish handling.
pub enum VlmStreamItem {
    TextDelta(String),
    ToolCall(ToolCall),
    Finish(String),
}

/// Direct HTTP client for an OpenAI-compatible chat-completions endpoint.
/// Cheap to clone — `async_openai::Client` wraps a `reqwest::Client` (an
/// `Arc<...>` internally). No mutex needed when sharing across tasks.
#[derive(Clone)]
pub struct VlmClient {
    inner: Client<OpenAIConfig>,
    model: String,
}

impl VlmClient {
    pub fn new(cfg: &VlmConfig) -> Self {
        let oa = OpenAIConfig::new()
            .with_api_base(cfg.upstream.trim_end_matches('/'))
            .with_api_key(&cfg.api_key);
        Self {
            inner: Client::with_config(oa),
            model: cfg.model.clone(),
        }
    }

    /// Open a streaming chat completion. Yields:
    ///   - `TextDelta` for every assistant content chunk
    ///   - `ToolCall` once per accumulated function call (after the upstream
    ///     finishes streaming all argument deltas)
    ///   - one final `Finish(reason)` (e.g. "stop", "tool_calls", "error")
    pub async fn chat_stream(
        &self,
        messages: &[Message],
        tools: &[ToolDef],
    ) -> Result<Pin<Box<dyn Stream<Item = Result<VlmStreamItem>> + Send>>> {
        let oai_messages = build_openai_messages(messages)?;
        let oai_tools = build_openai_tools(tools)?;

        let mut req_builder = CreateChatCompletionRequestArgs::default();
        req_builder
            .model(&self.model)
            .messages(oai_messages)
            .stream(true);
        if !oai_tools.is_empty() {
            req_builder.tools(oai_tools);
        }
        let request = req_builder
            .build()
            .context("build chat completion request")?;

        let mut upstream = self
            .inner
            .chat()
            .create_stream(request)
            .await
            .context("open VLM chat stream")?;

        // Walk the upstream chunk-by-chunk, accumulating tool-call deltas by
        // index until the upstream finishes; then emit one ToolCall per index
        // and a final Finish event. Use mpsc + spawn so we can return the
        // boxed Stream while the polling runs in the background.
        let (tx, rx) = tokio::sync::mpsc::channel::<Result<VlmStreamItem>>(64);
        tokio::spawn(async move {
            let mut tc_acc: BTreeMap<u32, AccumulatedToolCall> = BTreeMap::new();
            let mut finish = "stop".to_string();
            while let Some(chunk) = upstream.next().await {
                match chunk {
                    Ok(resp) => {
                        let Some(choice) = resp.choices.into_iter().next() else {
                            continue;
                        };
                        let delta = choice.delta;
                        if let Some(content) = delta.content
                            && !content.is_empty()
                            && tx
                                .send(Ok(VlmStreamItem::TextDelta(content)))
                                .await
                                .is_err()
                        {
                            return;
                        }
                        if let Some(tc_chunks) = delta.tool_calls {
                            for tc in tc_chunks {
                                let entry = tc_acc.entry(tc.index).or_default();
                                if let Some(id) = tc.id {
                                    entry.id = id;
                                }
                                if let Some(func) = tc.function {
                                    if let Some(name) = func.name {
                                        entry.name.push_str(&name);
                                    }
                                    if let Some(args) = func.arguments {
                                        entry.arguments.push_str(&args);
                                    }
                                }
                            }
                        }
                        if let Some(fr) = choice.finish_reason {
                            finish = format!("{fr:?}").to_lowercase();
                        }
                    }
                    Err(e) => {
                        let _ = tx
                            .send(Err(anyhow::anyhow!("VLM stream chunk error: {e}")))
                            .await;
                        return;
                    }
                }
            }

            for (_, tc) in tc_acc {
                if tc.id.is_empty() && tc.name.is_empty() {
                    continue;
                }
                let item = VlmStreamItem::ToolCall(ToolCall {
                    id: tc.id,
                    kind: "function".to_string(),
                    function: FnCall {
                        name: tc.name,
                        arguments: tc.arguments,
                    },
                });
                if tx.send(Ok(item)).await.is_err() {
                    return;
                }
            }
            let _ = tx.send(Ok(VlmStreamItem::Finish(finish))).await;
        });

        Ok(Box::pin(tokio_stream::wrappers::ReceiverStream::new(rx)))
    }
}

#[derive(Default)]
struct AccumulatedToolCall {
    id: String,
    name: String,
    arguments: String,
}

// ── Message + tool conversion ──────────────────────────────────────────────

fn build_openai_messages(messages: &[Message]) -> Result<Vec<ChatCompletionRequestMessage>> {
    let mut out = Vec::with_capacity(messages.len());
    for m in messages {
        let msg = match m.role.as_str() {
            "system" => ChatCompletionRequestSystemMessageArgs::default()
                .content(m.content.clone().unwrap_or_default())
                .build()?
                .into(),
            "user" => {
                if let Some(image) = &m.image_base64 {
                    let text = m.content.clone().unwrap_or_default();
                    let mut parts: Vec<ChatCompletionRequestUserMessageContentPart> = Vec::new();
                    if !text.is_empty() {
                        parts.push(ChatCompletionRequestUserMessageContentPart::Text(
                            ChatCompletionRequestMessageContentPartText { text },
                        ));
                    }
                    let url = format!("data:image/jpeg;base64,{image}");
                    parts.push(ChatCompletionRequestUserMessageContentPart::ImageUrl(
                        ChatCompletionRequestMessageContentPartImage {
                            image_url: ImageUrl {
                                url,
                                detail: Some(ImageDetail::Auto),
                            },
                        },
                    ));
                    ChatCompletionRequestUserMessageArgs::default()
                        .content(ChatCompletionRequestUserMessageContent::Array(parts))
                        .build()?
                        .into()
                } else {
                    ChatCompletionRequestUserMessageArgs::default()
                        .content(m.content.clone().unwrap_or_default())
                        .build()?
                        .into()
                }
            }
            "assistant" => {
                let mut b = ChatCompletionRequestAssistantMessageArgs::default();
                if let Some(c) = &m.content
                    && !c.is_empty()
                {
                    b.content(c.clone());
                }
                if let Some(tcs) = &m.tool_calls {
                    let oai_tcs: Vec<ChatCompletionMessageToolCalls> = tcs
                        .iter()
                        .map(|tc| {
                            ChatCompletionMessageToolCalls::Function(
                                ChatCompletionMessageToolCall {
                                    id: tc.id.clone(),
                                    function: FunctionCall {
                                        name: tc.function.name.clone(),
                                        arguments: tc.function.arguments.clone(),
                                    },
                                },
                            )
                        })
                        .collect();
                    b.tool_calls(oai_tcs);
                }
                b.build()?.into()
            }
            "tool" => {
                let id = m.tool_call_id.clone().unwrap_or_default();
                ChatCompletionRequestToolMessageArgs::default()
                    .tool_call_id(id)
                    .content(m.content.clone().unwrap_or_default())
                    .build()?
                    .into()
            }
            other => anyhow::bail!("unknown message role '{other}'"),
        };
        out.push(msg);
    }
    Ok(out)
}

fn build_openai_tools(tools: &[ToolDef]) -> Result<Vec<ChatCompletionTools>> {
    tools
        .iter()
        .map(|t| -> Result<ChatCompletionTools> {
            let func: FunctionObject = FunctionObjectArgs::default()
                .name(&t.function.name)
                .description(&t.function.description)
                .parameters(t.function.parameters.clone())
                .build()?;
            Ok(ChatCompletionTools::Function(ChatCompletionTool {
                function: func,
            }))
        })
        .collect()
}
