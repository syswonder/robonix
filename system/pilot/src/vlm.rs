// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Embedded OpenAI-compatible chat-completions client.
// TODO: maybe we will support Google/Anthropic/etc. in the future :D
use crate::config::VlmConfig;
use anyhow::{Context, Result, bail};
use async_openai::types::chat::{
    ChatCompletionMessageToolCall, ChatCompletionMessageToolCalls,
    ChatCompletionRequestAssistantMessageArgs, ChatCompletionRequestMessage,
    ChatCompletionRequestMessageContentPartImage, ChatCompletionRequestMessageContentPartText,
    ChatCompletionRequestSystemMessageArgs, ChatCompletionRequestToolMessageArgs,
    ChatCompletionRequestUserMessageArgs, ChatCompletionRequestUserMessageContent,
    ChatCompletionRequestUserMessageContentPart, ChatCompletionStreamOptions, ChatCompletionTool,
    ChatCompletionTools, CreateChatCompletionRequestArgs, FunctionCall, FunctionObject,
    FunctionObjectArgs, ImageDetail, ImageUrl, ResponseFormat,
};
use futures_util::stream::{Stream, StreamExt};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;
use std::pin::Pin;
use std::time::Duration;

const MAX_OPEN_RETRIES: usize = 3;

/// Compatibility fallback is only safe when a client-error response names an
/// optional field that Pilot added. Unrelated 4xx responses must retain their
/// original diagnosis instead of being retried with a misleading warning.
fn rejects_optional_request_fields(status: reqwest::StatusCode, body: &str) -> bool {
    if status != reqwest::StatusCode::BAD_REQUEST
        && status != reqwest::StatusCode::UNPROCESSABLE_ENTITY
    {
        return false;
    }
    let body = body.to_ascii_lowercase();
    ["stream_options", "include_usage", "prompt_cache_key"]
        .iter()
        .any(|field| body.contains(field))
}

fn open_retry_delay(
    status: reqwest::StatusCode,
    retry_after: Option<&str>,
    retry_index: usize,
) -> Option<Duration> {
    if retry_index >= MAX_OPEN_RETRIES
        || !(status == reqwest::StatusCode::TOO_MANY_REQUESTS || status.is_server_error())
    {
        return None;
    }
    let server_seconds = retry_after
        .and_then(|value| value.trim().parse::<u64>().ok())
        .map(|seconds| seconds.clamp(1, 10));
    let seconds = server_seconds.unwrap_or_else(|| 1_u64 << retry_index.min(3));
    Some(Duration::from_secs(seconds))
}

/// One message in an OpenAI Chat Completions conversation.
/// Spec: https://platform.openai.com/docs/api-reference/chat/create#chat/create-messages
///
/// One struct, four roles (`system` / `user` / `assistant` / `tool`); each
/// role uses a different subset of the optional fields. `skip_serializing_if`
/// on every Option prunes irrelevant fields at serialization, so the wire
/// JSON for each role only carries what OpenAI expects:
///
///   system    → role + content
///   user      → role + content (+ optional `name` for multi-user)
///   assistant → role + content (may be null when only tool_calls are emitted)
///                              + optional tool_calls[]
///   tool      → role + content + tool_call_id (must match an id in the
///                                              preceding assistant.tool_calls)
///
/// We use a flat struct with optional fields rather than a tagged enum because
/// the planner does a lot of generic Vec<Message> manipulation (trim, sanitize,
/// sliding-window slicing) that's awkward to express through `match` on every
/// access. Type-safety for "tool messages must have tool_call_id" is delegated
/// to runtime checks (`history::sanitize_for_vlm`) and the OpenAI server's
/// own validation.
///
/// `image_base64` is a robonix-side simplification, NOT part of the OpenAI
/// wire format. Callers set it on a `user` message; `build_openai_messages`
/// in this file repackages content + image into OpenAI's multimodal `content`
/// array (`[{type:"text",...}, {type:"image_url",...}]`) at request time.
#[derive(Serialize, Deserialize, Clone)]
pub struct Message {
    /// "system" / "user" / "assistant" / "tool". Determines which other
    /// fields are meaningful; OpenAI rejects mismatched combinations.
    pub role: String,

    /// Optional sender name. Used by `user`/`assistant` for multi-user
    /// disambiguation; rare in practice. Robonix doesn't set it today.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    /// Message text. Always present except on `assistant` messages whose
    /// only output is tool calls (then None / null on the wire).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub content: Option<String>,

    /// Tool calls the LLM decided to make. Present only on `assistant`
    /// messages. Each entry carries id + function.{name, arguments};
    /// the corresponding `tool` message links back via `tool_call_id`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_calls: Option<Vec<ToolCall>>,

    /// Correlates a `tool` result back to the `assistant.tool_calls[].id`
    /// that produced it. Required on `tool` messages; absent on others.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_call_id: Option<String>,

    /// Inline image attached to a `user` message (base64-encoded JPEG
    /// bytes). Robonix-only field; rewritten into OpenAI's multimodal
    /// content array at serialize time by `build_openai_messages`.
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

/// Item yielded by the chat completion stream. `planner.rs` matches on this
/// enum to drive token streaming, tool dispatch, and finish handling.
pub enum VlmStreamItem {
    TextDelta(String),
    ToolCall(ToolCall),
    /// Provider-reported usage for the complete streamed request. OpenAI sends
    /// it in a final choice-less chunk when `include_usage` is supported.
    Usage(VlmUsage),
    /// Stream complete. Finish reason ("stop" / "tool_calls" / "error") is
    /// not surfaced to consumers yet — add a field here when the planner or
    /// downstream PilotEvent grows a use for it.
    Finish,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VlmUsage {
    pub prompt_tokens: u64,
    pub completion_tokens: u64,
    pub cached_tokens: Option<u64>,
}

/// Direct HTTP client for an OpenAI-compatible chat-completions endpoint.
/// Cheap to clone — `async_openai::Client` wraps a `reqwest::Client` (an
/// `Arc<...>` internally). No mutex needed when sharing across tasks.
#[derive(Clone)]
pub struct VlmClient {
    inner: reqwest::Client,
    api_base: String,
    api_key: String,
    model: String,
}

impl VlmClient {
    pub fn new(cfg: &VlmConfig) -> Self {
        Self {
            inner: reqwest::Client::new(),
            api_base: cfg.upstream.trim_end_matches('/').to_string(),
            api_key: cfg.api_key.clone(),
            model: cfg.model.clone(),
        }
    }

    /// Open a streaming chat completion. Yields:
    ///   - `TextDelta` for every assistant content chunk
    ///   - `ToolCall` once per accumulated function call (after the upstream
    ///     finishes streaming all argument deltas)
    ///   - one final `Finish`
    pub async fn chat_stream(
        &self,
        messages: &[Message],
        tools: &[ToolDef],
        prompt_cache_key: Option<&str>,
    ) -> Result<Pin<Box<dyn Stream<Item = Result<VlmStreamItem>> + Send>>> {
        let oai_messages = build_openai_messages(messages)?;
        let oai_tools = build_openai_tools(tools)?;

        let mut req_builder = CreateChatCompletionRequestArgs::default();
        req_builder
            .model(&self.model)
            .messages(oai_messages)
            .stream(true)
            .stream_options(ChatCompletionStreamOptions {
                include_usage: Some(true),
                include_obfuscation: None,
            })
            .response_format(ResponseFormat::JsonObject);
        if !oai_tools.is_empty() {
            req_builder.tools(oai_tools);
        }
        if let Some(prompt_cache_key) = prompt_cache_key {
            req_builder.prompt_cache_key(prompt_cache_key);
        }
        let request = req_builder
            .build()
            .context("build chat completion request")?;
        let mut request_body = serde_json::to_value(request)
            .context("serialize chat completion request for transport")?;

        let url = format!("{}/chat/completions", self.api_base);
        let mut retry_index = 0;
        let mut compatibility_fallback_attempted = false;
        let response = loop {
            let response = self
                .inner
                .post(&url)
                .bearer_auth(&self.api_key)
                .header(reqwest::header::ACCEPT, "text/event-stream")
                .header(reqwest::header::CONTENT_TYPE, "application/json")
                .json(&request_body)
                .send()
                .await
                .context("open VLM chat stream")?;
            let status = response.status();
            if status.is_success() {
                break response;
            }
            let retry_after = response
                .headers()
                .get(reqwest::header::RETRY_AFTER)
                .and_then(|value| value.to_str().ok())
                .map(str::to_string);
            let text = response.text().await.unwrap_or_default();
            if rejects_optional_request_fields(status, &text) && !compatibility_fallback_attempted {
                let removed = request_body.as_object_mut().is_some_and(|body| {
                    let stream_options = body.remove("stream_options").is_some();
                    let prompt_cache_key = body.remove("prompt_cache_key").is_some();
                    stream_options || prompt_cache_key
                });
                if !removed {
                    bail!("open VLM chat stream: HTTP {status}: {text}");
                }
                robonix_scribe::warn!(
                    "[pilot/vlm] upstream rejected optional cache/usage fields with HTTP {status}; retrying without them"
                );
                compatibility_fallback_attempted = true;
                continue;
            }
            if let Some(delay) = open_retry_delay(status, retry_after.as_deref(), retry_index) {
                robonix_scribe::warn!(
                    "[pilot/vlm] open stream HTTP {status}; retry {}/{} in {:.1}s",
                    retry_index + 1,
                    MAX_OPEN_RETRIES,
                    delay.as_secs_f64()
                );
                tokio::time::sleep(delay).await;
                retry_index += 1;
                continue;
            }
            bail!("open VLM chat stream: HTTP {status}: {text}");
        };
        let mut upstream = response.bytes_stream();

        // Walk the upstream chunk-by-chunk, accumulating tool-call deltas by
        // index until the upstream finishes; then emit one ToolCall per index
        // and a final Finish event. Use mpsc + spawn so we can return the
        // boxed Stream while the polling runs in the background.
        let (tx, rx) = tokio::sync::mpsc::channel::<Result<VlmStreamItem>>(64);
        tokio::spawn(async move {
            let mut tc_acc: BTreeMap<u32, AccumulatedToolCall> = BTreeMap::new();
            let mut finish = "stop".to_string();
            let mut buf = String::new();
            let mut done = false;
            while !done {
                let chunk = tokio::select! {
                    _ = tx.closed() => return,
                    chunk = upstream.next() => chunk,
                };
                let Some(chunk) = chunk else {
                    break;
                };
                match chunk {
                    Ok(bytes) => {
                        buf.push_str(&String::from_utf8_lossy(&bytes));
                        while let Some(pos) = buf.find('\n') {
                            let line: String = buf.drain(..=pos).collect();
                            match process_stream_line(
                                line.trim_end(),
                                &mut tc_acc,
                                &mut finish,
                                &tx,
                            )
                            .await
                            {
                                Ok(true) => {
                                    done = true;
                                    break;
                                }
                                Ok(false) => {}
                                Err(e) => {
                                    let _ = tx.send(Err(e)).await;
                                    return;
                                }
                            }
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
            if !buf.trim().is_empty()
                && let Err(e) =
                    process_stream_line(buf.trim_end(), &mut tc_acc, &mut finish, &tx).await
            {
                let _ = tx.send(Err(e)).await;
                return;
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
            let _ = finish; // surface to PilotEvent later if needed
            let _ = tx.send(Ok(VlmStreamItem::Finish)).await;
        });

        Ok(Box::pin(tokio_stream::wrappers::ReceiverStream::new(rx)))
    }
}

#[cfg(test)]
mod tests {
    use super::{
        AccumulatedToolCall, MAX_OPEN_RETRIES, VlmStreamItem, VlmUsage, open_retry_delay,
        parse_usage, process_stream_line, rejects_optional_request_fields,
    };
    use serde_json::json;
    use std::collections::BTreeMap;
    use std::time::Duration;

    #[test]
    fn transient_open_errors_use_bounded_backoff() {
        assert_eq!(
            open_retry_delay(reqwest::StatusCode::TOO_MANY_REQUESTS, None, 0),
            Some(Duration::from_secs(1))
        );
        assert_eq!(
            open_retry_delay(reqwest::StatusCode::SERVICE_UNAVAILABLE, Some("7"), 1),
            Some(Duration::from_secs(7))
        );
        assert_eq!(
            open_retry_delay(reqwest::StatusCode::BAD_REQUEST, None, 0),
            None
        );
        assert_eq!(
            open_retry_delay(
                reqwest::StatusCode::TOO_MANY_REQUESTS,
                None,
                MAX_OPEN_RETRIES
            ),
            None
        );
    }

    #[test]
    fn usage_includes_provider_prompt_cache_tokens() {
        let usage = parse_usage(&json!({
            "choices": [],
            "usage": {
                "prompt_tokens": 1200,
                "completion_tokens": 80,
                "total_tokens": 1280,
                "prompt_tokens_details": {"cached_tokens": 900}
            }
        }));
        assert_eq!(
            usage,
            Some(VlmUsage {
                prompt_tokens: 1200,
                completion_tokens: 80,
                cached_tokens: Some(900),
            })
        );
    }

    #[test]
    fn optional_field_fallback_does_not_mask_unrelated_client_errors() {
        assert!(rejects_optional_request_fields(
            reqwest::StatusCode::BAD_REQUEST,
            "unknown field prompt_cache_key"
        ));
        assert!(rejects_optional_request_fields(
            reqwest::StatusCode::UNPROCESSABLE_ENTITY,
            "stream_options is not permitted"
        ));
        assert!(!rejects_optional_request_fields(
            reqwest::StatusCode::BAD_REQUEST,
            "invalid model name"
        ));
        assert!(!rejects_optional_request_fields(
            reqwest::StatusCode::UNAUTHORIZED,
            "prompt_cache_key"
        ));
    }

    #[tokio::test]
    async fn choice_less_usage_chunk_reaches_the_consumer() {
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);
        let mut calls = BTreeMap::<u32, AccumulatedToolCall>::new();
        let mut finish = String::new();
        process_stream_line(
            r#"data: {"choices":[],"usage":{"prompt_tokens":1200,"completion_tokens":80,"prompt_tokens_details":{"cached_tokens":900}}}"#,
            &mut calls,
            &mut finish,
            &tx,
        )
        .await
        .unwrap();
        assert!(matches!(
            rx.recv().await,
            Some(Ok(VlmStreamItem::Usage(VlmUsage {
                prompt_tokens: 1200,
                completion_tokens: 80,
                cached_tokens: Some(900),
            })))
        ));
    }
}

#[derive(Default)]
struct AccumulatedToolCall {
    id: String,
    name: String,
    arguments: String,
}

async fn process_stream_line(
    line: &str,
    tc_acc: &mut BTreeMap<u32, AccumulatedToolCall>,
    finish: &mut String,
    tx: &tokio::sync::mpsc::Sender<Result<VlmStreamItem>>,
) -> Result<bool> {
    let line = line.trim();
    if line.is_empty() || line.starts_with(':') {
        return Ok(false);
    }
    let Some(data) = line.strip_prefix("data:") else {
        return Ok(false);
    };
    let data = data.trim();
    if data == "[DONE]" {
        return Ok(true);
    }

    let v: Value = serde_json::from_str(data)
        .with_context(|| format!("deserialize VLM stream chunk: {data}"))?;
    if let Some(usage) = parse_usage(&v)
        && tx.send(Ok(VlmStreamItem::Usage(usage))).await.is_err()
    {
        return Ok(true);
    }
    let Some(choice) = v
        .get("choices")
        .and_then(Value::as_array)
        .and_then(|choices| choices.first())
    else {
        return Ok(false);
    };

    if let Some(content) = choice
        .get("delta")
        .and_then(|delta| delta.get("content"))
        .and_then(Value::as_str)
        && !content.is_empty()
        && tx
            .send(Ok(VlmStreamItem::TextDelta(content.to_string())))
            .await
            .is_err()
    {
        return Ok(true);
    }
    if let Some(tc_chunks) = choice
        .get("delta")
        .and_then(|delta| delta.get("tool_calls"))
        .and_then(Value::as_array)
    {
        for tc in tc_chunks {
            let index = tc.get("index").and_then(Value::as_u64).unwrap_or(0) as u32;
            let entry = tc_acc.entry(index).or_default();
            if let Some(id) = tc.get("id").and_then(Value::as_str) {
                entry.id = id.to_string();
            }
            if let Some(func) = tc.get("function") {
                if let Some(name) = func.get("name").and_then(Value::as_str) {
                    entry.name.push_str(name);
                }
                if let Some(args) = func.get("arguments").and_then(Value::as_str) {
                    entry.arguments.push_str(args);
                }
            }
        }
    }
    if let Some(fr) = choice.get("finish_reason").and_then(Value::as_str) {
        *finish = fr.to_string();
    }
    Ok(false)
}

/// Read the standard Chat Completions usage shape without requiring every
/// OpenAI-compatible provider to deserialize optional detail fields equally.
fn parse_usage(value: &Value) -> Option<VlmUsage> {
    let usage = value.get("usage")?.as_object()?;
    let prompt_tokens = usage.get("prompt_tokens")?.as_u64()?;
    let completion_tokens = usage.get("completion_tokens")?.as_u64()?;
    let cached_tokens = usage
        .get("prompt_tokens_details")
        .and_then(Value::as_object)
        .and_then(|details| details.get("cached_tokens"))
        .and_then(Value::as_u64);
    Some(VlmUsage {
        prompt_tokens,
        completion_tokens,
        cached_tokens,
    })
}

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
