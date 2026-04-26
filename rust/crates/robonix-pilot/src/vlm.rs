//! VLM gRPC client — contract `SrvCognitionReason` (`robonix_contracts.proto`).

use crate::contracts::srv_cognition_reason_client::SrvCognitionReasonClient;
use crate::robonix_msg::{
    ChatMessage as PbChatMessage, ChatPart as PbChatPart, ToolCall as PbWireToolCall,
    ToolSpec as PbToolSpec,
};
use crate::vlm_proto::{ChatStreamEvent, ChatStreamRequest};
use anyhow::{Context, Result};
use robonix_sdk::QueryNodesOpts;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use tonic::Request;

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
    #[serde(skip_serializing_if = "Option::is_none")]
    pub image_base64: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parts: Option<Vec<ChatPart>>,
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

#[derive(Serialize, Deserialize, Clone)]
pub struct ChatPart {
    pub kind: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub text: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mime_type: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data_base64: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uri: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_arguments_json: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_call_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_result_json: Option<String>,
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
            parts: Some(vec![ChatPart::text(content)]),
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
            parts: Some(vec![ChatPart::text(content)]),
        }
    }
    pub fn user_with_image(content: &str, image: String) -> Self {
        Self {
            role: "user".into(),
            name: None,
            content: Some(content.into()),
            tool_calls: None,
            tool_call_id: None,
            image_base64: Some(image.clone()),
            parts: Some(vec![
                ChatPart::text(content),
                ChatPart::inline_data("image/jpeg", image),
            ]),
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
            parts: Some(vec![ChatPart::text(content)]),
        }
    }
    pub fn assistant_tool_calls(tc: Vec<ToolCall>) -> Self {
        let parts = tc.iter().map(ChatPart::function_call).collect();
        Self {
            role: "assistant".into(),
            name: None,
            content: None,
            tool_calls: Some(tc),
            tool_call_id: None,
            image_base64: None,
            parts: Some(parts),
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
            parts: Some(vec![ChatPart::function_response(id, content)]),
        }
    }
    #[allow(dead_code)]
    pub fn tool_result_with_image(id: &str, content: &str, image: String) -> Self {
        Self {
            role: "tool".into(),
            name: None,
            content: Some(content.into()),
            tool_calls: None,
            tool_call_id: Some(id.into()),
            image_base64: Some(image.clone()),
            parts: Some(vec![
                ChatPart::function_response(id, content),
                ChatPart::inline_data("image/jpeg", image),
            ]),
        }
    }
}

impl ChatPart {
    pub fn text(text: &str) -> Self {
        Self {
            kind: "text".into(),
            text: Some(text.into()),
            mime_type: None,
            data_base64: None,
            uri: None,
            tool_name: None,
            tool_arguments_json: None,
            tool_call_id: None,
            tool_result_json: None,
        }
    }

    pub fn inline_data(mime_type: &str, data_base64: String) -> Self {
        Self {
            kind: "inline_data".into(),
            text: None,
            mime_type: Some(mime_type.into()),
            data_base64: Some(data_base64),
            uri: None,
            tool_name: None,
            tool_arguments_json: None,
            tool_call_id: None,
            tool_result_json: None,
        }
    }

    pub fn function_call(tc: &ToolCall) -> Self {
        Self {
            kind: "function_call".into(),
            text: None,
            mime_type: None,
            data_base64: None,
            uri: None,
            tool_name: Some(tc.function.name.clone()),
            tool_arguments_json: Some(tc.function.arguments.clone()),
            tool_call_id: Some(tc.id.clone()),
            tool_result_json: None,
        }
    }

    pub fn function_response(tool_call_id: &str, result_json: &str) -> Self {
        Self {
            kind: "function_response".into(),
            text: None,
            mime_type: None,
            data_base64: None,
            uri: None,
            tool_name: None,
            tool_arguments_json: None,
            tool_call_id: Some(tool_call_id.into()),
            tool_result_json: Some(result_json.into()),
        }
    }
}

/// Stable contract id for the VLM cognition/reason capability (path + interface leaf).
/// See `rust/docs/NAMESPACE.md` (“System abstract interfaces”).
pub const VLM_CONTRACT_ID: &str = "robonix/srv/cognition/reason";

/// Default `QueryNodes.namespace` prefix for legacy split discovery only.
pub const DEFAULT_VLM_NAMESPACE_PREFIX: &str = "robonix/srv/model/vlm";

fn vlm_contract_id_for_query() -> String {
    match std::env::var("ROBONIX_VLM_CONTRACT_ID") {
        Ok(s) => s.trim().to_string(),
        Err(_) => VLM_CONTRACT_ID.to_string(),
    }
}

fn vlm_query_namespace_prefix() -> String {
    match std::env::var("ROBONIX_VLM_NAMESPACE_PREFIX") {
        Ok(s) => s,
        Err(_) => DEFAULT_VLM_NAMESPACE_PREFIX.to_string(),
    }
}

fn vlm_interface_leaf() -> &'static str {
    VLM_CONTRACT_ID
        .rsplit_once('/')
        .map(|(_, leaf)| leaf)
        .unwrap_or("chat")
}

/// VLM client discovered through robonix-atlas's control plane.
pub struct VlmClient {
    inner: SrvCognitionReasonClient<tonic::transport::Channel>,
}

impl VlmClient {
    /// Discover a VLM/LLM service via robonix-atlas and negotiate a gRPC channel.
    ///
    /// The entire flow (query → negotiate → connect) is retried up to 30 times
    /// (2s interval, ~60s total) to tolerate services that haven't registered yet
    /// or stale registrations from a previous run.
    pub async fn discover(
        sdk: &mut robonix_sdk::RobonixClient,
        agent_node_id: &str,
    ) -> Result<Self> {
        const MAX_RETRIES: u32 = 30;
        const RETRY_INTERVAL: std::time::Duration = std::time::Duration::from_secs(2);

        let contract_id = vlm_contract_id_for_query();
        let mut last_err: Option<anyhow::Error> = None;

        for attempt in 1..=MAX_RETRIES {
            match Self::try_discover(sdk, agent_node_id, &contract_id).await {
                Ok(client) => return Ok(client),
                Err(e) => {
                    if attempt < MAX_RETRIES {
                        log::info!(
                            "VLM discovery attempt {}/{} failed: {}. Retrying in {}s...",
                            attempt,
                            MAX_RETRIES,
                            e,
                            RETRY_INTERVAL.as_secs()
                        );
                        last_err = Some(e);
                        tokio::time::sleep(RETRY_INTERVAL).await;
                    } else {
                        last_err = Some(e);
                    }
                }
            }
        }

        Err(last_err.unwrap_or_else(|| {
            anyhow::anyhow!("VLM discovery failed after {} attempts", MAX_RETRIES)
        }))
    }

    /// Single attempt: query nodes → negotiate channel → connect.
    async fn try_discover(
        sdk: &mut robonix_sdk::RobonixClient,
        agent_node_id: &str,
        contract_id: &str,
    ) -> Result<Self> {
        let mut nodes = if contract_id.is_empty() {
            let ns_prefix = vlm_query_namespace_prefix();
            let iface_leaf = vlm_interface_leaf();
            sdk.query_nodes(&ns_prefix, iface_leaf, "grpc")
                .await
                .with_context(|| "failed to query nodes (legacy split namespace + name)")?
        } else {
            sdk.query_nodes_opts(QueryNodesOpts {
                contract_id: contract_id.to_string(),
                transport: "grpc".into(),
                ..Default::default()
            })
            .await
            .with_context(|| format!("failed to query nodes for contract_id={contract_id}"))?
        };

        if nodes.is_empty() {
            anyhow::bail!(
                "no VLM node for contract {} (grpc)",
                if contract_id.is_empty() {
                    format!("{}+{}", vlm_query_namespace_prefix(), vlm_interface_leaf())
                } else {
                    contract_id.to_string()
                }
            );
        }

        if nodes.len() > 1 {
            nodes.sort_by_key(|b| std::cmp::Reverse(b.namespace.len()));
            log::warn!(
                "multiple VLM candidates ({}); using most specific namespace {:?}",
                nodes.len(),
                nodes[0].namespace
            );
        }

        let vlm_node = &nodes[0];

        log::info!(
            "discovered VLM service: node_id='{}' namespace='{}'",
            vlm_node.node_id,
            vlm_node.namespace
        );

        let iface_name: &str = if contract_id.is_empty() {
            let iface_leaf = vlm_interface_leaf();
            vlm_node
                .interfaces
                .iter()
                .find(|i| {
                    i.name == iface_leaf && i.supported_transports.contains(&"grpc".to_string())
                })
                .map(|i| i.name.as_str())
                .unwrap_or(iface_leaf)
        } else {
            vlm_node
                .interfaces
                .iter()
                .find(|i| {
                    i.contract_id == contract_id
                        && i.supported_transports.contains(&"grpc".to_string())
                })
                .map(|i| i.name.as_str())
                .ok_or_else(|| {
                    anyhow::anyhow!(
                        "no grpc interface with contract_id='{contract_id}' on node '{}'",
                        vlm_node.node_id
                    )
                })?
        };

        let channel = sdk
            .negotiate_channel(agent_node_id, &vlm_node.node_id, iface_name, "grpc")
            .await
            .context("failed to negotiate channel with VLM service")?;

        log::info!("VLM channel negotiated: endpoint='{}'", channel.endpoint);

        let endpoint = if channel.endpoint.contains("://") {
            channel.endpoint
        } else {
            format!("http://{}", channel.endpoint)
        };

        let tonic_channel = tonic::transport::Endpoint::new(endpoint)?
            .connect()
            .await
            .context("failed to connect to VLM service data plane")?;

        Ok(Self {
            inner: SrvCognitionReasonClient::new(tonic_channel),
        })
    }

    /// Open the contract `Stream` RPC and return the tonic `Streaming` handle.
    pub async fn chat_stream(
        &mut self,
        messages: &[Message],
        tools: &[ToolDef],
    ) -> Result<tonic::Streaming<ChatStreamEvent>> {
        let req = Self::build_chat_stream_request(messages, tools);
        let resp = self
            .inner
            .stream(Request::new(req))
            .await
            .map_err(|e| anyhow::anyhow!("VLM gRPC Stream failed: {e}"))?;
        Ok(resp.into_inner())
    }

    /// Parse a stream event into typed enum for convenience.
    pub fn parse_stream_event(event: ChatStreamEvent) -> VlmStreamItem {
        if !event.text_delta.is_empty() {
            VlmStreamItem::TextDelta(event.text_delta)
        } else if let Some(tc) = event.tool_call {
            VlmStreamItem::ToolCall(ToolCall {
                id: tc.id,
                kind: "function".to_string(),
                function: FnCall {
                    name: tc.name,
                    arguments: tc.arguments_json,
                },
            })
        } else {
            VlmStreamItem::Finish(())
        }
    }

    fn build_chat_stream_request(messages: &[Message], tools: &[ToolDef]) -> ChatStreamRequest {
        ChatStreamRequest {
            messages: Self::build_chat_messages(messages),
            tools: Self::build_tool_specs(tools),
            tool_choice: Self::build_tool_choice(tools),
            max_tokens: 0,
        }
    }

    fn build_chat_messages(messages: &[Message]) -> Vec<PbChatMessage> {
        messages
            .iter()
            .map(|m| PbChatMessage {
                role: m.role.clone(),
                name: m.name.clone().unwrap_or_default(),
                content: m.content.clone().unwrap_or_default(),
                image_base64: m.image_base64.clone().unwrap_or_default(),
                tool_call_id: m.tool_call_id.clone().unwrap_or_default(),
                tool_calls: m
                    .tool_calls
                    .clone()
                    .unwrap_or_default()
                    .into_iter()
                    .map(|tc| PbWireToolCall {
                        id: tc.id,
                        name: tc.function.name,
                        arguments_json: tc.function.arguments,
                    })
                    .collect(),
                parts: m
                    .parts
                    .clone()
                    .unwrap_or_default()
                    .into_iter()
                    .map(|p| PbChatPart {
                        kind: p.kind,
                        text: p.text.unwrap_or_default(),
                        mime_type: p.mime_type.unwrap_or_default(),
                        data_base64: p.data_base64.unwrap_or_default(),
                        uri: p.uri.unwrap_or_default(),
                        tool_name: p.tool_name.unwrap_or_default(),
                        tool_arguments_json: p.tool_arguments_json.unwrap_or_default(),
                        tool_call_id: p.tool_call_id.unwrap_or_default(),
                        tool_result_json: p.tool_result_json.unwrap_or_default(),
                    })
                    .collect(),
            })
            .collect()
    }

    fn build_tool_specs(tools: &[ToolDef]) -> Vec<PbToolSpec> {
        tools
            .iter()
            .map(|t| PbToolSpec {
                name: t.function.name.clone(),
                description: t.function.description.clone(),
                input_schema_json: t.function.parameters.to_string(),
            })
            .collect()
    }

    fn build_tool_choice(tools: &[ToolDef]) -> String {
        if tools.is_empty() {
            String::new()
        } else {
            "auto".to_string()
        }
    }
}

pub enum VlmStreamItem {
    TextDelta(String),
    ToolCall(ToolCall),
    Finish(()),
}
