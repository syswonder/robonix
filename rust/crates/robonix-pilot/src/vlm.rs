//! VLM gRPC client — contract `SysModelVlmChat` (`robonix_contracts.proto`).

use crate::contracts::sys_model_vlm_chat_client::SysModelVlmChatClient;
use crate::robonix_msg::{ChatMessage as PbChatMessage, ToolSpec as PbToolSpec};
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
    pub content: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_calls: Option<Vec<ToolCall>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_call_id: Option<String>,
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
            content: Some(content.into()),
            tool_calls: None,
            tool_call_id: None,
            image_base64: None,
        }
    }
    pub fn user(content: &str) -> Self {
        Self {
            role: "user".into(),
            content: Some(content.into()),
            tool_calls: None,
            tool_call_id: None,
            image_base64: None,
        }
    }
    pub fn assistant(content: &str) -> Self {
        Self {
            role: "assistant".into(),
            content: Some(content.into()),
            tool_calls: None,
            tool_call_id: None,
            image_base64: None,
        }
    }
    pub fn assistant_tool_calls(tc: Vec<ToolCall>) -> Self {
        Self {
            role: "assistant".into(),
            content: None,
            tool_calls: Some(tc),
            tool_call_id: None,
            image_base64: None,
        }
    }
    pub fn tool_result(id: &str, content: &str) -> Self {
        Self {
            role: "tool".into(),
            content: Some(content.into()),
            tool_calls: None,
            tool_call_id: Some(id.into()),
            image_base64: None,
        }
    }
    pub fn tool_result_with_image(id: &str, content: &str, image: String) -> Self {
        Self {
            role: "tool".into(),
            content: Some(content.into()),
            tool_calls: None,
            tool_call_id: Some(id.into()),
            image_base64: Some(image),
        }
    }
}

/// Stable contract id for the VLM chat capability (path + interface leaf).
/// See `rust/docs/NAMESPACE.md` (“System abstract interfaces”).
pub const VLM_CONTRACT_ID: &str = "robonix/sys/model/vlm/chat";

/// Default `QueryNodes.namespace` prefix for legacy split discovery only.
pub const DEFAULT_VLM_NAMESPACE_PREFIX: &str = "robonix/sys/model/vlm";

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
    inner: SysModelVlmChatClient<tonic::transport::Channel>,
}

impl VlmClient {
    /// Discover a VLM/LLM service via robonix-atlas and negotiate a gRPC channel.
    pub async fn discover(
        sdk: &mut robonix_sdk::RobonixClient,
        agent_node_id: &str,
    ) -> Result<Self> {
        let contract_id = vlm_contract_id_for_query();
        let mut nodes = if contract_id.is_empty() {
            let ns_prefix = vlm_query_namespace_prefix();
            let iface_leaf = vlm_interface_leaf();
            sdk.query_nodes(&ns_prefix, iface_leaf, "grpc")
                .await
                .with_context(|| "failed to query nodes (legacy split namespace + name)")?
        } else {
            sdk.query_nodes_opts(QueryNodesOpts {
                contract_id: contract_id.clone(),
                transport: "grpc".into(),
                ..Default::default()
            })
            .await
            .with_context(|| format!("failed to query nodes for contract_id={contract_id}"))?
        };

        if nodes.len() > 1 {
            nodes.sort_by(|a, b| b.namespace.len().cmp(&a.namespace.len()));
            log::warn!(
                "multiple VLM candidates ({}); using most specific namespace {:?}",
                nodes.len(),
                nodes[0].namespace
            );
        }

        let vlm_node = nodes.first().ok_or_else(|| {
            anyhow::anyhow!(
                "no VLM node for contract {} (grpc). See rust/docs/NAMESPACE.md; \
                 set ROBONIX_VLM_CONTRACT_ID or use legacy empty contract + ROBONIX_VLM_NAMESPACE_PREFIX.",
                if contract_id.is_empty() {
                    format!("{}+{}", vlm_query_namespace_prefix(), vlm_interface_leaf())
                } else {
                    contract_id.clone()
                }
            )
        })?;

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
            inner: SysModelVlmChatClient::new(tonic_channel),
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
                content: m.content.clone().unwrap_or_default(),
                image_base64: m.image_base64.clone().unwrap_or_default(),
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
