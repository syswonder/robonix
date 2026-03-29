//! VLM gRPC client using protos from `robonix-interfaces/robonix_proto` (ridlc → `vlm.proto` + deps).

use anyhow::{Context, Result};
use robonix_sdk::QueryNodesOpts;
use serde::{Deserialize, Serialize};
use serde_json::Value;

/// Generated prost + tonic modules (`tonic_prost_build` from `build.rs`).
mod robonix {
    #![allow(dead_code)]
    #![allow(clippy::all)]
    pub mod builtin_interfaces {
        include!(concat!(env!("OUT_DIR"), "/robonix.builtin_interfaces.rs"));
    }
    pub mod std_msgs {
        include!(concat!(env!("OUT_DIR"), "/robonix.std_msgs.rs"));
    }
    pub mod geometry_msgs {
        include!(concat!(env!("OUT_DIR"), "/robonix.geometry_msgs.rs"));
    }
    pub mod sensor_msgs {
        include!(concat!(env!("OUT_DIR"), "/robonix.sensor_msgs.rs"));
    }
    pub mod robonix_msg {
        include!(concat!(env!("OUT_DIR"), "/robonix.robonix_msg.rs"));
    }
    pub mod vlm {
        include!(concat!(env!("OUT_DIR"), "/robonix.vlm.rs"));
    }
}

use robonix::robonix_msg::{ChatMessage as PbChatMessage, ToolSpec as PbToolSpec};
use robonix::vlm::vlm_service_client::VlmServiceClient;
use robonix::vlm::{ChatRequest, ChatResponse, ChatStreamEvent};

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

/// System abstract interface ID for the VLM chat capability (path + interface leaf).
/// See `rust/docs/NAMESPACE.md` (“System abstract interfaces”).
pub const VLM_ABSTRACT_INTERFACE_ID: &str = "robonix/sys/model/vlm/chat";

/// Default `QueryNodes.namespace` prefix for legacy split discovery only.
pub const DEFAULT_VLM_NAMESPACE_PREFIX: &str = "robonix/sys/model/vlm";

/// `ROBONIX_VLM_ABSTRACT_INTERFACE_ID`: canonical protocol id (default [`VLM_ABSTRACT_INTERFACE_ID`]).
/// Sent as `QueryNodesRequest.abstract_interface_id` — must match server `InterfaceInfo.abstract_interface_id`.
///
/// If set to an empty string, discovery falls back to split `namespace` + `name` using
/// `ROBONIX_VLM_NAMESPACE_PREFIX` (default [`DEFAULT_VLM_NAMESPACE_PREFIX`]) + interface leaf from [`VLM_ABSTRACT_INTERFACE_ID`].
fn vlm_abstract_interface_id_for_query() -> String {
    match std::env::var("ROBONIX_VLM_ABSTRACT_INTERFACE_ID") {
        Ok(s) => s.trim().to_string(),
        Err(_) => VLM_ABSTRACT_INTERFACE_ID.to_string(),
    }
}

fn vlm_query_namespace_prefix() -> String {
    match std::env::var("ROBONIX_VLM_NAMESPACE_PREFIX") {
        Ok(s) => s,
        Err(_) => DEFAULT_VLM_NAMESPACE_PREFIX.to_string(),
    }
}

fn vlm_interface_leaf() -> &'static str {
    VLM_ABSTRACT_INTERFACE_ID
        .rsplit_once('/')
        .map(|(_, leaf)| leaf)
        .unwrap_or("chat")
}

/// VLM client discovered through robonix-server's control plane.
pub struct VlmClient {
    inner: VlmServiceClient<tonic::transport::Channel>,
}

impl VlmClient {
    /// Discover a VLM/LLM service via robonix-server and negotiate a gRPC channel.
    ///
    /// Uses `QueryNodesRequest.abstract_interface_id` when non-empty; matches `InterfaceInfo.abstract_interface_id`
    /// on the server. `NegotiateChannel` uses the matching interface's `name` (DeclareInterface leaf), not the full path.
    /// If several nodes match, picks the longest `namespace`.
    pub async fn discover(
        sdk: &mut robonix_sdk::RobonixClient,
        agent_node_id: &str,
    ) -> Result<Self> {
        let abstract_id = vlm_abstract_interface_id_for_query();
        let mut nodes = if abstract_id.is_empty() {
            let ns_prefix = vlm_query_namespace_prefix();
            let iface_leaf = vlm_interface_leaf();
            sdk.query_nodes(&ns_prefix, iface_leaf, "grpc")
                .await
                .with_context(|| "failed to query nodes (legacy split namespace + name)")?
        } else {
            sdk.query_nodes_opts(QueryNodesOpts {
                abstract_interface_id: abstract_id.clone(),
                transport: "grpc".into(),
                ..Default::default()
            })
            .await
            .with_context(|| {
                format!("failed to query nodes for abstract_interface_id={abstract_id}")
            })?
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
                "no VLM node for abstract interface {} (grpc). See rust/docs/NAMESPACE.md; \
                 set ROBONIX_VLM_ABSTRACT_INTERFACE_ID or use legacy empty abstract + ROBONIX_VLM_NAMESPACE_PREFIX.",
                if abstract_id.is_empty() {
                    format!("{}+{}", vlm_query_namespace_prefix(), vlm_interface_leaf())
                } else {
                    abstract_id.clone()
                }
            )
        })?;

        log::info!(
            "discovered VLM service: node_id='{}' namespace='{}'",
            vlm_node.node_id,
            vlm_node.namespace
        );

        let iface_name: &str = if abstract_id.is_empty() {
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
                    i.abstract_interface_id == abstract_id
                        && i.supported_transports.contains(&"grpc".to_string())
                })
                .map(|i| i.name.as_str())
                .ok_or_else(|| {
                    anyhow::anyhow!(
                        "no grpc interface with abstract_interface_id='{abstract_id}' on node '{}'",
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
            inner: VlmServiceClient::new(tonic_channel),
        })
    }

    pub async fn chat(
        &mut self,
        messages: &[Message],
        tools: &[ToolDef],
    ) -> Result<(Option<String>, Vec<ToolCall>)> {
        let req = Self::build_request(messages, tools);

        let resp: tonic::Response<ChatResponse> =
            self.inner
                .chat(tonic::Request::new(req))
                .await
                .map_err(|e| anyhow::anyhow!("VLM gRPC Chat failed: {e}"))?;

        let parsed = resp.into_inner();

        let tool_calls = parsed
            .tool_calls
            .into_iter()
            .map(|tc| ToolCall {
                id: tc.id,
                kind: "function".to_string(),
                function: FnCall {
                    name: tc.name,
                    arguments: tc.arguments_json,
                },
            })
            .collect::<Vec<_>>();

        let content = if parsed.content.is_empty() {
            None
        } else {
            Some(parsed.content)
        };
        Ok((content, tool_calls))
    }

    /// Open a ChatStream and return the raw tonic Streaming handle.
    /// The caller drives the stream to get real-time text deltas.
    pub async fn chat_stream(
        &mut self,
        messages: &[Message],
        tools: &[ToolDef],
    ) -> Result<tonic::Streaming<ChatStreamEvent>> {
        let req = Self::build_request(messages, tools);
        let resp = self
            .inner
            .chat_stream(tonic::Request::new(req))
            .await
            .map_err(|e| anyhow::anyhow!("VLM gRPC ChatStream failed: {e}"))?;
        Ok(resp.into_inner())
    }

    /// Parse a stream event into typed enum for convenience.
    pub fn parse_stream_event(event: ChatStreamEvent) -> VlmStreamItem {
        match event.event {
            Some(robonix::vlm::chat_stream_event::Event::TextDelta(d)) => {
                VlmStreamItem::TextDelta(d)
            }
            Some(robonix::vlm::chat_stream_event::Event::ToolCall(tc)) => {
                VlmStreamItem::ToolCall(ToolCall {
                    id: tc.id,
                    kind: "function".to_string(),
                    function: FnCall {
                        name: tc.name,
                        arguments: tc.arguments_json,
                    },
                })
            }
            Some(robonix::vlm::chat_stream_event::Event::FinishReason(_)) => {
                VlmStreamItem::Finish(())
            }
            None => VlmStreamItem::Finish(()),
        }
    }

    fn build_request(messages: &[Message], tools: &[ToolDef]) -> ChatRequest {
        ChatRequest {
            messages: messages
                .iter()
                .map(|m| PbChatMessage {
                    role: m.role.clone(),
                    content: m.content.clone().unwrap_or_default(),
                    image_base64: m.image_base64.clone().unwrap_or_default(),
                })
                .collect(),
            tools: tools
                .iter()
                .map(|t| PbToolSpec {
                    name: t.function.name.clone(),
                    description: t.function.description.clone(),
                    input_schema_json: t.function.parameters.to_string(),
                })
                .collect(),
            tool_choice: if tools.is_empty() {
                String::new()
            } else {
                "auto".to_string()
            },
            max_tokens: 0,
        }
    }
}

pub enum VlmStreamItem {
    TextDelta(String),
    ToolCall(ToolCall),
    Finish(()),
}
