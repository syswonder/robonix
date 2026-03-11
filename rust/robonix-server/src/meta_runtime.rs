// SPDX-License-Identifier: MulanPSL-2.0
// Robonix runtime meta API (gRPC)

use anyhow::{Context, Result};
use log::info;
use serde_json::json;
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::Arc;
use tokio::sync::RwLock;
use tonic::{Request, Response, Status};
use uuid::Uuid;

pub mod pb {
    tonic::include_proto!("robonix.runtime");
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum CapabilityKind {
    Stream,
    Command,
    Query,
}

impl CapabilityKind {
    fn short(self) -> &'static str {
        match self {
            CapabilityKind::Stream => "s",
            CapabilityKind::Command => "c",
            CapabilityKind::Query => "q",
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            CapabilityKind::Stream => "stream",
            CapabilityKind::Command => "command",
            CapabilityKind::Query => "query",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct CapabilityKey {
    kind: CapabilityKind,
    namespace: String,
    name: String,
    provider_node_id: String,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
struct NodeRecord {
    node_id: String,
    ridl_namespaces: Vec<String>,
    capabilities: Vec<String>,
    updated_at_ms: u128,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
struct ChannelRecord {
    channel_name: String,
    kind: CapabilityKind,
    namespace: String,
    name: String,
    provider_node_id: String,
    created_at_ms: u128,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
struct LinkRecord {
    requester_id: String,
    target: String,
    provider_node_id: String,
    kind: CapabilityKind,
    namespace: String,
    name: String,
    channel_name: String,
    created_at_ms: u128,
}

#[derive(Debug, Default)]
struct MetaRuntimeState {
    nodes: HashMap<String, NodeRecord>,
    channels: HashMap<CapabilityKey, ChannelRecord>,
    links: Vec<LinkRecord>,
}

#[derive(Debug)]
pub struct MetaRuntimeRegistry {
    inner: RwLock<MetaRuntimeState>,
    channel_prefix: String,
}

impl Default for MetaRuntimeRegistry {
    fn default() -> Self {
        Self::new("/rbnx/ch")
    }
}

impl MetaRuntimeRegistry {
    pub fn new(channel_prefix: impl Into<String>) -> Self {
        Self {
            inner: RwLock::new(MetaRuntimeState::default()),
            channel_prefix: channel_prefix.into(),
        }
    }

    fn now_ms() -> u128 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis()
    }

    fn validate_identifier(field: &str, value: &str, allow_empty: bool) -> Result<String, Status> {
        let value = value.trim();
        if value.is_empty() {
            if allow_empty {
                return Ok(String::new());
            }
            return Err(Status::invalid_argument(format!("{field} must not be empty")));
        }
        if value.len() > 128 {
            return Err(Status::invalid_argument(format!("{field} too long")));
        }
        let valid = value.chars().all(|c| {
            c.is_ascii_alphanumeric()
                || matches!(c, '_' | '-' | '/' | '.' | ':' | '*' )
        });
        if !valid {
            return Err(Status::invalid_argument(format!(
                "{field} contains unsupported characters"
            )));
        }
        Ok(value.to_string())
    }

    fn allocate_channel_name(&self, kind: CapabilityKind) -> String {
        format!(
            "{}/{}/n{}",
            self.channel_prefix,
            kind.short(),
            Uuid::new_v4().simple()
        )
    }

    async fn upsert_node(
        &self,
        node_id: String,
        ridl_namespaces: Vec<String>,
        capabilities: Vec<String>,
    ) -> String {
        let mut inner = self.inner.write().await;
        let node_id = if node_id.trim().is_empty() {
            format!("node-{}", Uuid::new_v4().simple())
        } else {
            node_id
        };
        inner.nodes.insert(
            node_id.clone(),
            NodeRecord {
                node_id: node_id.clone(),
                ridl_namespaces,
                capabilities,
                updated_at_ms: Self::now_ms(),
            },
        );
        node_id
    }

    async fn ensure_node_exists(&self, node_id: &str) {
        let mut inner = self.inner.write().await;
        inner
            .nodes
            .entry(node_id.to_string())
            .or_insert_with(|| NodeRecord {
                node_id: node_id.to_string(),
                ridl_namespaces: Vec::new(),
                capabilities: Vec::new(),
                updated_at_ms: Self::now_ms(),
            })
            .updated_at_ms = Self::now_ms();
    }

    async fn register_capability(
        &self,
        kind: CapabilityKind,
        node_id: String,
        namespace: String,
        name: String,
    ) -> String {
        self.ensure_node_exists(&node_id).await;
        let key = CapabilityKey {
            kind,
            namespace: namespace.clone(),
            name: name.clone(),
            provider_node_id: node_id.clone(),
        };

        let mut inner = self.inner.write().await;
        if let Some(existing) = inner.channels.get(&key) {
            info!(
                "meta-runtime: reused {:?} channel '{}' for provider='{}' namespace='{}' name='{}'",
                kind,
                existing.channel_name,
                node_id,
                namespace,
                name
            );
            return existing.channel_name.clone();
        }

        let channel_name = self.allocate_channel_name(kind);
        info!(
            "meta-runtime: allocated {:?} channel '{}' for provider='{}' namespace='{}' name='{}'",
            kind,
            channel_name,
            node_id,
            namespace,
            name
        );
        inner.channels.insert(
            key,
            ChannelRecord {
                channel_name: channel_name.clone(),
                kind,
                namespace,
                name,
                provider_node_id: node_id,
                created_at_ms: Self::now_ms(),
            },
        );
        channel_name
    }

    async fn resolve_capability(
        &self,
        kind: CapabilityKind,
        requester_id: String,
        target: String,
        namespace: String,
        name: String,
    ) -> Result<String, Status> {
        self.ensure_node_exists(&requester_id).await;

        let mut matches = {
            let inner = self.inner.read().await;
            let mut found: Vec<ChannelRecord> = inner
                .channels
                .values()
                .filter(|entry| {
                    entry.kind == kind
                        && entry.namespace == namespace
                        && entry.name == name
                        && (target.is_empty()
                            || target == "*"
                            || entry.provider_node_id == target)
                })
                .cloned()
                .collect();
            found.sort_by(|a, b| a.provider_node_id.cmp(&b.provider_node_id));
            found
        };

        let selected = matches
            .drain(..)
            .next()
            .ok_or_else(|| Status::not_found("no matching provider/channel found"))?;

        let mut inner = self.inner.write().await;
        inner.links.push(LinkRecord {
            requester_id,
            target,
            provider_node_id: selected.provider_node_id.clone(),
            kind,
            namespace: selected.namespace.clone(),
            name: selected.name.clone(),
            channel_name: selected.channel_name.clone(),
            created_at_ms: Self::now_ms(),
        });
        info!(
            "meta-runtime: resolved {:?} channel '{}' for requester='{}' target='{}' provider='{}' namespace='{}' name='{}'",
            kind,
            selected.channel_name,
            inner.links.last().map(|l| l.requester_id.as_str()).unwrap_or_default(),
            inner.links.last().map(|l| l.target.as_str()).unwrap_or_default(),
            selected.provider_node_id,
            selected.namespace,
            selected.name
        );

        Ok(selected.channel_name)
    }

    async fn inspect_runtime_json(&self) -> Result<String> {
        let inner = self.inner.read().await;

        let mut nodes = inner
            .nodes
            .values()
            .map(|node| {
                json!({
                    "node_id": node.node_id,
                    "ridl_namespaces": node.ridl_namespaces,
                    "capabilities": node.capabilities,
                    "updated_at_ms": node.updated_at_ms.to_string(),
                })
            })
            .collect::<Vec<_>>();
        nodes.sort_by(|a, b| {
            a.get("node_id")
                .and_then(serde_json::Value::as_str)
                .cmp(&b.get("node_id").and_then(serde_json::Value::as_str))
        });

        let mut channels = inner
            .channels
            .values()
            .map(|channel| {
                json!({
                    "channel_name": channel.channel_name,
                    "kind": channel.kind.as_str(),
                    "namespace": channel.namespace,
                    "name": channel.name,
                    "provider_node_id": channel.provider_node_id,
                    "created_at_ms": channel.created_at_ms.to_string(),
                })
            })
            .collect::<Vec<_>>();
        channels.sort_by(|a, b| {
            a.get("channel_name")
                .and_then(serde_json::Value::as_str)
                .cmp(&b.get("channel_name").and_then(serde_json::Value::as_str))
        });

        let mut links = inner
            .links
            .iter()
            .map(|link| {
                json!({
                    "requester_id": link.requester_id,
                    "target": link.target,
                    "provider_node_id": link.provider_node_id,
                    "kind": link.kind.as_str(),
                    "namespace": link.namespace,
                    "name": link.name,
                    "channel_name": link.channel_name,
                    "created_at_ms": link.created_at_ms.to_string(),
                })
            })
            .collect::<Vec<_>>();
        links.sort_by(|a, b| {
            let a_key = (
                a.get("requester_id")
                    .and_then(serde_json::Value::as_str)
                    .unwrap_or_default(),
                a.get("channel_name")
                    .and_then(serde_json::Value::as_str)
                    .unwrap_or_default(),
            );
            let b_key = (
                b.get("requester_id")
                    .and_then(serde_json::Value::as_str)
                    .unwrap_or_default(),
                b.get("channel_name")
                    .and_then(serde_json::Value::as_str)
                    .unwrap_or_default(),
            );
            a_key.cmp(&b_key)
        });

        serde_json::to_string_pretty(&json!({
            "nodes": nodes,
            "channels": channels,
            "links": links,
        }))
        .context("failed to serialize runtime snapshot")
    }
}

#[derive(Clone)]
pub struct MetaRuntimeService {
    registry: Arc<MetaRuntimeRegistry>,
    runtime_endpoint: String,
}

impl MetaRuntimeService {
    pub fn new(registry: Arc<MetaRuntimeRegistry>, runtime_endpoint: String) -> Self {
        Self {
            registry,
            runtime_endpoint,
        }
    }
}

#[tonic::async_trait]
impl pb::robonix_runtime_server::RobonixRuntime for MetaRuntimeService {
    async fn register_node(
        &self,
        request: Request<pb::RegisterNodeRequest>,
    ) -> Result<Response<pb::RegisterNodeResponse>, Status> {
        let req = request.into_inner();
        let node_id = Self::validate_identifier("node_id", &req.node_id, true)?;
        let ridl_namespaces = req
            .ridl_namespaces
            .into_iter()
            .map(|ns| MetaRuntimeRegistry::validate_identifier("ridl_namespace", &ns, false))
            .collect::<Result<Vec<_>, _>>()?;
        let capabilities = req
            .capabilities
            .into_iter()
            .map(|cap| MetaRuntimeRegistry::validate_identifier("capability", &cap, false))
            .collect::<Result<Vec<_>, _>>()?;

        let assigned = self
            .registry
            .upsert_node(node_id, ridl_namespaces, capabilities)
            .await;
        info!("meta-runtime: registered node '{}'", assigned);
        Ok(Response::new(pb::RegisterNodeResponse {
            node_id: assigned,
            runtime_endpoint: self.runtime_endpoint.clone(),
        }))
    }

    async fn register_stream(
        &self,
        request: Request<pb::RegisterStreamRequest>,
    ) -> Result<Response<pb::RegisterStreamResponse>, Status> {
        let req = request.into_inner();
        let node_id = MetaRuntimeRegistry::validate_identifier("node_id", &req.node_id, false)?;
        let namespace = MetaRuntimeRegistry::validate_identifier("namespace", &req.namespace, false)?;
        let stream_name =
            MetaRuntimeRegistry::validate_identifier("stream_name", &req.stream_name, false)?;
        let role = MetaRuntimeRegistry::validate_identifier("role", &req.role, false)?;
        if role != "provider" {
            return Err(Status::invalid_argument(
                "register_stream currently only supports role=provider",
            ));
        }
        let channel_name = self
            .registry
            .register_capability(CapabilityKind::Stream, node_id, namespace, stream_name)
            .await;
        Ok(Response::new(pb::RegisterStreamResponse { channel_name }))
    }

    async fn resolve_stream(
        &self,
        request: Request<pb::ResolveStreamRequest>,
    ) -> Result<Response<pb::ResolveStreamResponse>, Status> {
        let req = request.into_inner();
        let requester_id =
            MetaRuntimeRegistry::validate_identifier("requester_id", &req.requester_id, false)?;
        let target = MetaRuntimeRegistry::validate_identifier("target", &req.target, true)?;
        let namespace = MetaRuntimeRegistry::validate_identifier("namespace", &req.namespace, false)?;
        let stream_name =
            MetaRuntimeRegistry::validate_identifier("stream_name", &req.stream_name, false)?;
        let role = MetaRuntimeRegistry::validate_identifier("role", &req.role, false)?;
        if role != "consumer" {
            return Err(Status::invalid_argument(
                "resolve_stream currently only supports role=consumer",
            ));
        }
        let channel_name = self
            .registry
            .resolve_capability(
                CapabilityKind::Stream,
                requester_id,
                target,
                namespace,
                stream_name,
            )
            .await?;
        Ok(Response::new(pb::ResolveStreamResponse { channel_name }))
    }

    async fn register_command(
        &self,
        request: Request<pb::RegisterCommandRequest>,
    ) -> Result<Response<pb::RegisterCommandResponse>, Status> {
        let req = request.into_inner();
        let node_id = MetaRuntimeRegistry::validate_identifier("node_id", &req.node_id, false)?;
        let namespace = MetaRuntimeRegistry::validate_identifier("namespace", &req.namespace, false)?;
        let command_name =
            MetaRuntimeRegistry::validate_identifier("command_name", &req.command_name, false)?;
        let channel_name = self
            .registry
            .register_capability(CapabilityKind::Command, node_id, namespace, command_name)
            .await;
        Ok(Response::new(pb::RegisterCommandResponse { channel_name }))
    }

    async fn resolve_command(
        &self,
        request: Request<pb::ResolveCommandRequest>,
    ) -> Result<Response<pb::ResolveCommandResponse>, Status> {
        let req = request.into_inner();
        let requester_id =
            MetaRuntimeRegistry::validate_identifier("requester_id", &req.requester_id, false)?;
        let target = MetaRuntimeRegistry::validate_identifier("target", &req.target, true)?;
        let namespace = MetaRuntimeRegistry::validate_identifier("namespace", &req.namespace, false)?;
        let command_name =
            MetaRuntimeRegistry::validate_identifier("command_name", &req.command_name, false)?;
        let channel_name = self
            .registry
            .resolve_capability(
                CapabilityKind::Command,
                requester_id,
                target,
                namespace,
                command_name,
            )
            .await?;
        Ok(Response::new(pb::ResolveCommandResponse { channel_name }))
    }

    async fn register_query(
        &self,
        request: Request<pb::RegisterQueryRequest>,
    ) -> Result<Response<pb::RegisterQueryResponse>, Status> {
        let req = request.into_inner();
        let node_id = MetaRuntimeRegistry::validate_identifier("node_id", &req.node_id, false)?;
        let namespace = MetaRuntimeRegistry::validate_identifier("namespace", &req.namespace, false)?;
        let query_name =
            MetaRuntimeRegistry::validate_identifier("query_name", &req.query_name, false)?;
        let channel_name = self
            .registry
            .register_capability(CapabilityKind::Query, node_id, namespace, query_name)
            .await;
        Ok(Response::new(pb::RegisterQueryResponse { channel_name }))
    }

    async fn resolve_query(
        &self,
        request: Request<pb::ResolveQueryRequest>,
    ) -> Result<Response<pb::ResolveQueryResponse>, Status> {
        let req = request.into_inner();
        let requester_id =
            MetaRuntimeRegistry::validate_identifier("requester_id", &req.requester_id, false)?;
        let target = MetaRuntimeRegistry::validate_identifier("target", &req.target, true)?;
        let namespace = MetaRuntimeRegistry::validate_identifier("namespace", &req.namespace, false)?;
        let query_name =
            MetaRuntimeRegistry::validate_identifier("query_name", &req.query_name, false)?;
        let channel_name = self
            .registry
            .resolve_capability(
                CapabilityKind::Query,
                requester_id,
                target,
                namespace,
                query_name,
            )
            .await?;
        Ok(Response::new(pb::ResolveQueryResponse { channel_name }))
    }

    async fn inspect_runtime(
        &self,
        _request: Request<pb::InspectRuntimeRequest>,
    ) -> Result<Response<pb::InspectRuntimeResponse>, Status> {
        let json = self
            .registry
            .inspect_runtime_json()
            .await
            .map_err(|err| Status::internal(format!("failed to inspect runtime: {err:#}")))?;
        Ok(Response::new(pb::InspectRuntimeResponse { json }))
    }
}

pub async fn serve_meta_runtime(
    registry: Arc<MetaRuntimeRegistry>,
    listen_addr: SocketAddr,
    runtime_endpoint: String,
) -> Result<()> {
    let svc = MetaRuntimeService::new(registry, runtime_endpoint.clone());
    info!(
        "starting robonix runtime meta API (gRPC) on {}",
        runtime_endpoint
    );
    tonic::transport::Server::builder()
        .add_service(pb::robonix_runtime_server::RobonixRuntimeServer::new(svc))
        .serve(listen_addr)
        .await?;
    Ok(())
}

impl MetaRuntimeService {
    fn validate_identifier(field: &str, value: &str, allow_empty: bool) -> Result<String, Status> {
        MetaRuntimeRegistry::validate_identifier(field, value, allow_empty)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn register_reuses_existing_channel_for_same_provider() {
        let registry = MetaRuntimeRegistry::default();

        let first = registry
            .register_capability(
                CapabilityKind::Query,
                "node-a".to_string(),
                "robonix/prm/base".to_string(),
                "get_status".to_string(),
            )
            .await;
        let second = registry
            .register_capability(
                CapabilityKind::Query,
                "node-a".to_string(),
                "robonix/prm/base".to_string(),
                "get_status".to_string(),
            )
            .await;

        assert_eq!(first, second);
        assert!(first.starts_with("/rbnx/ch/q/"));
        let leaf = first.rsplit('/').next().expect("channel leaf token");
        assert!(leaf.starts_with('n'));
    }

    #[tokio::test]
    async fn resolve_prefers_explicit_target_node() {
        let registry = MetaRuntimeRegistry::default();

        let channel_a = registry
            .register_capability(
                CapabilityKind::Command,
                "node-a".to_string(),
                "robonix/prm/base".to_string(),
                "move".to_string(),
            )
            .await;
        let channel_b = registry
            .register_capability(
                CapabilityKind::Command,
                "node-b".to_string(),
                "robonix/prm/base".to_string(),
                "move".to_string(),
            )
            .await;

        let resolved = registry
            .resolve_capability(
                CapabilityKind::Command,
                "client-1".to_string(),
                "node-b".to_string(),
                "robonix/prm/base".to_string(),
                "move".to_string(),
            )
            .await
            .expect("resolution should succeed");

        assert_eq!(resolved, channel_b);
        assert_ne!(resolved, channel_a);
    }
}
