// SPDX-License-Identifier: MulanPSL-2.0
// Robonix runtime meta API (gRPC) — transport-agnostic node/interface/channel registry

use anyhow::{Context, Result};
use log::info;
use serde::Serialize;
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::atomic::{AtomicU16, Ordering};
use tokio::sync::RwLock;
use tonic::{Request, Response, Status};
use uuid::Uuid;

pub mod pb {
    tonic::include_proto!("robonix.runtime");
}

// ── Data model ──────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize)]
struct InterfaceRecord {
    name: String,
    /// Resolved contract ID for QueryNodes (see `DeclareInterfaceRequest.contract_id`).
    contract_id: String,
    supported_transports: Vec<String>,
    metadata_json: String,
    allocated_endpoint: String,
}

#[derive(Debug, Clone, Serialize)]
struct SkillInfoRecord {
    name: String,
    description: String,
    path: String,
    metadata_json: String,
}

#[derive(Debug, Clone, Serialize)]
struct NodeRecord {
    node_id: String,
    namespace: String,
    kind: String,
    interfaces: Vec<InterfaceRecord>,
    #[serde(skip_serializing_if = "Option::is_none")]
    skill_md: Option<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    skills: Vec<SkillInfoRecord>,
    #[serde(skip_serializing_if = "String::is_empty")]
    distro: String,
    #[serde(skip_serializing_if = "String::is_empty")]
    container_id: String,
    heartbeat_ms: u128,
}

#[derive(Debug, Clone, Serialize)]
struct ChannelRecord {
    channel_id: String,
    transport: String,
    endpoint: String,
    provider_node_id: String,
    consumer_id: String,
    interface_name: String,
    metadata_json: String,
}

#[derive(Debug, Default, Serialize)]
pub(crate) struct State {
    nodes: HashMap<String, NodeRecord>,
    channels: HashMap<String, ChannelRecord>,
}

// ── Registry ────────────────────────────────────────────────────────────────

const PORT_RANGE_START: u16 = 50100;

/// Known stable contract paths (same as `[contract] id` in `rust/contracts/`).
/// grpc/ros2 declarations emit a warning when their contract_id is not in this list, but
/// are still accepted — unknown contracts are allowed to support rapid iteration.
const ROBO_SYSTEM_INTERFACE_CATALOG: &[&str] = &[
    // ── primitives (sensors / base) ───────────────────────────────────────────
    "robonix/prm/base/move",
    "robonix/prm/base/odom",
    "robonix/prm/camera/rgb",
    "robonix/prm/camera/depth",
    "robonix/prm/sensor/imu",
    "robonix/prm/sensor/lidar",
    // ── primitives — MCP tool groups (rpc, wire_profile=mcp) ─────────────────
    "robonix/prm/sim/env/tools", // env_node:  get_camera_image, get_robot_state, step_action
    "robonix/prm/perception/tools", // perception_node: detect_objects
    "robonix/prm/manipulation/tools", // vla_node: execute_instruction, move_base
    // ── system services ───────────────────────────────────────────────────────
    "robonix/sys/runtime/pilot",
    "robonix/sys/runtime/executor",
    "robonix/sys/runtime/liaison",
    "robonix/sys/model/vlm/chat",
    "robonix/sys/memory/search", // legacy search-only contract (kept for compat)
    "robonix/sys/memory/tools",  // memsearch_service: search_memory, save_memory, compact_memory
];

#[derive(Debug)]
pub struct MetaRuntimeRegistry {
    pub(crate) inner: RwLock<State>,
    next_port: AtomicU16,
}

impl Default for MetaRuntimeRegistry {
    fn default() -> Self {
        Self {
            inner: RwLock::new(State::default()),
            next_port: AtomicU16::new(PORT_RANGE_START),
        }
    }
}

impl MetaRuntimeRegistry {
    fn join_namespace_leaf(namespace: &str, interface_leaf: &str) -> String {
        let ns = namespace.trim().trim_end_matches('/');
        let leaf = interface_leaf.trim().trim_start_matches('/');
        format!("{ns}/{leaf}")
    }

    fn transports_require_robonix_catalog(transports: &[String]) -> bool {
        transports
            .iter()
            .any(|t| t == "grpc" || t == "ros2" || t == "mcp")
    }

    fn is_catalogued_robonix_interface(abstract_path: &str) -> bool {
        ROBO_SYSTEM_INTERFACE_CATALOG
            .iter()
            .any(|&id| id == abstract_path)
    }

    /// For `grpc` / `ros2` / `mcp`, namespace must be under `robonix/...`.
    /// Explicit `contract_id` (stable path) is preferred; if absent the path derived from
    /// namespace+name is used.  Unknown contracts emit a warning but are still accepted.
    ///
    /// MCP-transport interfaces additionally warn when `contract_id` is absent, because each MCP
    /// tool group must be registered under a stable capability path from `rust/contracts/`.
    fn validate_robonix_system_interface_for_transports(
        node_namespace: &str,
        interface_leaf: &str,
        contract_id_override: &str,
        transports: &[String],
    ) -> Result<(), Status> {
        if !Self::transports_require_robonix_catalog(transports) {
            return Ok(());
        }
        let ns = node_namespace.trim();
        if !ns.starts_with("robonix/") {
            return Err(Status::invalid_argument(
                "grpc/ros2/mcp DeclareInterface requires RegisterNode.namespace under robonix/...",
            ));
        }

        let is_mcp = transports.iter().any(|t| t == "mcp");
        let has_explicit_contract = !contract_id_override.trim().is_empty();

        if is_mcp && !has_explicit_contract {
            // MCP tool groups must bind to a stable contract_id so Executor can route them
            // correctly and the capability can be discovered by contract path.
            log::warn!(
                "mcp DeclareInterface on '{ns}/{interface_leaf}' has no contract_id — \
                 set contract_id to e.g. 'robonix/prm/…/tools' (see rust/contracts/)"
            );
        }

        let effective = if has_explicit_contract {
            contract_id_override.trim().to_string()
        } else {
            Self::join_namespace_leaf(ns, interface_leaf)
        };
        if !Self::is_catalogued_robonix_interface(&effective) {
            let transport_label = if is_mcp { "mcp" } else { "grpc/ros2" };
            log::warn!(
                "unknown contract \"{effective}\" for {transport_label} (not in catalog) — allowing anyway"
            );
        }
        Ok(())
    }

    fn now_ms() -> u128 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis()
    }

    /// Stable capability path: `robonix/.../leaf` (no version suffix — version is in contract TOML).
    fn validate_contract_id(field: &str, value: &str) -> Result<(), Status> {
        let s = value.trim();
        if s.is_empty() {
            return Err(Status::invalid_argument(format!(
                "{field} must not be empty"
            )));
        }
        if s.len() > 512 {
            return Err(Status::invalid_argument(format!(
                "{field} is too long (max 512)"
            )));
        }
        if s.contains("..") || s.contains("//") {
            return Err(Status::invalid_argument(format!(
                "{field} must not contain '..' or '//'"
            )));
        }
        let first = s.chars().next().unwrap();
        let last = s.chars().last().unwrap();
        if first == '/' || last == '/' {
            return Err(Status::invalid_argument(format!(
                "{field} must not start or end with '/'"
            )));
        }
        Ok(())
    }

    fn validate_id(field: &str, value: &str, allow_empty: bool) -> Result<String, Status> {
        let v = value.trim();
        if v.is_empty() {
            if allow_empty {
                return Ok(String::new());
            }
            return Err(Status::invalid_argument(format!(
                "{field} must not be empty"
            )));
        }
        if v.len() > 128 {
            return Err(Status::invalid_argument(format!("{field} too long")));
        }
        if !v
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '/' | '.' | ':' | '*'))
        {
            return Err(Status::invalid_argument(format!(
                "{field} contains unsupported characters"
            )));
        }
        Ok(v.to_string())
    }

    /// `node_id`, `consumer_id`, and `provider_node_id` use **reverse-DNS** form so they are
    /// globally unique and human-readable: `com.<org>.<product>[.<role>]` (minimum 3 dot labels).
    ///
    /// Segments after `com` are ASCII alphanumeric or `_`. Empty `node_id` on register is allowed
    /// and the server assigns `com.robonix.ephemeral.<uuid>`.
    pub(crate) fn validate_node_identity(
        field: &str,
        value: &str,
        allow_empty: bool,
    ) -> Result<String, Status> {
        let v = value.trim();
        if v.is_empty() {
            if allow_empty {
                return Ok(String::new());
            }
            return Err(Status::invalid_argument(format!(
                "{field} must not be empty"
            )));
        }
        if v.len() > 128 {
            return Err(Status::invalid_argument(format!("{field} too long")));
        }
        let parts: Vec<&str> = v.split('.').collect();
        if parts.iter().any(|s| s.is_empty()) {
            return Err(Status::invalid_argument(format!(
                "{field} must not contain empty segments"
            )));
        }
        if parts.len() < 3 {
            return Err(Status::invalid_argument(format!(
                "{field} must be reverse-DNS with at least 3 labels, e.g. com.vendor.service (see NAMESPACE.md)"
            )));
        }
        if parts[0] != "com" {
            return Err(Status::invalid_argument(format!(
                "{field} must start with \"com.\""
            )));
        }
        if !parts[1..]
            .iter()
            .all(|p| p.chars().all(|c| c.is_ascii_alphanumeric() || c == '_'))
        {
            return Err(Status::invalid_argument(format!(
                "{field} segments after \"com\" must be ASCII alphanumeric or underscore"
            )));
        }
        Ok(v.to_string())
    }

    fn allocate_endpoint_static(transport: &str) -> String {
        let id = Uuid::new_v4().simple().to_string();
        match transport {
            "ros2" => format!("/rbnx/ch/n{id}"),
            "shared_memory" => format!("/rbnx_shm_{id}"),
            other => format!("/rbnx/{other}/{id}"),
        }
    }

    /// Allocate a data-plane endpoint for a (node, transport) pair.
    ///
    /// For port-based transports (grpc, mcp): if the node already has an
    /// interface on the same transport, reuse the existing port so one
    /// server process can serve all interfaces.  Otherwise increment the
    /// global port counter.
    fn allocate_endpoint_for_node(
        &self,
        transport: &str,
        existing_interfaces: &[InterfaceRecord],
        listen_port: u32,
    ) -> String {
        match transport {
            "grpc" | "mcp" => {
                if listen_port != 0 {
                    let host = std::env::var("ROBONIX_DATA_PLANE_HOST")
                        .unwrap_or_else(|_| "localhost".to_string());
                    return format!("{host}:{listen_port}");
                }
                // Reuse the port if this node already has an interface on the
                // same transport.
                for iface in existing_interfaces {
                    if iface.supported_transports.contains(&transport.to_string())
                        && !iface.allocated_endpoint.is_empty()
                    {
                        return iface.allocated_endpoint.clone();
                    }
                }
                let port = self.next_port.fetch_add(1, Ordering::Relaxed);
                let host = std::env::var("ROBONIX_DATA_PLANE_HOST")
                    .unwrap_or_else(|_| "localhost".to_string());
                format!("{host}:{port}")
            }
            _ => Self::allocate_endpoint_static(transport),
        }
    }

    async fn register_node(
        &self,
        node_id: String,
        namespace: String,
        kind: String,
        skill_md: Option<String>,
        skills: Vec<SkillInfoRecord>,
        distro: String,
        container_id: String,
    ) -> String {
        let mut st = self.inner.write().await;
        let node_id = if node_id.is_empty() {
            format!("com.robonix.ephemeral.{}", Uuid::new_v4().simple())
        } else {
            node_id
        };
        st.nodes
            .entry(node_id.clone())
            .and_modify(|n| {
                n.namespace = namespace.clone();
                n.kind = kind.clone();
                n.skill_md = skill_md.clone();
                // Only overwrite skills if the caller provides a non-empty list.
                // This allows `rbnx start` to pre-register skills, and the node
                // process's own RegisterNode (with empty skills) won't wipe them.
                if !skills.is_empty() {
                    n.skills = skills.clone();
                }
                n.distro = distro.clone();
                n.container_id = container_id.clone();
                n.heartbeat_ms = Self::now_ms();
            })
            .or_insert_with(|| NodeRecord {
                node_id: node_id.clone(),
                namespace,
                kind,
                interfaces: Vec::new(),
                skill_md,
                skills,
                distro,
                container_id,
                heartbeat_ms: Self::now_ms(),
            });
        node_id
    }

    async fn declare_interface(
        &self,
        node_id: &str,
        name: String,
        transports: Vec<String>,
        metadata_json: String,
        listen_port: u32,
        contract_id: String,
    ) -> Result<String, Status> {
        if listen_port != 0 && !(1..=65535).contains(&listen_port) {
            return Err(Status::invalid_argument(
                "listen_port must be 0 or in 1..=65535",
            ));
        }
        let mut st = self.inner.write().await;
        let node = st
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| Status::not_found(format!("node '{node_id}' not registered")))?;

        let trimmed_contract = contract_id.trim();
        Self::validate_robonix_system_interface_for_transports(
            &node.namespace,
            &name,
            trimmed_contract,
            &transports,
        )?;

        // Prefer explicit contract_id (stable path). Else derive namespace+name.
        let effective_contract = if trimmed_contract.is_empty() {
            Self::join_namespace_leaf(&node.namespace, &name)
        } else {
            trimmed_contract.to_string()
        };
        Self::validate_contract_id("contract_id", &effective_contract)?;

        let primary_transport = transports.first().map(|s| s.as_str()).unwrap_or("");
        let endpoint =
            self.allocate_endpoint_for_node(primary_transport, &node.interfaces, listen_port);

        let enriched_meta = Self::inject_endpoint(&metadata_json, &endpoint);

        if let Some(iface) = node
            .interfaces
            .iter_mut()
            .find(|i| i.name == name && i.supported_transports == transports)
        {
            iface.contract_id = effective_contract;
            iface.metadata_json = enriched_meta;
            iface.allocated_endpoint = endpoint.clone();
        } else {
            node.interfaces.push(InterfaceRecord {
                name,
                contract_id: effective_contract,
                supported_transports: transports,
                metadata_json: enriched_meta,
                allocated_endpoint: endpoint.clone(),
            });
        }
        node.heartbeat_ms = Self::now_ms();
        Ok(endpoint)
    }

    async fn unregister_node(&self, node_id: &str) -> bool {
        let mut st = self.inner.write().await;
        if st.nodes.remove(node_id).is_none() {
            return false;
        }
        let ids: Vec<String> = st
            .channels
            .iter()
            .filter(|(_, ch)| ch.provider_node_id == node_id || ch.consumer_id == node_id)
            .map(|(k, _)| k.clone())
            .collect();
        for k in ids {
            st.channels.remove(&k);
        }
        true
    }

    async fn node_heartbeat(&self, node_id: &str) -> Result<u128, Status> {
        let mut st = self.inner.write().await;
        let node = st
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| Status::not_found(format!("node '{node_id}' not found")))?;
        let t = Self::now_ms();
        node.heartbeat_ms = t;
        Ok(t)
    }

    fn inject_endpoint(metadata_json: &str, endpoint: &str) -> String {
        if let Ok(mut meta) = serde_json::from_str::<serde_json::Value>(metadata_json) {
            if let Some(obj) = meta.as_object_mut() {
                obj.insert(
                    "endpoint".into(),
                    serde_json::Value::String(endpoint.to_string()),
                );
                return serde_json::to_string(&meta).unwrap_or_else(|_| metadata_json.to_string());
            }
        }
        // If metadata_json is empty or not an object, create a minimal one.
        serde_json::json!({ "endpoint": endpoint }).to_string()
    }

    async fn negotiate_channel(
        &self,
        consumer_id: String,
        provider_node_id: String,
        interface_name: String,
        transport: String,
    ) -> Result<ChannelRecord, Status> {
        let st = self.inner.read().await;
        let node = st
            .nodes
            .get(&provider_node_id)
            .ok_or_else(|| Status::not_found(format!("provider '{provider_node_id}' not found")))?;
        let iface = node
            .interfaces
            .iter()
            .find(|i| i.name == interface_name && i.supported_transports.contains(&transport))
            .ok_or_else(|| {
                let available: Vec<Vec<String>> = node
                    .interfaces
                    .iter()
                    .filter(|i| i.name == interface_name)
                    .map(|i| i.supported_transports.clone())
                    .collect();
                if available.is_empty() {
                    Status::not_found(format!(
                        "interface '{interface_name}' not found on '{provider_node_id}'"
                    ))
                } else {
                    Status::invalid_argument(format!(
                        "transport '{transport}' not supported by interface '{interface_name}' (available transport sets: {available:?})"
                    ))
                }
            })?;
        // Reuse the producer's endpoint for transports that share state
        // (grpc, mcp, shared_memory). Other transports get a fresh name.
        let endpoint = match transport.as_str() {
            "grpc" | "mcp" | "shared_memory" => iface.allocated_endpoint.clone(),
            _ => Self::allocate_endpoint_static(&transport),
        };
        let metadata_json = iface.metadata_json.clone();
        drop(st);

        let channel_id = format!("ch-{}", Uuid::new_v4().simple());
        let rec = ChannelRecord {
            channel_id: channel_id.clone(),
            transport,
            endpoint,
            provider_node_id,
            consumer_id,
            interface_name,
            metadata_json,
        };

        let mut st = self.inner.write().await;
        st.channels.insert(channel_id, rec.clone());
        Ok(rec)
    }

    async fn release_channel(&self, channel_id: &str) -> bool {
        let mut st = self.inner.write().await;
        st.channels.remove(channel_id).is_some()
    }

    async fn query_skill_md(&self, node_id: &str) -> Result<String, Status> {
        let st = self.inner.read().await;
        let node = st
            .nodes
            .get(node_id)
            .ok_or_else(|| Status::not_found(format!("node '{node_id}' not found")))?;
        Ok(node.skill_md.clone().unwrap_or_default())
    }

    async fn query_all_skills(
        &self,
    ) -> Vec<(String, String, String, String, Vec<SkillInfoRecord>)> {
        let st = self.inner.read().await;
        st.nodes
            .values()
            .filter(|n| n.skill_md.is_some() || !n.skills.is_empty())
            .map(|n| {
                (
                    n.node_id.clone(),
                    n.namespace.clone(),
                    n.kind.clone(),
                    n.skill_md.clone().unwrap_or_default(),
                    n.skills.clone(),
                )
            })
            .collect()
    }

    async fn inspect_json(&self) -> Result<String> {
        let st = self.inner.read().await;
        serde_json::to_string_pretty(&*st).context("failed to serialize runtime snapshot")
    }
}

// ── gRPC service ────────────────────────────────────────────────────────────

#[derive(Clone)]
pub struct MetaRuntimeService {
    registry: Arc<MetaRuntimeRegistry>,
}

impl MetaRuntimeService {
    pub fn new(registry: Arc<MetaRuntimeRegistry>) -> Self {
        Self { registry }
    }

    fn v(field: &str, value: &str, allow_empty: bool) -> Result<String, Status> {
        MetaRuntimeRegistry::validate_id(field, value, allow_empty)
    }
}

#[tonic::async_trait]
impl pb::robonix_runtime_server::RobonixRuntime for MetaRuntimeService {
    async fn register_node(
        &self,
        request: Request<pb::RegisterNodeRequest>,
    ) -> Result<Response<pb::RegisterNodeResponse>, Status> {
        let r = request.into_inner();
        let node_id = MetaRuntimeRegistry::validate_node_identity("node_id", &r.node_id, true)?;
        let namespace = Self::v("namespace", &r.namespace, true)?;
        let kind = Self::v("kind", &r.kind, true)?;
        let skill_md = if r.skill_md.is_empty() {
            None
        } else {
            Some(r.skill_md)
        };
        let skills: Vec<SkillInfoRecord> = r
            .skills
            .into_iter()
            .map(|s| SkillInfoRecord {
                name: s.name,
                description: s.description,
                path: s.path,
                metadata_json: s.metadata_json,
            })
            .collect();
        let distro = r.distro.trim().to_string();
        let container_id = r.container_id.trim().to_string();
        let id = self
            .registry
            .register_node(
                node_id,
                namespace,
                kind,
                skill_md,
                skills.clone(),
                distro.clone(),
                container_id.clone(),
            )
            .await;
        let extra = if distro.is_empty() {
            String::new()
        } else {
            format!(" distro={distro}")
        };
        let skill_extra = if skills.is_empty() {
            String::new()
        } else {
            format!(" skills={}", skills.len())
        };
        info!("meta-runtime: registered node '{id}'{extra}{skill_extra}");
        Ok(Response::new(pb::RegisterNodeResponse { node_id: id }))
    }

    async fn unregister_node(
        &self,
        request: Request<pb::UnregisterNodeRequest>,
    ) -> Result<Response<pb::UnregisterNodeResponse>, Status> {
        let r = request.into_inner();
        let node_id = MetaRuntimeRegistry::validate_node_identity("node_id", &r.node_id, false)?;
        let ok = self.registry.unregister_node(&node_id).await;
        if ok {
            info!("meta-runtime: unregistered node '{node_id}'");
        }
        Ok(Response::new(pb::UnregisterNodeResponse { ok }))
    }

    async fn node_heartbeat(
        &self,
        request: Request<pb::NodeHeartbeatRequest>,
    ) -> Result<Response<pb::NodeHeartbeatResponse>, Status> {
        let r = request.into_inner();
        let node_id = MetaRuntimeRegistry::validate_node_identity("node_id", &r.node_id, false)?;
        let t = self.registry.node_heartbeat(&node_id).await?;
        let server_time_ms = u64::try_from(t).unwrap_or(u64::MAX);
        Ok(Response::new(pb::NodeHeartbeatResponse {
            ok: true,
            server_time_ms,
        }))
    }

    async fn declare_interface(
        &self,
        request: Request<pb::DeclareInterfaceRequest>,
    ) -> Result<Response<pb::DeclareInterfaceResponse>, Status> {
        let r = request.into_inner();
        let node_id = MetaRuntimeRegistry::validate_node_identity("node_id", &r.node_id, false)?;
        let name = Self::v("name", &r.name, false)?;
        let endpoint = self
            .registry
            .declare_interface(
                &node_id,
                name.clone(),
                r.supported_transports,
                r.metadata_json,
                r.listen_port,
                r.contract_id,
            )
            .await?;
        info!(
            "meta-runtime: declared interface '{name}' on node '{node_id}' → endpoint '{endpoint}'"
        );
        Ok(Response::new(pb::DeclareInterfaceResponse {
            ok: true,
            allocated_endpoint: endpoint,
        }))
    }

    async fn query_nodes(
        &self,
        request: Request<pb::QueryNodesRequest>,
    ) -> Result<Response<pb::QueryNodesResponse>, Status> {
        let r = request.into_inner();
        let distro_prefix = r.distro_prefix.trim();
        let container_filter = r.container_id.trim();
        let contract_filter = r.contract_id.trim();
        let st = self.registry.inner.read().await;
        let nodes: Vec<pb::NodeInfo> = st
            .nodes
            .values()
            .filter(|n| {
                if contract_filter.is_empty() {
                    (r.namespace.is_empty() || n.namespace.starts_with(&r.namespace))
                        && (r.name.is_empty() || n.interfaces.iter().any(|i| i.name == r.name))
                } else {
                    n.interfaces.iter().any(|i| {
                        i.contract_id == contract_filter
                            && (r.transport.is_empty()
                                || i.supported_transports.contains(&r.transport))
                    })
                }
            })
            .filter(|n| {
                if contract_filter.is_empty() {
                    r.transport.is_empty()
                        || n.interfaces
                            .iter()
                            .any(|i| i.supported_transports.contains(&r.transport))
                } else {
                    true
                }
            })
            .filter(|n| distro_prefix.is_empty() || n.distro.starts_with(distro_prefix))
            .filter(|n| container_filter.is_empty() || n.container_id == container_filter)
            .map(|n| pb::NodeInfo {
                node_id: n.node_id.clone(),
                namespace: n.namespace.clone(),
                kind: n.kind.clone(),
                has_skill_md: n.skill_md.is_some(),
                interfaces: n
                    .interfaces
                    .iter()
                    .map(|i| pb::InterfaceInfo {
                        name: i.name.clone(),
                        supported_transports: i.supported_transports.clone(),
                        metadata_json: i.metadata_json.clone(),
                        contract_id: i.contract_id.clone(),
                    })
                    .collect(),
                distro: n.distro.clone(),
                container_id: n.container_id.clone(),
                last_heartbeat_ms: u64::try_from(n.heartbeat_ms).unwrap_or(0),
                skills: n
                    .skills
                    .iter()
                    .map(|s| pb::SkillInfo {
                        name: s.name.clone(),
                        description: s.description.clone(),
                        path: s.path.clone(),
                        metadata_json: s.metadata_json.clone(),
                    })
                    .collect(),
            })
            .collect();
        Ok(Response::new(pb::QueryNodesResponse { nodes }))
    }

    async fn negotiate_channel(
        &self,
        request: Request<pb::NegotiateChannelRequest>,
    ) -> Result<Response<pb::NegotiateChannelResponse>, Status> {
        let r = request.into_inner();
        let consumer_id =
            MetaRuntimeRegistry::validate_node_identity("consumer_id", &r.consumer_id, false)?;
        let provider = MetaRuntimeRegistry::validate_node_identity(
            "provider_node_id",
            &r.provider_node_id,
            false,
        )?;
        let iface = Self::v("interface_name", &r.interface_name, false)?;
        let transport = Self::v("transport", &r.transport, false)?;
        let ch = self
            .registry
            .negotiate_channel(consumer_id, provider, iface, transport)
            .await?;
        info!(
            "meta-runtime: negotiated channel '{}' endpoint='{}'",
            ch.channel_id, ch.endpoint
        );
        Ok(Response::new(pb::NegotiateChannelResponse {
            channel_id: ch.channel_id,
            transport: ch.transport,
            endpoint: ch.endpoint,
            metadata_json: ch.metadata_json,
        }))
    }

    async fn release_channel(
        &self,
        request: Request<pb::ReleaseChannelRequest>,
    ) -> Result<Response<pb::ReleaseChannelResponse>, Status> {
        let r = request.into_inner();
        let ok = self.registry.release_channel(&r.channel_id).await;
        Ok(Response::new(pb::ReleaseChannelResponse { ok }))
    }

    async fn query_skill_md(
        &self,
        request: Request<pb::QuerySkillMdRequest>,
    ) -> Result<Response<pb::QuerySkillMdResponse>, Status> {
        let r = request.into_inner();
        let node_id = MetaRuntimeRegistry::validate_node_identity("node_id", &r.node_id, false)?;
        let skill_md = self.registry.query_skill_md(&node_id).await?;
        Ok(Response::new(pb::QuerySkillMdResponse { skill_md }))
    }

    async fn query_all_skills(
        &self,
        _request: Request<pb::QueryAllSkillsRequest>,
    ) -> Result<Response<pb::QueryAllSkillsResponse>, Status> {
        let skills = self
            .registry
            .query_all_skills()
            .await
            .into_iter()
            .map(
                |(node_id, namespace, kind, skill_md, skill_infos)| pb::SkillEntry {
                    node_id,
                    namespace,
                    kind,
                    skill_md,
                    skills: skill_infos
                        .into_iter()
                        .map(|s| pb::SkillInfo {
                            name: s.name,
                            description: s.description,
                            path: s.path,
                            metadata_json: s.metadata_json,
                        })
                        .collect(),
                },
            )
            .collect();
        Ok(Response::new(pb::QueryAllSkillsResponse { skills }))
    }

    async fn inspect_runtime(
        &self,
        _request: Request<pb::InspectRuntimeRequest>,
    ) -> Result<Response<pb::InspectRuntimeResponse>, Status> {
        let json = self
            .registry
            .inspect_json()
            .await
            .map_err(|e| Status::internal(format!("inspect failed: {e:#}")))?;
        Ok(Response::new(pb::InspectRuntimeResponse { json }))
    }
}

// ── Server entrypoint ───────────────────────────────────────────────────────

pub async fn serve_meta_runtime(
    registry: Arc<MetaRuntimeRegistry>,
    listen_addr: SocketAddr,
    runtime_endpoint: String,
) -> Result<()> {
    let svc = MetaRuntimeService::new(registry);
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

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    async fn reg_node(reg: &MetaRuntimeRegistry, id: &str, ns: &str, kind: &str) -> String {
        reg.register_node(
            id.into(),
            ns.into(),
            kind.into(),
            None,
            Vec::new(),
            String::new(),
            String::new(),
        )
        .await
    }

    async fn reg_node_distro(
        reg: &MetaRuntimeRegistry,
        id: &str,
        ns: &str,
        kind: &str,
        distro: &str,
        container: &str,
    ) -> String {
        reg.register_node(
            id.into(),
            ns.into(),
            kind.into(),
            None,
            Vec::new(),
            distro.into(),
            container.into(),
        )
        .await
    }

    #[tokio::test]
    async fn register_node_assigns_id_when_empty() {
        let reg = MetaRuntimeRegistry::default();
        let id = reg_node(&reg, "", "ns", "primitive").await;
        assert!(id.starts_with("com.robonix.ephemeral."));
    }

    #[tokio::test]
    async fn register_node_preserves_explicit_id() {
        let reg = MetaRuntimeRegistry::default();
        let id = reg_node(&reg, "com.test.camera", "ns", "primitive").await;
        assert_eq!(id, "com.test.camera");
    }

    #[tokio::test]
    async fn declare_interface_and_negotiate() {
        let reg = MetaRuntimeRegistry::default();
        reg_node(&reg, "com.test.provider", "robonix/prm/camera", "primitive").await;
        reg.declare_interface(
            "com.test.provider",
            "rgb".into(),
            vec!["ros2".into(), "shared_memory".into()],
            "{}".into(),
            0,
            String::new(),
        )
        .await
        .unwrap();

        let ch = reg
            .negotiate_channel(
                "com.test.consumer".into(),
                "com.test.provider".into(),
                "rgb".into(),
                "ros2".into(),
            )
            .await
            .unwrap();
        assert!(ch.endpoint.starts_with("/rbnx/ch/n"));
        assert_eq!(ch.transport, "ros2");

        let ch2 = reg
            .negotiate_channel(
                "com.test.consumer".into(),
                "com.test.provider".into(),
                "rgb".into(),
                "shared_memory".into(),
            )
            .await
            .unwrap();
        assert!(ch2.endpoint.starts_with("/rbnx_shm_"));
    }

    #[tokio::test]
    async fn negotiate_rejects_unsupported_transport() {
        let reg = MetaRuntimeRegistry::default();
        reg_node(&reg, "com.test.provider", "robonix/prm/camera", "primitive").await;
        reg.declare_interface(
            "com.test.provider",
            "rgb".into(),
            vec!["ros2".into()],
            "{}".into(),
            0,
            String::new(),
        )
        .await
        .unwrap();

        let err = reg
            .negotiate_channel(
                "com.test.consumer".into(),
                "com.test.provider".into(),
                "rgb".into(),
                "shared_memory".into(),
            )
            .await;
        assert!(err.is_err());
    }

    #[tokio::test]
    async fn release_channel_removes_it() {
        let reg = MetaRuntimeRegistry::default();
        reg_node(&reg, "com.test.provider", "robonix/prm/sensor", "primitive").await;
        reg.declare_interface(
            "com.test.provider",
            "lidar".into(),
            vec!["ros2".into()],
            String::new(),
            0,
            String::new(),
        )
        .await
        .unwrap();
        let ch = reg
            .negotiate_channel(
                "com.test.consumer".into(),
                "com.test.provider".into(),
                "lidar".into(),
                "ros2".into(),
            )
            .await
            .unwrap();

        assert!(reg.release_channel(&ch.channel_id).await);
        assert!(!reg.release_channel(&ch.channel_id).await);
    }

    #[tokio::test]
    async fn skill_md_round_trip() {
        let reg = MetaRuntimeRegistry::default();
        reg.register_node(
            "com.test.llm".into(),
            "ns".into(),
            "skill".into(),
            Some("# My Skill\nDoes stuff.".into()),
            Vec::new(),
            String::new(),
            String::new(),
        )
        .await;

        let md = reg.query_skill_md("com.test.llm").await.unwrap();
        assert!(md.contains("My Skill"));

        let all = reg.query_all_skills().await;
        assert_eq!(all.len(), 1);
        assert_eq!(all[0].0, "com.test.llm");
    }

    #[tokio::test]
    async fn declare_interface_uses_listen_port_when_set() {
        let reg = MetaRuntimeRegistry::default();
        reg_node(&reg, "com.test.p", "robonix/sys/model/vlm", "primitive").await;
        let ep = reg
            .declare_interface(
                "com.test.p",
                "chat".into(),
                vec!["grpc".into()],
                "{}".into(),
                55221,
                String::new(),
            )
            .await
            .unwrap();
        assert_eq!(ep, "localhost:55221");
    }

    #[tokio::test]
    async fn server_allocates_grpc_port() {
        let reg = MetaRuntimeRegistry::default();
        reg_node(&reg, "com.test.sim", "robonix/prm/base", "primitive").await;

        let ep = reg
            .declare_interface(
                "com.test.sim",
                "navigate".into(),
                vec!["grpc".into()],
                "{}".into(),
                0,
                String::new(),
            )
            .await
            .unwrap();
        assert!(
            ep.starts_with("localhost:"),
            "expected localhost:<port>, got {ep}"
        );

        let ep2 = reg
            .declare_interface(
                "com.test.sim",
                "stop".into(),
                vec!["grpc".into()],
                "{}".into(),
                0,
                String::new(),
            )
            .await
            .unwrap();
        assert_eq!(ep, ep2);
    }

    #[tokio::test]
    async fn server_allocates_mcp_port() {
        let reg = MetaRuntimeRegistry::default();
        reg_node(&reg, "com.test.vla", "ns", "service").await;
        let ep = reg
            .declare_interface(
                "com.test.vla",
                "mcp_tools".into(),
                vec!["mcp".into()],
                "{}".into(),
                0,
                String::new(),
            )
            .await
            .unwrap();
        assert!(
            ep.starts_with("localhost:"),
            "expected localhost:<port>, got {ep}"
        );

        let ch = reg
            .negotiate_channel(
                "com.test.agent".into(),
                "com.test.vla".into(),
                "mcp_tools".into(),
                "mcp".into(),
            )
            .await
            .unwrap();
        assert_eq!(ch.endpoint, ep);
    }

    #[tokio::test]
    async fn different_nodes_get_different_ports() {
        let reg = MetaRuntimeRegistry::default();
        reg_node(&reg, "com.test.node_a", "robonix/prm/base", "primitive").await;
        reg_node(&reg, "com.test.node_b", "robonix/prm/base", "primitive").await;

        let ep_a = reg
            .declare_interface(
                "com.test.node_a",
                "navigate".into(),
                vec!["grpc".into()],
                "{}".into(),
                0,
                String::new(),
            )
            .await
            .unwrap();
        let ep_b = reg
            .declare_interface(
                "com.test.node_b",
                "stop".into(),
                vec!["grpc".into()],
                "{}".into(),
                0,
                String::new(),
            )
            .await
            .unwrap();
        assert_ne!(ep_a, ep_b);
    }

    #[tokio::test]
    async fn inspect_returns_valid_json() {
        let reg = MetaRuntimeRegistry::default();
        reg_node(&reg, "com.test.n1", "ns", "service").await;
        let json_str = reg.inspect_json().await.unwrap();
        let v: serde_json::Value = serde_json::from_str(&json_str).unwrap();
        assert!(v.get("nodes").is_some());
        assert!(v.get("channels").is_some());
    }

    #[tokio::test]
    async fn distro_and_container_stored() {
        let reg = MetaRuntimeRegistry::default();
        reg_node_distro(
            &reg,
            "com.test.nav2",
            "ns",
            "primitive",
            "humble",
            "nav2-container",
        )
        .await;
        reg_node_distro(
            &reg,
            "com.test.isaac",
            "ns",
            "primitive",
            "jazzy",
            "isaac-container",
        )
        .await;

        let st = reg.inner.read().await;
        assert_eq!(st.nodes["com.test.nav2"].distro, "humble");
        assert_eq!(st.nodes["com.test.nav2"].container_id, "nav2-container");
        assert_eq!(st.nodes["com.test.isaac"].distro, "jazzy");
        assert_eq!(st.nodes["com.test.isaac"].container_id, "isaac-container");
    }

    #[tokio::test]
    async fn inspect_includes_distro() {
        let reg = MetaRuntimeRegistry::default();
        reg_node_distro(&reg, "com.test.n1", "ns", "service", "humble", "ctr-1").await;
        let json_str = reg.inspect_json().await.unwrap();
        let v: serde_json::Value = serde_json::from_str(&json_str).unwrap();
        let node = &v["nodes"]["com.test.n1"];
        assert_eq!(node["distro"], "humble");
        assert_eq!(node["container_id"], "ctr-1");
    }

    #[tokio::test]
    async fn unregister_node_removes_node_and_channels() {
        let reg = MetaRuntimeRegistry::default();
        reg_node(&reg, "com.test.provider", "robonix/prm/camera", "primitive").await;
        reg_node(
            &reg,
            "com.test.consumer",
            "robonix/sys/runtime/agent",
            "tool",
        )
        .await;
        reg.declare_interface(
            "com.test.provider",
            "rgb".into(),
            vec!["ros2".into()],
            "{}".into(),
            0,
            String::new(),
        )
        .await
        .unwrap();
        let ch = reg
            .negotiate_channel(
                "com.test.consumer".into(),
                "com.test.provider".into(),
                "rgb".into(),
                "ros2".into(),
            )
            .await
            .unwrap();
        assert!(reg.inner.read().await.channels.contains_key(&ch.channel_id));

        assert!(reg.unregister_node("com.test.provider").await);
        assert!(
            !reg.inner
                .read()
                .await
                .nodes
                .contains_key("com.test.provider")
        );
        assert!(!reg.inner.read().await.channels.contains_key(&ch.channel_id));

        assert!(!reg.unregister_node("com.test.provider").await);
    }

    #[tokio::test]
    async fn node_heartbeat_updates_timestamp() {
        let reg = MetaRuntimeRegistry::default();
        reg_node(&reg, "com.test.node", "ns", "primitive").await;
        let t0 = reg.inner.read().await.nodes["com.test.node"].heartbeat_ms;
        tokio::time::sleep(std::time::Duration::from_millis(5)).await;
        let t1 = reg.node_heartbeat("com.test.node").await.unwrap();
        assert!(t1 >= t0);
        assert_eq!(
            reg.inner.read().await.nodes["com.test.node"].heartbeat_ms,
            t1
        );
    }

    #[test]
    fn validate_node_identity_rejects_non_reverse_dns() {
        assert!(
            MetaRuntimeRegistry::validate_node_identity("node_id", "tiago-node", false).is_err()
        );
        assert!(
            MetaRuntimeRegistry::validate_node_identity("node_id", "com.short", false).is_err()
        );
        assert!(
            MetaRuntimeRegistry::validate_node_identity("node_id", "com.test.ok", false).is_ok()
        );
    }

    #[tokio::test]
    async fn contract_id_legacy_derives_from_namespace_and_name() {
        // When contract_id is empty, Atlas derives it from namespace+name (legacy behavior).
        let reg = MetaRuntimeRegistry::default();
        reg.register_node(
            "com.test.vlm".into(),
            "robonix/sys/model/vlm".into(),
            "service".into(),
            None,
            Vec::new(),
            String::new(),
            String::new(),
        )
        .await;
        reg.declare_interface(
            "com.test.vlm",
            "chat".into(),
            vec!["grpc".into()],
            "{}".into(),
            0,
            String::new(), // empty → auto-derive
        )
        .await
        .unwrap();
        let st = reg.inner.read().await;
        // Legacy derived contract_id uses slash notation.
        let derived = "robonix/sys/model/vlm/chat";
        let count = st
            .nodes
            .values()
            .filter(|n| {
                n.interfaces.iter().any(|i| {
                    i.contract_id == derived && i.supported_transports.contains(&"grpc".to_string())
                })
            })
            .count();
        assert_eq!(count, 1);
    }

    #[tokio::test]
    async fn explicit_contract_id_overrides_derived_path() {
        let reg = MetaRuntimeRegistry::default();
        reg_node(&reg, "com.test.tiago", "robonix/prm/camera", "primitive").await;
        reg.declare_interface(
            "com.test.tiago",
            "rgb".into(),
            vec!["ros2".into()],
            "{}".into(),
            0,
            "robonix/prm/camera/rgb".into(),
        )
        .await
        .unwrap();
        let st = reg.inner.read().await;
        let iface = &st.nodes["com.test.tiago"].interfaces[0];
        assert_eq!(iface.name, "rgb");
        assert_eq!(iface.contract_id, "robonix/prm/camera/rgb");
        let hit = st.nodes.values().any(|n| {
            n.interfaces
                .iter()
                .any(|i| i.contract_id == "robonix/prm/camera/rgb")
        });
        assert!(hit);
    }

    #[tokio::test]
    async fn grpc_ros2_warns_for_unknown_contract_but_accepts() {
        // Unknown contract IDs are accepted with a warning, not rejected.
        let reg = MetaRuntimeRegistry::default();
        reg_node(&reg, "com.test.custom", "robonix/prm/custom", "primitive").await;
        let result = reg
            .declare_interface(
                "com.test.custom",
                "data".into(),
                vec!["grpc".into()],
                "{}".into(),
                0,
                "robonix/prm/custom/data".into(),
            )
            .await;
        assert!(
            result.is_ok(),
            "unknown contract should be accepted with warning"
        );
    }
}
