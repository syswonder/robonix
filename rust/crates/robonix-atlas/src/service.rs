// SPDX-License-Identifier: MulanPSL-2.0
// Atlas — Robonix capability registry (gRPC service).
//
// One running unit = one *capability instance*, registered under a
// reverse-DNS `capability_id` and a `namespace` (e.g. "robonix/primitive/base").
// An instance announces a list of `CapabilityInfo` (semantic) and then
// binds wire endpoints via DeclareInterface (one (contract_id, transport)
// pair per call). See rust/proto/atlas.proto for the wire schema.

use anyhow::{Context, Result};
use log::{info, warn};
use serde::Serialize;
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::atomic::{AtomicU16, Ordering};
use tokio::sync::RwLock;
use tonic::{Request, Response, Status};
use uuid::Uuid;

pub mod pb {
    tonic::include_proto!("robonix.atlas");
}

// ── Data model ──────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize)]
struct CapabilityInfoRec {
    name: String,
    contract_id: String,
    description: String,
    capability_md_path: String,
    metadata_json: String,
}

impl From<pb::CapabilityInfo> for CapabilityInfoRec {
    fn from(c: pb::CapabilityInfo) -> Self {
        Self {
            name: c.name,
            contract_id: c.contract_id,
            description: c.description,
            capability_md_path: c.capability_md_path,
            metadata_json: c.metadata_json,
        }
    }
}

impl From<&CapabilityInfoRec> for pb::CapabilityInfo {
    fn from(c: &CapabilityInfoRec) -> Self {
        Self {
            name: c.name.clone(),
            contract_id: c.contract_id.clone(),
            description: c.description.clone(),
            capability_md_path: c.capability_md_path.clone(),
            metadata_json: c.metadata_json.clone(),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
struct EndpointRec {
    contract_id: String,
    transport: String,
    endpoint: String,
    metadata_json: String,
}

impl From<&EndpointRec> for pb::InterfaceEndpoint {
    fn from(e: &EndpointRec) -> Self {
        Self {
            contract_id: e.contract_id.clone(),
            transport: e.transport.clone(),
            endpoint: e.endpoint.clone(),
            metadata_json: e.metadata_json.clone(),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
struct CapRecord {
    capability_id: String,
    namespace: String,
    last_heartbeat_ms: u64,
    capabilities: Vec<CapabilityInfoRec>,
    endpoints: Vec<EndpointRec>,
}

#[derive(Debug, Default, Serialize)]
pub(crate) struct State {
    caps: HashMap<String, CapRecord>,
}

// ── Registry ────────────────────────────────────────────────────────────────

const PORT_RANGE_START: u16 = 50100;

#[derive(Debug)]
pub struct AtlasRegistry {
    pub(crate) inner: RwLock<State>,
    next_port: AtomicU16,
}

impl Default for AtlasRegistry {
    fn default() -> Self {
        Self {
            inner: RwLock::new(State::default()),
            next_port: AtomicU16::new(PORT_RANGE_START),
        }
    }
}

impl AtlasRegistry {
    fn now_ms() -> u64 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64
    }

    fn assign_id() -> String {
        format!("com.robonix.ephemeral.{}", Uuid::new_v4())
    }

    fn next_port(&self) -> u16 {
        self.next_port.fetch_add(1, Ordering::Relaxed)
    }

    fn require(field: &str, value: &str) -> Result<String, Status> {
        let v = value.trim();
        if v.is_empty() {
            return Err(Status::invalid_argument(format!("{field} required")));
        }
        Ok(v.to_string())
    }
}

// ── gRPC service ────────────────────────────────────────────────────────────

#[derive(Debug)]
pub struct AtlasService {
    registry: Arc<AtlasRegistry>,
    /// Host (without port) used to build grpc data-plane endpoints, e.g.
    /// "127.0.0.1" or "atlas.lab.local". Atlas appends `:port` per
    /// DeclareInterface.
    data_plane_host: String,
}

impl AtlasService {
    pub fn new(registry: Arc<AtlasRegistry>, data_plane_host: String) -> Self {
        // Strip a trailing :port if caller passed a host:port string.
        let data_plane_host = data_plane_host
            .rsplit_once(':')
            .map(|(h, _)| h.to_string())
            .unwrap_or(data_plane_host);
        Self { registry, data_plane_host }
    }
}

#[tonic::async_trait]
impl pb::atlas_server::Atlas for AtlasService {
    async fn register_capability(
        &self,
        req: Request<pb::RegisterCapabilityRequest>,
    ) -> Result<Response<pb::RegisterCapabilityResponse>, Status> {
        let r = req.into_inner();
        let cap_id = if r.capability_id.trim().is_empty() {
            AtlasRegistry::assign_id()
        } else {
            r.capability_id.trim().to_string()
        };
        let namespace = AtlasRegistry::require("namespace", &r.namespace)?;
        let capabilities: Vec<CapabilityInfoRec> =
            r.capabilities.into_iter().map(Into::into).collect();

        // Each declared contract_id should sit under the namespace prefix —
        // warn but don't reject (loose policy keeps iteration cheap).
        for c in &capabilities {
            if !c.contract_id.is_empty() && !c.contract_id.starts_with(&namespace) {
                warn!(
                    "[atlas] {} contract_id '{}' is not under namespace '{}'",
                    cap_id, c.contract_id, namespace
                );
            }
        }

        let mut state = self.registry.inner.write().await;
        state.caps.insert(
            cap_id.clone(),
            CapRecord {
                capability_id: cap_id.clone(),
                namespace,
                last_heartbeat_ms: AtlasRegistry::now_ms(),
                capabilities,
                endpoints: Vec::new(),
            },
        );
        info!("[atlas] register {cap_id}");
        Ok(Response::new(pb::RegisterCapabilityResponse {
            capability_id: cap_id,
        }))
    }

    async fn unregister_capability(
        &self,
        req: Request<pb::UnregisterCapabilityRequest>,
    ) -> Result<Response<pb::UnregisterCapabilityResponse>, Status> {
        let r = req.into_inner();
        let cap_id = AtlasRegistry::require("capability_id", &r.capability_id)?;
        let mut state = self.registry.inner.write().await;
        let removed = state.caps.remove(&cap_id).is_some();
        info!("[atlas] unregister {cap_id} (was_present={removed})");
        Ok(Response::new(pb::UnregisterCapabilityResponse { ok: removed }))
    }

    async fn heartbeat(
        &self,
        req: Request<pb::HeartbeatRequest>,
    ) -> Result<Response<pb::HeartbeatResponse>, Status> {
        let r = req.into_inner();
        let cap_id = AtlasRegistry::require("capability_id", &r.capability_id)?;
        let now = AtlasRegistry::now_ms();
        let mut state = self.registry.inner.write().await;
        match state.caps.get_mut(&cap_id) {
            Some(rec) => {
                rec.last_heartbeat_ms = now;
                Ok(Response::new(pb::HeartbeatResponse {
                    ok: true,
                    server_time_ms: now,
                }))
            }
            None => Err(Status::not_found(format!("unknown capability_id: {cap_id}"))),
        }
    }

    async fn declare_interface(
        &self,
        req: Request<pb::DeclareInterfaceRequest>,
    ) -> Result<Response<pb::DeclareInterfaceResponse>, Status> {
        let r = req.into_inner();
        let cap_id = AtlasRegistry::require("capability_id", &r.capability_id)?;
        let contract_id = AtlasRegistry::require("contract_id", &r.contract_id)?;
        let transport = AtlasRegistry::require("transport", &r.transport)?;

        let endpoint = self.resolve_endpoint(&transport, r.listen_port, &contract_id, &r.metadata_json);

        let mut state = self.registry.inner.write().await;
        let rec = state
            .caps
            .get_mut(&cap_id)
            .ok_or_else(|| Status::not_found(format!("unknown capability_id: {cap_id}")))?;

        if !rec.capabilities.iter().any(|c| c.contract_id == contract_id) {
            return Err(Status::failed_precondition(format!(
                "contract_id '{contract_id}' was not announced in RegisterCapability for {cap_id}"
            )));
        }
        if rec
            .endpoints
            .iter()
            .any(|e| e.contract_id == contract_id && e.transport == transport)
        {
            return Err(Status::already_exists(format!(
                "({contract_id}, {transport}) already declared by {cap_id}"
            )));
        }

        rec.endpoints.push(EndpointRec {
            contract_id: contract_id.clone(),
            transport: transport.clone(),
            endpoint: endpoint.clone(),
            metadata_json: r.metadata_json,
        });
        info!("[atlas] declare {cap_id} {contract_id} via {transport} -> {endpoint}");
        Ok(Response::new(pb::DeclareInterfaceResponse { ok: true, endpoint }))
    }

    async fn query_capabilities(
        &self,
        req: Request<pb::QueryCapabilitiesRequest>,
    ) -> Result<Response<pb::QueryCapabilitiesResponse>, Status> {
        let r = req.into_inner();
        let f_contract = r.contract_id.trim();
        let f_namespace = r.namespace.trim();
        let f_transport = r.transport.trim();
        let state = self.registry.inner.read().await;
        let mut records = Vec::new();
        for rec in state.caps.values() {
            if !f_namespace.is_empty() && !rec.namespace.starts_with(f_namespace) {
                continue;
            }
            if !f_contract.is_empty()
                && !rec.capabilities.iter().any(|c| c.contract_id == f_contract)
            {
                continue;
            }
            let endpoints: Vec<pb::InterfaceEndpoint> = rec
                .endpoints
                .iter()
                .filter(|e| {
                    (f_contract.is_empty() || e.contract_id == f_contract)
                        && (f_transport.is_empty() || e.transport == f_transport)
                })
                .map(Into::into)
                .collect();
            records.push(pb::CapabilityRecord {
                capability_id: rec.capability_id.clone(),
                namespace: rec.namespace.clone(),
                last_heartbeat_ms: rec.last_heartbeat_ms,
                capabilities: rec.capabilities.iter().map(Into::into).collect(),
                endpoints,
            });
        }
        Ok(Response::new(pb::QueryCapabilitiesResponse { records }))
    }

    async fn query_capability_md(
        &self,
        req: Request<pb::QueryCapabilityMdRequest>,
    ) -> Result<Response<pb::QueryCapabilityMdResponse>, Status> {
        let r = req.into_inner();
        let cap_id = AtlasRegistry::require("capability_id", &r.capability_id)?;
        let state = self.registry.inner.read().await;
        let rec = state
            .caps
            .get(&cap_id)
            .ok_or_else(|| Status::not_found(format!("unknown capability_id: {cap_id}")))?;
        let mut md = String::new();
        for c in &rec.capabilities {
            if c.capability_md_path.trim().is_empty() {
                continue;
            }
            if let Ok(content) = std::fs::read_to_string(&c.capability_md_path) {
                if !md.is_empty() {
                    md.push_str("\n\n---\n\n");
                }
                md.push_str(&content);
            }
        }
        Ok(Response::new(pb::QueryCapabilityMdResponse {
            capability_md: md,
        }))
    }

    async fn inspect_atlas(
        &self,
        _req: Request<pb::InspectAtlasRequest>,
    ) -> Result<Response<pb::InspectAtlasResponse>, Status> {
        let state = self.registry.inner.read().await;
        let json =
            serde_json::to_string_pretty(&*state).unwrap_or_else(|_| "{}".to_string());
        Ok(Response::new(pb::InspectAtlasResponse { json }))
    }
}

impl AtlasService {
    fn resolve_endpoint(
        &self,
        transport: &str,
        listen_port: u32,
        contract_id: &str,
        metadata_json: &str,
    ) -> String {
        let meta = serde_json::from_str::<serde_json::Value>(metadata_json).ok();
        let meta_str = |key: &str| -> Option<String> {
            meta.as_ref()
                .and_then(|v| v.get(key))
                .and_then(|x| x.as_str())
                .map(|s| s.to_string())
        };

        match transport {
            "grpc" => {
                let port = if listen_port > 0 {
                    listen_port as u16
                } else {
                    self.registry.next_port()
                };
                format!("{}:{}", self.data_plane_host, port)
            }
            "ros2" => meta_str("ros2_topic")
                .or_else(|| meta_str("ros2_service"))
                .unwrap_or_else(|| {
                    format!("/rbnx/{}", contract_id.replace('/', "."))
                }),
            "shared_memory" => meta_str("shm_key")
                .unwrap_or_else(|| format!("/rbnx_shm_{}", Uuid::new_v4().simple())),
            // Custom transport: caller must supply `endpoint` in metadata_json.
            _ => meta_str("endpoint").unwrap_or_default(),
        }
    }
}

// ── Server entrypoint ───────────────────────────────────────────────────────

pub async fn serve_atlas(
    registry: Arc<AtlasRegistry>,
    listen: SocketAddr,
    data_plane_host: String,
) -> Result<()> {
    let svc = AtlasService::new(registry, data_plane_host);
    info!("[atlas] gRPC listening on {listen}");
    tonic::transport::Server::builder()
        .add_service(pb::atlas_server::AtlasServer::new(svc))
        .serve(listen)
        .await
        .context("Atlas server failed")?;
    Ok(())
}
