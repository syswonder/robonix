// SPDX-License-Identifier: MulanPSL-2.0
// Atlas — Robonix capability registry (gRPC service).
//
// One running unit = one *capability instance*, registered under a
// reverse-DNS `capability_id` and a `namespace` (e.g. "robonix/primitive/base").
// An instance announces a list of `CapabilityInterface` (semantic) and then
// binds wire endpoints via DeclareInterface (one (contract_id, transport)
// pair per call). See rust/proto/atlas.proto for the wire schema.

use anyhow::{Context, Result};
use log::{info, warn};
use serde::Serialize;
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::Arc;
use tokio::sync::RwLock;
use tonic::{Request, Response, Status};
use uuid::Uuid;

pub mod pb {
    tonic::include_proto!("robonix.atlas");
}

// ── Data model ──────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize)]
struct CapabilityInterfaceRec {
    name: String,
    contract_id: String,
    capability_md_path: String,
    metadata_json: String,
}

impl From<pb::CapabilityInterface> for CapabilityInterfaceRec {
    fn from(c: pb::CapabilityInterface) -> Self {
        Self {
            name: c.name,
            contract_id: c.contract_id,
            capability_md_path: c.capability_md_path,
            metadata_json: c.metadata_json,
        }
    }
}

impl From<&CapabilityInterfaceRec> for pb::CapabilityInterface {
    fn from(c: &CapabilityInterfaceRec) -> Self {
        Self {
            name: c.name.clone(),
            contract_id: c.contract_id.clone(),
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
    interfaces: Vec<CapabilityInterfaceRec>,
    endpoints: Vec<EndpointRec>,
}

#[derive(Debug, Default, Serialize)]
pub(crate) struct State {
    caps: HashMap<String, CapRecord>,
}

// ── Registry ────────────────────────────────────────────────────────────────

#[derive(Debug, Default)]
pub struct AtlasRegistry {
    pub(crate) inner: RwLock<State>,
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
}

impl AtlasService {
    pub fn new(registry: Arc<AtlasRegistry>) -> Self {
        Self { registry }
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
        let interfaces: Vec<CapabilityInterfaceRec> =
            r.interfaces.into_iter().map(Into::into).collect();

        // Each declared contract_id should sit under the namespace prefix —
        // warn but don't reject (loose policy keeps iteration cheap).
        for c in &interfaces {
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
                interfaces,
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
        let proposed = r.endpoint.trim().to_string();

        let mut state = self.registry.inner.write().await;

        // Cap must exist and have announced this contract.
        let rec = state
            .caps
            .get(&cap_id)
            .ok_or_else(|| Status::not_found(format!("unknown capability_id: {cap_id}")))?;
        if !rec.interfaces.iter().any(|c| c.contract_id == contract_id) {
            return Err(Status::failed_precondition(format!(
                "contract_id '{contract_id}' was not announced in RegisterCapability for {cap_id}"
            )));
        }
        // Same cap can't declare the same (contract, transport) twice.
        if rec
            .endpoints
            .iter()
            .any(|e| e.contract_id == contract_id && e.transport == transport)
        {
            return Err(Status::already_exists(format!(
                "({contract_id}, {transport}) already declared by {cap_id}"
            )));
        }

        // Resolve a globally-unique endpoint per the rules in atlas.proto.
        let endpoint = resolve_endpoint(&state, &transport, &proposed, &contract_id)?;

        let rec = state.caps.get_mut(&cap_id).expect("checked above");
        rec.endpoints.push(EndpointRec {
            contract_id: contract_id.clone(),
            transport: transport.clone(),
            endpoint: endpoint.clone(),
            metadata_json: r.metadata_json,
        });
        info!("[atlas] declare {cap_id} {contract_id} via {transport} -> {endpoint}");
        Ok(Response::new(pb::DeclareInterfaceResponse {
            ok: true,
            endpoint,
        }))
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
                && !rec.interfaces.iter().any(|c| c.contract_id == f_contract)
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
                interfaces: rec.interfaces.iter().map(Into::into).collect(),
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
        for c in &rec.interfaces {
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

/// Pick a globally-unique endpoint for (transport, proposed) per the rules
/// documented on `DeclareInterfaceRequest` in atlas.proto.
///
///   - empty proposal + mintable transport       → mint "/rbnx/<dotted>/<uuid8>"
///   - empty proposal + non-mintable transport   → InvalidArgument
///   - non-empty + no collision                  → use as-is
///   - non-empty + collision + mintable          → "<proposed>~<uuid8>"
///   - non-empty + collision + non-mintable      → AlreadyExists
fn resolve_endpoint(
    state: &State,
    transport: &str,
    proposed: &str,
    contract_id: &str,
) -> Result<String, Status> {
    // Atlas can mint names freely for transports whose address space does NOT
    // require an OS resource bind at registration time.
    let atlas_can_mint = matches!(transport, "ros2" | "shared_memory");

    let collides = |s: &str| -> bool {
        state.caps.values().any(|rec| {
            rec.endpoints
                .iter()
                .any(|e| e.transport == transport && e.endpoint == s)
        })
    };
    let short_uuid = || -> String {
        Uuid::new_v4()
            .simple()
            .to_string()
            .chars()
            .take(8)
            .collect()
    };
    let dotted = contract_id.replace('/', ".");

    if proposed.is_empty() {
        if !atlas_can_mint {
            return Err(Status::invalid_argument(format!(
                "transport '{transport}' requires caller-supplied endpoint; \
                 Atlas cannot allocate (e.g. caller must bind a port and pass host:port)"
            )));
        }
        for _ in 0..16 {
            let candidate = format!("/rbnx/{dotted}/{}", short_uuid());
            if !collides(&candidate) {
                return Ok(candidate);
            }
        }
        return Err(Status::internal(
            "could not mint unique endpoint after 16 attempts",
        ));
    }

    if !collides(proposed) {
        return Ok(proposed.to_string());
    }

    if !atlas_can_mint {
        return Err(Status::already_exists(format!(
            "endpoint '{proposed}' already registered on transport '{transport}'; \
             pick a new address (rebind a different port / use a different name) and retry"
        )));
    }

    for _ in 0..16 {
        let candidate = format!("{proposed}~{}", short_uuid());
        if !collides(&candidate) {
            return Ok(candidate);
        }
    }
    Err(Status::internal(
        "could not mint unique endpoint after 16 attempts",
    ))
}

// ── Server entrypoint ───────────────────────────────────────────────────────

pub async fn serve_atlas(
    registry: Arc<AtlasRegistry>,
    listen: SocketAddr,
) -> Result<()> {
    let svc = AtlasService::new(registry);
    info!("[atlas] gRPC listening on {listen}");
    tonic::transport::Server::builder()
        .add_service(pb::atlas_server::AtlasServer::new(svc))
        .serve(listen)
        .await
        .context("Atlas server failed")?;
    Ok(())
}
