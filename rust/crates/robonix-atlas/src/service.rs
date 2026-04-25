// SPDX-License-Identifier: MulanPSL-2.0
// Atlas — Robonix capability registry (gRPC service).
//
// One running unit = one *capability instance*, registered under a
// reverse-DNS `capability_id` and a `namespace` (e.g. "robonix/primitive/base").
// Interfaces are declared lazily via DeclareInterface (one (contract_id,
// transport) pair per call) — typically the cap declares its driver
// interface first to let `rbnx start` invoke hardware init, then declares
// the rest after it has discovered what it can support. See
// rust/proto/atlas.proto for the wire schema.

use anyhow::{Context, Result};
use log::{info, warn};
use serde::Serialize;
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::Arc;
use tokio::sync::RwLock;
use tonic::{Request, Response, Status};
use uuid::Uuid;

use crate::pb;
use pb::Transport;

/// How many times Atlas tries to mint a unique endpoint before giving up.
/// At 8-hex-char UUID suffixes the keyspace is 2^32; collisions are
/// dominated by bugs in the collide check, not randomness.
const MINT_ATTEMPTS: usize = 16;

// ── Transport-specific params (typed mirror of proto oneof) ────────────────

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "transport", rename_all = "snake_case")]
enum TransportParamsRec {
    Grpc {
        proto_file: String,
        service_name: String,
        method: String,
    },
    Ros2 {
        qos_profile: String,
    },
    Mcp {
        description: String,
        input_schema_json: String,
    },
}

impl TransportParamsRec {
    fn transport(&self) -> Transport {
        match self {
            Self::Grpc { .. } => Transport::Grpc,
            Self::Ros2 { .. } => Transport::Ros2,
            Self::Mcp { .. } => Transport::Mcp,
        }
    }
}

impl From<&TransportParamsRec> for pb::TransportParams {
    fn from(r: &TransportParamsRec) -> Self {
        use pb::transport_params::Kind;
        let kind = match r {
            TransportParamsRec::Grpc {
                proto_file,
                service_name,
                method,
            } => Kind::Grpc(pb::GrpcParams {
                proto_file: proto_file.clone(),
                service_name: service_name.clone(),
                method: method.clone(),
            }),
            TransportParamsRec::Ros2 { qos_profile } => Kind::Ros2(pb::Ros2Params {
                qos_profile: qos_profile.clone(),
            }),
            TransportParamsRec::Mcp {
                description,
                input_schema_json,
            } => Kind::Mcp(pb::McpParams {
                description: description.clone(),
                input_schema_json: input_schema_json.clone(),
            }),
        };
        Self { kind: Some(kind) }
    }
}

#[derive(Debug, Clone, Serialize)]
struct EndpointRec {
    contract_id: String,
    #[serde(serialize_with = "serialize_transport")]
    transport: Transport,
    endpoint: String,
    params: TransportParamsRec,
}

fn serialize_transport<S: serde::Serializer>(
    t: &Transport,
    ser: S,
) -> Result<S::Ok, S::Error> {
    ser.serialize_str(t.as_str_name())
}

impl From<&EndpointRec> for pb::InterfaceEndpoint {
    fn from(e: &EndpointRec) -> Self {
        Self {
            contract_id: e.contract_id.clone(),
            transport: e.transport as i32,
            endpoint: e.endpoint.clone(),
            params: Some((&e.params).into()),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
struct CapRecord {
    capability_id: String,
    namespace: String,
    capability_md_path: String,
    last_heartbeat_ms: u64,
    endpoints: Vec<EndpointRec>,
}

#[derive(Debug, Default, Serialize)]
pub(crate) struct State {
    caps: HashMap<String, CapRecord>,
}

// ── Registry ────────────────────────────────────────────────────────────────

/// In-memory state shared by all gRPC handlers.
///
/// Hold the `Arc<AtlasRegistry>` once at process startup; clone the `Arc`
/// (not the struct) when sharing across tasks. The interior `RwLock`
/// serialises access to the cap table.
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

    fn require<'a>(field: &str, value: &'a str) -> Result<&'a str, Status> {
        let v = value.trim();
        if v.is_empty() {
            return Err(Status::invalid_argument(format!("{field} required")));
        }
        Ok(v)
    }
}

/// Whether Atlas can mint a fresh endpoint name for this transport without
/// requiring a prior OS-level bind by the caller. ros2 is a pure-name
/// address space; grpc and mcp need a host:port that
/// only the caller can produce.
fn atlas_can_mint(transport: Transport) -> bool {
    matches!(transport, Transport::Ros2)
}

/// Convert wire-form `pb::TransportParams` into the typed Rust enum and
/// verify that its `oneof` variant matches the requested `transport`.
fn parse_params(
    transport: Transport,
    params: Option<pb::TransportParams>,
) -> Result<TransportParamsRec, Status> {
    use pb::transport_params::Kind;
    let kind = params.and_then(|p| p.kind).ok_or_else(|| {
        Status::invalid_argument(
            "params required: set TransportParams.kind to the variant matching `transport`",
        )
    })?;
    let rec = match kind {
        Kind::Grpc(g) => TransportParamsRec::Grpc {
            proto_file: g.proto_file,
            service_name: g.service_name,
            method: g.method,
        },
        Kind::Ros2(r) => TransportParamsRec::Ros2 {
            qos_profile: r.qos_profile,
        },
        Kind::Mcp(m) => {
            // Validate input_schema_json parses as a JSON object.
            if !m.input_schema_json.is_empty() {
                let v: serde_json::Value = serde_json::from_str(&m.input_schema_json)
                    .map_err(|e| {
                        Status::invalid_argument(format!(
                            "mcp input_schema_json invalid: {e}"
                        ))
                    })?;
                if !v.is_object() {
                    return Err(Status::invalid_argument(
                        "mcp input_schema_json must be a JSON object",
                    ));
                }
            }
            TransportParamsRec::Mcp {
                description: m.description,
                input_schema_json: m.input_schema_json,
            }
        }
    };
    if rec.transport() != transport {
        return Err(Status::invalid_argument(format!(
            "params: oneof variant {:?} does not match transport {:?}",
            rec.transport(),
            transport
        )));
    }
    Ok(rec)
}

fn parse_transport(t: i32) -> Result<Transport, Status> {
    let v = Transport::try_from(t).map_err(|_| {
        Status::invalid_argument(format!("transport: unknown enum value {t}"))
    })?;
    if v == Transport::Unspecified {
        return Err(Status::invalid_argument("transport: must not be UNSPECIFIED"));
    }
    Ok(v)
}

// ── gRPC service ────────────────────────────────────────────────────────────

/// gRPC service implementing the Atlas control plane (see
/// `rust/proto/atlas.proto`). Stateless wrapper around an `AtlasRegistry`.
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
        let namespace = AtlasRegistry::require("namespace", &r.namespace)?.to_string();

        let mut state = self.registry.inner.write().await;
        if state.caps.contains_key(&cap_id) {
            return Err(Status::already_exists(format!(
                "capability_id '{cap_id}' already registered; Unregister first"
            )));
        }
        state.caps.insert(
            cap_id.clone(),
            CapRecord {
                capability_id: cap_id.clone(),
                namespace,
                capability_md_path: r.capability_md_path.trim().to_string(),
                last_heartbeat_ms: AtlasRegistry::now_ms(),
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
        let cap_id = AtlasRegistry::require("capability_id", &r.capability_id)?.to_string();
        let mut state = self.registry.inner.write().await;
        let was_present = state.caps.remove(&cap_id).is_some();
        info!("[atlas] unregister {cap_id} (was_present={was_present})");
        Ok(Response::new(pb::UnregisterCapabilityResponse {
            was_present,
        }))
    }

    async fn heartbeat(
        &self,
        req: Request<pb::HeartbeatRequest>,
    ) -> Result<Response<pb::HeartbeatResponse>, Status> {
        let r = req.into_inner();
        let cap_id = AtlasRegistry::require("capability_id", &r.capability_id)?.to_string();
        let now = AtlasRegistry::now_ms();
        let mut state = self.registry.inner.write().await;
        let rec = state
            .caps
            .get_mut(&cap_id)
            .ok_or_else(|| Status::not_found(format!("unknown capability_id: {cap_id}")))?;
        rec.last_heartbeat_ms = now;
        Ok(Response::new(pb::HeartbeatResponse {}))
    }

    async fn declare_interface(
        &self,
        req: Request<pb::DeclareInterfaceRequest>,
    ) -> Result<Response<pb::DeclareInterfaceResponse>, Status> {
        let r = req.into_inner();
        let cap_id = AtlasRegistry::require("capability_id", &r.capability_id)?.to_string();
        let contract_id = AtlasRegistry::require("contract_id", &r.contract_id)?.to_string();
        let transport = parse_transport(r.transport)?;
        let params = parse_params(transport, r.params)?;
        let proposed = r.endpoint.trim().to_string();

        let mut state = self.registry.inner.write().await;

        // Cap must exist; contract_id must fall under the cap's namespace.
        let rec = state
            .caps
            .get(&cap_id)
            .ok_or_else(|| Status::not_found(format!("unknown capability_id: {cap_id}")))?;
        if !contract_id.starts_with(&rec.namespace) {
            return Err(Status::invalid_argument(format!(
                "contract_id '{contract_id}' is not under namespace '{}' of capability '{cap_id}'",
                rec.namespace
            )));
        }
        if rec
            .endpoints
            .iter()
            .any(|e| e.contract_id == contract_id && e.transport == transport)
        {
            return Err(Status::already_exists(format!(
                "({contract_id}, {transport:?}) already declared by {cap_id}"
            )));
        }

        let endpoint = resolve_endpoint(&state, transport, &proposed, &contract_id)?;

        // Lock guard hasn't been dropped since the read above; ok_or_else
        // here is defensive against future refactors that split the lock.
        let rec = state
            .caps
            .get_mut(&cap_id)
            .ok_or_else(|| Status::internal("capability vanished mid-declare"))?;
        rec.endpoints.push(EndpointRec {
            contract_id: contract_id.clone(),
            transport,
            endpoint: endpoint.clone(),
            params,
        });
        info!(
            "[atlas] declare {cap_id} {contract_id} via {transport:?} -> {endpoint}"
        );
        Ok(Response::new(pb::DeclareInterfaceResponse { endpoint }))
    }

    async fn query_capabilities(
        &self,
        req: Request<pb::QueryCapabilitiesRequest>,
    ) -> Result<Response<pb::QueryCapabilitiesResponse>, Status> {
        let r = req.into_inner();
        let f_cap_id = r.capability_id.trim();
        let f_contract = r.contract_id.trim();
        let f_transport = Transport::try_from(r.transport)
            .ok()
            .filter(|&t| t != Transport::Unspecified);

        let state = self.registry.inner.read().await;
        let mut records = Vec::new();
        for rec in state.caps.values() {
            if !f_cap_id.is_empty() && rec.capability_id != f_cap_id {
                continue;
            }
            // contract_id filter applies to the endpoints list — the set
            // of contracts a cap currently offers is `{e.contract_id ∈ endpoints}`.
            if !f_contract.is_empty()
                && !rec.endpoints.iter().any(|e| e.contract_id == f_contract)
            {
                continue;
            }
            let endpoints: Vec<pb::InterfaceEndpoint> = rec
                .endpoints
                .iter()
                .filter(|e| {
                    (f_contract.is_empty() || e.contract_id == f_contract)
                        && f_transport.is_none_or(|t| e.transport == t)
                })
                .map(Into::into)
                .collect();
            records.push(pb::CapabilityRecord {
                capability_id: rec.capability_id.clone(),
                namespace: rec.namespace.clone(),
                capability_md_path: rec.capability_md_path.clone(),
                last_heartbeat_ms: rec.last_heartbeat_ms,
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
        let cap_id = AtlasRegistry::require("capability_id", &r.capability_id)?.to_string();

        // Snapshot path under the read lock, then drop it before doing
        // (potentially blocking) filesystem I/O.
        let path = {
            let state = self.registry.inner.read().await;
            let rec = state
                .caps
                .get(&cap_id)
                .ok_or_else(|| Status::not_found(format!("unknown capability_id: {cap_id}")))?;
            rec.capability_md_path.clone()
        };

        if path.is_empty() {
            return Ok(Response::new(pb::QueryCapabilityMdResponse {
                capability_md: String::new(),
            }));
        }
        match tokio::fs::read_to_string(&path).await {
            Ok(capability_md) => Ok(Response::new(pb::QueryCapabilityMdResponse {
                capability_md,
            })),
            Err(e) => {
                warn!("[atlas] {cap_id}: read CAPABILITY.md '{path}' failed: {e}");
                Err(Status::internal(format!(
                    "failed to read CAPABILITY.md for {cap_id}: {e}"
                )))
            }
        }
    }

    async fn inspect_atlas(
        &self,
        _req: Request<pb::InspectAtlasRequest>,
    ) -> Result<Response<pb::InspectAtlasResponse>, Status> {
        let state = self.registry.inner.read().await;
        let json = serde_json::to_string_pretty(&*state)
            .map_err(|e| Status::internal(format!("inspect serialise failed: {e}")))?;
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
    transport: Transport,
    proposed: &str,
    contract_id: &str,
) -> Result<String, Status> {
    let mintable = atlas_can_mint(transport);

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
        if !mintable {
            return Err(Status::invalid_argument(format!(
                "transport '{transport:?}' requires caller-supplied endpoint; \
                 Atlas cannot allocate (e.g. caller must bind a port and pass host:port)"
            )));
        }
        for _ in 0..MINT_ATTEMPTS {
            let candidate = format!("/rbnx/{dotted}/{}", short_uuid());
            if !collides(&candidate) {
                return Ok(candidate);
            }
        }
        return Err(Status::internal(format!(
            "could not mint unique endpoint for ({transport:?}, {contract_id}) \
             after {MINT_ATTEMPTS} attempts (existing caps: {})",
            state.caps.len()
        )));
    }

    if !collides(proposed) {
        return Ok(proposed.to_string());
    }

    if !mintable {
        return Err(Status::already_exists(format!(
            "endpoint '{proposed}' already registered on transport '{transport:?}'; \
             pick a new address (rebind a different port / use a different name) and retry"
        )));
    }

    for _ in 0..MINT_ATTEMPTS {
        let candidate = format!("{proposed}~{}", short_uuid());
        if !collides(&candidate) {
            return Ok(candidate);
        }
    }
    Err(Status::internal(format!(
        "could not disambiguate '{proposed}' on transport '{transport:?}' \
         after {MINT_ATTEMPTS} attempts (existing caps: {})",
        state.caps.len()
    )))
}

// ── Heartbeat eviction ──────────────────────────────────────────────────────
//
// `RegisterCapability` keeps a cap in the registry until either Unregister
// is called OR its heartbeat lapses past the eviction threshold. The proto
// promises this; without an eviction task the registry would only shrink
// on explicit Unregister, leaking entries from crashed/disconnected caps.
//
// Tunables (env, read once at serve_atlas startup):
//   ROBONIX_ATLAS_HEARTBEAT_TIMEOUT_MS  default 60000 (0 disables eviction)
//   ROBONIX_ATLAS_EVICTION_INTERVAL_MS  default 10000 (sweep cadence)
//
// Eviction is non-monotonic: a cap that comes back and re-registers under
// the same id reuses the slot. Stale entries between crash and re-register
// remain visible to consumers — they should still tolerate `connect_to_*`
// failures and retry.

const DEFAULT_HEARTBEAT_TIMEOUT_MS: u64 = 60_000;
const DEFAULT_EVICTION_INTERVAL_MS: u64 = 10_000;

fn read_env_u64(name: &str, default: u64) -> u64 {
    std::env::var(name)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(default)
}

/// Run the eviction loop until cancelled. `timeout_ms == 0` skips the loop
/// entirely — useful for tests and dev where caps may legitimately go silent.
async fn eviction_loop(registry: Arc<AtlasRegistry>, timeout_ms: u64, interval_ms: u64) {
    if timeout_ms == 0 {
        info!("[atlas] heartbeat eviction disabled (timeout=0)");
        return;
    }
    info!(
        "[atlas] heartbeat eviction: timeout={}ms interval={}ms",
        timeout_ms, interval_ms
    );
    let interval = std::time::Duration::from_millis(interval_ms);
    loop {
        tokio::time::sleep(interval).await;
        let now = AtlasRegistry::now_ms();
        let mut state = registry.inner.write().await;
        let evicted: Vec<String> = state
            .caps
            .iter()
            .filter(|(_, rec)| now.saturating_sub(rec.last_heartbeat_ms) > timeout_ms)
            .map(|(id, _)| id.clone())
            .collect();
        for id in &evicted {
            state.caps.remove(id);
            warn!(
                "[atlas] evicted '{id}' (heartbeat lapsed > {}ms)",
                timeout_ms
            );
        }
    }
}

// ── Server entrypoint ───────────────────────────────────────────────────────

/// Start the Atlas gRPC server on `listen` using the given registry.
/// Spawns a background heartbeat-eviction task; both tasks block until
/// the gRPC server stops or returns an error.
pub async fn serve_atlas(registry: Arc<AtlasRegistry>, listen: SocketAddr) -> Result<()> {
    let timeout_ms = read_env_u64("ROBONIX_ATLAS_HEARTBEAT_TIMEOUT_MS", DEFAULT_HEARTBEAT_TIMEOUT_MS);
    let interval_ms = read_env_u64("ROBONIX_ATLAS_EVICTION_INTERVAL_MS", DEFAULT_EVICTION_INTERVAL_MS);
    let _eviction_task = tokio::spawn(eviction_loop(Arc::clone(&registry), timeout_ms, interval_ms));

    let svc = AtlasService::new(registry);
    info!("[atlas] gRPC listening on {listen}");
    tonic::transport::Server::builder()
        .add_service(pb::atlas_server::AtlasServer::new(svc))
        .serve(listen)
        .await
        .context("Atlas server failed")?;
    Ok(())
}
