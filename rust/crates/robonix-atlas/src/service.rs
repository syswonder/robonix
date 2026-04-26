// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Atlas — Robonix capability registry (gRPC service).
//
// `AtlasRegistry` owns the in-memory state and exposes typed async methods
// for each operation. `AtlasService` is a thin facade that parses wire
// types out of `pb::*` requests and calls the registry. The legacy
// `RobonixRuntime` shim (see crate::legacy) calls the same registry
// methods after translating old fields, which is how backward compat is
// implemented without a parallel state.

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
pub use pb::Transport;

/// How many times Atlas tries to mint a unique endpoint before giving up.
const MINT_ATTEMPTS: usize = 16;

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

fn serialize_transport<S: serde::Serializer>(t: &Transport, ser: S) -> Result<S::Ok, S::Error> {
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

impl From<&CapRecord> for pb::CapabilityRecord {
    fn from(rec: &CapRecord) -> Self {
        Self {
            capability_id: rec.capability_id.clone(),
            namespace: rec.namespace.clone(),
            capability_md_path: rec.capability_md_path.clone(),
            last_heartbeat_ms: rec.last_heartbeat_ms,
            endpoints: rec.endpoints.iter().map(Into::into).collect(),
            state: rec.state() as i32,
        }
    }
}

impl CapRecord {
    /// Observed lifecycle state. INITIALIZED iff the cap has declared at
    /// least one interface whose contract_id is NOT a `*/driver` (the
    /// lifecycle hook). By convention, primitives + scene services declare
    /// only `<kind>/driver` until rbnx calls `Driver(CMD_INIT)` succeeds,
    /// after which they lazy-declare their real interfaces. System caps
    /// (atlas/pilot/executor/memory/...) skip the driver step and declare
    /// their real interfaces directly, transitioning to INITIALIZED on
    /// first declare.
    fn state(&self) -> pb::CapabilityState {
        let any_non_driver = self
            .endpoints
            .iter()
            .any(|e| !is_driver_contract(&e.contract_id));
        if any_non_driver {
            pb::CapabilityState::StateInitialized
        } else {
            pb::CapabilityState::StateRegistered
        }
    }
}

fn is_driver_contract(contract_id: &str) -> bool {
    // `robonix/primitive/driver` and `robonix/service/driver` (the canonical
    // lifecycle contracts), plus historical `<area>/driver` ids that
    // pre-9c22145 packages may still declare.
    contract_id == "robonix/primitive/driver"
        || contract_id == "robonix/service/driver"
        || contract_id.ends_with("/driver")
}

#[derive(Debug, Default, Serialize)]
pub(crate) struct State {
    caps: HashMap<String, CapRecord>,
}

// ── Registry ────────────────────────────────────────────────────────────────

/// In-memory state shared by all gRPC handlers (new + legacy). All ops go
/// through one of the typed async methods below. `Arc<AtlasRegistry>` is
/// cheap to clone; the interior `RwLock` serialises mutations.
#[derive(Debug, Default)]
pub struct AtlasRegistry {
    pub(crate) inner: RwLock<State>,
}

impl AtlasRegistry {
    pub fn now_ms() -> u64 {
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

    /// Register a new capability instance. Empty `cap_id` triggers Atlas-
    /// assigned ephemeral id. Returns the resolved id.
    pub async fn register(
        &self,
        cap_id: &str,
        namespace: &str,
        capability_md_path: &str,
    ) -> Result<String, Status> {
        let cap_id = if cap_id.trim().is_empty() {
            Self::assign_id()
        } else {
            cap_id.trim().to_string()
        };
        let namespace = Self::require("namespace", namespace)?.to_string();
        let mut state = self.inner.write().await;
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
                capability_md_path: capability_md_path.trim().to_string(),
                last_heartbeat_ms: Self::now_ms(),
                endpoints: Vec::new(),
            },
        );
        info!("[atlas] register {cap_id}");
        Ok(cap_id)
    }

    /// Idempotent: returns `true` if a record was removed, `false` if the id
    /// was unknown.
    pub async fn unregister(&self, cap_id: &str) -> bool {
        let cap_id = cap_id.trim();
        if cap_id.is_empty() {
            return false;
        }
        let mut state = self.inner.write().await;
        let was_present = state.caps.remove(cap_id).is_some();
        info!("[atlas] unregister {cap_id} (was_present={was_present})");
        was_present
    }

    /// Updates `last_heartbeat_ms` to now. Returns the timestamp it set.
    pub async fn heartbeat(&self, cap_id: &str) -> Result<u64, Status> {
        let cap_id = Self::require("capability_id", cap_id)?;
        let now = Self::now_ms();
        let mut state = self.inner.write().await;
        let rec = state
            .caps
            .get_mut(cap_id)
            .ok_or_else(|| Status::not_found(format!("unknown capability_id: {cap_id}")))?;
        rec.last_heartbeat_ms = now;
        Ok(now)
    }

    /// Declare ONE transport for ONE contract on a registered cap. Returns
    /// the authoritative endpoint string (may differ from `proposed` when
    /// Atlas rewrote it to disambiguate on a mintable transport).
    pub async fn declare(
        &self,
        cap_id: &str,
        contract_id: &str,
        transport: Transport,
        proposed: &str,
        params: pb::TransportParams,
    ) -> Result<String, Status> {
        let cap_id = Self::require("capability_id", cap_id)?;
        let contract_id = Self::require("contract_id", contract_id)?.to_string();
        let params = parse_params(transport, Some(params))?;
        let proposed = proposed.trim().to_string();

        let mut state = self.inner.write().await;
        let rec = state
            .caps
            .get(cap_id)
            .ok_or_else(|| Status::not_found(format!("unknown capability_id: {cap_id}")))?;
        // Drivers live above per-area namespaces by design — every primitive
        // (regardless of area) shares `robonix/primitive/driver`, every scene
        // service shares `robonix/service/driver`. The namespace check covers
        // only the cap's real interfaces.
        if !is_driver_contract(&contract_id) && !contract_id.starts_with(&rec.namespace) {
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

        let endpoint = resolve_endpoint(&state, transport, &proposed, &contract_id, cap_id)?;
        let rec = state
            .caps
            .get_mut(cap_id)
            .ok_or_else(|| Status::internal("capability vanished mid-declare"))?;
        rec.endpoints.push(EndpointRec {
            contract_id: contract_id.clone(),
            transport,
            endpoint: endpoint.clone(),
            params,
        });
        info!("[atlas] declare {cap_id} {contract_id} via {transport:?} -> {endpoint}");
        Ok(endpoint)
    }

    /// Snapshot of registered caps matching the given filters. Empty
    /// `cap_id` / empty `contract` / `Transport::Unspecified` mean "no
    /// filter on that field". Records carry only endpoints that satisfy
    /// `contract` + `transport` filters.
    pub async fn query(
        &self,
        cap_id: &str,
        contract: &str,
        transport: Transport,
    ) -> Vec<pb::CapabilityRecord> {
        let f_cap_id = cap_id.trim();
        let f_contract = contract.trim();
        let f_transport = if transport == Transport::Unspecified {
            None
        } else {
            Some(transport)
        };

        let state = self.inner.read().await;
        let mut out = Vec::new();
        for rec in state.caps.values() {
            if !f_cap_id.is_empty() && rec.capability_id != f_cap_id {
                continue;
            }
            if !f_contract.is_empty() && !rec.endpoints.iter().any(|e| e.contract_id == f_contract)
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
            out.push(pb::CapabilityRecord {
                capability_id: rec.capability_id.clone(),
                namespace: rec.namespace.clone(),
                capability_md_path: rec.capability_md_path.clone(),
                last_heartbeat_ms: rec.last_heartbeat_ms,
                state: rec.state() as i32,
                endpoints,
            });
        }
        out
    }

    /// Read the cap's CAPABILITY.md content. Returns "" when the cap
    /// registered without a path.
    pub async fn capability_md(&self, cap_id: &str) -> Result<String, Status> {
        let cap_id = Self::require("capability_id", cap_id)?;
        let path = {
            let state = self.inner.read().await;
            let rec = state
                .caps
                .get(cap_id)
                .ok_or_else(|| Status::not_found(format!("unknown capability_id: {cap_id}")))?;
            rec.capability_md_path.clone()
        };
        if path.is_empty() {
            return Ok(String::new());
        }
        match tokio::fs::read_to_string(&path).await {
            Ok(s) => Ok(s),
            Err(e) => {
                warn!("[atlas] {cap_id}: read CAPABILITY.md '{path}' failed: {e}");
                Err(Status::internal(format!(
                    "failed to read CAPABILITY.md for {cap_id}: {e}"
                )))
            }
        }
    }

    /// Debug-only JSON dump of the entire registry. Schema is unstable.
    pub async fn inspect_json(&self) -> Result<String, Status> {
        let state = self.inner.read().await;
        serde_json::to_string_pretty(&*state)
            .map_err(|e| Status::internal(format!("inspect serialise failed: {e}")))
    }

    // ── Legacy-shim helpers (keep parallel state out of legacy.rs) ─────────

    /// Snapshot all `(cap_id, namespace, capability_md_path, endpoints-as-pb)`
    /// for the legacy `RobonixRuntime` shim's QueryNodes / QueryAllSkills /
    /// NegotiateChannel translations. Uses the shared registry state; no copy
    /// of state lives in legacy.rs.
    pub async fn snapshot_for_legacy(&self) -> Vec<pb::CapabilityRecord> {
        self.query("", "", Transport::Unspecified).await
    }

    /// Same as `unregister` but also returns whether the slot existed —
    /// here for symmetry with the legacy shim, which uses the boolean to
    /// decide whether to clean up its skill_md temp file.
    pub async fn unregister_with_path(&self, cap_id: &str) -> (bool, Option<String>) {
        let cap_id = cap_id.trim();
        if cap_id.is_empty() {
            return (false, None);
        }
        let mut state = self.inner.write().await;
        let prior = state.caps.remove(cap_id);
        let path = prior.as_ref().map(|r| r.capability_md_path.clone());
        info!(
            "[atlas] unregister {cap_id} (was_present={})",
            prior.is_some()
        );
        (prior.is_some(), path)
    }
}

/// Whether Atlas can mint a fresh endpoint name for this transport without
/// requiring a prior OS-level bind by the caller. ros2 is a pure-name
/// address space; grpc and mcp need a host:port that only the caller can
/// produce.
fn atlas_can_mint(transport: Transport) -> bool {
    matches!(transport, Transport::Ros2)
}

/// Convert wire `pb::TransportParams` into the typed Rust enum and verify
/// the `oneof` variant matches `transport`.
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
            if !m.input_schema_json.is_empty() {
                let v: serde_json::Value =
                    serde_json::from_str(&m.input_schema_json).map_err(|e| {
                        Status::invalid_argument(format!("mcp input_schema_json invalid: {e}"))
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
    let v = Transport::try_from(t)
        .map_err(|_| Status::invalid_argument(format!("transport: unknown enum value {t}")))?;
    if v == Transport::Unspecified {
        return Err(Status::invalid_argument(
            "transport: must not be UNSPECIFIED",
        ));
    }
    Ok(v)
}

/// Pick a globally-unique endpoint per the rules on `DeclareInterfaceRequest`.
///
/// "Globally unique" here means: no *other* cap may already own this
/// `(transport, endpoint)` pair. The cap itself is allowed to expose multiple
/// contracts on the same endpoint — that's the dominant pattern for MCP
/// (one MCP server URL hosts many tools) and a legitimate one for gRPC
/// (one tonic Server with multiple services). The per-cap
/// `(contract_id, transport)` uniqueness check happens earlier in `declare`.
fn resolve_endpoint(
    state: &State,
    transport: Transport,
    proposed: &str,
    contract_id: &str,
    own_cap_id: &str,
) -> Result<String, Status> {
    let mintable = atlas_can_mint(transport);
    let collides = |s: &str| -> bool {
        state.caps.iter().any(|(other_id, rec)| {
            other_id != own_cap_id
                && rec
                    .endpoints
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

// ── gRPC service (thin facade over AtlasRegistry) ──────────────────────────

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
        let capability_id = self
            .registry
            .register(&r.capability_id, &r.namespace, &r.capability_md_path)
            .await?;
        Ok(Response::new(pb::RegisterCapabilityResponse {
            capability_id,
        }))
    }

    async fn unregister_capability(
        &self,
        req: Request<pb::UnregisterCapabilityRequest>,
    ) -> Result<Response<pb::UnregisterCapabilityResponse>, Status> {
        let r = req.into_inner();
        let was_present = self.registry.unregister(&r.capability_id).await;
        Ok(Response::new(pb::UnregisterCapabilityResponse {
            was_present,
        }))
    }

    async fn heartbeat(
        &self,
        req: Request<pb::HeartbeatRequest>,
    ) -> Result<Response<pb::HeartbeatResponse>, Status> {
        let r = req.into_inner();
        self.registry.heartbeat(&r.capability_id).await?;
        Ok(Response::new(pb::HeartbeatResponse {}))
    }

    async fn declare_interface(
        &self,
        req: Request<pb::DeclareInterfaceRequest>,
    ) -> Result<Response<pb::DeclareInterfaceResponse>, Status> {
        let r = req.into_inner();
        let transport = parse_transport(r.transport)?;
        let endpoint = self
            .registry
            .declare(
                &r.capability_id,
                &r.contract_id,
                transport,
                &r.endpoint,
                r.params.unwrap_or_default(),
            )
            .await?;
        Ok(Response::new(pb::DeclareInterfaceResponse { endpoint }))
    }

    async fn query_capabilities(
        &self,
        req: Request<pb::QueryCapabilitiesRequest>,
    ) -> Result<Response<pb::QueryCapabilitiesResponse>, Status> {
        let r = req.into_inner();
        let transport = Transport::try_from(r.transport).unwrap_or(Transport::Unspecified);
        let records = self
            .registry
            .query(&r.capability_id, &r.contract_id, transport)
            .await;
        Ok(Response::new(pb::QueryCapabilitiesResponse { records }))
    }

    async fn query_capability_md(
        &self,
        req: Request<pb::QueryCapabilityMdRequest>,
    ) -> Result<Response<pb::QueryCapabilityMdResponse>, Status> {
        let r = req.into_inner();
        let capability_md = self.registry.capability_md(&r.capability_id).await?;
        Ok(Response::new(pb::QueryCapabilityMdResponse {
            capability_md,
        }))
    }

    async fn inspect_atlas(
        &self,
        _req: Request<pb::InspectAtlasRequest>,
    ) -> Result<Response<pb::InspectAtlasResponse>, Status> {
        let json = self.registry.inspect_json().await?;
        Ok(Response::new(pb::InspectAtlasResponse { json }))
    }
}

// ── Heartbeat eviction ──────────────────────────────────────────────────────

const DEFAULT_HEARTBEAT_TIMEOUT_MS: u64 = 60_000;
const DEFAULT_EVICTION_INTERVAL_MS: u64 = 10_000;

fn read_env_u64(name: &str, default: u64) -> u64 {
    std::env::var(name)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(default)
}

async fn eviction_loop(registry: Arc<AtlasRegistry>, timeout_ms: u64, interval_ms: u64) {
    if timeout_ms == 0 {
        info!("[atlas] heartbeat eviction disabled (timeout=0)");
        return;
    }
    info!("[atlas] heartbeat eviction: timeout={timeout_ms}ms interval={interval_ms}ms");
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
            warn!("[atlas] evicted '{id}' (heartbeat lapsed > {timeout_ms}ms)");
        }
    }
}

// ── Server entrypoint ───────────────────────────────────────────────────────

/// Start the Atlas gRPC server on `listen` using the given registry.
/// Spawns a background heartbeat-eviction task and exposes BOTH the new
/// `Atlas` service and the deprecated `RobonixRuntime` shim on the same
/// gRPC port.
pub async fn serve_atlas(registry: Arc<AtlasRegistry>, listen: SocketAddr) -> Result<()> {
    let timeout_ms = read_env_u64(
        "ROBONIX_ATLAS_HEARTBEAT_TIMEOUT_MS",
        DEFAULT_HEARTBEAT_TIMEOUT_MS,
    );
    let interval_ms = read_env_u64(
        "ROBONIX_ATLAS_EVICTION_INTERVAL_MS",
        DEFAULT_EVICTION_INTERVAL_MS,
    );
    let _eviction_task = tokio::spawn(eviction_loop(
        Arc::clone(&registry),
        timeout_ms,
        interval_ms,
    ));

    let svc = AtlasService::new(Arc::clone(&registry));
    let legacy = crate::legacy::LegacyRuntimeService::new(Arc::clone(&registry));

    info!("[atlas] gRPC listening on {listen} (Atlas + legacy RobonixRuntime)");
    tonic::transport::Server::builder()
        .add_service(pb::atlas_server::AtlasServer::new(svc))
        .add_service(crate::legacy_pb::robonix_runtime_server::RobonixRuntimeServer::new(legacy))
        .serve(listen)
        .await
        .context("Atlas server failed")?;
    Ok(())
}
