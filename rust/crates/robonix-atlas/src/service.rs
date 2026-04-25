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

use pb::Transport;

/// How many times Atlas tries to mint a unique endpoint before giving up.
/// At 8-hex-char UUID suffixes the keyspace is 2^32; collisions are
/// dominated by bugs in the collide check, not randomness.
const MINT_ATTEMPTS: usize = 16;

// ── Data model ──────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize)]
struct CapabilityInterfaceRec {
    name: String,
    contract_id: String,
    extra_json: String,
}

impl From<pb::CapabilityInterface> for CapabilityInterfaceRec {
    fn from(c: pb::CapabilityInterface) -> Self {
        Self {
            name: c.name,
            contract_id: c.contract_id,
            extra_json: c.extra_json,
        }
    }
}

impl From<&CapabilityInterfaceRec> for pb::CapabilityInterface {
    fn from(c: &CapabilityInterfaceRec) -> Self {
        Self {
            name: c.name.clone(),
            contract_id: c.contract_id.clone(),
            extra_json: c.extra_json.clone(),
        }
    }
}

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
        is_service: bool,
        qos_profile: String,
    },
    SharedMemory {
        size_bytes: u64,
        ringbuf_capacity: u32,
    },
    RawTcp,
    Websocket {
        subprotocol: String,
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
            Self::SharedMemory { .. } => Transport::SharedMemory,
            Self::RawTcp => Transport::RawTcp,
            Self::Websocket { .. } => Transport::Websocket,
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
            TransportParamsRec::Ros2 {
                is_service,
                qos_profile,
            } => Kind::Ros2(pb::Ros2Params {
                is_service: *is_service,
                qos_profile: qos_profile.clone(),
            }),
            TransportParamsRec::SharedMemory {
                size_bytes,
                ringbuf_capacity,
            } => Kind::SharedMemory(pb::SharedMemoryParams {
                size_bytes: *size_bytes,
                ringbuf_capacity: *ringbuf_capacity,
            }),
            TransportParamsRec::RawTcp => Kind::RawTcp(pb::RawTcpParams {}),
            TransportParamsRec::Websocket { subprotocol } => Kind::Websocket(pb::WebsocketParams {
                subprotocol: subprotocol.clone(),
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
    interfaces: Vec<CapabilityInterfaceRec>,
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
/// requiring a prior OS-level bind by the caller. ros2/shared_memory are
/// pure-name address spaces; grpc/raw_tcp/websocket need a host:port that
/// only the caller can produce.
fn atlas_can_mint(transport: Transport) -> bool {
    matches!(transport, Transport::Ros2 | Transport::SharedMemory)
}

/// Validate `s` is empty or a parseable JSON object (not bare scalar /
/// array — the proto fields are spec'd as objects).
fn validate_extra_json(field: &str, s: &str) -> Result<(), Status> {
    let s = s.trim();
    if s.is_empty() {
        return Ok(());
    }
    let v: serde_json::Value = serde_json::from_str(s)
        .map_err(|e| Status::invalid_argument(format!("{field}: invalid JSON ({e})")))?;
    if !v.is_object() {
        return Err(Status::invalid_argument(format!(
            "{field}: must be a JSON object"
        )));
    }
    Ok(())
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
            is_service: r.is_service,
            qos_profile: r.qos_profile,
        },
        Kind::SharedMemory(s) => TransportParamsRec::SharedMemory {
            size_bytes: s.size_bytes,
            ringbuf_capacity: s.ringbuf_capacity,
        },
        Kind::RawTcp(_) => TransportParamsRec::RawTcp,
        Kind::Websocket(w) => TransportParamsRec::Websocket {
            subprotocol: w.subprotocol,
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

        // Hard-reject any contract_id that's not under the announced
        // namespace. Proto says "must"; honour that contract.
        for c in &r.interfaces {
            if c.contract_id.trim().is_empty() {
                return Err(Status::invalid_argument(
                    "interface contract_id must not be empty",
                ));
            }
            if !c.contract_id.starts_with(&namespace) {
                return Err(Status::invalid_argument(format!(
                    "contract_id '{}' is not under namespace '{}'",
                    c.contract_id, namespace
                )));
            }
            validate_extra_json(
                &format!("interface '{}' extra_json", c.contract_id),
                &c.extra_json,
            )?;
        }

        let interfaces: Vec<CapabilityInterfaceRec> =
            r.interfaces.into_iter().map(Into::into).collect();

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
        let f_contract = r.contract_id.trim();
        let f_namespace = r.namespace.trim();
        let f_transport = Transport::try_from(r.transport)
            .ok()
            .filter(|&t| t != Transport::Unspecified);

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
                        && f_transport.is_none_or(|t| e.transport == t)
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

// ── Server entrypoint ───────────────────────────────────────────────────────

/// Start the Atlas gRPC server on `listen` using the given registry.
/// Blocks until the server stops or returns an error.
pub async fn serve_atlas(registry: Arc<AtlasRegistry>, listen: SocketAddr) -> Result<()> {
    let svc = AtlasService::new(registry);
    info!("[atlas] gRPC listening on {listen}");
    tonic::transport::Server::builder()
        .add_service(pb::atlas_server::AtlasServer::new(svc))
        .serve(listen)
        .await
        .context("Atlas server failed")?;
    Ok(())
}
