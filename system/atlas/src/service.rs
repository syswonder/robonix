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

use crate::contract_registry::ContractRegistry;
use crate::pb;
pub use pb::Transport;

/// How many times Atlas tries to mint a unique endpoint before giving up.
const MINT_ATTEMPTS: usize = 16;

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "transport", rename_all = "snake_case")]
enum TransportParamsState {
    Grpc {
        proto_file: String,
        service_name: String,
        method: String,
    },
    Ros2 {
        qos_profile: String,
    },
    Mcp {
        input_schema_json: String,
    },
}

impl TransportParamsState {
    fn transport(&self) -> Transport {
        match self {
            Self::Grpc { .. } => Transport::Grpc,
            Self::Ros2 { .. } => Transport::Ros2,
            Self::Mcp { .. } => Transport::Mcp,
        }
    }
}

impl From<&TransportParamsState> for pb::TransportParams {
    fn from(r: &TransportParamsState) -> Self {
        use pb::transport_params::Kind;
        let kind = match r {
            TransportParamsState::Grpc {
                proto_file,
                service_name,
                method,
            } => Kind::Grpc(pb::GrpcParams {
                proto_file: proto_file.clone(),
                service_name: service_name.clone(),
                method: method.clone(),
            }),
            TransportParamsState::Ros2 { qos_profile } => Kind::Ros2(pb::Ros2Params {
                qos_profile: qos_profile.clone(),
            }),
            TransportParamsState::Mcp { input_schema_json } => Kind::Mcp(pb::McpParams {
                input_schema_json: input_schema_json.clone(),
            }),
        };
        Self { kind: Some(kind) }
    }
}

#[derive(Debug, Clone, Serialize)]
struct DeclaredEndpoint {
    contract_id: String,
    #[serde(serialize_with = "serialize_transport")]
    transport: Transport,
    endpoint: String,
    params: TransportParamsState,
    /// Provider-supplied natural-language description for this
    /// Capability. Empty means "fall back to contract default".
    description: String,
}

fn serialize_transport<S: serde::Serializer>(t: &Transport, ser: S) -> Result<S::Ok, S::Error> {
    ser.serialize_str(t.as_str_name())
}

fn serialize_pushed_state<S: serde::Serializer>(
    s: &Option<pb::LifecycleState>,
    ser: S,
) -> Result<S::Ok, S::Error> {
    match s {
        Some(v) => ser.serialize_str(v.as_str_name()),
        None => ser.serialize_none(),
    }
}

fn serialize_provider_kind<S: serde::Serializer>(k: &pb::Kind, ser: S) -> Result<S::Ok, S::Error> {
    ser.serialize_str(k.as_str_name())
}

#[derive(Debug, Clone, Serialize)]
struct CapabilityProviderState {
    id: String,
    #[serde(serialize_with = "serialize_provider_kind")]
    kind: pb::Kind,
    namespace: String,
    capability_md_path: String,
    last_heartbeat_ms: u64,
    endpoints: Vec<DeclaredEndpoint>,
    /// Last value reported by SetLifecycleState. None for Providers
    /// that never push, in which case `state()` falls back to the
    /// "first non-driver capability declare -> INACTIVE" inference.
    #[serde(serialize_with = "serialize_pushed_state")]
    pushed_state: Option<pb::LifecycleState>,
    state_detail: String,
}

impl CapabilityProviderState {
    /// Convert one of this record's endpoints to a wire `pb::Capability`,
    /// stamping provider_id / provider_kind from the parent record.
    fn capability_at(&self, e: &DeclaredEndpoint) -> pb::Capability {
        pb::Capability {
            provider_id: self.id.clone(),
            provider_kind: self.kind as i32,
            contract_id: e.contract_id.clone(),
            transport: e.transport as i32,
            params: Some((&e.params).into()),
            description: e.description.clone(),
        }
    }
}

impl From<&CapabilityProviderState> for pb::CapabilityProvider {
    fn from(provider: &CapabilityProviderState) -> Self {
        Self {
            id: provider.id.clone(),
            kind: provider.kind as i32,
            namespace: provider.namespace.clone(),
            capability_md_path: provider.capability_md_path.clone(),
            last_heartbeat_ms: provider.last_heartbeat_ms,
            capabilities: provider
                .endpoints
                .iter()
                .map(|e| provider.capability_at(e))
                .collect(),
            state: provider.state() as i32,
            state_detail: provider.state_detail.clone(),
        }
    }
}

impl CapabilityProviderState {
    /// Lifecycle state. The provider pushes via SetLifecycleState whenever its
    /// on_init / on_activate / on_deactivate handler returns; that pushed
    /// value wins. For legacy providers that never push, fall back to "any
    /// non-driver capability declared → INACTIVE" (preserves behaviour
    /// for code still on the old register-then-declare-only flow).
    fn state(&self) -> pb::LifecycleState {
        if let Some(s) = self.pushed_state {
            return s;
        }
        let any_non_driver = self
            .endpoints
            .iter()
            .any(|e| !is_driver_contract(&e.contract_id));
        if any_non_driver {
            pb::LifecycleState::StateInactive
        } else {
            pb::LifecycleState::StateRegistered
        }
    }
}

/// Whether `prev → next` matches the v0.1 lifecycle FSM (see
/// `docs/src/architecture/provider-lifecycle.md`). `Unspecified` as `prev`
/// means "fresh provider, never pushed state" — accept any first transition.
fn is_legal_transition(prev: pb::LifecycleState, next: pb::LifecycleState) -> bool {
    use pb::LifecycleState::*;
    if next == StateError || next == StateTerminated {
        return true; // any state may fail or shut down
    }
    match (prev, next) {
        (StateUnspecified, _) => true,
        (StateRegistered, StateInactive) => true,
        (StateInactive, StateActive) => true,
        (StateActive, StateInactive) => true,
        (StateError, StateInactive) => true,
        // self-transitions are no-ops, accept silently
        (a, b) if a == b => true,
        _ => false,
    }
}

fn is_driver_contract(contract_id: &str) -> bool {
    // Per-area lifecycle drivers: `robonix/primitive/<area>/driver`,
    // `robonix/service/<area>/driver`. The `{CAP_CLASS}/driver` template in
    // `capabilities/{primitive,service}/driver.v1.toml` is concretised by
    // the provider to its area at DeclareCapability time.
    contract_id.ends_with("/driver")
}

/// One open consumer→provider edge. Allocated by `ConnectCapability`,
/// dropped by `DisconnectCapability` or when the provider unregisters /
/// is evicted by heartbeat lapse.
#[derive(Debug, Clone, Serialize)]
struct OpenChannel {
    channel_id: String,
    consumer_id: String,
    provider_id: String,
    contract_id: String,
    #[serde(serialize_with = "serialize_transport")]
    transport: Transport,
    endpoint: String,
    opened_at_ms: u64,
}

#[derive(Debug, Default, Serialize)]
pub(crate) struct State {
    providers: HashMap<String, CapabilityProviderState>,
    channels: HashMap<String, OpenChannel>,
}

impl State {
    /// Drop every channel whose provider is `provider_id`. Called from
    /// unregister / heartbeat eviction so dead providers don't leak
    /// channel providers. Returns the number of channels dropped.
    fn drop_channels_of(&mut self, provider_id: &str) -> usize {
        let before = self.channels.len();
        self.channels.retain(|_, ch| ch.provider_id != provider_id);
        before - self.channels.len()
    }
}

/// In-memory state shared by all gRPC handlers (new + legacy). All ops go
/// through one of the typed async methods below. `Arc<AtlasRegistry>` is
/// cheap to clone; the interior `RwLock` serialises mutations.
///
/// `contracts` is loaded once at startup from `<robonix_source>/capabilities/**`
/// and is read-only for the lifetime of the process — handlers serve
/// QueryContract / ListContracts directly off it without locking.
#[derive(Debug, Default)]
pub struct AtlasRegistry {
    pub(crate) inner: RwLock<State>,
    contracts: ContractRegistry,
}

impl AtlasRegistry {
    /// Construct a registry with an empty capability/channel state plus a
    /// pre-loaded contract registry. Use this from `main.rs` once the
    /// capabilities dir has been resolved.
    pub fn with_contracts(contracts: ContractRegistry) -> Self {
        Self {
            inner: RwLock::new(State::default()),
            contracts,
        }
    }

    pub fn contracts(&self) -> &ContractRegistry {
        &self.contracts
    }
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

    /// Register a new capability instance, OR take over an existing
    /// provider_id slot whose previous provider is gone. Empty `provider_id` triggers
    /// Atlas-assigned ephemeral id. Returns the resolved id.
    ///
    /// Takeover semantics — a re-Register on an existing provider_id is NOT
    /// an error. We assume the old process is dead (or about to be), so
    /// we drop its endpoints + state and reset last_heartbeat to now.
    /// The caller is then expected to redeclare capabilities with its own
    /// fresh endpoints. Without this, an orphan provider (heartbeat eviction
    /// hasn't fired yet — 60s default) blocks every subsequent boot of
    /// the same package: rbnx waits for a "new" provider to appear in atlas,
    /// the python framework only retries register once and then quietly
    /// keeps going, and atlas keeps pointing consumers at the dead
    /// process's endpoints. The previous "ALREADY_EXISTS" failure mode
    /// caught accidental dual deployments but the cure was worse than
    /// the disease — silent boot failures are nearly impossible to
    /// diagnose. If two live processes claim the same id, both will
    /// heartbeat into the same record and the latest declare wins; that
    /// case shows up as scattered/mysterious endpoint flips, which is
    /// at least visible.
    pub async fn register(
        &self,
        provider_id: &str,
        kind: pb::Kind,
        namespace: &str,
        capability_md_path: &str,
    ) -> Result<String, Status> {
        let provider_id = if provider_id.trim().is_empty() {
            Self::assign_id()
        } else {
            provider_id.trim().to_string()
        };
        let namespace = Self::require("namespace", namespace)?.to_string();
        let mut state = self.inner.write().await;
        if let Some(existing) = state.providers.get_mut(&provider_id) {
            // Cross-kind collision is rejected per proto contract.
            if existing.kind != kind {
                return Err(Status::already_exists(format!(
                    "'{provider_id}' already registered as {:?}; cannot re-register as {:?}",
                    existing.kind, kind
                )));
            }
            // Same-kind takeover. Drop the previous provider's endpoints
            // and pushed state; the caller will redeclare what it owns.
            // Channels targeting dropped Capabilities are also auto-closed.
            let prev_iface_count = existing.endpoints.len();
            existing.namespace = namespace;
            existing.capability_md_path = capability_md_path.trim().to_string();
            existing.last_heartbeat_ms = Self::now_ms();
            existing.endpoints.clear();
            existing.pushed_state = None;
            existing.state_detail.clear();
            let dropped = state.drop_channels_of(&provider_id);
            info!(
                "[atlas] register {provider_id} (takeover; dropped {prev_iface_count} \
                 stale capabilities, {dropped} channels)"
            );
            return Ok(provider_id);
        }
        state.providers.insert(
            provider_id.clone(),
            CapabilityProviderState {
                id: provider_id.clone(),
                kind,
                namespace,
                capability_md_path: capability_md_path.trim().to_string(),
                last_heartbeat_ms: Self::now_ms(),
                endpoints: Vec::new(),
                pushed_state: None,
                state_detail: String::new(),
            },
        );
        info!("[atlas] register {provider_id} kind={kind:?}");
        Ok(provider_id)
    }

    /// Idempotent: returns `true` if a record was removed, `false` if the id
    /// was unknown. Also drops any channels where this provider was the provider —
    /// consumers will get NOT_FOUND on their next call and can re-discover.
    pub async fn unregister(&self, provider_id: &str) -> bool {
        let provider_id = provider_id.trim();
        if provider_id.is_empty() {
            return false;
        }
        let mut state = self.inner.write().await;
        let was_present = state.providers.remove(provider_id).is_some();
        let dropped = state.drop_channels_of(provider_id);
        info!(
            "[atlas] unregister {provider_id} (was_present={was_present}, channels_dropped={dropped})"
        );
        was_present
    }

    /// Update the provider's lifecycle state. Returns the previous value (or
    /// the inferred fallback when nothing's been pushed yet) so callers
    /// can log "X went INACTIVE → ACTIVE" without a separate query.
    /// Validation is **soft**: illegal transitions log a warn but the
    /// new state is still stored. Strict validation will land later
    /// once telemetry confirms there are no spurious illegal
    /// transitions during atlas/provider startup-race conditions.
    pub async fn set_lifecycle_state(
        &self,
        provider_id: &str,
        new_state: pb::LifecycleState,
        detail: &str,
    ) -> Result<pb::LifecycleState, Status> {
        let provider_id = Self::require("provider_id", provider_id)?;
        if new_state == pb::LifecycleState::StateUnspecified {
            return Err(Status::invalid_argument(
                "state: must not be STATE_UNSPECIFIED",
            ));
        }
        let mut state = self.inner.write().await;
        let provider = state
            .providers
            .get_mut(provider_id)
            .ok_or_else(|| Status::not_found(format!("unknown provider_id: {provider_id}")))?;
        let prev = provider.state();
        if !is_legal_transition(prev, new_state) {
            warn!(
                "[atlas] illegal transition {provider_id}: {:?} -> {:?} (storing anyway, soft-validation v0.1)",
                prev, new_state
            );
        }
        provider.pushed_state = Some(new_state);
        provider.state_detail = detail.trim().to_string();
        info!(
            "[atlas] state {provider_id}: {:?} -> {:?}{}",
            prev,
            new_state,
            if provider.state_detail.is_empty() {
                String::new()
            } else {
                format!(" ({})", provider.state_detail)
            }
        );
        Ok(prev)
    }

    /// Updates `last_heartbeat_ms` to now. Returns the timestamp it set.
    pub async fn heartbeat(&self, provider_id: &str) -> Result<u64, Status> {
        let provider_id = Self::require("provider_id", provider_id)?;
        let now = Self::now_ms();
        let mut state = self.inner.write().await;
        let provider = state
            .providers
            .get_mut(provider_id)
            .ok_or_else(|| Status::not_found(format!("unknown provider_id: {provider_id}")))?;
        provider.last_heartbeat_ms = now;
        Ok(now)
    }

    /// Declare ONE transport for ONE contract on a registered provider. Returns
    /// the authoritative endpoint string (may differ from `proposed` when
    /// Atlas rewrote it to disambiguate on a mintable transport).
    pub async fn declare(
        &self,
        provider_id: &str,
        contract_id: &str,
        transport: Transport,
        proposed: &str,
        params: pb::TransportParams,
        description: &str,
    ) -> Result<String, Status> {
        let provider_id = Self::require("provider_id", provider_id)?;
        let contract_id = Self::require("contract_id", contract_id)?.to_string();
        let params = parse_params(transport, Some(params))?;
        let proposed = proposed.trim().to_string();

        let mut state = self.inner.write().await;
        let provider = state
            .providers
            .get(provider_id)
            .ok_or_else(|| Status::not_found(format!("unknown provider_id: {provider_id}")))?;
        if !contract_id.starts_with(&provider.namespace) {
            return Err(Status::invalid_argument(format!(
                "contract_id '{contract_id}' is not under namespace '{}' of capability '{provider_id}'",
                provider.namespace
            )));
        }
        if provider
            .endpoints
            .iter()
            .any(|e| e.contract_id == contract_id && e.transport == transport)
        {
            return Err(Status::already_exists(format!(
                "({contract_id}, {transport:?}) already declared by {provider_id}"
            )));
        }

        let endpoint = resolve_endpoint(&state, transport, &proposed, &contract_id, provider_id)?;
        let provider = state
            .providers
            .get_mut(provider_id)
            .ok_or_else(|| Status::internal("capability vanished mid-declare"))?;
        provider.endpoints.push(DeclaredEndpoint {
            contract_id: contract_id.clone(),
            transport,
            endpoint: endpoint.clone(),
            params,
            description: description.to_string(),
        });
        info!("[atlas] declare {provider_id} {contract_id} via {transport:?} -> {endpoint}");
        Ok(endpoint)
    }

    /// Snapshot of registered Providers matching the given filters. Empty
    /// `provider_id` / empty `contract` / `Transport::Unspecified` /
    /// `Kind::Unspecified` mean "no filter on that field". Each
    /// returned record carries only the Capabilities that satisfy the
    /// `contract` + `transport` filters.
    pub async fn query(
        &self,
        provider_id: &str,
        kind: pb::Kind,
        contract: &str,
        transport: Transport,
    ) -> Vec<pb::CapabilityProvider> {
        self.query_with_prefix(provider_id, kind, contract, "", transport)
            .await
    }

    pub async fn query_with_prefix(
        &self,
        provider_id: &str,
        kind: pb::Kind,
        contract: &str,
        namespace_prefix: &str,
        transport: Transport,
    ) -> Vec<pb::CapabilityProvider> {
        let f_cap_id = provider_id.trim();
        let f_contract = contract.trim();
        let f_ns_prefix = namespace_prefix.trim();
        let f_transport = if transport == Transport::Unspecified {
            None
        } else {
            Some(transport)
        };
        let f_kind = if kind == pb::Kind::Unspecified {
            None
        } else {
            Some(kind)
        };

        let state = self.inner.read().await;
        let mut out = Vec::new();
        for provider in state.providers.values() {
            if !f_cap_id.is_empty() && provider.id != f_cap_id {
                continue;
            }
            if let Some(k) = f_kind
                && provider.kind != k
            {
                continue;
            }
            if !f_ns_prefix.is_empty() && !provider.namespace.starts_with(f_ns_prefix) {
                continue;
            }
            if !f_contract.is_empty()
                && !provider
                    .endpoints
                    .iter()
                    .any(|e| e.contract_id == f_contract)
            {
                continue;
            }
            let capabilities: Vec<pb::Capability> = provider
                .endpoints
                .iter()
                .filter(|e| {
                    (f_contract.is_empty() || e.contract_id == f_contract)
                        && f_transport.is_none_or(|t| e.transport == t)
                })
                .map(|e| provider.capability_at(e))
                .collect();
            out.push(pb::CapabilityProvider {
                id: provider.id.clone(),
                kind: provider.kind as i32,
                namespace: provider.namespace.clone(),
                capability_md_path: provider.capability_md_path.clone(),
                last_heartbeat_ms: provider.last_heartbeat_ms,
                state: provider.state() as i32,
                state_detail: provider.state_detail.clone(),
                capabilities,
            });
        }
        out
    }

    /// Open a channel to one (provider provider, contract, transport). Atlas
    /// only providers the edge — the consumer dials the returned endpoint
    /// itself (each transport has its own connect protocol; atlas can't
    /// dial generically). Returns the allocated channel handle and the
    /// full binding the consumer needs.
    pub async fn connect(
        &self,
        consumer_id: &str,
        provider_id: &str,
        contract_id: &str,
        transport: Transport,
    ) -> Result<(String, String, pb::TransportParams), Status> {
        let consumer_id = Self::require("consumer_id", consumer_id)?.to_string();
        let provider_id = Self::require("provider_id", provider_id)?.to_string();
        let contract_id = Self::require("contract_id", contract_id)?.to_string();
        if transport == Transport::Unspecified {
            return Err(Status::invalid_argument(
                "transport: must not be UNSPECIFIED",
            ));
        }

        let mut state = self.inner.write().await;
        let provider = state
            .providers
            .get(&provider_id)
            .ok_or_else(|| Status::not_found(format!("unknown provider_id: {provider_id}")))?;
        let ep = provider
            .endpoints
            .iter()
            .find(|e| e.contract_id == contract_id && e.transport == transport)
            .ok_or_else(|| {
                Status::not_found(format!(
                    "provider '{provider_id}' has not declared ({contract_id}, {transport:?})"
                ))
            })?;
        let endpoint = ep.endpoint.clone();
        let params: pb::TransportParams = (&ep.params).into();

        let channel_id = format!("ch-{}", Uuid::new_v4().simple());
        state.channels.insert(
            channel_id.clone(),
            OpenChannel {
                channel_id: channel_id.clone(),
                consumer_id: consumer_id.clone(),
                provider_id: provider_id.clone(),
                contract_id: contract_id.clone(),
                transport,
                endpoint: endpoint.clone(),
                opened_at_ms: Self::now_ms(),
            },
        );
        info!(
            "[atlas] connect '{consumer_id}' -> '{provider_id}' \
             {contract_id} via {transport:?} -> {endpoint} ({channel_id})"
        );
        Ok((channel_id, endpoint, params))
    }

    /// Idempotent: returns `true` if a channel was removed, `false` if
    /// the id was unknown (already released, or auto-dropped when its
    /// provider went away).
    pub async fn disconnect(&self, channel_id: &str) -> bool {
        let channel_id = channel_id.trim();
        if channel_id.is_empty() {
            return false;
        }
        let mut state = self.inner.write().await;
        let was_open = state.channels.remove(channel_id).is_some();
        info!("[atlas] disconnect {channel_id} (was_open={was_open})");
        was_open
    }

    /// Read the provider's CAPABILITY.md content. Returns "" when the provider
    /// registered without a path.
    pub async fn capability_md(&self, provider_id: &str) -> Result<String, Status> {
        let provider_id = Self::require("provider_id", provider_id)?;
        let path = {
            let state = self.inner.read().await;
            let provider = state
                .providers
                .get(provider_id)
                .ok_or_else(|| Status::not_found(format!("unknown provider_id: {provider_id}")))?;
            provider.capability_md_path.clone()
        };
        if path.is_empty() {
            return Ok(String::new());
        }
        match tokio::fs::read_to_string(&path).await {
            Ok(s) => Ok(s),
            Err(e) => {
                warn!("[atlas] {provider_id}: read CAPABILITY.md '{path}' failed: {e}");
                Err(Status::internal(format!(
                    "failed to read CAPABILITY.md for {provider_id}: {e}"
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
) -> Result<TransportParamsState, Status> {
    use pb::transport_params::Kind;
    let kind = params.and_then(|p| p.kind).ok_or_else(|| {
        Status::invalid_argument(
            "params required: set TransportParams.kind to the variant matching `transport`",
        )
    })?;
    let params_state = match kind {
        Kind::Grpc(g) => TransportParamsState::Grpc {
            proto_file: g.proto_file,
            service_name: g.service_name,
            method: g.method,
        },
        Kind::Ros2(r) => TransportParamsState::Ros2 {
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
            TransportParamsState::Mcp {
                input_schema_json: m.input_schema_json,
            }
        }
    };
    if params_state.transport() != transport {
        return Err(Status::invalid_argument(format!(
            "params: oneof variant {:?} does not match transport {:?}",
            params_state.transport(),
            transport
        )));
    }
    Ok(params_state)
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

/// Pick a globally-unique endpoint per the rules on `DeclareCapabilityRequest`.
///
/// "Globally unique" here means: no *other* provider may already own this
/// `(transport, endpoint)` pair. The provider itself is allowed to expose multiple
/// contracts on the same endpoint — that's the dominant pattern for MCP
/// (one MCP server URL hosts many tools) and a legitimate one for gRPC
/// (one tonic Server with multiple services). The per-provider
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
        state.providers.iter().any(|(other_id, provider)| {
            other_id != own_cap_id
                && provider
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
             after {MINT_ATTEMPTS} attempts (existing providers: {})",
            state.providers.len()
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
         after {MINT_ATTEMPTS} attempts (existing providers: {})",
        state.providers.len()
    )))
}

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
    async fn register_primitive(
        &self,
        req: Request<pb::RegisterRequest>,
    ) -> Result<Response<pb::RegisterResponse>, Status> {
        let r = req.into_inner();
        let id = self
            .registry
            .register(
                &r.id,
                pb::Kind::Primitive,
                &r.namespace,
                &r.capability_md_path,
            )
            .await?;
        Ok(Response::new(pb::RegisterResponse { id }))
    }

    async fn register_service(
        &self,
        req: Request<pb::RegisterRequest>,
    ) -> Result<Response<pb::RegisterResponse>, Status> {
        let r = req.into_inner();
        let id = self
            .registry
            .register(
                &r.id,
                pb::Kind::Service,
                &r.namespace,
                &r.capability_md_path,
            )
            .await?;
        Ok(Response::new(pb::RegisterResponse { id }))
    }

    async fn register_skill(
        &self,
        req: Request<pb::RegisterRequest>,
    ) -> Result<Response<pb::RegisterResponse>, Status> {
        let r = req.into_inner();
        let id = self
            .registry
            .register(&r.id, pb::Kind::Skill, &r.namespace, &r.capability_md_path)
            .await?;
        Ok(Response::new(pb::RegisterResponse { id }))
    }

    async fn unregister(
        &self,
        req: Request<pb::UnregisterRequest>,
    ) -> Result<Response<pb::UnregisterResponse>, Status> {
        let r = req.into_inner();
        let was_present = self.registry.unregister(&r.id).await;
        Ok(Response::new(pb::UnregisterResponse { was_present }))
    }

    async fn heartbeat(
        &self,
        req: Request<pb::HeartbeatRequest>,
    ) -> Result<Response<pb::HeartbeatResponse>, Status> {
        let r = req.into_inner();
        self.registry.heartbeat(&r.id).await?;
        Ok(Response::new(pb::HeartbeatResponse {}))
    }

    async fn set_lifecycle_state(
        &self,
        req: Request<pb::SetLifecycleStateRequest>,
    ) -> Result<Response<pb::SetLifecycleStateResponse>, Status> {
        let r = req.into_inner();
        let new_state = pb::LifecycleState::try_from(r.state).map_err(|_| {
            Status::invalid_argument(format!("unknown LifecycleState value: {}", r.state))
        })?;
        let prev = self
            .registry
            .set_lifecycle_state(&r.id, new_state, &r.detail)
            .await?;
        Ok(Response::new(pb::SetLifecycleStateResponse {
            previous_state: prev as i32,
        }))
    }

    async fn declare_capability(
        &self,
        req: Request<pb::DeclareCapabilityRequest>,
    ) -> Result<Response<pb::DeclareCapabilityResponse>, Status> {
        let r = req.into_inner();
        let transport = parse_transport(r.transport)?;
        let endpoint = self
            .registry
            .declare(
                &r.provider_id,
                &r.contract_id,
                transport,
                &r.endpoint,
                r.params.unwrap_or_default(),
                &r.description,
            )
            .await?;
        Ok(Response::new(pb::DeclareCapabilityResponse { endpoint }))
    }

    async fn query(
        &self,
        req: Request<pb::QueryRequest>,
    ) -> Result<Response<pb::QueryResponse>, Status> {
        let r = req.into_inner();
        let transport = Transport::try_from(r.transport).unwrap_or(Transport::Unspecified);
        let kind = pb::Kind::try_from(r.kind).unwrap_or(pb::Kind::Unspecified);
        let providers = self
            .registry
            .query_with_prefix(&r.id, kind, &r.contract_id, &r.namespace_prefix, transport)
            .await;
        Ok(Response::new(pb::QueryResponse { providers }))
    }

    async fn connect_capability(
        &self,
        req: Request<pb::ConnectCapabilityRequest>,
    ) -> Result<Response<pb::ConnectCapabilityResponse>, Status> {
        let r = req.into_inner();
        let transport = parse_transport(r.transport)?;
        let (channel_id, endpoint, params) = self
            .registry
            .connect(&r.consumer_id, &r.provider_id, &r.contract_id, transport)
            .await?;
        Ok(Response::new(pb::ConnectCapabilityResponse {
            channel_id,
            endpoint,
            params: Some(params),
        }))
    }

    async fn disconnect_capability(
        &self,
        req: Request<pb::DisconnectCapabilityRequest>,
    ) -> Result<Response<pb::DisconnectCapabilityResponse>, Status> {
        let r = req.into_inner();
        let was_open = self.registry.disconnect(&r.channel_id).await;
        Ok(Response::new(pb::DisconnectCapabilityResponse { was_open }))
    }

    async fn inspect_atlas(
        &self,
        _req: Request<pb::InspectAtlasRequest>,
    ) -> Result<Response<pb::InspectAtlasResponse>, Status> {
        let json = self.registry.inspect_json().await?;
        Ok(Response::new(pb::InspectAtlasResponse { json }))
    }

    async fn query_contract(
        &self,
        req: Request<pb::QueryContractRequest>,
    ) -> Result<Response<pb::QueryContractResponse>, Status> {
        let r = req.into_inner();
        let id = r.contract_id.trim();
        if id.is_empty() {
            return Err(Status::invalid_argument("contract_id required"));
        }
        let resp = match self.registry.contracts().get(id) {
            Some(d) => pb::QueryContractResponse {
                contract: Some(d.clone()),
                found: true,
            },
            None => pb::QueryContractResponse {
                contract: Some(pb::ContractDescriptor::default()),
                found: false,
            },
        };
        Ok(Response::new(resp))
    }

    async fn list_contracts(
        &self,
        req: Request<pb::ListContractsRequest>,
    ) -> Result<Response<pb::ListContractsResponse>, Status> {
        let r = req.into_inner();
        let contracts = self
            .registry
            .contracts()
            .list_with_prefix(&r.namespace_prefix);
        Ok(Response::new(pb::ListContractsResponse { contracts }))
    }
}

const DEFAULT_EVICTION_INTERVAL_MS: u64 = 10_000; // check every 10s
const DEFAULT_HEARTBEAT_TIMEOUT_MS: u64 = 90_000; // mark TERMINATED after 90s
const DEFAULT_GC_AFTER_TERMINATED_MS: u64 = 600_000; // drop record 10 min after TERMINATED

fn read_env_u64(name: &str, default: u64) -> u64 {
    std::env::var(name)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(default)
}

/// Two-phase eviction:
///   1. heartbeat lapsed > `timeout_ms` AND state is not yet TERMINATED →
///      transition to TERMINATED, drop the provider's channels (so consumers
///      stop dialing a corpse).
///   2. state is TERMINATED AND last_heartbeat older than `gc_after_ms` →
///      remove the record entirely.
///
/// Phase 1 keeps a debug-friendly "yes that provider died, here's why" view in
/// `rbnx caps` for `gc_after_ms` after the lapse; phase 2 frees memory.
async fn eviction_loop(
    registry: Arc<AtlasRegistry>,
    timeout_ms: u64,
    gc_after_ms: u64,
    interval_ms: u64,
) {
    if timeout_ms == 0 {
        info!("[atlas] heartbeat eviction disabled (timeout=0)");
        return;
    }
    info!(
        "[atlas] heartbeat eviction: terminate-after={timeout_ms}ms \
         gc-after-terminated={gc_after_ms}ms interval={interval_ms}ms"
    );
    let interval = std::time::Duration::from_millis(interval_ms);
    loop {
        tokio::time::sleep(interval).await;
        let now = AtlasRegistry::now_ms();
        let mut state = registry.inner.write().await;

        // Phase 1: lapsed → TERMINATED.
        let lapsed: Vec<String> = state
            .providers
            .iter()
            .filter(|(_, provider)| {
                now.saturating_sub(provider.last_heartbeat_ms) > timeout_ms
                    && provider.state() != pb::LifecycleState::StateTerminated
            })
            .map(|(id, _)| id.clone())
            .collect();
        for id in &lapsed {
            if let Some(provider) = state.providers.get_mut(id) {
                provider.pushed_state = Some(pb::LifecycleState::StateTerminated);
                provider.state_detail = format!("heartbeat lapsed > {timeout_ms}ms");
            }
            let dropped = state.drop_channels_of(id);
            warn!(
                "[atlas] '{id}' → TERMINATED (heartbeat lapsed > {timeout_ms}ms, \
                 channels_dropped={dropped})"
            );
        }

        // Phase 2: TERMINATED long enough → drop the record.
        let stale: Vec<String> = state
            .providers
            .iter()
            .filter(|(_, provider)| {
                provider.state() == pb::LifecycleState::StateTerminated
                    && now.saturating_sub(provider.last_heartbeat_ms) > timeout_ms + gc_after_ms
            })
            .map(|(id, _)| id.clone())
            .collect();
        for id in &stale {
            state.providers.remove(id);
            warn!("[atlas] '{id}' GC'd from registry (TERMINATED > {gc_after_ms}ms)");
        }
    }
}

/// Start the Atlas gRPC server on `listen` using the given registry.
/// Spawns a background heartbeat-eviction task and exposes BOTH the new
/// `Atlas` service and the deprecated `RobonixRuntime` shim on the same
/// gRPC port.
pub async fn serve_atlas(registry: Arc<AtlasRegistry>, listen: SocketAddr) -> Result<()> {
    let timeout_ms = read_env_u64(
        "ROBONIX_ATLAS_HEARTBEAT_TIMEOUT_MS",
        DEFAULT_HEARTBEAT_TIMEOUT_MS,
    );
    let gc_after_ms = read_env_u64(
        "ROBONIX_ATLAS_GC_AFTER_TERMINATED_MS",
        DEFAULT_GC_AFTER_TERMINATED_MS,
    );
    let interval_ms = read_env_u64(
        "ROBONIX_ATLAS_EVICTION_INTERVAL_MS",
        DEFAULT_EVICTION_INTERVAL_MS,
    );
    let _eviction_task = tokio::spawn(eviction_loop(
        Arc::clone(&registry),
        timeout_ms,
        gc_after_ms,
        interval_ms,
    ));

    let svc = AtlasService::new(Arc::clone(&registry));

    info!("[atlas] gRPC listening on {listen}");
    tonic::transport::Server::builder()
        .add_service(pb::atlas_server::AtlasServer::new(svc))
        .serve(listen)
        .await
        .context("Atlas server failed")?;
    Ok(())
}
