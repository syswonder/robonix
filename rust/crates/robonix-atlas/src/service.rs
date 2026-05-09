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

fn serialize_pushed_state<S: serde::Serializer>(
    s: &Option<pb::CapabilityState>,
    ser: S,
) -> Result<S::Ok, S::Error> {
    match s {
        Some(v) => ser.serialize_str(v.as_str_name()),
        None => ser.serialize_none(),
    }
}

impl From<&EndpointRec> for pb::InterfaceMetadata {
    fn from(e: &EndpointRec) -> Self {
        Self {
            contract_id: e.contract_id.clone(),
            transport: e.transport as i32,
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
    /// Last value reported by SetCapabilityState. None for legacy caps
    /// that never push, in which case `state()` falls back to the old
    /// "first non-driver interface declare → INITIALIZED" inference.
    #[serde(serialize_with = "serialize_pushed_state")]
    pushed_state: Option<pb::CapabilityState>,
    state_detail: String,
}

impl From<&CapRecord> for pb::CapabilityRecord {
    fn from(rec: &CapRecord) -> Self {
        Self {
            capability_id: rec.capability_id.clone(),
            namespace: rec.namespace.clone(),
            capability_md_path: rec.capability_md_path.clone(),
            last_heartbeat_ms: rec.last_heartbeat_ms,
            interfaces: rec.endpoints.iter().map(Into::into).collect(),
            state: rec.state() as i32,
            state_detail: rec.state_detail.clone(),
        }
    }
}

impl CapRecord {
    /// Lifecycle state. The cap pushes via SetCapabilityState whenever its
    /// on_init / on_activate / on_deactivate handler returns; that pushed
    /// value wins. For legacy caps that never push, fall back to "any
    /// non-driver interface declared → INITIALIZED" (preserves behaviour
    /// for code still on the old register-then-declare-only flow).
    fn state(&self) -> pb::CapabilityState {
        if let Some(s) = self.pushed_state {
            return s;
        }
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

/// Whether `prev → next` matches the v0.1 lifecycle FSM (see
/// `docs/src/architecture/cap-lifecycle.md`). `Unspecified` as `prev`
/// means "fresh cap, never pushed state" — accept any first transition.
fn is_legal_transition(prev: pb::CapabilityState, next: pb::CapabilityState) -> bool {
    use pb::CapabilityState::*;
    if next == StateError || next == StateTerminated {
        return true; // any state may fail or shut down
    }
    match (prev, next) {
        (StateUnspecified, _) => true,
        (StateRegistered, StateInitialized) => true,
        (StateInitialized, StateRunning) => true,
        (StateRunning, StateInitialized) => true,
        (StateError, StateInitialized) => true,
        // self-transitions are no-ops, accept silently
        (a, b) if a == b => true,
        _ => false,
    }
}

fn is_driver_contract(contract_id: &str) -> bool {
    // Per-area lifecycle drivers: `robonix/primitive/<area>/driver`,
    // `robonix/service/<area>/driver`. The `{CAP_CLASS}/driver` template in
    // `capabilities/{primitive,service}/driver.v1.toml` is concretised by
    // the cap to its area at DeclareInterface time.
    contract_id.ends_with("/driver")
}

/// One open consumer→provider edge. Allocated by `ConnectCapability`,
/// dropped by `DisconnectCapability` or when the provider unregisters /
/// is evicted by heartbeat lapse.
#[derive(Debug, Clone, Serialize)]
struct ChannelRec {
    channel_id: String,
    consumer_id: String,
    provider_cap_id: String,
    contract_id: String,
    #[serde(serialize_with = "serialize_transport")]
    transport: Transport,
    endpoint: String,
    opened_at_ms: u64,
}

#[derive(Debug, Default, Serialize)]
pub(crate) struct State {
    caps: HashMap<String, CapRecord>,
    channels: HashMap<String, ChannelRec>,
}

impl State {
    /// Drop every channel whose provider is `cap_id`. Called from
    /// unregister / heartbeat eviction so dead providers don't leak
    /// channel records. Returns the number of channels dropped.
    fn drop_channels_of(&mut self, cap_id: &str) -> usize {
        let before = self.channels.len();
        self.channels.retain(|_, ch| ch.provider_cap_id != cap_id);
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
    /// cap_id slot whose previous owner is gone. Empty `cap_id` triggers
    /// Atlas-assigned ephemeral id. Returns the resolved id.
    ///
    /// Takeover semantics — a re-Register on an existing cap_id is NOT
    /// an error. We assume the old process is dead (or about to be), so
    /// we drop its endpoints + state and reset last_heartbeat to now.
    /// The caller is then expected to redeclare interfaces with its own
    /// fresh endpoints. Without this, an orphan cap (heartbeat eviction
    /// hasn't fired yet — 60s default) blocks every subsequent boot of
    /// the same package: rbnx waits for a "new" cap to appear in atlas,
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
        if let Some(existing) = state.caps.get_mut(&cap_id) {
            // Take over the slot. Drop the previous owner's endpoints
            // and pushed state; the caller will redeclare what it owns.
            // Heartbeat moves to `now` so wait_for_registration spotters
            // can tell this register call from the dead-cap residue.
            let prev_iface_count = existing.endpoints.len();
            existing.namespace = namespace;
            existing.capability_md_path = capability_md_path.trim().to_string();
            existing.last_heartbeat_ms = Self::now_ms();
            existing.endpoints.clear();
            existing.pushed_state = None;
            existing.state_detail.clear();
            info!(
                "[atlas] register {cap_id} (takeover; dropped {prev_iface_count} \
                 stale interfaces)"
            );
            return Ok(cap_id);
        }
        state.caps.insert(
            cap_id.clone(),
            CapRecord {
                capability_id: cap_id.clone(),
                namespace,
                capability_md_path: capability_md_path.trim().to_string(),
                last_heartbeat_ms: Self::now_ms(),
                endpoints: Vec::new(),
                pushed_state: None,
                state_detail: String::new(),
            },
        );
        info!("[atlas] register {cap_id}");
        Ok(cap_id)
    }

    /// Idempotent: returns `true` if a record was removed, `false` if the id
    /// was unknown. Also drops any channels where this cap was the provider —
    /// consumers will get NOT_FOUND on their next call and can re-discover.
    pub async fn unregister(&self, cap_id: &str) -> bool {
        let cap_id = cap_id.trim();
        if cap_id.is_empty() {
            return false;
        }
        let mut state = self.inner.write().await;
        let was_present = state.caps.remove(cap_id).is_some();
        let dropped = state.drop_channels_of(cap_id);
        info!(
            "[atlas] unregister {cap_id} (was_present={was_present}, channels_dropped={dropped})"
        );
        was_present
    }

    /// Update the cap's lifecycle state. Returns the previous value (or
    /// the inferred fallback when nothing's been pushed yet) so callers
    /// can log "X went INITIALIZED → RUNNING" without a separate query.
    /// Validation is **soft** in v0.1: illegal transitions log a warn
    /// but the new state is still stored. Strict validation will land
    /// in v0.2 once telemetry confirms there are no spurious illegal
    /// transitions during atlas/cap startup-race conditions.
    pub async fn set_capability_state(
        &self,
        cap_id: &str,
        new_state: pb::CapabilityState,
        detail: &str,
    ) -> Result<pb::CapabilityState, Status> {
        let cap_id = Self::require("capability_id", cap_id)?;
        if new_state == pb::CapabilityState::StateUnspecified {
            return Err(Status::invalid_argument(
                "state: must not be STATE_UNSPECIFIED",
            ));
        }
        let mut state = self.inner.write().await;
        let rec = state
            .caps
            .get_mut(cap_id)
            .ok_or_else(|| Status::not_found(format!("unknown capability_id: {cap_id}")))?;
        let prev = rec.state();
        if !is_legal_transition(prev, new_state) {
            warn!(
                "[atlas] illegal transition {cap_id}: {:?} -> {:?} (storing anyway, soft-validation v0.1)",
                prev, new_state
            );
        }
        rec.pushed_state = Some(new_state);
        rec.state_detail = detail.trim().to_string();
        info!(
            "[atlas] state {cap_id}: {:?} -> {:?}{}",
            prev,
            new_state,
            if rec.state_detail.is_empty() {
                String::new()
            } else {
                format!(" ({})", rec.state_detail)
            }
        );
        Ok(prev)
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
        self.query_with_prefix(cap_id, contract, "", transport)
            .await
    }

    pub async fn query_with_prefix(
        &self,
        cap_id: &str,
        contract: &str,
        namespace_prefix: &str,
        transport: Transport,
    ) -> Vec<pb::CapabilityRecord> {
        let f_cap_id = cap_id.trim();
        let f_contract = contract.trim();
        let f_ns_prefix = namespace_prefix.trim();
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
            if !f_ns_prefix.is_empty() && !rec.namespace.starts_with(f_ns_prefix) {
                continue;
            }
            if !f_contract.is_empty() && !rec.endpoints.iter().any(|e| e.contract_id == f_contract)
            {
                continue;
            }
            let interfaces: Vec<pb::InterfaceMetadata> = rec
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
                state_detail: rec.state_detail.clone(),
                interfaces,
            });
        }
        out
    }

    /// Open a channel to one (provider cap, contract, transport). Atlas
    /// only records the edge — the consumer dials the returned endpoint
    /// itself (each transport has its own connect protocol; atlas can't
    /// dial generically). Returns the allocated channel handle and the
    /// full binding the consumer needs.
    pub async fn connect(
        &self,
        consumer_id: &str,
        provider_cap_id: &str,
        contract_id: &str,
        transport: Transport,
    ) -> Result<(String, String, pb::TransportParams), Status> {
        let consumer_id = Self::require("consumer_id", consumer_id)?.to_string();
        let provider_cap_id = Self::require("capability_id", provider_cap_id)?.to_string();
        let contract_id = Self::require("contract_id", contract_id)?.to_string();
        if transport == Transport::Unspecified {
            return Err(Status::invalid_argument(
                "transport: must not be UNSPECIFIED",
            ));
        }

        let mut state = self.inner.write().await;
        let rec = state.caps.get(&provider_cap_id).ok_or_else(|| {
            Status::not_found(format!("unknown capability_id: {provider_cap_id}"))
        })?;
        let ep = rec
            .endpoints
            .iter()
            .find(|e| e.contract_id == contract_id && e.transport == transport)
            .ok_or_else(|| {
                Status::not_found(format!(
                    "cap '{provider_cap_id}' has not declared ({contract_id}, {transport:?})"
                ))
            })?;
        let endpoint = ep.endpoint.clone();
        let params: pb::TransportParams = (&ep.params).into();

        let channel_id = format!("ch-{}", Uuid::new_v4().simple());
        state.channels.insert(
            channel_id.clone(),
            ChannelRec {
                channel_id: channel_id.clone(),
                consumer_id: consumer_id.clone(),
                provider_cap_id: provider_cap_id.clone(),
                contract_id: contract_id.clone(),
                transport,
                endpoint: endpoint.clone(),
                opened_at_ms: Self::now_ms(),
            },
        );
        info!(
            "[atlas] connect '{consumer_id}' -> '{provider_cap_id}' \
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
        let dropped = state.drop_channels_of(cap_id);
        info!(
            "[atlas] unregister {cap_id} (was_present={}, channels_dropped={dropped})",
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

    async fn set_capability_state(
        &self,
        req: Request<pb::SetCapabilityStateRequest>,
    ) -> Result<Response<pb::SetCapabilityStateResponse>, Status> {
        let r = req.into_inner();
        let new_state = pb::CapabilityState::try_from(r.state).map_err(|_| {
            Status::invalid_argument(format!("unknown CapabilityState value: {}", r.state))
        })?;
        let prev = self
            .registry
            .set_capability_state(&r.capability_id, new_state, &r.detail)
            .await?;
        Ok(Response::new(pb::SetCapabilityStateResponse {
            previous_state: prev as i32,
        }))
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
            .query_with_prefix(
                &r.capability_id,
                &r.contract_id,
                &r.namespace_prefix,
                transport,
            )
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

    async fn connect_capability(
        &self,
        req: Request<pb::ConnectCapabilityRequest>,
    ) -> Result<Response<pb::ConnectCapabilityResponse>, Status> {
        let r = req.into_inner();
        let transport = parse_transport(r.transport)?;
        let (channel_id, endpoint, params) = self
            .registry
            .connect(&r.consumer_id, &r.capability_id, &r.contract_id, transport)
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
///      transition to TERMINATED, drop the cap's channels (so consumers
///      stop dialing a corpse).
///   2. state is TERMINATED AND last_heartbeat older than `gc_after_ms` →
///      remove the record entirely.
/// Phase 1 keeps a debug-friendly "yes that cap died, here's why" view in
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
            .caps
            .iter()
            .filter(|(_, rec)| {
                now.saturating_sub(rec.last_heartbeat_ms) > timeout_ms
                    && rec.state() != pb::CapabilityState::StateTerminated
            })
            .map(|(id, _)| id.clone())
            .collect();
        for id in &lapsed {
            if let Some(rec) = state.caps.get_mut(id) {
                rec.pushed_state = Some(pb::CapabilityState::StateTerminated);
                rec.state_detail = format!("heartbeat lapsed > {timeout_ms}ms");
            }
            let dropped = state.drop_channels_of(id);
            warn!(
                "[atlas] '{id}' → TERMINATED (heartbeat lapsed > {timeout_ms}ms, \
                 channels_dropped={dropped})"
            );
        }

        // Phase 2: TERMINATED long enough → drop the record.
        let stale: Vec<String> = state
            .caps
            .iter()
            .filter(|(_, rec)| {
                rec.state() == pb::CapabilityState::StateTerminated
                    && now.saturating_sub(rec.last_heartbeat_ms) > timeout_ms + gc_after_ms
            })
            .map(|(id, _)| id.clone())
            .collect();
        for id in &stale {
            state.caps.remove(id);
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
    let legacy = crate::legacy::LegacyRuntimeService::new(Arc::clone(&registry));

    info!("[atlas] gRPC listening on {listen} (Atlas + legacy)");
    tonic::transport::Server::builder()
        .add_service(pb::atlas_server::AtlasServer::new(svc))
        .add_service(crate::legacy_pb::robonix_runtime_server::RobonixRuntimeServer::new(legacy))
        .serve(listen)
        .await
        .context("Atlas server failed")?;
    Ok(())
}
