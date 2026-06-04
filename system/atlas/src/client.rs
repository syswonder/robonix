// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Atlas client-side helpers shared by every Robonix component that talks
// to Atlas (pilot, executor, cli, system services, …).
//
// All helpers return `anyhow::Error` wrapping the underlying
// `tonic::Status` so callers can attach context with `.with_context(...)`.

use crate::pb;
use anyhow::{Context, Result};
use std::time::Duration;
use tonic::transport::{Channel, Endpoint};

/// Wrapped `pb::atlas_client::AtlasClient` with helpers.
///
/// Cheap to clone — the inner generated client wraps a `Channel`, which
/// is itself just a handle to a connection pool. Share clones across
/// tasks rather than wrapping in a `Mutex`.
#[derive(Clone)]
pub struct AtlasClient {
    inner: pb::atlas_client::AtlasClient<Channel>,
}

impl AtlasClient {
    /// Connect once. Accepts bare `host:port` or full `http://host:port`.
    pub async fn connect(endpoint: impl AsRef<str>) -> Result<Self> {
        let normalized = normalize_grpc_endpoint(endpoint.as_ref());
        let channel = Endpoint::new(normalized.clone())
            .with_context(|| format!("invalid Atlas endpoint '{}'", normalized))?
            .connect()
            .await
            .with_context(|| format!("connect to Atlas at '{}'", normalized))?;
        Ok(Self {
            inner: pb::atlas_client::AtlasClient::new(channel),
        })
    }

    /// `connect`, retrying up to `attempts` times with `delay` between tries.
    pub async fn connect_with_retry(
        endpoint: impl AsRef<str>,
        attempts: u32,
        delay: Duration,
    ) -> Result<Self> {
        let endpoint = endpoint.as_ref();
        let mut last_err: Option<anyhow::Error> = None;
        for i in 0..attempts.max(1) {
            match Self::connect(endpoint).await {
                Ok(c) => return Ok(c),
                Err(e) => {
                    log::debug!(
                        "[atlas-client] connect attempt {}/{} failed: {e:#}",
                        i + 1,
                        attempts
                    );
                    last_err = Some(e);
                    if i + 1 < attempts {
                        tokio::time::sleep(delay).await;
                    }
                }
            }
        }
        Err(last_err.unwrap_or_else(|| anyhow::anyhow!("connect_with_retry: 0 attempts")))
    }

    pub fn inner(&self) -> pb::atlas_client::AtlasClient<Channel> {
        self.inner.clone()
    }

    // ── Registration (one RPC per kind, shared request/response type) ──────

    fn build_register_req(
        id: &str,
        namespace: &str,
        capability_md_path: &str,
    ) -> pb::RegisterRequest {
        pb::RegisterRequest {
            id: id.to_string(),
            namespace: namespace.to_string(),
            capability_md_path: capability_md_path.to_string(),
        }
    }

    /// Register a Primitive. Returns the (possibly Atlas-assigned) id.
    pub async fn register_primitive(
        &mut self,
        id: &str,
        namespace: &str,
        capability_md_path: &str,
    ) -> Result<String> {
        let resp = self
            .inner
            .register_primitive(Self::build_register_req(id, namespace, capability_md_path))
            .await
            .with_context(|| format!("RegisterPrimitive '{id}'"))?;
        Ok(resp.into_inner().id)
    }

    /// Register a Service.
    pub async fn register_service(
        &mut self,
        id: &str,
        namespace: &str,
        capability_md_path: &str,
    ) -> Result<String> {
        let resp = self
            .inner
            .register_service(Self::build_register_req(id, namespace, capability_md_path))
            .await
            .with_context(|| format!("RegisterService '{id}'"))?;
        Ok(resp.into_inner().id)
    }

    /// Register a Skill.
    pub async fn register_skill(
        &mut self,
        id: &str,
        namespace: &str,
        capability_md_path: &str,
    ) -> Result<String> {
        let resp = self
            .inner
            .register_skill(Self::build_register_req(id, namespace, capability_md_path))
            .await
            .with_context(|| format!("RegisterSkill '{id}'"))?;
        Ok(resp.into_inner().id)
    }

    /// Unregister any registered entity. Returns `true` if a record was
    /// removed, `false` if the id was unknown (idempotent).
    pub async fn unregister(&mut self, id: &str) -> Result<bool> {
        let resp = self
            .inner
            .unregister(pb::UnregisterRequest { id: id.to_string() })
            .await
            .with_context(|| format!("Unregister '{id}'"))?;
        Ok(resp.into_inner().was_present)
    }

    // ── Liveness + lifecycle ───────────────────────────────────────────────

    pub async fn heartbeat(&mut self, id: &str) -> Result<()> {
        self.inner
            .heartbeat(pb::HeartbeatRequest { id: id.to_string() })
            .await
            .with_context(|| format!("Heartbeat '{id}'"))?;
        Ok(())
    }

    /// Push a lifecycle state transition. `detail` is a free-form
    /// human-readable note (e.g. "missing /opt/models/...") that
    /// `rbnx caps` surfaces verbatim; pass empty when there's nothing.
    pub async fn set_lifecycle_state(
        &mut self,
        id: &str,
        new_state: pb::LifecycleState,
        detail: &str,
    ) -> Result<()> {
        self.inner
            .set_lifecycle_state(pb::SetLifecycleStateRequest {
                id: id.to_string(),
                state: new_state as i32,
                detail: detail.to_string(),
            })
            .await
            .with_context(|| format!("SetLifecycleState '{id}' -> {new_state:?}"))?;
        Ok(())
    }

    // ── Capability binding ─────────────────────────────────────────────────

    /// Declare one Capability (transport-bound endpoint) on an entity.
    /// Returns the *authoritative* endpoint (may differ from the request
    /// when Atlas rewrote to disambiguate). `description` is the optional
    /// natural-language description for this Capability.
    pub async fn declare_capability(
        &mut self,
        provider_id: &str,
        contract_id: &str,
        transport: pb::Transport,
        endpoint: &str,
        params: pb::TransportParams,
    ) -> Result<String> {
        self.declare_capability_with_description(
            provider_id,
            contract_id,
            transport,
            endpoint,
            params,
            "",
        )
        .await
    }

    /// Same as `declare_capability` but with the instance-specific
    /// description string (see DeclareCapabilityRequest.description).
    pub async fn declare_capability_with_description(
        &mut self,
        provider_id: &str,
        contract_id: &str,
        transport: pb::Transport,
        endpoint: &str,
        params: pb::TransportParams,
        description: &str,
    ) -> Result<String> {
        let resp = self
            .inner
            .declare_capability(pb::DeclareCapabilityRequest {
                provider_id: provider_id.to_string(),
                contract_id: contract_id.to_string(),
                transport: transport as i32,
                endpoint: endpoint.to_string(),
                params: Some(params),
                description: description.to_string(),
            })
            .await
            .with_context(|| {
                format!(
                    "DeclareCapability provider='{provider_id}' contract='{contract_id}' \
                     transport={transport:?}"
                )
            })?;
        Ok(resp.into_inner().endpoint)
    }

    // ── Discovery ──────────────────────────────────────────────────────────

    /// Generic Query. `kind == Kind::Unspecified` = no kind filter (all
    /// kinds returned; each `CapabilityProvider.kind` carries its kind).
    /// Empty strings / `Transport::Unspecified` = no filter on that field.
    pub async fn query(
        &mut self,
        kind: pb::Kind,
        id: &str,
        contract_id: &str,
        namespace_prefix: &str,
        transport: pb::Transport,
    ) -> Result<Vec<pb::CapabilityProvider>> {
        let resp = self
            .inner
            .query(pb::QueryRequest {
                kind: kind as i32,
                id: id.to_string(),
                contract_id: contract_id.to_string(),
                namespace_prefix: namespace_prefix.to_string(),
                transport: transport as i32,
            })
            .await
            .with_context(|| format!("Query kind={kind:?}"))?;
        Ok(resp.into_inner().providers)
    }

    /// Convenience — find Primitives (kind filter applied).
    pub async fn query_primitives(
        &mut self,
        id: &str,
        contract_id: &str,
        namespace_prefix: &str,
        transport: pb::Transport,
    ) -> Result<Vec<pb::CapabilityProvider>> {
        self.query(
            pb::Kind::Primitive,
            id,
            contract_id,
            namespace_prefix,
            transport,
        )
        .await
    }

    /// Convenience — find Services.
    pub async fn query_services(
        &mut self,
        id: &str,
        contract_id: &str,
        namespace_prefix: &str,
        transport: pb::Transport,
    ) -> Result<Vec<pb::CapabilityProvider>> {
        self.query(
            pb::Kind::Service,
            id,
            contract_id,
            namespace_prefix,
            transport,
        )
        .await
    }

    /// Convenience — find Skills.
    pub async fn query_skills(
        &mut self,
        id: &str,
        contract_id: &str,
        namespace_prefix: &str,
        transport: pb::Transport,
    ) -> Result<Vec<pb::CapabilityProvider>> {
        self.query(
            pb::Kind::Skill,
            id,
            contract_id,
            namespace_prefix,
            transport,
        )
        .await
    }

    /// Consumer-facing discovery: flat list of Capabilities across all
    /// kinds. Walks `Query(kind=Unspecified)` and flattens each
    /// CapabilityProvider's nested capabilities. Each returned
    /// `Capability` already carries `provider_id` + `provider_kind`.
    pub async fn flatten_capabilities(
        &mut self,
        contract_id: &str,
        namespace_prefix: &str,
        transport: pb::Transport,
    ) -> Result<Vec<pb::Capability>> {
        let providers = self
            .query(
                pb::Kind::Unspecified,
                "",
                contract_id,
                namespace_prefix,
                transport,
            )
            .await?;
        Ok(providers
            .into_iter()
            .flat_map(|p| p.capabilities.into_iter())
            .collect())
    }

    /// Back-compat alias for the legacy 3-arg signature returning a list
    /// of CapabilityProviders. Equivalent to
    /// `query(Kind::Unspecified, id, contract_id, "", transport)`.
    pub async fn query_capabilities(
        &mut self,
        id: &str,
        contract_id: &str,
        transport: pb::Transport,
    ) -> Result<Vec<pb::CapabilityProvider>> {
        self.query(pb::Kind::Unspecified, id, contract_id, "", transport)
            .await
    }

    // ── Channels ───────────────────────────────────────────────────────────

    /// Open a channel to one (provider, contract, transport). Atlas only
    /// providers the edge — the consumer dials the returned endpoint
    /// itself using whatever transport-appropriate mechanism (tonic for
    /// grpc, rclrs for ros2, fastmcp for mcp, …).
    /// Returns `(channel_id, endpoint, params)`.
    pub async fn connect_capability(
        &mut self,
        consumer_id: &str,
        provider_id: &str,
        contract_id: &str,
        transport: pb::Transport,
    ) -> Result<(String, String, pb::TransportParams)> {
        let resp = self
            .inner
            .connect_capability(pb::ConnectCapabilityRequest {
                consumer_id: consumer_id.to_string(),
                provider_id: provider_id.to_string(),
                contract_id: contract_id.to_string(),
                transport: transport as i32,
            })
            .await
            .with_context(|| {
                format!(
                    "ConnectCapability consumer='{consumer_id}' provider='{provider_id}' \
                     contract='{contract_id}' transport={transport:?}"
                )
            })?;
        let r = resp.into_inner();
        Ok((r.channel_id, r.endpoint, r.params.unwrap_or_default()))
    }

    /// Release a previously-opened channel. Idempotent: returns `false`
    /// when the channel_id was unknown.
    pub async fn disconnect_capability(&mut self, channel_id: &str) -> Result<bool> {
        let resp = self
            .inner
            .disconnect_capability(pb::DisconnectCapabilityRequest {
                channel_id: channel_id.to_string(),
            })
            .await
            .with_context(|| format!("DisconnectCapability '{channel_id}'"))?;
        Ok(resp.into_inner().was_open)
    }

    // ── Contract registry ──────────────────────────────────────────────────

    /// Look up one contract by id.
    pub async fn query_contract(
        &mut self,
        contract_id: &str,
    ) -> Result<Option<pb::ContractDescriptor>> {
        let resp = self
            .inner
            .query_contract(pb::QueryContractRequest {
                contract_id: contract_id.to_string(),
            })
            .await
            .with_context(|| format!("QueryContract '{contract_id}'"))?;
        let inner = resp.into_inner();
        Ok(if inner.found { inner.contract } else { None })
    }

    pub async fn list_contracts(
        &mut self,
        namespace_prefix: &str,
    ) -> Result<Vec<pb::ContractDescriptor>> {
        let resp = self
            .inner
            .list_contracts(pb::ListContractsRequest {
                namespace_prefix: namespace_prefix.to_string(),
            })
            .await
            .with_context(|| format!("ListContracts prefix='{namespace_prefix}'"))?;
        Ok(resp.into_inner().contracts)
    }
}

/// gRPC-only convenience: pick the first Capability matching `contract_id`
/// over gRPC, call `ConnectCapability` to register the edge, dial the
/// returned host:port, and hand back a tonic Channel + the channel_id
/// (so the caller can DisconnectCapability on shutdown).
///
/// Returns `(channel_id, provider_id, Channel)`.
pub async fn connect_to_capability(
    atlas: &mut AtlasClient,
    consumer_id: &str,
    contract_id: &str,
) -> Result<(String, String, Channel)> {
    let rows = atlas
        .flatten_capabilities(contract_id, "", pb::Transport::Grpc)
        .await?;
    if rows.is_empty() {
        anyhow::bail!(
            "no Capability offering contract_id='{contract_id}' over gRPC; \
             registered entities may not have declared this Capability yet"
        );
    }
    if rows.len() > 1 {
        log::warn!(
            "[atlas-client] {} entities offer '{contract_id}' over gRPC; \
             picking first ('{}'). Use query_capabilities + connect_capability \
             for deterministic selection.",
            rows.len(),
            rows[0].provider_id,
        );
    }
    let provider_id = rows
        .into_iter()
        .next()
        .expect("non-empty checked above")
        .provider_id;
    let (channel_id, endpoint_str, _params) = atlas
        .connect_capability(consumer_id, &provider_id, contract_id, pb::Transport::Grpc)
        .await?;
    let normalized = normalize_grpc_endpoint(&endpoint_str);
    let channel = Endpoint::new(normalized.clone())
        .with_context(|| {
            format!(
                "invalid endpoint '{}' for provider '{}'",
                normalized, provider_id
            )
        })?
        .connect()
        .await
        .with_context(|| {
            format!(
                "connect to provider '{provider_id}' at '{normalized}' for contract '{contract_id}'"
            )
        })?;
    Ok((channel_id, provider_id, channel))
}

// ── TransportParams constructors ───────────────────────────────────────────

pub fn grpc_params(
    proto_file: impl Into<String>,
    service_name: impl Into<String>,
    method: impl Into<String>,
) -> pb::TransportParams {
    pb::TransportParams {
        kind: Some(pb::transport_params::Kind::Grpc(pb::GrpcParams {
            proto_file: proto_file.into(),
            service_name: service_name.into(),
            method: method.into(),
        })),
    }
}

pub fn ros2_params(qos_profile: impl Into<String>) -> pb::TransportParams {
    pb::TransportParams {
        kind: Some(pb::transport_params::Kind::Ros2(pb::Ros2Params {
            qos_profile: qos_profile.into(),
        })),
    }
}

/// Build `TransportParams` for an MCP tool Capability. The natural-
/// language description now lives on `DeclareCapabilityRequest.description`,
/// not inside `McpParams`.
pub fn mcp_params(input_schema_json: impl Into<String>) -> pb::TransportParams {
    pb::TransportParams {
        kind: Some(pb::transport_params::Kind::Mcp(pb::McpParams {
            input_schema_json: input_schema_json.into(),
        })),
    }
}

// ── Helpers ────────────────────────────────────────────────────────────────

fn normalize_grpc_endpoint(s: &str) -> String {
    let s = s.trim();
    if s.starts_with("http://") || s.starts_with("https://") {
        s.to_string()
    } else {
        format!("http://{s}")
    }
}
