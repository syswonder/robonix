// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Atlas client-side helpers shared by every Robonix component that talks to
// Atlas (pilot, executor, cli, system services, …).
//
// Two layers:
//   * `AtlasClient` — thin wrapper over the generated stub with retry-able
//     connect and one helper per Atlas RPC. Use this for register / declare /
//     query / heartbeat / connect / disconnect / unregister.
//   * `connect_to_capability` — gRPC-only convenience: pick the first cap
//     offering the contract, call `ConnectCapability` to record the edge,
//     dial the returned host:port, and hand back a tonic Channel + the
//     channel_id (so the caller can DisconnectCapability on shutdown).
//     For ROS 2 / MCP consumers, call `ConnectCapability` directly and
//     dial yourself — atlas can't dial generically across transports.
//
// All helpers return `anyhow::Error` wrapping the underlying `tonic::Status`
// so callers can attach context with `.with_context(...)`.

use crate::pb;
use anyhow::{Context, Result};
use std::time::Duration;
use tonic::transport::{Channel, Endpoint};

/// Wrapped `pb::atlas_client::AtlasClient` with helpers.
///
/// Cheap to clone — the inner generated client wraps a `Channel`, which is
/// itself just a handle to a connection pool. Share clones across tasks
/// rather than wrapping in a `Mutex`.
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
    /// Returns the last error if all attempts fail.
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

    /// Underlying generated client. Use for raw RPCs not covered by helpers.
    pub fn inner(&self) -> pb::atlas_client::AtlasClient<Channel> {
        self.inner.clone()
    }

    /// Register a capability instance. Returns the (possibly Atlas-assigned)
    /// `capability_id`.
    pub async fn register_capability(
        &mut self,
        capability_id: &str,
        namespace: &str,
        capability_md_path: &str,
    ) -> Result<String> {
        let resp = self
            .inner
            .register_capability(pb::RegisterCapabilityRequest {
                capability_id: capability_id.to_string(),
                namespace: namespace.to_string(),
                capability_md_path: capability_md_path.to_string(),
            })
            .await
            .with_context(|| format!("RegisterCapability '{capability_id}'"))?;
        Ok(resp.into_inner().capability_id)
    }

    /// Declare one (transport, endpoint) binding for one interface.
    /// Returns the *authoritative* endpoint (may differ from `endpoint`
    /// when Atlas rewrote to disambiguate).
    pub async fn declare_interface(
        &mut self,
        capability_id: &str,
        contract_id: &str,
        transport: pb::Transport,
        endpoint: &str,
        params: pb::TransportParams,
    ) -> Result<String> {
        let resp = self
            .inner
            .declare_interface(pb::DeclareInterfaceRequest {
                capability_id: capability_id.to_string(),
                contract_id: contract_id.to_string(),
                transport: transport as i32,
                endpoint: endpoint.to_string(),
                params: Some(params),
            })
            .await
            .with_context(|| {
                format!(
                    "DeclareInterface cap='{capability_id}' contract='{contract_id}' \
                     transport={transport:?}"
                )
            })?;
        Ok(resp.into_inner().endpoint)
    }

    /// Query capabilities. Empty / `Transport::Unspecified` means "no filter
    /// on that field".
    pub async fn query_capabilities(
        &mut self,
        capability_id: &str,
        contract_id: &str,
        transport: pb::Transport,
    ) -> Result<Vec<pb::CapabilityRecord>> {
        let resp = self
            .inner
            .query_capabilities(pb::QueryCapabilitiesRequest {
                capability_id: capability_id.to_string(),
                contract_id: contract_id.to_string(),
                transport: transport as i32,
            })
            .await
            .with_context(|| {
                format!(
                    "QueryCapabilities cap='{capability_id}' contract='{contract_id}' \
                     transport={transport:?}"
                )
            })?;
        Ok(resp.into_inner().records)
    }

    /// Fetch the package's CAPABILITY.md content for a registered cap.
    /// Returns "" when the cap was registered with no `capability_md_path`.
    pub async fn query_capability_md(&mut self, capability_id: &str) -> Result<String> {
        let resp = self
            .inner
            .query_capability_md(pb::QueryCapabilityMdRequest {
                capability_id: capability_id.to_string(),
            })
            .await
            .with_context(|| format!("QueryCapabilityMd '{capability_id}'"))?;
        Ok(resp.into_inner().capability_md)
    }

    pub async fn heartbeat(&mut self, capability_id: &str) -> Result<()> {
        self.inner
            .heartbeat(pb::HeartbeatRequest {
                capability_id: capability_id.to_string(),
            })
            .await
            .with_context(|| format!("Heartbeat '{capability_id}'"))?;
        Ok(())
    }

    /// Push a lifecycle state transition. Atlas does NOT validate the
    /// transition graph — the cap is the source of truth. `detail` is a
    /// free-form human-readable note (e.g. "missing /opt/models/...") that
    /// `rbnx caps` surfaces verbatim; pass empty when there's nothing to add.
    pub async fn set_capability_state(
        &mut self,
        capability_id: &str,
        new_state: pb::CapabilityState,
        detail: &str,
    ) -> Result<()> {
        self.inner
            .set_capability_state(pb::SetCapabilityStateRequest {
                capability_id: capability_id.to_string(),
                state: new_state as i32,
                detail: detail.to_string(),
            })
            .await
            .with_context(|| {
                format!("SetCapabilityState '{capability_id}' -> {new_state:?}")
            })?;
        Ok(())
    }

    /// Open a channel to one cap's interface. Atlas records the
    /// consumer→provider edge and returns the binding. Caller dials the
    /// returned `endpoint` themselves using whatever transport-appropriate
    /// mechanism (tonic for grpc, rclrs for ros2, fastmcp for mcp, …).
    /// Returns `(channel_id, endpoint, params)`.
    pub async fn connect_capability(
        &mut self,
        consumer_id: &str,
        capability_id: &str,
        contract_id: &str,
        transport: pb::Transport,
    ) -> Result<(String, String, pb::TransportParams)> {
        let resp = self
            .inner
            .connect_capability(pb::ConnectCapabilityRequest {
                consumer_id: consumer_id.to_string(),
                capability_id: capability_id.to_string(),
                contract_id: contract_id.to_string(),
                transport: transport as i32,
            })
            .await
            .with_context(|| {
                format!(
                    "ConnectCapability consumer='{consumer_id}' provider='{capability_id}' \
                     contract='{contract_id}' transport={transport:?}"
                )
            })?;
        let r = resp.into_inner();
        Ok((r.channel_id, r.endpoint, r.params.unwrap_or_default()))
    }

    /// Release a previously-opened channel. Idempotent: returns `false` when
    /// the channel_id was unknown (already released, or auto-dropped because
    /// the provider unregistered / was evicted).
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

    /// Unregister. Returns `true` if a record was removed, `false` if the
    /// id was unknown (idempotent caller-side semantics).
    pub async fn unregister_capability(&mut self, capability_id: &str) -> Result<bool> {
        let resp = self
            .inner
            .unregister_capability(pb::UnregisterCapabilityRequest {
                capability_id: capability_id.to_string(),
            })
            .await
            .with_context(|| format!("UnregisterCapability '{capability_id}'"))?;
        Ok(resp.into_inner().was_present)
    }
}

/// gRPC-only convenience: pick the first cap offering `contract_id` over
/// gRPC, call `ConnectCapability` to register the edge, dial the returned
/// host:port, and hand back the bookkeeping handle + tonic Channel.
///
/// Returns `(channel_id, capability_id, Channel)`. The caller MUST call
/// `disconnect_capability(channel_id)` when it's done so atlas can drop
/// the bookkeeping. If multiple caps offer the contract atlas's order is
/// unspecified — callers needing deterministic selection should call
/// `query_capabilities` + `connect_capability` themselves.
///
/// Not for ROS 2 / MCP consumers — those transports have their own
/// connect protocols. Use `AtlasClient::connect_capability` directly and
/// dial yourself with rclrs / fastmcp / etc.
pub async fn connect_to_capability(
    atlas: &mut AtlasClient,
    consumer_id: &str,
    contract_id: &str,
) -> Result<(String, String, Channel)> {
    let records = atlas
        .query_capabilities("", contract_id, pb::Transport::Grpc)
        .await?;
    if records.is_empty() {
        anyhow::bail!(
            "no capability offering contract_id='{contract_id}' over gRPC; \
             registered caps may not have declared this interface yet"
        );
    }
    if records.len() > 1 {
        log::warn!(
            "[atlas-client] {} caps offer '{contract_id}' over gRPC; \
             picking first ('{}'). Use query_capabilities + connect_capability \
             for deterministic selection.",
            records.len(),
            records[0].capability_id,
        );
    }
    let cap_id = records
        .into_iter()
        .next()
        .expect("non-empty checked above")
        .capability_id;
    let (channel_id, endpoint_str, _params) = atlas
        .connect_capability(consumer_id, &cap_id, contract_id, pb::Transport::Grpc)
        .await?;
    let normalized = normalize_grpc_endpoint(&endpoint_str);
    let channel = Endpoint::new(normalized.clone())
        .with_context(|| format!("invalid endpoint '{}' for cap '{}'", normalized, cap_id))?
        .connect()
        .await
        .with_context(|| {
            format!("connect to cap '{cap_id}' at '{normalized}' for contract '{contract_id}'")
        })?;
    Ok((channel_id, cap_id, channel))
}

// ── TransportParams constructors ────────────────────────────────────────────
//
// Tiny helpers so callers don't have to write the `oneof` wrapper boilerplate
// at every DeclareInterface site.

/// Build `TransportParams` for a gRPC interface.
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

/// Build `TransportParams` for a ROS 2 interface.
pub fn ros2_params(qos_profile: impl Into<String>) -> pb::TransportParams {
    pb::TransportParams {
        kind: Some(pb::transport_params::Kind::Ros2(pb::Ros2Params {
            qos_profile: qos_profile.into(),
        })),
    }
}

/// Build `TransportParams` for an MCP tool interface.
pub fn mcp_params(
    description: impl Into<String>,
    input_schema_json: impl Into<String>,
) -> pb::TransportParams {
    pb::TransportParams {
        kind: Some(pb::transport_params::Kind::Mcp(pb::McpParams {
            description: description.into(),
            input_schema_json: input_schema_json.into(),
        })),
    }
}

// ── Helpers ─────────────────────────────────────────────────────────────────

fn normalize_grpc_endpoint(s: &str) -> String {
    let s = s.trim();
    if s.starts_with("http://") || s.starts_with("https://") {
        s.to_string()
    } else {
        format!("http://{s}")
    }
}
