// SPDX-License-Identifier: MulanPSL-2.0
// Atlas client-side helpers shared by every Robonix component that talks to
// Atlas (pilot, executor, cli, system services, …).
//
// Two layers:
//   * `AtlasClient` — thin wrapper over the generated stub with retry-able
//     connect and one helper per Atlas RPC. Use this for register / declare /
//     query / heartbeat / unregister.
//   * `connect_to_capability` — opinionated "find one cap by contract +
//     transport, open a tonic Channel to its endpoint" combo. The bread-and-
//     butter call site for every consumer.
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

/// "Find a cap offering this contract over gRPC, then connect to it."
///
/// The bread-and-butter discovery primitive — every consumer that wants to
/// call a Robonix gRPC interface goes through this. Returns the matched
/// `capability_id` and a connected `Channel` ready for stub construction.
///
/// Selection policy when multiple caps match: longest-namespace-wins (most
/// specific provider). Override by passing `capability_id` filter through
/// `AtlasClient::query_capabilities` directly when you need exact targeting.
pub async fn connect_to_capability(
    atlas: &mut AtlasClient,
    contract_id: &str,
) -> Result<(String, Channel)> {
    let mut records = atlas
        .query_capabilities("", contract_id, pb::Transport::Grpc)
        .await?;
    if records.is_empty() {
        anyhow::bail!(
            "no capability offering contract_id='{contract_id}' over gRPC; \
             registered caps may not have declared this interface yet"
        );
    }
    // Most specific provider wins (longest namespace).
    records.sort_by(|a, b| b.namespace.len().cmp(&a.namespace.len()));
    if records.len() > 1 {
        log::warn!(
            "[atlas-client] {} caps offer '{contract_id}' over gRPC; \
             picking '{}' (namespace='{}')",
            records.len(),
            records[0].capability_id,
            records[0].namespace
        );
    }
    let cap = records.into_iter().next().expect("non-empty checked above");
    let endpoint_str = cap
        .endpoints
        .iter()
        .find(|e| e.contract_id == contract_id && e.transport == pb::Transport::Grpc as i32)
        .map(|e| e.endpoint.clone())
        .ok_or_else(|| {
            anyhow::anyhow!(
                "cap '{}' returned by query has no matching gRPC endpoint for '{contract_id}'",
                cap.capability_id
            )
        })?;
    let normalized = normalize_grpc_endpoint(&endpoint_str);
    let channel = Endpoint::new(normalized.clone())
        .with_context(|| format!("invalid endpoint '{}' for cap '{}'", normalized, cap.capability_id))?
        .connect()
        .await
        .with_context(|| {
            format!(
                "connect to cap '{}' at '{}' for contract '{contract_id}'",
                cap.capability_id, normalized
            )
        })?;
    Ok((cap.capability_id, channel))
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
