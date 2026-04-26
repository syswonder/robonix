// SPDX-License-Identifier: MulanPSL-2.0
//
// Backward-compatibility shim for the pre-refactor `RobonixRuntime` service
// (see `proto/atlas_legacy.proto`). Atlas serves both this and the new
// `Atlas` service on the same gRPC port; old callers (liaison, audio
// bridges, any package not yet migrated) keep working until they're
// rewritten against the new schema.
//
// Every method:
//   * logs `WARN` once with the cap_id / contract_id so operators can see
//     who is still on the legacy wire,
//   * translates the old fields into the new vocabulary,
//   * delegates to the same `AtlasRegistry` the new service uses (no
//     parallel state).
//
// Removal target: track migration progress via the WARN log; drop once
// it's silent across a full deployment.

use crate::legacy_pb as legacy;
use crate::pb;
use crate::service::{AtlasRegistry, Transport};
use log::warn;
use std::path::PathBuf;
use std::sync::Arc;
use tonic::{Request, Response, Status};
use uuid::Uuid;

// ── skill_md staging (old "inline string" → new "filesystem path") ─────────

fn legacy_md_dir() -> PathBuf {
    std::env::var_os("ROBONIX_ATLAS_LEGACY_MD_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join("robonix-legacy-md"))
}

fn legacy_md_path_for(cap_id: &str) -> PathBuf {
    let safe = cap_id.replace(['/', '\\'], "_");
    legacy_md_dir().join(format!("{safe}.md"))
}

fn write_skill_md_to_temp(cap_id: &str, content: &str) -> std::io::Result<String> {
    let dir = legacy_md_dir();
    std::fs::create_dir_all(&dir)?;
    let path = legacy_md_path_for(cap_id);
    std::fs::write(&path, content)?;
    Ok(path.display().to_string())
}

fn delete_skill_md_for(cap_id: &str) {
    let _ = std::fs::remove_file(legacy_md_path_for(cap_id));
}

// ── Service ─────────────────────────────────────────────────────────────────

pub struct LegacyRuntimeService {
    registry: Arc<AtlasRegistry>,
}

impl LegacyRuntimeService {
    pub fn new(registry: Arc<AtlasRegistry>) -> Self {
        Self { registry }
    }
}

#[tonic::async_trait]
impl legacy::robonix_runtime_server::RobonixRuntime for LegacyRuntimeService {
    // ── Node lifecycle ──────────────────────────────────────────────────────

    async fn register_node(
        &self,
        req: Request<legacy::RegisterNodeRequest>,
    ) -> Result<Response<legacy::RegisterNodeResponse>, Status> {
        let r = req.into_inner();
        warn!(
            "[atlas-legacy] RegisterNode node_id='{}' namespace='{}' kind='{}' \
             — DEPRECATED, migrate to Atlas.RegisterCapability",
            r.node_id, r.namespace, r.kind
        );

        let cap_id = if r.node_id.trim().is_empty() {
            format!("com.robonix.ephemeral.{}", Uuid::new_v4())
        } else {
            r.node_id.trim().to_string()
        };

        // Stage inline skill_md as a file so QueryCapabilityMd can serve it.
        let capability_md_path = if !r.skill_md.is_empty() {
            write_skill_md_to_temp(&cap_id, &r.skill_md)
                .map_err(|e| Status::internal(format!("legacy skill_md staging: {e}")))?
        } else {
            String::new()
        };

        let resolved = self
            .registry
            .register(&cap_id, &r.namespace, &capability_md_path)
            .await?;
        Ok(Response::new(legacy::RegisterNodeResponse {
            node_id: resolved,
        }))
    }

    async fn unregister_node(
        &self,
        req: Request<legacy::UnregisterNodeRequest>,
    ) -> Result<Response<legacy::UnregisterNodeResponse>, Status> {
        let r = req.into_inner();
        warn!(
            "[atlas-legacy] UnregisterNode '{}' — DEPRECATED, migrate to Atlas.UnregisterCapability",
            r.node_id
        );
        let (was_present, _path) = self.registry.unregister_with_path(&r.node_id).await;
        if was_present {
            delete_skill_md_for(&r.node_id);
        }
        Ok(Response::new(legacy::UnregisterNodeResponse {
            ok: was_present,
        }))
    }

    async fn node_heartbeat(
        &self,
        req: Request<legacy::NodeHeartbeatRequest>,
    ) -> Result<Response<legacy::NodeHeartbeatResponse>, Status> {
        let r = req.into_inner();
        let server_time_ms = self.registry.heartbeat(&r.node_id).await?;
        Ok(Response::new(legacy::NodeHeartbeatResponse {
            ok: true,
            server_time_ms,
        }))
    }

    // ── Interface declaration / discovery ───────────────────────────────────

    async fn declare_interface(
        &self,
        req: Request<legacy::DeclareInterfaceRequest>,
    ) -> Result<Response<legacy::DeclareInterfaceResponse>, Status> {
        let r = req.into_inner();
        warn!(
            "[atlas-legacy] DeclareInterface node='{}' name='{}' contract='{}' transports={:?} \
             — DEPRECATED, migrate to Atlas.DeclareInterface",
            r.node_id, r.name, r.contract_id, r.supported_transports
        );

        if r.supported_transports.len() > 1 {
            return Err(Status::invalid_argument(
                "legacy shim supports exactly one transport per DeclareInterface call; \
                 split multi-transport declarations into separate calls",
            ));
        }
        let transport_str = r
            .supported_transports
            .first()
            .map(|s| s.as_str())
            .unwrap_or("grpc");
        let transport = parse_transport_str(transport_str)?;

        // contract_id: explicit wins; otherwise synthesise from cap.namespace + "/" + name.
        let contract_id = if !r.contract_id.is_empty() {
            r.contract_id.clone()
        } else {
            let recs = self
                .registry
                .query(&r.node_id, "", Transport::Unspecified)
                .await;
            let rec = recs
                .first()
                .ok_or_else(|| Status::not_found(format!("unknown node_id: {}", r.node_id)))?;
            format!("{}/{}", rec.namespace, r.name)
        };

        let proposed_endpoint = if r.listen_port != 0 {
            format!("127.0.0.1:{}", r.listen_port)
        } else {
            String::new()
        };
        let params = build_legacy_params(transport, &r.metadata_json);

        let resolved = self
            .registry
            .declare(
                &r.node_id,
                &contract_id,
                transport,
                &proposed_endpoint,
                params,
            )
            .await?;
        Ok(Response::new(legacy::DeclareInterfaceResponse {
            ok: true,
            allocated_endpoint: resolved,
        }))
    }

    async fn query_nodes(
        &self,
        req: Request<legacy::QueryNodesRequest>,
    ) -> Result<Response<legacy::QueryNodesResponse>, Status> {
        let r = req.into_inner();
        warn!(
            "[atlas-legacy] QueryNodes namespace='{}' name='{}' contract='{}' transport='{}' \
             — DEPRECATED, migrate to Atlas.QueryCapabilities",
            r.namespace, r.name, r.contract_id, r.transport
        );

        // Map legacy transport string → typed filter; "" means no filter.
        let transport = if r.transport.is_empty() {
            Transport::Unspecified
        } else {
            parse_transport_str(&r.transport).unwrap_or(Transport::Unspecified)
        };

        // contract_id wins; else namespace+name path.
        let records = if !r.contract_id.is_empty() {
            self.registry.query("", &r.contract_id, transport).await
        } else {
            self.registry.query("", "", transport).await
        };

        let mut nodes = Vec::with_capacity(records.len());
        for rec in records {
            // namespace prefix filter (only when contract_id is empty).
            if r.contract_id.is_empty()
                && !r.namespace.is_empty()
                && !rec.namespace.starts_with(&r.namespace)
            {
                continue;
            }
            // name filter (only when contract_id is empty).
            if r.contract_id.is_empty() && !r.name.is_empty() {
                let matches = rec.interfaces.iter().any(|m| {
                    m.contract_id
                        .rsplit_once('/')
                        .map(|(_, leaf)| leaf == r.name)
                        .unwrap_or(false)
                });
                if !matches {
                    continue;
                }
            }
            nodes.push(record_to_node_info(rec, &r.transport));
        }
        Ok(Response::new(legacy::QueryNodesResponse { nodes }))
    }

    // ── Channel negotiation ────────────────────────────────────────────────
    // Translates to the new `ConnectCapability` / `DisconnectCapability`
    // calls so legacy callers get the same atlas-side bookkeeping the new
    // API does (channel record, provider-eviction cleanup).

    async fn negotiate_channel(
        &self,
        req: Request<legacy::NegotiateChannelRequest>,
    ) -> Result<Response<legacy::NegotiateChannelResponse>, Status> {
        let r = req.into_inner();
        // Legacy interface_name is just the contract_id leaf; resolve it to
        // the full contract_id by looking at what the provider declared.
        let recs = self
            .registry
            .query(&r.provider_node_id, "", Transport::Unspecified)
            .await;
        let rec = recs.first().ok_or_else(|| {
            Status::not_found(format!("unknown provider_node_id: {}", r.provider_node_id))
        })?;
        let transport = match r.transport.as_str() {
            "grpc" => Transport::Grpc,
            "ros2" => Transport::Ros2,
            "mcp" => Transport::Mcp,
            other => {
                return Err(Status::invalid_argument(format!(
                    "unknown legacy transport '{other}' (expected grpc | ros2 | mcp)"
                )));
            }
        };
        let contract_id = rec
            .interfaces
            .iter()
            .find(|m| {
                m.transport == transport as i32
                    && m.contract_id
                        .rsplit_once('/')
                        .map(|(_, leaf)| leaf == r.interface_name)
                        .unwrap_or(false)
            })
            .map(|m| m.contract_id.clone())
            .ok_or_else(|| {
                Status::not_found(format!(
                    "no interface '{}' over '{}' on '{}'",
                    r.interface_name, r.transport, r.provider_node_id
                ))
            })?;

        let (channel_id, endpoint, _params) = self
            .registry
            .connect(&r.consumer_id, &r.provider_node_id, &contract_id, transport)
            .await?;
        Ok(Response::new(legacy::NegotiateChannelResponse {
            channel_id,
            transport: r.transport,
            endpoint,
            metadata_json: String::new(),
        }))
    }

    async fn release_channel(
        &self,
        req: Request<legacy::ReleaseChannelRequest>,
    ) -> Result<Response<legacy::ReleaseChannelResponse>, Status> {
        let r = req.into_inner();
        let was_open = self.registry.disconnect(&r.channel_id).await;
        Ok(Response::new(legacy::ReleaseChannelResponse { ok: was_open }))
    }

    // ── Skill markdown ──────────────────────────────────────────────────────

    async fn query_skill_md(
        &self,
        req: Request<legacy::QuerySkillMdRequest>,
    ) -> Result<Response<legacy::QuerySkillMdResponse>, Status> {
        let r = req.into_inner();
        warn!(
            "[atlas-legacy] QuerySkillMd '{}' — DEPRECATED, migrate to Atlas.QueryCapabilityMd",
            r.node_id
        );
        let skill_md = self
            .registry
            .capability_md(&r.node_id)
            .await
            .unwrap_or_default();
        Ok(Response::new(legacy::QuerySkillMdResponse { skill_md }))
    }

    async fn query_all_skills(
        &self,
        _req: Request<legacy::QueryAllSkillsRequest>,
    ) -> Result<Response<legacy::QueryAllSkillsResponse>, Status> {
        warn!("[atlas-legacy] QueryAllSkills — DEPRECATED, agents now read CAPABILITY.md per-cap");
        let recs = self.registry.snapshot_for_legacy().await;
        let mut skills = Vec::with_capacity(recs.len());
        for rec in recs {
            if rec.capability_md_path.is_empty() {
                continue;
            }
            let md = self
                .registry
                .capability_md(&rec.capability_id)
                .await
                .unwrap_or_default();
            skills.push(legacy::SkillEntry {
                node_id: rec.capability_id,
                namespace: rec.namespace,
                kind: "service".to_string(),
                skill_md: md,
                skills: vec![],
            });
        }
        Ok(Response::new(legacy::QueryAllSkillsResponse { skills }))
    }

    async fn inspect_runtime(
        &self,
        _req: Request<legacy::InspectRuntimeRequest>,
    ) -> Result<Response<legacy::InspectRuntimeResponse>, Status> {
        warn!("[atlas-legacy] InspectRuntime — DEPRECATED, migrate to Atlas.InspectAtlas");
        let json = self.registry.inspect_json().await?;
        Ok(Response::new(legacy::InspectRuntimeResponse { json }))
    }
}

// ── Field translation helpers ───────────────────────────────────────────────

fn parse_transport_str(s: &str) -> Result<Transport, Status> {
    match s {
        "grpc" => Ok(Transport::Grpc),
        "ros2" => Ok(Transport::Ros2),
        "mcp" => Ok(Transport::Mcp),
        other => Err(Status::invalid_argument(format!(
            "legacy shim: transport '{other}' not supported by new API"
        ))),
    }
}

fn transport_int_to_str(t: i32) -> &'static str {
    match Transport::try_from(t).unwrap_or(Transport::Unspecified) {
        Transport::Grpc => "grpc",
        Transport::Ros2 => "ros2",
        Transport::Mcp => "mcp",
        Transport::Unspecified => "",
    }
}

/// Old `metadata_json` was opaque; vlm_service / other callers stuffed
/// transport-specific config into it. Best-effort parse into typed
/// `TransportParams`; missing keys → empty defaults.
fn build_legacy_params(transport: Transport, metadata_json: &str) -> pb::TransportParams {
    use pb::transport_params::Kind;
    let v: serde_json::Value = serde_json::from_str(metadata_json.trim()).unwrap_or_default();
    let kind = match transport {
        Transport::Grpc => {
            let c = v.get("contract").unwrap_or(&serde_json::Value::Null);
            Kind::Grpc(pb::GrpcParams {
                proto_file: c
                    .get("proto_file")
                    .and_then(|x| x.as_str())
                    .unwrap_or("")
                    .to_string(),
                service_name: c
                    .get("service")
                    .and_then(|x| x.as_str())
                    .unwrap_or("")
                    .to_string(),
                method: c
                    .get("streaming_rpc_method")
                    .or_else(|| c.get("rpc_method"))
                    .and_then(|x| x.as_str())
                    .unwrap_or("")
                    .to_string(),
            })
        }
        Transport::Ros2 => Kind::Ros2(pb::Ros2Params {
            qos_profile: v
                .get("qos_profile")
                .and_then(|x| x.as_str())
                .unwrap_or("")
                .to_string(),
        }),
        Transport::Mcp => {
            let schema = v.get("input_schema").or_else(|| v.get("input_schema_json"));
            Kind::Mcp(pb::McpParams {
                description: v
                    .get("description")
                    .and_then(|x| x.as_str())
                    .unwrap_or("")
                    .to_string(),
                input_schema_json: match schema {
                    Some(serde_json::Value::String(s)) => s.clone(),
                    Some(other) => other.to_string(),
                    None => String::new(),
                },
            })
        }
        Transport::Unspecified => Kind::Grpc(pb::GrpcParams::default()),
    };
    pb::TransportParams { kind: Some(kind) }
}

/// Project a `pb::CapabilityRecord` back into the legacy `NodeInfo` shape.
/// Interfaces are grouped by `contract_id`; one `InterfaceInfo` per group.
fn record_to_node_info(rec: pb::CapabilityRecord, transport_filter: &str) -> legacy::NodeInfo {
    use std::collections::BTreeMap;

    let mut by_contract: BTreeMap<String, Vec<i32>> = BTreeMap::new();
    for m in &rec.interfaces {
        if !transport_filter.is_empty() && transport_int_to_str(m.transport) != transport_filter {
            continue;
        }
        by_contract
            .entry(m.contract_id.clone())
            .or_default()
            .push(m.transport);
    }

    let interfaces: Vec<legacy::InterfaceInfo> = by_contract
        .into_iter()
        .map(|(contract_id, transports)| {
            let name = contract_id
                .rsplit_once('/')
                .map(|(_, leaf)| leaf.to_string())
                .unwrap_or_else(|| contract_id.clone());
            legacy::InterfaceInfo {
                name,
                supported_transports: transports
                    .iter()
                    .map(|t| transport_int_to_str(*t).to_string())
                    .collect(),
                metadata_json: String::new(),
                contract_id,
            }
        })
        .collect();

    legacy::NodeInfo {
        node_id: rec.capability_id,
        namespace: rec.namespace,
        kind: "service".to_string(),
        interfaces,
        has_skill_md: !rec.capability_md_path.is_empty(),
        distro: String::new(),
        container_id: String::new(),
        last_heartbeat_ms: rec.last_heartbeat_ms,
        skills: vec![],
    }
}
