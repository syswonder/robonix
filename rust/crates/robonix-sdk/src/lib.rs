mod proto {
    tonic::include_proto!("robonix.runtime");
}

use anyhow::{Context, Result};
use std::time::Duration;

pub struct InterfaceInfo {
    pub name: String,
    pub supported_transports: Vec<String>,
    pub metadata_json: String,
    /// Stable capability path (same as `QueryNodesOpts.contract_id` when matching).
    /// Example: `robonix/prm/camera/rgb`. Versioning is **not** part of this string — it lives
    /// in the contract TOML (`[contract] version`) and in generated IDL; discovery uses the
    /// stable path only.
    pub contract_id: String,
}

pub struct NodeInfo {
    pub node_id: String,
    pub namespace: String,
    pub kind: String,
    pub interfaces: Vec<InterfaceInfo>,
    pub has_skill_md: bool,
    pub skills: Vec<SkillInfoItem>,
    pub distro: String,
    pub container_id: String,
    pub last_heartbeat_ms: u64,
}

pub struct Channel {
    pub channel_id: String,
    pub transport: String,
    pub endpoint: String,
}

pub struct SkillInfoItem {
    pub name: String,
    pub description: String,
    pub path: String,
    /// JSON (e.g. `disable_model_invocation`).
    pub metadata_json: String,
}

pub struct SkillEntry {
    pub node_id: String,
    pub namespace: String,
    pub kind: String,
    pub skill_md: String,
    pub skills: Vec<SkillInfoItem>,
}

pub struct RobonixClient {
    inner: proto::robonix_runtime_client::RobonixRuntimeClient<tonic::transport::Channel>,
}

#[derive(Debug, Clone, Default)]
pub struct QueryNodesOpts {
    pub namespace: String,
    pub interface_name: String,
    pub transport: String,
    pub distro_prefix: String,
    pub container_id: String,
    /// When non-empty, server matches `InterfaceInfo.contract_id` exactly; `namespace` and `interface_name` are ignored.
    pub contract_id: String,
}

impl RobonixClient {
    pub async fn connect(endpoint: &str) -> Result<Self> {
        let inner =
            proto::robonix_runtime_client::RobonixRuntimeClient::connect(endpoint.to_owned())
                .await
                .context("failed to connect to robonix-atlas")?;
        Ok(Self { inner })
    }

    pub async fn connect_with_retry(
        endpoint: &str,
        attempts: usize,
        delay: Duration,
    ) -> Result<Self> {
        for i in 0..attempts {
            match Self::connect(endpoint).await {
                Ok(c) => return Ok(c),
                Err(e) if i + 1 < attempts => {
                    eprintln!("connect attempt {}/{attempts} failed: {e:#}", i + 1);
                    tokio::time::sleep(delay).await;
                }
                Err(e) => return Err(e),
            }
        }
        anyhow::bail!("connect_with_retry: no attempts requested")
    }

    pub async fn register_node(
        &mut self,
        node_id: impl Into<String>,
        namespace: impl Into<String>,
        kind: impl Into<String>,
        skill_md: impl Into<String>,
    ) -> Result<String> {
        self.register_node_full(node_id, namespace, kind, skill_md, "", "")
            .await
    }

    pub async fn register_node_full(
        &mut self,
        node_id: impl Into<String>,
        namespace: impl Into<String>,
        kind: impl Into<String>,
        skill_md: impl Into<String>,
        distro: impl Into<String>,
        container_id: impl Into<String>,
    ) -> Result<String> {
        self.register_node_with_skills(
            node_id,
            namespace,
            kind,
            skill_md,
            Vec::new(),
            distro,
            container_id,
        )
        .await
    }

    pub async fn register_node_with_skills(
        &mut self,
        node_id: impl Into<String>,
        namespace: impl Into<String>,
        kind: impl Into<String>,
        skill_md: impl Into<String>,
        skills: Vec<SkillInfoItem>,
        distro: impl Into<String>,
        container_id: impl Into<String>,
    ) -> Result<String> {
        let resp = self
            .inner
            .register_node(proto::RegisterNodeRequest {
                node_id: node_id.into(),
                namespace: namespace.into(),
                kind: kind.into(),
                skill_md: skill_md.into(),
                skills: skills
                    .into_iter()
                    .map(|s| proto::SkillInfo {
                        name: s.name,
                        description: s.description,
                        path: s.path,
                        metadata_json: s.metadata_json,
                    })
                    .collect(),
                distro: distro.into(),
                container_id: container_id.into(),
            })
            .await
            .context("register_node RPC failed")?;
        Ok(resp.into_inner().node_id)
    }

    pub async fn unregister_node(&mut self, node_id: impl Into<String>) -> Result<bool> {
        let resp = self
            .inner
            .unregister_node(proto::UnregisterNodeRequest {
                node_id: node_id.into(),
            })
            .await
            .context("unregister_node RPC failed")?;
        Ok(resp.into_inner().ok)
    }

    pub async fn node_heartbeat(&mut self, node_id: impl Into<String>) -> Result<u64> {
        let resp = self
            .inner
            .node_heartbeat(proto::NodeHeartbeatRequest {
                node_id: node_id.into(),
            })
            .await
            .context("node_heartbeat RPC failed")?;
        Ok(resp.into_inner().server_time_ms)
    }

    pub async fn declare_interface(
        &mut self,
        node_id: impl Into<String>,
        name: impl Into<String>,
        supported_transports: Vec<String>,
        metadata_json: impl Into<String>,
    ) -> Result<()> {
        self.declare_interface_with_listen_port(
            node_id,
            name,
            supported_transports,
            metadata_json,
            0,
        )
        .await
    }

    /// `listen_port`: 0 = let the server pick a port; non-zero = data plane listens on this port (must match a port you already bound).
    pub async fn declare_interface_with_listen_port(
        &mut self,
        node_id: impl Into<String>,
        name: impl Into<String>,
        supported_transports: Vec<String>,
        metadata_json: impl Into<String>,
        listen_port: u32,
    ) -> Result<()> {
        self.declare_interface_full(
            node_id,
            name,
            supported_transports,
            metadata_json,
            listen_port,
            "",
        )
        .await
    }

    /// Same as [`Self::declare_interface_with_listen_port`], plus optional `contract_id`
    /// (stable path e.g. `"robonix/prm/camera/rgb"`; empty = server derives from namespace+name).
    pub async fn declare_interface_full(
        &mut self,
        node_id: impl Into<String>,
        name: impl Into<String>,
        supported_transports: Vec<String>,
        metadata_json: impl Into<String>,
        listen_port: u32,
        contract_id: impl Into<String>,
    ) -> Result<()> {
        self.inner
            .declare_interface(proto::DeclareInterfaceRequest {
                node_id: node_id.into(),
                name: name.into(),
                supported_transports,
                metadata_json: metadata_json.into(),
                listen_port,
                contract_id: contract_id.into(),
            })
            .await
            .context("declare_interface RPC failed")?;
        Ok(())
    }

    pub async fn query_nodes(
        &mut self,
        namespace: impl Into<String>,
        name: impl Into<String>,
        transport: impl Into<String>,
    ) -> Result<Vec<NodeInfo>> {
        self.query_nodes_opts(QueryNodesOpts {
            namespace: namespace.into(),
            interface_name: name.into(),
            transport: transport.into(),
            ..Default::default()
        })
        .await
    }

    pub async fn query_nodes_opts(&mut self, opts: QueryNodesOpts) -> Result<Vec<NodeInfo>> {
        let resp = self
            .inner
            .query_nodes(proto::QueryNodesRequest {
                namespace: opts.namespace,
                name: opts.interface_name,
                transport: opts.transport,
                distro_prefix: opts.distro_prefix,
                container_id: opts.container_id,
                contract_id: opts.contract_id,
            })
            .await
            .context("query_nodes RPC failed")?;
        Ok(resp
            .into_inner()
            .nodes
            .into_iter()
            .map(|n| NodeInfo {
                node_id: n.node_id,
                namespace: n.namespace,
                kind: n.kind,
                interfaces: n
                    .interfaces
                    .into_iter()
                    .map(|i| InterfaceInfo {
                        name: i.name,
                        supported_transports: i.supported_transports,
                        metadata_json: i.metadata_json,
                        contract_id: i.contract_id,
                    })
                    .collect(),
                has_skill_md: n.has_skill_md,
                skills: n
                    .skills
                    .into_iter()
                    .map(|s| SkillInfoItem {
                        name: s.name,
                        description: s.description,
                        path: s.path,
                        metadata_json: s.metadata_json,
                    })
                    .collect(),
                distro: n.distro,
                container_id: n.container_id,
                last_heartbeat_ms: n.last_heartbeat_ms,
            })
            .collect())
    }

    pub async fn negotiate_channel(
        &mut self,
        consumer_id: impl Into<String>,
        provider_node_id: impl Into<String>,
        interface_name: impl Into<String>,
        transport: impl Into<String>,
    ) -> Result<Channel> {
        let resp = self
            .inner
            .negotiate_channel(proto::NegotiateChannelRequest {
                consumer_id: consumer_id.into(),
                provider_node_id: provider_node_id.into(),
                interface_name: interface_name.into(),
                transport: transport.into(),
            })
            .await
            .context("negotiate_channel RPC failed")?;
        let r = resp.into_inner();
        Ok(Channel {
            channel_id: r.channel_id,
            transport: r.transport,
            endpoint: r.endpoint,
        })
    }

    pub async fn release_channel(&mut self, channel_id: impl Into<String>) -> Result<()> {
        self.inner
            .release_channel(proto::ReleaseChannelRequest {
                channel_id: channel_id.into(),
            })
            .await
            .context("release_channel RPC failed")?;
        Ok(())
    }

    pub async fn query_skill_md(&mut self, node_id: impl Into<String>) -> Result<String> {
        let resp = self
            .inner
            .query_skill_md(proto::QuerySkillMdRequest {
                node_id: node_id.into(),
            })
            .await
            .context("query_skill_md RPC failed")?;
        Ok(resp.into_inner().skill_md)
    }

    pub async fn query_all_skills(&mut self) -> Result<Vec<SkillEntry>> {
        let resp = self
            .inner
            .query_all_skills(proto::QueryAllSkillsRequest {})
            .await
            .context("query_all_skills RPC failed")?;
        Ok(resp
            .into_inner()
            .skills
            .into_iter()
            .map(|s| SkillEntry {
                node_id: s.node_id,
                namespace: s.namespace,
                kind: s.kind,
                skill_md: s.skill_md,
                skills: s
                    .skills
                    .into_iter()
                    .map(|si| SkillInfoItem {
                        name: si.name,
                        description: si.description,
                        path: si.path,
                        metadata_json: si.metadata_json,
                    })
                    .collect(),
            })
            .collect())
    }

    pub async fn inspect_runtime(&mut self) -> Result<String> {
        let resp = self
            .inner
            .inspect_runtime(proto::InspectRuntimeRequest {})
            .await
            .context("inspect_runtime RPC failed")?;
        Ok(resp.into_inner().json)
    }
}
