// SPDX-License-Identifier: MulanPSL-2.0

use anyhow::{Context, Result};
use clap::Parser;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use robonix_soma::config::{Args, SomaConfig};
use robonix_soma::deployment::DeploymentStore;
use robonix_soma::launcher::PackageLauncher;
use robonix_soma::pb::contracts::{
    robonix_system_soma_get_urdf_server::RobonixSystemSomaGetUrdfServer,
    robonix_system_soma_get_yaml_server::RobonixSystemSomaGetYamlServer,
};
use robonix_soma::service::SomaService;
use robonix_soma::store::SomaStore;
use robonix_soma::supervisor::PackageSupervisor;
use robonix_soma::{GET_URDF_CONTRACT, GET_YAML_CONTRACT, SOMA_NAMESPACE};
use std::sync::Arc;
use std::time::Duration;

const GET_YAML_TOML: &str = "capabilities/system/soma/get_yaml.v1.toml";
const GET_URDF_TOML: &str = "capabilities/system/soma/get_urdf.v1.toml";

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();

    let config = SomaConfig::resolve(args).context("resolve Soma config")?;
    let deployments = DeploymentStore::load(&config).context("load deployment manifests")?;
    let store = Arc::new(SomaStore::load(&config).context("load Soma YAML/URDF data")?);

    let launcher = PackageLauncher::new(config.rbnx_bin.clone(), config.atlas_endpoint.clone());
    let mut supervisor = PackageSupervisor::new();
    let startup_report =
        supervisor.start_from_deployments(&deployments, &launcher, config.start_packages);
    startup_report.print_to_terminal();

    let listen_addr: std::net::SocketAddr = config
        .listen
        .parse()
        .with_context(|| format!("invalid Soma listen address '{}'", config.listen))?;
    let advertised = format!("127.0.0.1:{}", listen_addr.port());
    let atlas_http = normalize_endpoint(&config.atlas_endpoint);

    let mut atlas = AtlasClient::connect_with_retry(&atlas_http, 10, Duration::from_secs(2))
        .await
        .context("connect to Atlas")?;
    atlas
        .register_service(&config.provider_id, SOMA_NAMESPACE, "")
        .await
        .context("register Soma service")?;
    atlas
        .declare_capability(
            &config.provider_id,
            GET_YAML_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                GET_YAML_TOML,
                "robonix.contracts.RobonixSystemSomaGetYaml",
                "/robonix.contracts.RobonixSystemSomaGetYaml/GetYaml",
            ),
        )
        .await
        .context("declare Soma get_yaml gRPC capability")?;
    atlas
        .declare_capability(
            &config.provider_id,
            GET_URDF_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                GET_URDF_TOML,
                "robonix.contracts.RobonixSystemSomaGetUrdf",
                "/robonix.contracts.RobonixSystemSomaGetUrdf/GetUrdf",
            ),
        )
        .await
        .context("declare Soma get_urdf gRPC capability")?;
    if let Err(e) = atlas
        .set_lifecycle_state(
            &config.provider_id,
            atlas_pb::LifecycleState::StateActive,
            "",
        )
        .await
    {
        eprintln!("SetLifecycleState(ACTIVE) failed: {e:#}");
    }
    {
        let mut hb = atlas.clone();
        let provider_id = config.provider_id.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await;
            loop {
                tick.tick().await;
                if let Err(e) = hb.heartbeat(&provider_id).await {
                    eprintln!("heartbeat failed: {e:#}");
                }
            }
        });
    }

    let svc = Arc::new(SomaService::new(store));
    eprintln!(
        "robonix-soma ready on {}  (robots loaded, provider_id={})",
        config.listen, config.provider_id
    );
    tonic::transport::Server::builder()
        .add_service(RobonixSystemSomaGetYamlServer::from_arc(Arc::clone(&svc)))
        .add_service(RobonixSystemSomaGetUrdfServer::from_arc(svc))
        .serve(listen_addr)
        .await?;
    Ok(())
}

fn normalize_endpoint(endpoint: &str) -> String {
    if endpoint.starts_with("http") {
        endpoint.to_string()
    } else {
        format!("http://{endpoint}")
    }
}
