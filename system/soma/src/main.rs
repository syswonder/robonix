// SPDX-License-Identifier: MulanPSL-2.0

use anyhow::{Context, Result};
use clap::Parser;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{info, warn};
use robonix_soma::config::{Args, SomaConfig};
use robonix_soma::deployment::DeploymentStore;
use robonix_soma::launcher::PackageLauncher;
use robonix_soma::pb::contracts::{
    robonix_system_soma_get_urdf_server::RobonixSystemSomaGetUrdfServer,
    robonix_system_soma_get_yaml_server::RobonixSystemSomaGetYamlServer,
};
use robonix_soma::service::SomaService;
use robonix_soma::store::SomaStore;
use robonix_soma::{GET_URDF_CONTRACT, GET_YAML_CONTRACT, SOMA_NAMESPACE};
use std::sync::Arc;
use std::time::Duration;
use tokio::signal::unix::{SignalKind, signal};

const GET_YAML_TOML: &str = "capabilities/system/soma/get_yaml.v1.toml";
const GET_URDF_TOML: &str = "capabilities/system/soma/get_urdf.v1.toml";

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    if let Some(level) = args.log.as_deref() {
        unsafe {
            std::env::set_var("SCRIBE_FILE_LEVEL", level);
        }
    }
    robonix_scribe::init("soma");
    info!("robonix-soma starting");

    let config = SomaConfig::resolve(args).context("resolve Soma config")?;
    let deployments = DeploymentStore::load(&config).context("load deployment manifests")?;
    let store = Arc::new(SomaStore::load(&config).context("load Soma YAML/URDF data")?);

    let mut launcher = PackageLauncher::new(
        std::env::var("SCRIBE_LOG_DIR")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|_| std::path::PathBuf::from("./logs")),
        config.rbnx_bin.clone(),
        config.atlas_endpoint.clone(),
    )
    .context("create package process manager")?;
    let startup_report = launcher
        .start_from_deployments(&deployments, config.start_packages)
        .await;
    startup_report.print_to_terminal();
    if startup_report.has_failures() {
        launcher.stop_all().await?;
        anyhow::bail!("Soma package startup failed");
    }

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
    atlas
        .set_lifecycle_state(
            &config.provider_id,
            atlas_pb::LifecycleState::StateActive,
            "",
        )
        .await
        .context("set Soma lifecycle ACTIVE")?;
    let heartbeat_failure = {
        let mut hb = atlas.clone();
        let provider_id = config.provider_id.clone();
        let (tx, rx) = tokio::sync::oneshot::channel();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            let mut failures = 0usize;
            let mut tx = Some(tx);
            tick.tick().await;
            loop {
                tick.tick().await;
                if let Err(e) = hb.heartbeat(&provider_id).await {
                    failures += 1;
                    warn!("heartbeat failed ({failures}/3): {e:#}");
                    if failures >= 3 {
                        if let Some(tx) = tx.take() {
                            let _ = tx.send(());
                        }
                        break;
                    }
                } else {
                    failures = 0;
                }
            }
        });
        rx
    };

    let svc = Arc::new(SomaService::new(store));
    info!(
        "robonix-soma ready on {}  (robots loaded, provider_id={})",
        config.listen, config.provider_id
    );
    let shutdown = shutdown_signal(heartbeat_failure);
    tonic::transport::Server::builder()
        .add_service(RobonixSystemSomaGetYamlServer::from_arc(Arc::clone(&svc)))
        .add_service(RobonixSystemSomaGetUrdfServer::from_arc(svc))
        .serve_with_shutdown(listen_addr, shutdown)
        .await?;
    launcher.stop_all().await?;
    Ok(())
}

fn normalize_endpoint(endpoint: &str) -> String {
    if endpoint.starts_with("http") {
        endpoint.to_string()
    } else {
        format!("http://{endpoint}")
    }
}

async fn shutdown_signal(mut heartbeat_failure: tokio::sync::oneshot::Receiver<()>) {
    let mut sigint = signal(SignalKind::interrupt()).expect("install SIGINT handler");
    let mut sigterm = signal(SignalKind::terminate()).expect("install SIGTERM handler");
    tokio::select! {
        _ = sigint.recv() => {
            info!("received SIGINT");
        }
        _ = sigterm.recv() => {
            info!("received SIGTERM");
        }
        _ = &mut heartbeat_failure => {
            warn!("heartbeat failed repeatedly; shutting down Soma");
        }
    }
}
