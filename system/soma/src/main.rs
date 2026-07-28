// SPDX-License-Identifier: MulanPSL-2.0
//
// Robonix Soma — body description and live body-state service.
//
// rbnx is the deployment orchestrator and owns all primitive, service and
// skill package processes. Soma deliberately does not read a deployment
// manifest or spawn children.

use anyhow::{Context, Result};
use clap::Parser;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{info, warn};
use robonix_soma::config::{Args, SomaConfig};
use robonix_soma::pb::contracts::{
    robonix_system_soma_footprint_server::RobonixSystemSomaFootprintServer,
    robonix_system_soma_get_health_server::RobonixSystemSomaGetHealthServer,
    robonix_system_soma_get_urdf_server::RobonixSystemSomaGetUrdfServer,
    robonix_system_soma_get_yaml_server::RobonixSystemSomaGetYamlServer,
    robonix_system_soma_health_server::RobonixSystemSomaHealthServer,
};
use robonix_soma::service::SomaService;
use robonix_soma::store::SomaBody;
use robonix_soma::{
    GET_FOOTPRINT_CONTRACT, GET_HEALTH_CONTRACT, GET_URDF_CONTRACT, GET_YAML_CONTRACT,
    HEALTH_CONTRACT, SOMA_NAMESPACE,
};
use std::sync::Arc;
use std::time::Duration;
use tokio::signal::unix::{SignalKind, signal};

const GET_YAML_TOML: &str = "capabilities/system/soma/get_yaml.v1.toml";
const GET_URDF_TOML: &str = "capabilities/system/soma/get_urdf.v1.toml";
const GET_FOOTPRINT_TOML: &str = "capabilities/system/soma/footprint.v1.toml";
const GET_HEALTH_TOML: &str = "capabilities/system/soma/get_health.v1.toml";
const HEALTH_TOML: &str = "capabilities/system/soma/health.v1.toml";
#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    // Apply the manifest's per-component `log:` level (delivered inside
    // --config-json) to scribe's file sink before the first log line, so it
    // controls what this component persists; `rbnx logs --level` still
    // filters at read time.
    robonix_scribe::init_from_config("soma", args.config_json.as_deref());
    info!("robonix-soma starting");

    let config = SomaConfig::resolve(args).context("resolve Soma config")?;
    let body = Arc::new(SomaBody::load(&config.robot_yaml).context("load Soma YAML/URDF data")?);
    let log_dir = std::env::var("SCRIBE_LOG_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| std::path::PathBuf::from("./logs"));
    let atlas_http = normalize_endpoint(&config.atlas_endpoint);
    let mut atlas = AtlasClient::connect_with_retry(&atlas_http, 10, Duration::from_secs(2))
        .await
        .context("connect to Atlas")?;

    let listen_addr: std::net::SocketAddr = config
        .listen
        .parse()
        .with_context(|| format!("invalid Soma listen address '{}'", config.listen))?;
    {
        let probe = std::net::TcpListener::bind(listen_addr)
            .with_context(|| format!("bind Soma listen address {}", config.listen))?;
        drop(probe);
    }

    // Start the body API before Atlas registration so consumers never observe
    // Soma ACTIVE until its endpoint is actually accepting connections.
    let svc = Arc::new(SomaService::new(Arc::clone(&body)));
    let runtime_state = svc.runtime();
    let snapshot_service = Arc::clone(&svc);
    let snapshot_task = tokio::spawn(async move {
        let mut tick = tokio::time::interval(Duration::from_millis(500));
        let mut seq = 0_u64;
        loop {
            tick.tick().await;
            seq += 1;
            snapshot_service.publish_runtime_snapshot(seq).await;
        }
    });
    let (body_shutdown_tx, body_shutdown_rx) = tokio::sync::oneshot::channel();
    let mut body_server = tokio::spawn(async move {
        tonic::transport::Server::builder()
            .add_service(RobonixSystemSomaGetYamlServer::from_arc(Arc::clone(&svc)))
            .add_service(RobonixSystemSomaGetUrdfServer::from_arc(Arc::clone(&svc)))
            .add_service(RobonixSystemSomaFootprintServer::from_arc(Arc::clone(&svc)))
            .add_service(RobonixSystemSomaGetHealthServer::from_arc(Arc::clone(&svc)))
            .add_service(RobonixSystemSomaHealthServer::from_arc(svc))
            .serve_with_shutdown(listen_addr, async {
                let _ = body_shutdown_rx.await;
            })
            .await
    });
    wait_for_body_api(listen_addr).await?;
    info!("Soma body API ready on {}", config.listen);

    // Register only after the body API is reachable.
    let advertised = format!("127.0.0.1:{}", listen_addr.port());
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
        .declare_capability(
            &config.provider_id,
            GET_FOOTPRINT_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                GET_FOOTPRINT_TOML,
                "robonix.contracts.RobonixSystemSomaFootprint",
                "/robonix.contracts.RobonixSystemSomaFootprint/GetFootprint",
            ),
        )
        .await
        .context("declare Soma footprint gRPC capability")?;
    atlas
        .declare_capability(
            &config.provider_id,
            GET_HEALTH_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                GET_HEALTH_TOML,
                "robonix.contracts.RobonixSystemSomaGetHealth",
                "/robonix.contracts.RobonixSystemSomaGetHealth/GetHealth",
            ),
        )
        .await
        .context("declare Soma get_health gRPC capability")?;
    atlas
        .declare_capability(
            &config.provider_id,
            HEALTH_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                HEALTH_TOML,
                "robonix.contracts.RobonixSystemSomaHealth",
                "/robonix.contracts.RobonixSystemSomaHealth/StreamHealth",
            ),
        )
        .await
        .context("declare Soma health stream gRPC capability")?;
    atlas
        .set_lifecycle_state(
            &config.provider_id,
            atlas_pb::LifecycleState::StateActive,
            "",
        )
        .await
        .context("set Soma lifecycle ACTIVE")?;
    let runtime_dir = log_dir.join("soma-runtime");
    let runtime_monitor = match robonix_soma::runtime_monitor::start(
        &mut atlas,
        &config.provider_id,
        runtime_state,
        &runtime_dir,
    )
    .await
    {
        Ok(monitor) => monitor,
        Err(error) => {
            warn!("[soma/state] runtime monitor unavailable: {error:#}");
            None
        }
    };
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

    info!(
        "robonix-soma ready on {}  (robot loaded, provider_id={})",
        config.listen, config.provider_id
    );
    let serve_result: Result<()> = tokio::select! {
        _ = shutdown_signal(heartbeat_failure) => {
            let _ = body_shutdown_tx.send(());
            body_server
                .await
                .context("join Soma body API task")?
                .context("serve Soma body API")
        }
        result = &mut body_server => {
            match result {
                Ok(Ok(())) => Err(anyhow::anyhow!("Soma body API stopped unexpectedly")),
                Ok(Err(e)) => Err(e).context("serve Soma body API"),
                Err(e) => Err(e).context("join Soma body API task"),
            }
        }
    };
    snapshot_task.abort();
    if let Some(runtime_monitor) = runtime_monitor {
        runtime_monitor.shutdown(&mut atlas).await;
    }
    serve_result?;
    Ok(())
}

async fn wait_for_body_api(listen_addr: std::net::SocketAddr) -> Result<()> {
    let dial_addr = std::net::SocketAddr::from(([127, 0, 0, 1], listen_addr.port()));
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    loop {
        match tokio::net::TcpStream::connect(dial_addr).await {
            Ok(_) => return Ok(()),
            Err(_) if tokio::time::Instant::now() < deadline => {
                tokio::time::sleep(Duration::from_millis(50)).await;
            }
            Err(e) => {
                return Err(e).with_context(|| format!("wait for Soma body API at {dial_addr}"));
            }
        }
    }
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
