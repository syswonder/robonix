// SPDX-License-Identifier: MulanPSL-2.0
//
// robonix-health-primitive — wraps robot sysfs/hwmon sensors behind unified gRPC contracts.
// On startup:
//   1. Connects to Atlas, registers as `health_primitive`.
//   2. Declares two gRPC capabilities:
//      - robonix/primitive/health/state  (rpc: GetHealthState)
//      - robonix/primitive/health/stream (server_stream: StreamHealthState)
//   3. Starts the sysfs collect loop.
//   4. Serves both gRPC services on port 50092.

mod config;
mod pb;
mod service;

use anyhow::{Context, Result};
use clap::Parser;
use config::{Args, HealthConfig, PROVIDER_NAMESPACE};
use log::info;
use pb::contracts::robonix_primitive_health_state_server::RobonixPrimitiveHealthStateServer;
use pb::contracts::robonix_primitive_health_stream_server::RobonixPrimitiveHealthStreamServer;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use service::HealthPrimitiveService;
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<()> {
    let parsed = Args::parse();
    let log_filter = parsed
        .log
        .clone()
        .or_else(|| std::env::var("RUST_LOG").ok())
        .unwrap_or_else(|| "robonix_health_primitive=info".to_string());
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(log_filter)).init();

    let cfg = HealthConfig::resolve(parsed)?;

    info!("connecting to Atlas at {}", cfg.atlas_endpoint);
    let mut atlas =
        AtlasClient::connect_with_retry(&cfg.atlas_endpoint, 10, Duration::from_secs(2))
            .await
            .context("connect to Atlas")?;

    atlas
        .register_primitive(&cfg.id, PROVIDER_NAMESPACE, "")
        .await?;
    info!("registered as '{}' under '{PROVIDER_NAMESPACE}'", cfg.id);

    let listen_addr: std::net::SocketAddr = cfg
        .listen
        .parse()
        .with_context(|| format!("invalid listen address '{}'", cfg.listen))?;
    let advertised = match listen_addr.ip() {
        std::net::IpAddr::V4(ip) if ip.is_unspecified() => {
            format!("127.0.0.1:{}", listen_addr.port())
        }
        _ => listen_addr.to_string(),
    };

    // Declare GetHealthState RPC.
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/primitive/health/state",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/primitive/health/state.v1.toml",
                "robonix.contracts.RobonixPrimitiveHealthState",
                "/robonix.contracts.RobonixPrimitiveHealthState/GetHealthState",
            ),
        )
        .await?;
    info!("declared GetHealthState at {advertised}");

    // Declare StreamHealthState server_stream.
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/primitive/health/stream",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/primitive/health/stream.v1.toml",
                "robonix.contracts.RobonixPrimitiveHealthStream",
                "/robonix.contracts.RobonixPrimitiveHealthStream/StreamHealthState",
            ),
        )
        .await?;
    info!("declared StreamHealthState at {advertised}");

    if let Err(e) = atlas
        .set_lifecycle_state(&cfg.id, atlas_pb::LifecycleState::StateActive, "")
        .await
    {
        log::warn!("SetLifecycleState(ACTIVE) failed: {e:#}");
    }

    // Heartbeat.
    {
        let mut hb = atlas.clone();
        let provider_id = cfg.id.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await;
            loop {
                tick.tick().await;
                if let Err(e) = hb.heartbeat(&provider_id).await {
                    log::warn!("heartbeat failed: {e:#}");
                }
            }
        });
    }

    let svc = HealthPrimitiveService::new();

    // Collect loop.
    {
        let svc = svc.clone();
        let interval = Duration::from_millis(cfg.collect_interval_ms);
        tokio::spawn(async move {
            let collector = match service::Collector::new() {
                Ok(c) => c,
                Err(e) => {
                    log::error!("collector init failed: {e:#}");
                    return;
                }
            };
            let mut tick = tokio::time::interval(interval);
            tick.tick().await;
            loop {
                tick.tick().await;
                let state = collector.collect();
                svc.update_state(state).await;
            }
        });
    }

    info!("Health primitive gRPC on {listen_addr}");
    eprintln!("robonix-health-primitive ready on {listen_addr}");

    tonic::transport::Server::builder()
        .add_service(RobonixPrimitiveHealthStateServer::new(svc.clone()))
        .add_service(RobonixPrimitiveHealthStreamServer::new(svc))
        .serve(listen_addr)
        .await
        .context("health primitive gRPC server failed")?;

    Ok(())
}
