// SPDX-License-Identifier: MulanPSL-2.0
// robonix-pilot — reasoning, planning, and session management.
//
// Run standalone (manual smoke testing):
//   robonix-pilot --vlm-upstream https://api.openai.com/v1 \
//                 --vlm-api-key sk-... \
//                 --vlm-model gpt-4o-mini
//   # atlas defaults to 127.0.0.1:50051; everything else has sane defaults.
//
// Run under rbnx:
//   ROBONIX_ATLAS_ENDPOINT=… ROBONIX_CONFIG_PATH=/run/robonix/pilot.yaml robonix-pilot
//
// Either way, on startup pilot:
//   1. Connects to atlas, registers its capability, declares the
//      `robonix/system/pilot` gRPC interface.
//   2. Constructs an embedded LLM client from the resolved VLM config.
//   3. Serves SystemPilot on `listen`. Executor address is discovered
//      through atlas at every Stream RPC, not configured statically.

mod config;
mod pb;
mod planner;
mod service;
mod vlm;

use anyhow::{Context, Result};
use clap::Parser;
use config::{Args, PILOT_NAMESPACE, PilotConfig};
use log::info;
use pb::contracts::system_pilot_server::SystemPilotServer;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use service::PilotServiceImpl;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("robonix_pilot=info"),
    )
    .init();

    let cfg = PilotConfig::resolve(Args::parse())?;

    info!("connecting to atlas at {}", cfg.atlas_endpoint);
    let mut atlas =
        AtlasClient::connect_with_retry(&cfg.atlas_endpoint, 10, Duration::from_secs(2))
            .await
            .context("connect to atlas")?;

    atlas
        .register_capability(&cfg.capability_id, PILOT_NAMESPACE, "")
        .await?;
    info!(
        "registered as '{}' under '{PILOT_NAMESPACE}'",
        cfg.capability_id
    );

    let listen_addr: std::net::SocketAddr = cfg
        .listen
        .parse()
        .with_context(|| format!("invalid pilot listen address '{}'", cfg.listen))?;
    // Use 127.0.0.1 (not "localhost") in the advertised endpoint so consumers
    // don't resolve ::1 while we listen on IPv4.
    let advertised = match listen_addr.ip() {
        std::net::IpAddr::V4(ip) if ip.is_unspecified() => {
            format!("127.0.0.1:{}", listen_addr.port())
        }
        _ => listen_addr.to_string(),
    };
    atlas
        .declare_interface(
            &cfg.capability_id,
            "robonix/system/pilot",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/pilot.v1.toml",
                "robonix.contracts.SystemPilot",
                "/robonix.contracts.SystemPilot/Stream",
            ),
        )
        .await?;
    info!("declared SystemPilot gRPC at {advertised}");

    let vlm = vlm::VlmClient::new(&cfg.vlm);
    info!(
        "VLM upstream='{}' model='{}'",
        cfg.vlm.upstream, cfg.vlm.model
    );

    let atlas = Arc::new(Mutex::new(atlas));
    let svc = PilotServiceImpl::new(atlas, vlm);

    info!("SystemPilot gRPC on {listen_addr}");
    eprintln!("robonix-pilot ready on {listen_addr}");

    tonic::transport::Server::builder()
        .add_service(SystemPilotServer::new(svc))
        .serve(listen_addr)
        .await
        .context("pilot gRPC server failed")?;

    Ok(())
}
