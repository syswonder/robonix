// SPDX-License-Identifier: MulanPSL-2.0
// robonix-executor — tool-call dispatch runtime.
//
// Bootstrap (every Robonix process):
//   ROBONIX_ATLAS_ENDPOINT  — atlas control-plane host:port (CLI / env)
//   ROBONIX_CONFIG_PATH     — optional YAML config slice (rbnx-written)
//
// On startup executor:
//   1. Connects to atlas, registers as `com.robonix.system.executor`.
//   2. Declares two contract interfaces over gRPC:
//        - robonix/system/executor              (Stream — TaskGraph dispatch)
//        - robonix/system/executor/list_tools   (Call   — tool catalogue)
//   3. Serves both on `listen`. Tool dispatch resolves MCP / gRPC / builtin
//      backends via atlas at every Stream RPC.

mod config;
mod dispatch;
mod exec_wire;
mod pb;
mod routing_kind;
mod service;
mod tools;

use anyhow::{Context, Result};
use clap::Parser;
use config::{Args, EXECUTOR_NAMESPACE, ExecutorConfig};
use log::info;
use pb::contracts::{
    system_executor_list_tools_server::SystemExecutorListToolsServer,
    system_executor_server::SystemExecutorServer,
};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use service::ExecutorServiceImpl;
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("robonix_executor=info"),
    )
    .init();

    let cfg = ExecutorConfig::resolve(Args::parse())?;

    info!("connecting to atlas at {}", cfg.atlas_endpoint);
    let mut atlas =
        AtlasClient::connect_with_retry(&cfg.atlas_endpoint, 10, Duration::from_secs(2))
            .await
            .context("connect to atlas")?;

    atlas
        .register_capability(&cfg.capability_id, EXECUTOR_NAMESPACE, "")
        .await?;
    info!(
        "registered as '{}' under '{EXECUTOR_NAMESPACE}'",
        cfg.capability_id
    );

    let listen_addr: std::net::SocketAddr = cfg
        .listen
        .parse()
        .with_context(|| format!("invalid executor listen address '{}'", cfg.listen))?;
    let advertised = match listen_addr.ip() {
        std::net::IpAddr::V4(ip) if ip.is_unspecified() => {
            format!("127.0.0.1:{}", listen_addr.port())
        }
        _ => listen_addr.to_string(),
    };

    // Executor exposes TWO contracts on the same gRPC server: stream-dispatch
    // and list_tools. Declare both so consumers (pilot) can connect to each.
    atlas
        .declare_interface(
            &cfg.capability_id,
            "robonix/system/executor",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/executor.v1.toml",
                "robonix.contracts.SystemExecutor",
                "/robonix.contracts.SystemExecutor/Stream",
            ),
        )
        .await?;
    atlas
        .declare_interface(
            &cfg.capability_id,
            "robonix/system/executor/list_tools",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/executor_list_tools.v1.toml",
                "robonix.contracts.SystemExecutorListTools",
                "/robonix.contracts.SystemExecutorListTools/Call",
            ),
        )
        .await?;
    info!("declared SystemExecutor + SystemExecutorListTools at {advertised}");

    let svc = ExecutorServiceImpl::new(atlas, cfg.capability_id.clone());
    info!("SystemExecutor gRPC on {listen_addr}");
    eprintln!("robonix-executor ready on {listen_addr}");

    tonic::transport::Server::builder()
        .add_service(SystemExecutorServer::new(svc.clone()))
        .add_service(SystemExecutorListToolsServer::new(svc))
        .serve(listen_addr)
        .await
        .context("executor gRPC server failed")?;

    Ok(())
}
