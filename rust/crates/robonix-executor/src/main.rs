// SPDX-License-Identifier: MulanPSL-2.0
// robonix-executor — capability-call dispatch runtime.
//
// On startup executor:
//   1. Connects to atlas, registers as `com.robonix.system.executor`.
//   2. Declares its gRPC Execute interface (Plan → CapabilityCallEvent stream).
//   3. Declares 5 built-in capabilities under `robonix/system/executor/builtin/<op>`
//      so pilot's atlas-driven discovery surfaces them to the LLM as plain
//      capabilities. Calls hitting these contracts short-circuit to in-process
//      handlers in `dispatch::builtin` — no MCP loopback.
//   4. Serves Execute on `listen`. Per-call dispatch resolves provider via
//      `ConnectCapability(cap_id, contract_id, MCP)` on atlas.

mod config;
mod dispatch;
mod exec_wire;
mod pb;
mod service;

use anyhow::{Context, Result};
use clap::Parser;
use config::{Args, EXECUTOR_NAMESPACE, ExecutorConfig};
use dispatch::builtin::BUILTINS;
use log::info;
use pb::contracts::system_executor_server::SystemExecutorServer;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use service::ExecutorServiceImpl;
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<()> {
    let parsed = Args::parse();
    let log_filter = parsed
        .log
        .clone()
        .or_else(|| std::env::var("RUST_LOG").ok())
        .unwrap_or_else(|| "robonix_executor=info".to_string());
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(log_filter)).init();

    let cfg = ExecutorConfig::resolve(parsed)?;

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

    // Execute RPC: pilot → executor for plan dispatch.
    atlas
        .declare_interface(
            &cfg.capability_id,
            "robonix/system/executor",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/executor.v1.toml",
                "robonix.contracts.SystemExecutor",
                "/robonix.contracts.SystemExecutor/Execute",
            ),
        )
        .await?;

    // Built-in capabilities: declared as MCP-transport interfaces so pilot's
    // catalog discovery sees them like any user MCP cap. The endpoint is a
    // sentinel — dispatch never dials it; calls hitting these contracts hit
    // the cap_id == self short-circuit in `dispatch::dispatch`.
    let builtin_endpoint = format!("internal://{}/builtin", cfg.capability_id);
    for spec in BUILTINS {
        let contract_id = format!("{EXECUTOR_NAMESPACE}/builtin/{}", spec.op);
        atlas
            .declare_interface(
                &cfg.capability_id,
                &contract_id,
                atlas_pb::Transport::Mcp,
                &builtin_endpoint,
                atlas_client::mcp_params(spec.description, spec.input_schema_json),
            )
            .await
            .with_context(|| format!("declare builtin '{}'", contract_id))?;
    }
    info!(
        "declared SystemExecutor + {} builtin capabilities at {advertised}",
        BUILTINS.len()
    );

    let svc = ExecutorServiceImpl::new(atlas, cfg.capability_id.clone());
    info!("SystemExecutor gRPC on {listen_addr}");
    eprintln!("robonix-executor ready on {listen_addr}");

    tonic::transport::Server::builder()
        .add_service(SystemExecutorServer::new(svc))
        .serve(listen_addr)
        .await
        .context("executor gRPC server failed")?;

    Ok(())
}
