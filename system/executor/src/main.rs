// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// robonix-executor — capability-call dispatch runtime.
// On startup executor:
//   1. Connects to atlas, registers as `com.robonix.system.executor`.
//   2. Declares its gRPC Execute capability (Plan → CapabilityCallEvent stream).
//   3. Declares 5 built-in capabilities under `robonix/system/executor/builtin/<op>`
//      so pilot's atlas-driven discovery surfaces them to the LLM as plain
//      capabilities. Calls hitting these contracts short-circuit to in-process
//      handlers in `dispatch::builtin` — no MCP loopback.
//   4. Serves Execute on `listen`. Per-call dispatch resolves provider via
//      `ConnectCapability(provider_id, contract_id, MCP)` on atlas.

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
use pb::contracts::robonix_system_executor_server::RobonixSystemExecutorServer;
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

    // Sentinel POC — load rules from SENTINEL_RULES_FILE before serving any
    // dispatch. Failures are non-fatal (allow-all fallback); see dispatch/sentinel.rs.
    dispatch::sentinel::init_from_env();

    let cfg = ExecutorConfig::resolve(parsed)?;

    info!("connecting to atlas at {}", cfg.atlas_endpoint);
    let mut atlas =
        AtlasClient::connect_with_retry(&cfg.atlas_endpoint, 10, Duration::from_secs(2))
            .await
            .context("connect to atlas")?;

    atlas
        .register_service(&cfg.id, EXECUTOR_NAMESPACE, "")
        .await?;
    info!("registered as '{}' under '{EXECUTOR_NAMESPACE}'", cfg.id);

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
        .declare_capability(
            &cfg.id,
            "robonix/system/executor",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/executor.v1.toml",
                "robonix.contracts.RobonixSystemExecutor",
                "/robonix.contracts.RobonixSystemExecutor/Execute",
            ),
        )
        .await?;

    // Built-in capabilities: declared as MCP-transport capabilities so pilot's
    // catalog discovery sees them like any user MCP provider. The endpoint is a
    // sentinel — dispatch never dials it; calls hitting these contracts hit
    // the provider_id == self short-circuit in `dispatch::dispatch`.
    let builtin_endpoint = format!("internal://{}/builtin", cfg.id);
    for spec in BUILTINS {
        let contract_id = format!("{EXECUTOR_NAMESPACE}/builtin/{}", spec.op);
        atlas
            .declare_capability_with_description(
                &cfg.id,
                &contract_id,
                atlas_pb::Transport::Mcp,
                &builtin_endpoint,
                atlas_client::mcp_params(spec.input_schema_json),
                spec.description,
            )
            .await
            .with_context(|| format!("declare builtin '{}'", contract_id))?;
    }
    info!(
        "declared RobonixSystemExecutor + {} builtin capabilities at {advertised}",
        BUILTINS.len()
    );

    // Executor has no Driver lifecycle handshake — it's ready as soon as
    // the gRPC server is up. Push ACTIVE so `rbnx caps` doesn't show the
    // legacy-fallback INACTIVE forever.
    if let Err(e) = atlas
        .set_lifecycle_state(&cfg.id, atlas_pb::LifecycleState::StateActive, "")
        .await
    {
        log::warn!("SetLifecycleState(ACTIVE) failed: {e:#}");
    }

    // Atlas evicts providers after ~60s without a heartbeat. Send one every
    // 20s so we stay registered for the lifetime of the process.
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

    let svc = ExecutorServiceImpl::new(atlas, cfg.id.clone());
    info!("RobonixSystemExecutor gRPC on {listen_addr}");
    eprintln!("robonix-executor ready on {listen_addr}");

    tonic::transport::Server::builder()
        .add_service(RobonixSystemExecutorServer::new(svc))
        .serve(listen_addr)
        .await
        .context("executor gRPC server failed")?;

    Ok(())
}
