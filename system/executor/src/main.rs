// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// robonix-executor — capability-call dispatch runtime.
// On startup executor:
//   1. Connects to atlas, registers as `com.robonix.system.executor`.
//   2. Declares its gRPC Execute and CancelAllPlans capabilities.
//   3. Declares built-in capabilities under `robonix/system/executor/builtin/<op>`
//      so pilot's atlas-driven discovery surfaces them to the LLM as plain
//      capabilities. Calls hitting these contracts short-circuit to in-process
//      handlers in `dispatch::builtin` — no MCP loopback.
//   4. Serves Execute on `listen`. Per-call dispatch resolves provider via
//      `ConnectCapability(provider_id, contract_id, MCP)` on atlas.

mod active_plans;
mod capability_execution;
mod config;
mod dispatch;
mod execution_context;
mod expression;
mod grpc_server;
mod lifecycle;
mod pb;
mod plan_input;
mod plan_scheduler;
mod rtdl_events;
mod verification;

// Preserve the unchanged dispatch module's imports during this reorganization.
use active_plans as plan_runtime;
use rtdl_events as rtdl_wire;

use anyhow::{Context, Result};
use clap::Parser;
use config::{Args, EXECUTOR_NAMESPACE, ExecutorConfig};
use dispatch::builtin::BUILTINS;
use grpc_server::ExecutorServiceImpl;
use lifecycle::{ExecutorLifecycleDriver, SHARED_DRIVER_CONTRACT};
use pb::contracts::robonix_lifecycle_driver_server::RobonixLifecycleDriverServer;
use pb::contracts::robonix_system_executor_cancel_all_plans_server::RobonixSystemExecutorCancelAllPlansServer;
use pb::contracts::robonix_system_executor_control_plan_server::RobonixSystemExecutorControlPlanServer;
use pb::contracts::robonix_system_executor_execute_server::RobonixSystemExecutorExecuteServer;
use pb::contracts::robonix_system_executor_execute_stream_server::RobonixSystemExecutorExecuteStreamServer;
use pb::contracts::robonix_system_executor_get_health_server::RobonixSystemExecutorGetHealthServer;
use pb::contracts::robonix_system_executor_list_active_plans_server::RobonixSystemExecutorListActivePlansServer;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{info, warn};
use std::sync::Arc;
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<()> {
    let parsed = Args::parse();
    // Apply the manifest's per-component `log:` level (delivered inside
    // --config-json) to scribe's file sink before the first log line.
    robonix_scribe::init_from_config("executor", parsed.config_json.as_deref());
    info!("robonix-executor starting");

    let cfg = ExecutorConfig::resolve(parsed)?;

    info!("connecting to atlas at {}", cfg.atlas_endpoint);
    let mut atlas =
        AtlasClient::connect_with_retry(&cfg.atlas_endpoint, 10, Duration::from_secs(2))
            .await
            .context("connect to atlas")?;

    let listen_addr: std::net::SocketAddr = cfg
        .listen
        .parse()
        .with_context(|| format!("invalid executor listen address '{}'", cfg.listen))?;
    // Own the port before self-dialing, so startup cannot reach another server.
    let incoming = tonic::transport::server::TcpIncoming::bind(listen_addr)
        .with_context(|| format!("bind executor gRPC at {listen_addr}"))?
        .with_nodelay(Some(true));
    let listen_addr = incoming.local_addr().context("executor listener address")?;
    let advertised = lifecycle::advertised_endpoint(listen_addr);
    atlas
        .register_service(&cfg.id, EXECUTOR_NAMESPACE, "")
        .await?;
    info!("registered as '{}' under '{EXECUTOR_NAMESPACE}'", cfg.id);
    atlas
        .declare_capability(
            &cfg.id,
            SHARED_DRIVER_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/lifecycle/driver.v1.toml",
                "robonix.contracts.RobonixLifecycleDriver",
                "/robonix.contracts.RobonixLifecycleDriver/Driver",
            ),
        )
        .await
        .context("declare Executor shared lifecycle Driver")?;

    // Execute RPC: pilot → executor for plan dispatch.
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/system/executor/execute",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/executor/execute.v1.toml",
                "robonix.contracts.RobonixSystemExecutorExecute",
                "/robonix.contracts.RobonixSystemExecutorExecute/Execute",
            ),
        )
        .await?;

    atlas
        .declare_capability(
            &cfg.id,
            "robonix/system/executor/execute_stream",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/executor/execute_stream.v1.toml",
                "robonix.contracts.RobonixSystemExecutorExecuteStream",
                "/robonix.contracts.RobonixSystemExecutorExecuteStream/ExecuteStream",
            ),
        )
        .await?;

    // Out-of-band RTDL meta operations. These never enter PlanRuntime as a
    // new plan, so canceling work cannot create a self-referential cancel tree.
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/system/executor/control_plan",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/executor/control_plan.v1.toml",
                "robonix.contracts.RobonixSystemExecutorControlPlan",
                "/robonix.contracts.RobonixSystemExecutorControlPlan/ControlPlan",
            ),
        )
        .await?;

    // Read-only control path for clients and observability. Polling it must not
    // create an RTDL query plan of its own.
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/system/executor/list_active_plans",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/executor/list_active_plans.v1.toml",
                "robonix.contracts.RobonixSystemExecutorListActivePlans",
                "/robonix.contracts.RobonixSystemExecutorListActivePlans/ListActivePlans",
            ),
        )
        .await?;

    // CancelAllPlans RPC: control path for cancelling every active RTDL plan.
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/system/executor/cancel_all_plans",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/executor/cancel_all_plans.v1.toml",
                "robonix.contracts.RobonixSystemExecutorCancelAllPlans",
                "/robonix.contracts.RobonixSystemExecutorCancelAllPlans/CancelAll",
            ),
        )
        .await?;

    // Module health RPC: Vitals polls this for system-module health.
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/system/executor/get_health",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/executor/get_health.toml",
                "robonix.contracts.RobonixSystemExecutorGetHealth",
                "/robonix.contracts.RobonixSystemExecutorGetHealth/GetModuleHealth",
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
        "declared executor gRPC capabilities + {} builtin capabilities at {advertised}",
        BUILTINS.len()
    );

    let verification = Arc::new(verification::VerificationPolicy::new(
        cfg.verification.overlap,
        cfg.verification.rules,
    ));
    info!(
        "loaded {} executor verification rule(s)",
        verification.len()
    );
    let svc = ExecutorServiceImpl::new(atlas.clone(), cfg.id.clone(), verification);
    svc.runtime
        .close_admission()
        .await
        .map_err(anyhow::Error::msg)?;
    let lifecycle =
        ExecutorLifecycleDriver::new(atlas.clone(), cfg.id.clone(), svc.runtime.clone());
    let server_shutdown = lifecycle.subscribe_shutdown();
    let server_lifecycle = lifecycle.clone();
    let mut server_task = tokio::spawn(async move {
        tonic::transport::Server::builder()
            .add_service(RobonixLifecycleDriverServer::new(server_lifecycle))
            .add_service(RobonixSystemExecutorExecuteServer::new(svc.clone()))
            .add_service(RobonixSystemExecutorExecuteStreamServer::new(svc.clone()))
            .add_service(RobonixSystemExecutorCancelAllPlansServer::new(svc.clone()))
            .add_service(RobonixSystemExecutorControlPlanServer::new(svc.clone()))
            .add_service(RobonixSystemExecutorListActivePlansServer::new(svc.clone()))
            .add_service(RobonixSystemExecutorGetHealthServer::new(svc))
            .serve_with_incoming_shutdown(incoming, lifecycle::wait_for_shutdown(server_shutdown))
            .await
    });
    tokio::select! {
        result = lifecycle::initialize_and_activate(&advertised) => result?,
        result = &mut server_task => {
            result.context("join Executor gRPC server")?
                .context("Executor gRPC server failed before readiness")?;
            anyhow::bail!("Executor gRPC server stopped before readiness");
        }
    }

    // Atlas evicts providers after ~60s without a heartbeat. Send one every
    // 20s so we stay registered for the lifetime of the process.
    {
        let mut hb = atlas.clone();
        let provider_id = cfg.id.clone();
        let shutdown = lifecycle::wait_for_shutdown(lifecycle.subscribe_shutdown());
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await;
            tokio::pin!(shutdown);
            loop {
                tokio::select! {
                    _ = &mut shutdown => break,
                    _ = tick.tick() => {
                        if let Err(e) = hb.heartbeat(&provider_id).await {
                            warn!("heartbeat failed: {e:#}");
                        }
                    }
                }
            }
        });
    }

    info!("executor gRPC on {listen_addr}");
    info!("robonix-executor ready on {listen_addr}");

    let result = tokio::select! {
        result = &mut server_task => result,
        _ = lifecycle::wait_for_shutdown(lifecycle.subscribe_shutdown()) => {
            // Completed plans can leave unread responses blocked by a client's flow control.
            match tokio::time::timeout(Duration::from_secs(5), &mut server_task).await {
                Ok(result) => result,
                Err(_) => {
                    server_task.abort();
                    let _ = server_task.await;
                    anyhow::bail!("Executor gRPC shutdown timed out draining connections");
                }
            }
        }
    };
    result
        .context("join Executor gRPC server")?
        .context("executor gRPC server failed")?;

    Ok(())
}
