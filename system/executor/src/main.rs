// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// robonix-executor — capability-call dispatch runtime.
// On startup Executor:
//   1. Registers the Executor and Sentinel providers with Atlas.
//   2. Declares their gRPC capabilities.
//   3. Declares built-in capabilities under `robonix/system/executor/builtin/<op>`
//      so pilot's atlas-driven discovery surfaces them to the LLM as plain
//      capabilities. Calls hitting these contracts short-circuit to in-process
//      handlers in `dispatch::builtin` — no MCP loopback.
//   4. Serves Execute on `listen`. Per-call dispatch resolves provider via
//      `ConnectCapability(provider_id, contract_id, MCP)` on atlas.

mod config;
mod dispatch;
mod pb;
mod plan_runtime;
mod rtdl_wire;
mod sentinel_runtime;
mod service;
mod verification;

use anyhow::{Context, Result};
use clap::Parser;
use config::{Args, EXECUTOR_NAMESPACE, ExecutorConfig};
use dispatch::builtin::BUILTINS;
use pb::contracts::robonix_system_executor_cancel_all_plans_server::RobonixSystemExecutorCancelAllPlansServer;
use pb::contracts::robonix_system_executor_control_plan_server::RobonixSystemExecutorControlPlanServer;
use pb::contracts::robonix_system_executor_execute_server::RobonixSystemExecutorExecuteServer;
use pb::contracts::robonix_system_executor_get_health_server::RobonixSystemExecutorGetHealthServer;
use pb::contracts::robonix_system_executor_list_active_plans_server::RobonixSystemExecutorListActivePlansServer;
use pb::contracts::robonix_system_sentinel_list_rules_server::RobonixSystemSentinelListRulesServer;
use pb::contracts::robonix_system_sentinel_replace_rules_server::RobonixSystemSentinelReplaceRulesServer;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{info, warn};
use sentinel_runtime::{
    SENTINEL_NAMESPACE, SENTINEL_PROVIDER_ID, SentinelRuntime, SentinelServiceImpl,
};
use service::ExecutorServiceImpl;
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
    config::bootstrap_sentinel_rules(&cfg.sentinel_rules, cfg.sentinel_rules_seed.as_deref())?;
    let sentinel = SentinelRuntime::load(cfg.sentinel_rules.clone())
        .with_context(|| format!("load Sentinel rules '{}'", cfg.sentinel_rules.display()))?;

    info!("connecting to atlas at {}", cfg.atlas_endpoint);
    let mut atlas =
        AtlasClient::connect_with_retry(&cfg.atlas_endpoint, 10, Duration::from_secs(2))
            .await
            .context("connect to atlas")?;

    atlas
        .register_service(&cfg.id, EXECUTOR_NAMESPACE, "")
        .await?;
    info!("registered as '{}' under '{EXECUTOR_NAMESPACE}'", cfg.id);
    atlas
        .register_service(SENTINEL_PROVIDER_ID, SENTINEL_NAMESPACE, "")
        .await?;
    info!("registered as '{SENTINEL_PROVIDER_ID}' under '{SENTINEL_NAMESPACE}'");

    let listen_addr: std::net::SocketAddr = cfg
        .listen
        .parse()
        .with_context(|| format!("invalid executor listen address '{}'", cfg.listen))?;
    let sentinel_listen_addr: std::net::SocketAddr = cfg
        .sentinel_listen
        .parse()
        .with_context(|| format!("invalid Sentinel listen address '{}'", cfg.sentinel_listen))?;
    let advertised = advertised_address(listen_addr);
    let sentinel_advertised = advertised_address(sentinel_listen_addr);

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

    for (contract_id, contract_path, service_name, method_path) in [
        (
            "robonix/system/sentinel/list_rules",
            "capabilities/system/sentinel/list_rules.v1.toml",
            "robonix.contracts.RobonixSystemSentinelListRules",
            "/robonix.contracts.RobonixSystemSentinelListRules/ListRules",
        ),
        (
            "robonix/system/sentinel/replace_rules",
            "capabilities/system/sentinel/replace_rules.v1.toml",
            "robonix.contracts.RobonixSystemSentinelReplaceRules",
            "/robonix.contracts.RobonixSystemSentinelReplaceRules/ReplaceRules",
        ),
    ] {
        atlas
            .declare_capability(
                SENTINEL_PROVIDER_ID,
                contract_id,
                atlas_pb::Transport::Grpc,
                &sentinel_advertised,
                atlas_client::grpc_params(contract_path, service_name, method_path),
            )
            .await?;
    }

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
    // catalog discovery sees them like any user MCP provider. Dispatch never
    // dials the internal endpoint; calls targeting this provider run in-process.
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

    // Executor has no Driver lifecycle handshake — it's ready as soon as
    // the gRPC server is up. Push ACTIVE so `rbnx caps` doesn't show the
    // legacy-fallback INACTIVE forever.
    if let Err(e) = atlas
        .set_lifecycle_state(&cfg.id, atlas_pb::LifecycleState::StateActive, "")
        .await
    {
        warn!("SetLifecycleState(ACTIVE) failed: {e:#}");
    }

    // Atlas evicts providers after ~60s without a heartbeat. Send one every
    // 20s so we stay registered for the lifetime of the process.
    {
        let mut hb = atlas.clone();
        let provider_ids = [cfg.id.clone(), SENTINEL_PROVIDER_ID.to_owned()];
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await;
            loop {
                tick.tick().await;
                for provider_id in &provider_ids {
                    if let Err(e) = hb.heartbeat(provider_id).await {
                        warn!("heartbeat for '{provider_id}' failed: {e:#}");
                    }
                }
            }
        });
    }

    let verification = Arc::new(verification::VerificationPolicy::new(
        cfg.verification.overlap,
        cfg.verification.rules,
    ));
    info!(
        "loaded {} executor verification rule(s)",
        verification.len()
    );
    let sentinel_svc = SentinelServiceImpl::new(atlas.clone(), sentinel.clone());
    let svc = ExecutorServiceImpl::new(
        atlas,
        cfg.id.clone(),
        sentinel,
        cfg.require_keystone,
        verification,
    );
    info!("executor gRPC on {listen_addr}");
    info!("sentinel gRPC on {sentinel_listen_addr}");
    info!("robonix-executor ready on {listen_addr}");

    let executor_server = tonic::transport::Server::builder()
        .add_service(RobonixSystemExecutorExecuteServer::new(svc.clone()))
        .add_service(RobonixSystemExecutorCancelAllPlansServer::new(svc.clone()))
        .add_service(RobonixSystemExecutorControlPlanServer::new(svc.clone()))
        .add_service(RobonixSystemExecutorListActivePlansServer::new(svc.clone()))
        .add_service(RobonixSystemExecutorGetHealthServer::new(svc.clone()))
        .serve(listen_addr);
    let sentinel_server = tonic::transport::Server::builder()
        .add_service(RobonixSystemSentinelListRulesServer::new(
            sentinel_svc.clone(),
        ))
        .add_service(RobonixSystemSentinelReplaceRulesServer::new(sentinel_svc))
        .serve(sentinel_listen_addr);
    tokio::try_join!(executor_server, sentinel_server)
        .context("Executor or Sentinel gRPC server failed")?;

    Ok(())
}

fn advertised_address(address: std::net::SocketAddr) -> String {
    match address.ip() {
        std::net::IpAddr::V4(ip) if ip.is_unspecified() => {
            format!("127.0.0.1:{}", address.port())
        }
        _ => address.to_string(),
    }
}
