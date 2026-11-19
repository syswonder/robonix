// SPDX-License-Identifier: MulanPSL-2.0

//! Linux Chronos executable with gRPC and MCP endpoints.

mod config;
mod encoding;
mod mcp;
mod pb;
mod publication;
mod service;

use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};
use std::time::Duration;

use anyhow::{Context, Result};
use clap::Parser;
use robonix_atlas::{
    client::{self as atlas_client, AtlasClient},
    pb as atlas_pb,
};
use tokio::net::TcpListener;
use tokio::signal::unix::{SignalKind, signal};
use tokio::sync::watch;
use tokio_stream::wrappers::TcpListenerStream;

use pb::contracts::{
    robonix_lifecycle_driver_client::RobonixLifecycleDriverClient,
    robonix_lifecycle_driver_server::RobonixLifecycleDriverServer,
    robonix_system_chronos_quality_server::RobonixSystemChronosQualityServer,
    robonix_system_chronos_reference_time_server::RobonixSystemChronosReferenceTimeServer,
};
use pb::lifecycle::DriverRequest;
use service::ChronosService;

/// Run the standalone daemon with CLI configuration and bounded shutdown.
pub async fn run() -> Result<()> {
    let cfg = config::resolve(config::Args::parse())?;
    robonix_scribe::init_from_config(&cfg.provider_id, Some(&cfg.log_json));
    let mut sigint = signal(SignalKind::interrupt())?;
    let mut sigterm = signal(SignalKind::terminate())?;
    let listener = TcpListener::bind(cfg.listen)
        .await
        .context("bind Chronos listener")?;
    let advertised = dial_address(listener.local_addr()?);
    let mut atlas = tokio::select! {
        result = tokio::time::timeout(Duration::from_secs(10),
            AtlasClient::connect_with_retry(&cfg.atlas_endpoint, 10, Duration::from_millis(200))) => {
            result.context("Chronos Atlas connection timed out")??
        }
        _ = sigint.recv() => anyhow::bail!("Chronos startup interrupted"),
        _ = sigterm.recv() => anyhow::bail!("Chronos startup terminated"),
    };
    let svc = ChronosService::new(cfg.chronos, atlas.clone(), cfg.provider_id.clone());
    let server_svc = svc.clone();
    let server_shutdown = svc.shutdown_receiver();
    let mut server = tokio::spawn(async move {
        let routes =
            tonic::service::Routes::new(RobonixLifecycleDriverServer::new(server_svc.clone()))
                .add_service(RobonixSystemChronosQualityServer::new(server_svc.clone()))
                .add_service(RobonixSystemChronosReferenceTimeServer::new(
                    server_svc.clone(),
                ))
                .into_axum_router()
                .route_service(mcp::PATH, mcp::http_service(server_svc, advertised.ip()));
        tonic::transport::Server::builder()
            .accept_http1(true)
            .add_routes(routes.into())
            .serve_with_incoming_shutdown(
                TcpListenerStream::new(listener),
                wait_shutdown(server_shutdown),
            )
            .await
    });
    let mut registered = false;
    let startup = async {
        atlas
            .register_service(&cfg.provider_id, "robonix/system/chronos", "")
            .await?;
        registered = true;
        declare(&mut atlas, &cfg.provider_id, &advertised.to_string()).await?;
        let mut driver =
            RobonixLifecycleDriverClient::connect(format!("http://{advertised}")).await?;
        for command in [0, 1] {
            let reply = driver
                .driver(DriverRequest {
                    command,
                    config_json: "{}".into(),
                })
                .await?
                .into_inner();
            anyhow::ensure!(reply.ok, "Chronos startup Driver failed: {}", reply.error);
        }
        Ok::<_, anyhow::Error>(())
    };
    let startup_result = tokio::select! {
        result = tokio::time::timeout(Duration::from_secs(10), startup) => result.context("Chronos startup timed out").and_then(|r| r),
        _ = sigint.recv() => Err(anyhow::anyhow!("Chronos startup interrupted")),
        _ = sigterm.recv() => Err(anyhow::anyhow!("Chronos startup terminated")),
    };
    let mut server_joined = false;
    let outcome = if let Err(error) = startup_result {
        Err(error)
    } else {
        robonix_scribe::info!("Chronos ready on {advertised} as '{}'", cfg.provider_id);
        let mut tick = tokio::time::interval(Duration::from_secs(20));
        tick.tick().await;
        let shutdown = wait_shutdown(svc.shutdown_receiver());
        tokio::pin!(shutdown);
        loop {
            tokio::select! {
                _ = sigint.recv() => break Ok(()),
                _ = sigterm.recv() => break Ok(()),
                _ = &mut shutdown => break Ok(()),
                result = &mut server => {
                    server_joined = true;
                    break result.context("join Chronos server")
                        .and_then(|r| r.context("Chronos server failed"));
                }
                _ = tick.tick() => {
                    match tokio::time::timeout(Duration::from_secs(5), atlas.heartbeat(&cfg.provider_id)).await {
                        Ok(Ok(())) => {}
                        Ok(Err(error)) => robonix_scribe::warn!("Chronos heartbeat failed: {error:#}"),
                        Err(error) => robonix_scribe::warn!("Chronos heartbeat timed out: {error}"),
                    }
                }
            }
        }
    };
    let cleanup = svc.close().await;
    if let Err(error) = &cleanup {
        robonix_scribe::error!("Chronos worker cleanup failed: {error:#}");
    }
    if registered {
        let result = tokio::time::timeout(
            Duration::from_secs(5),
            atlas.set_lifecycle_state(
                &cfg.provider_id,
                atlas_pb::LifecycleState::StateTerminated,
                "",
            ),
        )
        .await;
        if !matches!(result, Ok(Ok(()))) {
            robonix_scribe::warn!("Chronos could not report termination to Atlas: {result:?}");
        }
    }
    if !server_joined {
        match tokio::time::timeout(Duration::from_secs(5), &mut server).await {
            Ok(result) => result.context("join Chronos server cleanup")??,
            Err(_) => {
                robonix_scribe::warn!(
                    "Chronos shutdown exceeded five seconds; closing remaining connections"
                );
                server.abort();
                let _ = server.await;
            }
        }
    }
    outcome.and(cleanup)
}

async fn declare(atlas: &mut AtlasClient, id: &str, endpoint: &str) -> Result<()> {
    for (contract, service, method, contract_path) in [
        (
            "robonix/lifecycle/driver",
            "RobonixLifecycleDriver",
            "Driver",
            "capabilities/lifecycle/driver.v1.toml",
        ),
        (
            "robonix/system/chronos/quality",
            "RobonixSystemChronosQuality",
            "Quality",
            "capabilities/system/chronos/quality.v1.toml",
        ),
        (
            "robonix/system/chronos/reference_time",
            "RobonixSystemChronosReferenceTime",
            "ReferenceTime",
            "capabilities/system/chronos/reference_time.v1.toml",
        ),
    ] {
        atlas
            .declare_capability(
                id,
                contract,
                atlas_pb::Transport::Grpc,
                endpoint,
                atlas_client::grpc_params(
                    contract_path,
                    format!("robonix.contracts.{service}"),
                    format!("/robonix.contracts.{service}/{method}"),
                ),
            )
            .await?;
    }
    let tool = mcp::wall_time_tool();
    atlas
        .declare_capability_with_description(
            id,
            mcp::CONTRACT_ID,
            atlas_pb::Transport::Mcp,
            &format!("http://{endpoint}{}", mcp::PATH),
            atlas_client::mcp_params(serde_json::to_string(&tool.input_schema)?),
            mcp::DESCRIPTION,
        )
        .await?;
    Ok(())
}

/// Unspecified binds are advertised locally; use an explicit host IP for remote clients.
fn dial_address(address: SocketAddr) -> SocketAddr {
    let ip = match address.ip() {
        IpAddr::V4(ip) if ip.is_unspecified() => IpAddr::V4(Ipv4Addr::LOCALHOST),
        IpAddr::V6(ip) if ip.is_unspecified() => IpAddr::V6(Ipv6Addr::LOCALHOST),
        ip => ip,
    };
    SocketAddr::new(ip, address.port())
}

/// Preserve shutdown signals delivered before this waiter begins polling.
async fn wait_shutdown(mut receiver: watch::Receiver<bool>) {
    while !*receiver.borrow_and_update() {
        if receiver.changed().await.is_err() {
            break;
        }
    }
}

#[cfg(test)]
mod tests;
