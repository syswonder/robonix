// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// robonix-pilot — reasoning, planning, and session management.
//
// Run standalone (manual smoke testing):
//   robonix-pilot --vlm-upstream https://api.openai.com/v1 \
//                 --vlm-api-key sk-... \
//                 --vlm-model gpt-5.5
//   # atlas defaults to 127.0.0.1:50051; everything else has sane defaults.
//
// Run under rbnx:
//   ROBONIX_ATLAS_ENDPOINT=… ROBONIX_CONFIG_PATH=/run/robonix/pilot.yaml robonix-pilot
//
// Either way, on startup pilot:
//   1. Connects to atlas, registers its capability, declares the
//      `robonix/system/pilot` gRPC capability.
//   2. Constructs an embedded LLM client from the resolved VLM config.
//   3. Serves RobonixSystemPilot on `listen`. Executor address is discovered
//      through atlas at every Stream RPC, not configured statically.

mod config;
mod discovery;
mod history;
mod memory;
mod pb;
mod planner;
mod service;
mod soma_context;
mod state_context;
mod vlm;

use anyhow::{Context, Result};
use clap::Parser;
use config::{Args, PILOT_NAMESPACE, PilotConfig};
use pb::contracts::robonix_lifecycle_driver_client::RobonixLifecycleDriverClient;
use pb::contracts::robonix_lifecycle_driver_server::{
    RobonixLifecycleDriver, RobonixLifecycleDriverServer,
};
use pb::contracts::robonix_system_pilot_get_health_server::RobonixSystemPilotGetHealthServer;
use pb::contracts::robonix_system_pilot_server::RobonixSystemPilotServer;
use pb::lifecycle::{DriverRequest, DriverResponse};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{info, warn};
use service::PilotServiceImpl;
use std::time::Duration;
use tonic::{Request, Response, Status};

const SHARED_DRIVER_CONTRACT: &str = "robonix/lifecycle/driver";
const CMD_INIT: u32 = 0;
const CMD_ACTIVATE: u32 = 1;
const CMD_DEACTIVATE: u32 = 2;
const CMD_SHUTDOWN: u32 = 3;

#[derive(Clone)]
struct SystemLifecycleDriver {
    atlas: AtlasClient,
    provider_id: String,
    shutdown_tx: tokio::sync::watch::Sender<bool>,
}

impl SystemLifecycleDriver {
    fn new(atlas: AtlasClient, provider_id: String) -> Self {
        let (shutdown_tx, _) = tokio::sync::watch::channel(false);
        Self {
            atlas,
            provider_id,
            shutdown_tx,
        }
    }

    /// Apply one no-op lifecycle callback and publish its authoritative state
    /// to Atlas. Unknown commands and rejected Atlas transitions are errors.
    async fn transition(&self, command: u32) -> Result<&'static str> {
        let (state, label) = lifecycle_target(command)
            .ok_or_else(|| anyhow::anyhow!("unknown lifecycle command code {command}"))?;
        let mut atlas = self.atlas.clone();
        atlas
            .set_lifecycle_state(&self.provider_id, state, "")
            .await
            .with_context(|| format!("publish lifecycle state for '{}'", self.provider_id))?;
        if command == CMD_SHUTDOWN {
            self.shutdown_tx.send_replace(true);
        }
        Ok(label)
    }

    fn subscribe_shutdown(&self) -> tokio::sync::watch::Receiver<bool> {
        self.shutdown_tx.subscribe()
    }
}

#[tonic::async_trait]
impl RobonixLifecycleDriver for SystemLifecycleDriver {
    /// Serve the shared Driver RPC and report callback/transition failures in
    /// the stable DriverResponse envelope used by launchers.
    async fn driver(
        &self,
        request: Request<DriverRequest>,
    ) -> std::result::Result<Response<DriverResponse>, Status> {
        let response = match self.transition(request.into_inner().command).await {
            Ok(state) => DriverResponse {
                ok: true,
                state: state.to_string(),
                error: String::new(),
            },
            Err(error) => DriverResponse {
                ok: false,
                state: "error".to_string(),
                error: format!("{error:#}"),
            },
        };
        Ok(Response::new(response))
    }
}

fn lifecycle_target(command: u32) -> Option<(atlas_pb::LifecycleState, &'static str)> {
    match command {
        CMD_INIT => Some((atlas_pb::LifecycleState::StateInactive, "inactive")),
        CMD_ACTIVATE => Some((atlas_pb::LifecycleState::StateActive, "active")),
        CMD_DEACTIVATE => Some((atlas_pb::LifecycleState::StateInactive, "inactive")),
        CMD_SHUTDOWN => Some((atlas_pb::LifecycleState::StateTerminated, "terminated")),
        _ => None,
    }
}

/// Wait until Driver(CMD_SHUTDOWN) has published TERMINATED. A watch channel
/// preserves a shutdown sent before this particular waiter starts polling.
async fn wait_for_driver_shutdown(mut shutdown: tokio::sync::watch::Receiver<bool>) {
    if *shutdown.borrow() {
        return;
    }
    while shutdown.changed().await.is_ok() {
        if *shutdown.borrow() {
            return;
        }
    }
}

/// Return a loopback endpoint for an unspecified bind address so the process
/// can self-dial without changing the endpoint it advertises through Atlas.
fn startup_driver_endpoint(listen_addr: std::net::SocketAddr) -> String {
    let ip = match listen_addr.ip() {
        std::net::IpAddr::V4(ip) if ip.is_unspecified() => {
            std::net::IpAddr::V4(std::net::Ipv4Addr::LOCALHOST)
        }
        std::net::IpAddr::V6(ip) if ip.is_unspecified() => {
            std::net::IpAddr::V6(std::net::Ipv6Addr::LOCALHOST)
        }
        ip => ip,
    };
    std::net::SocketAddr::new(ip, listen_addr.port()).to_string()
}

/// Connect to this process's just-spawned Driver endpoint. Retries only the
/// bounded bind/startup window; a connected endpoint is returned immediately.
async fn connect_startup_driver(
    endpoint: &str,
) -> Result<RobonixLifecycleDriverClient<tonic::transport::Channel>> {
    let endpoint = if endpoint.starts_with("http") {
        endpoint.to_string()
    } else {
        format!("http://{endpoint}")
    };
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    loop {
        match RobonixLifecycleDriverClient::connect(endpoint.clone()).await {
            Ok(client) => return Ok(client),
            Err(_) if tokio::time::Instant::now() < deadline => {
                tokio::time::sleep(Duration::from_millis(25)).await;
            }
            Err(error) => return Err(error).context("connect startup lifecycle Driver"),
        }
    }
}

/// Invoke one startup lifecycle command through the real generated Driver RPC
/// and reject an `ok=false` response as a startup failure.
async fn call_startup_driver(
    client: &mut RobonixLifecycleDriverClient<tonic::transport::Channel>,
    command: u32,
) -> Result<String> {
    let response = client
        .driver(DriverRequest {
            command,
            config_json: "{}".to_string(),
        })
        .await
        .context("call startup lifecycle Driver")?
        .into_inner();
    if !response.ok {
        anyhow::bail!("startup lifecycle Driver failed: {}", response.error);
    }
    Ok(response.state)
}

#[tokio::main]
async fn main() -> Result<()> {
    let parsed = Args::parse();
    // Apply the manifest's per-component `log:` level (delivered inside
    // --config-json) to scribe's file sink before the first log line.
    robonix_scribe::init_from_config("pilot", parsed.config_json.as_deref());
    info!("robonix-pilot starting");

    let cfg = PilotConfig::resolve(parsed)?;

    info!("connecting to atlas at {}", cfg.atlas_endpoint);
    let mut atlas =
        AtlasClient::connect_with_retry(&cfg.atlas_endpoint, 10, Duration::from_secs(2))
            .await
            .context("connect to atlas")?;

    atlas.register_service(&cfg.id, PILOT_NAMESPACE, "").await?;
    info!("registered as '{}' under '{PILOT_NAMESPACE}'", cfg.id);

    let soma_prompt_block = match soma_context::fetch_system_prompt_block(&mut atlas, &cfg.id).await
    {
        Ok(Some(block)) => {
            info!("loaded Soma body context into Pilot system prompt");
            block
        }
        Ok(None) => String::new(),
        Err(e) => {
            warn!("Soma body context load failed; continuing without it: {e:#}");
            String::new()
        }
    };

    let listen_addr: std::net::SocketAddr = cfg
        .listen
        .parse()
        .with_context(|| format!("invalid pilot listen address '{}'", cfg.listen))?;
    // Advertise a concrete loopback for wildcard binds. Consumers cannot dial
    // 0.0.0.0 or [::], and the address family must match the listening socket.
    let advertised = startup_driver_endpoint(listen_addr);
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
        .context("declare Pilot shared lifecycle Driver")?;
    let lifecycle = SystemLifecycleDriver::new(atlas.clone(), cfg.id.clone());
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/system/pilot",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/pilot.v1.toml",
                "robonix.contracts.RobonixSystemPilot",
                "/robonix.contracts.RobonixSystemPilot/SubmitTask",
            ),
        )
        .await?;
    info!("declared RobonixSystemPilot gRPC at {advertised}");

    atlas
        .declare_capability(
            &cfg.id,
            "robonix/system/pilot/get_health",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/pilot/get_health.toml",
                "robonix.contracts.RobonixSystemPilotGetHealth",
                "/robonix.contracts.RobonixSystemPilotGetHealth/GetModuleHealth",
            ),
        )
        .await?;
    info!("declared RobonixSystemPilotGetHealth gRPC at {advertised}");

    let vlm = vlm::VlmClient::new(&cfg.vlm);
    info!(
        "VLM upstream='{}' model='{}'",
        cfg.vlm.upstream, cfg.vlm.model
    );

    let svc = PilotServiceImpl::new(atlas.clone(), cfg.id.clone(), vlm, soma_prompt_block);
    let server_shutdown = lifecycle.subscribe_shutdown();
    let server_lifecycle = lifecycle.clone();
    let mut server_task = tokio::spawn(async move {
        tonic::transport::Server::builder()
            .add_service(RobonixLifecycleDriverServer::new(server_lifecycle))
            .add_service(RobonixSystemPilotServer::new(svc.clone()))
            .add_service(RobonixSystemPilotGetHealthServer::new(svc))
            .serve_with_shutdown(listen_addr, wait_for_driver_shutdown(server_shutdown))
            .await
    });
    let startup_endpoint = startup_driver_endpoint(listen_addr);
    let mut startup_driver = tokio::select! {
        client = connect_startup_driver(&startup_endpoint) => client?,
        result = &mut server_task => {
            result.context("join Pilot gRPC server")?
                .context("Pilot gRPC server failed before readiness")?;
            anyhow::bail!("Pilot gRPC server stopped before readiness");
        }
    };
    call_startup_driver(&mut startup_driver, CMD_INIT)
        .await
        .context("initialize Pilot lifecycle")?;
    call_startup_driver(&mut startup_driver, CMD_ACTIVATE)
        .await
        .context("activate Pilot lifecycle")?;
    drop(startup_driver);

    // Atlas evicts providers after ~60s without a heartbeat. Send one every
    // 20s so we stay registered for the lifetime of the process.
    {
        let mut hb = atlas.clone();
        let provider_id = cfg.id.clone();
        let shutdown = lifecycle.subscribe_shutdown();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await; // first tick fires immediately; skip
            let shutdown = wait_for_driver_shutdown(shutdown);
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

    info!("RobonixSystemPilot gRPC on {listen_addr}");
    info!("robonix-pilot ready on {listen_addr}");

    server_task
        .await
        .context("join Pilot gRPC server")?
        .context("pilot gRPC server failed")?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use pb::contracts::robonix_lifecycle_driver_client::RobonixLifecycleDriverClient;
    use robonix_atlas::service::{AtlasRegistry, serve_atlas};
    use std::net::SocketAddr;
    use std::sync::Arc;

    fn reserve_address() -> SocketAddr {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("reserve port");
        listener.local_addr().expect("reserved address")
    }

    #[test]
    fn startup_dial_rewrites_unspecified_addresses() {
        assert_eq!(
            startup_driver_endpoint("0.0.0.0:51001".parse().unwrap()),
            "127.0.0.1:51001"
        );
        assert_eq!(
            startup_driver_endpoint("[::]:51001".parse().unwrap()),
            "[::1]:51001"
        );
    }

    /// Dial a just-spawned Driver server, retrying only the short startup race.
    async fn connect_driver(
        endpoint: String,
    ) -> RobonixLifecycleDriverClient<tonic::transport::Channel> {
        let mut last_error = None;
        for _ in 0..50 {
            match RobonixLifecycleDriverClient::connect(endpoint.clone()).await {
                Ok(client) => return client,
                Err(error) => last_error = Some(error),
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        panic!(
            "connect Driver: {:#}",
            last_error.expect("connection error")
        );
    }

    /// Exercise the generated Driver client against the real tonic endpoint
    /// and verify that both RPCs publish their lifecycle states to Atlas.
    #[tokio::test]
    async fn shared_driver_rpc_is_callable() {
        let atlas_addr = reserve_address();
        let registry = Arc::new(AtlasRegistry::default());
        let atlas_server = tokio::spawn(serve_atlas(Arc::clone(&registry), atlas_addr));
        let mut atlas = AtlasClient::connect_with_retry(
            format!("http://{atlas_addr}"),
            50,
            Duration::from_millis(10),
        )
        .await
        .expect("connect test Atlas");
        let provider_id = "pilot-driver-rpc-test";
        atlas
            .register_service(provider_id, PILOT_NAMESPACE, "")
            .await
            .expect("register test Provider");

        let driver_addr = reserve_address();
        atlas
            .declare_capability(
                provider_id,
                SHARED_DRIVER_CONTRACT,
                atlas_pb::Transport::Grpc,
                &driver_addr.to_string(),
                atlas_client::grpc_params(
                    "capabilities/lifecycle/driver.v1.toml",
                    "robonix.contracts.RobonixLifecycleDriver",
                    "/robonix.contracts.RobonixLifecycleDriver/Driver",
                ),
            )
            .await
            .expect("declare shared Driver");
        let lifecycle = SystemLifecycleDriver::new(atlas.clone(), provider_id.to_string());
        let server_shutdown = lifecycle.subscribe_shutdown();
        let driver_server = tokio::spawn(
            tonic::transport::Server::builder()
                .add_service(RobonixLifecycleDriverServer::new(lifecycle))
                .serve_with_shutdown(driver_addr, wait_for_driver_shutdown(server_shutdown)),
        );
        let mut client = connect_driver(format!("http://{driver_addr}")).await;

        let init = client
            .driver(DriverRequest {
                command: CMD_INIT,
                config_json: "{}".to_string(),
            })
            .await
            .expect("Driver INIT RPC")
            .into_inner();
        assert!(init.ok, "{}", init.error);
        assert_eq!(init.state, "inactive");
        let activate = client
            .driver(DriverRequest {
                command: CMD_ACTIVATE,
                config_json: String::new(),
            })
            .await
            .expect("Driver ACTIVATE RPC")
            .into_inner();
        assert!(activate.ok, "{}", activate.error);
        assert_eq!(activate.state, "active");

        let shutdown = client
            .driver(DriverRequest {
                command: CMD_SHUTDOWN,
                config_json: String::new(),
            })
            .await
            .expect("Driver SHUTDOWN RPC")
            .into_inner();
        assert!(shutdown.ok, "{}", shutdown.error);
        assert_eq!(shutdown.state, "terminated");
        tokio::time::timeout(Duration::from_secs(2), driver_server)
            .await
            .expect("Driver server did not stop after SHUTDOWN")
            .expect("join Driver server")
            .expect("graceful Driver server shutdown");

        let provider = atlas
            .query(
                atlas_pb::Kind::Service,
                provider_id,
                "",
                "",
                atlas_pb::Transport::Unspecified,
            )
            .await
            .expect("query test Provider")
            .pop()
            .expect("test Provider");
        assert_eq!(
            provider.state,
            atlas_pb::LifecycleState::StateTerminated as i32
        );

        atlas_server.abort();
    }
}
