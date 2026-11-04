// SPDX-License-Identifier: MulanPSL-2.0
//
// robonix-vitals — health monitoring: power state, component health, threshold alerts.
// On startup vitals:
//   1. Connects to atlas, registers as `vitals`.
//   2. Declares gRPC capabilities:
//      - robonix/system/vitals/get    (rpc: GetVitals → VitalsSnapshot)
//      - robonix/system/vitals/stream (topic_out: StreamVitals → stream VitalsSnapshot)
//      - robonix/system/vitals/modules/get (rpc: GetModuleHealthSnapshot)
//   3. Consumes Soma's StreamHealth gRPC stream (real or mock), normalizes
//      SomaHealthSnapshot → VitalsSnapshot via threshold rules.
//   4. Serves all gRPC services on `listen`.
//
// Vitals requires Soma.  Use --mock-soma to run an embedded mock, or point
// --soma-endpoint at a real Soma instance.

mod config;
mod mock_soma;
pub mod module_health;
mod module_health_poll;
mod pb;
mod service;
mod soma_ingest;
mod subprocess;

use anyhow::{Context, Result};
use clap::Parser;
use config::{Args, VITALS_NAMESPACE, VitalsConfig};
use log::info;
use pb::contracts::robonix_lifecycle_driver_client::RobonixLifecycleDriverClient;
use pb::contracts::robonix_lifecycle_driver_server::{
    RobonixLifecycleDriver, RobonixLifecycleDriverServer,
};
use pb::contracts::robonix_system_vitals_get_server::RobonixSystemVitalsGetServer;
use pb::contracts::robonix_system_vitals_modules_get_server::RobonixSystemVitalsModulesGetServer;
use pb::contracts::robonix_system_vitals_stream_server::RobonixSystemVitalsStreamServer;
use pb::lifecycle::{DriverRequest, DriverResponse};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use service::VitalsServiceImpl;
use std::sync::OnceLock;
use std::time::Duration;
use std::time::Instant;
use tonic::{Request, Response, Status};

pub(crate) const SHARED_DRIVER_CONTRACT: &str = "robonix/lifecycle/driver";
pub(crate) const CMD_INIT: u32 = 0;
pub(crate) const CMD_ACTIVATE: u32 = 1;
const CMD_DEACTIVATE: u32 = 2;
const CMD_SHUTDOWN: u32 = 3;

#[derive(Clone)]
pub(crate) struct SystemLifecycleDriver {
    atlas: AtlasClient,
    provider_id: String,
    shutdown_tx: tokio::sync::watch::Sender<bool>,
}

impl SystemLifecycleDriver {
    pub(crate) fn new(atlas: AtlasClient, provider_id: String) -> Self {
        let (shutdown_tx, _) = tokio::sync::watch::channel(false);
        Self {
            atlas,
            provider_id,
            shutdown_tx,
        }
    }

    /// Apply one no-op lifecycle callback and publish its authoritative state
    /// to Atlas. Unknown commands and rejected Atlas transitions are errors.
    pub(crate) async fn transition(&self, command: u32) -> Result<&'static str> {
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

    pub(crate) fn subscribe_shutdown(&self) -> tokio::sync::watch::Receiver<bool> {
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
pub(crate) async fn wait_for_driver_shutdown(mut shutdown: tokio::sync::watch::Receiver<bool>) {
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
pub(crate) fn startup_driver_endpoint(listen_addr: std::net::SocketAddr) -> String {
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
pub(crate) async fn connect_startup_driver(
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
pub(crate) async fn call_startup_driver(
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
    let log_filter = parsed
        .log
        .clone()
        .or_else(|| std::env::var("RUST_LOG").ok())
        .unwrap_or_else(|| "robonix_vitals=info".to_string());
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(log_filter)).init();

    let cfg = VitalsConfig::resolve(parsed)?;
    if cfg.mock_soma {
        return mock_soma::run_mock_soma(
            &cfg.atlas_endpoint,
            &cfg.mock_soma_id,
            &cfg.mock_soma_listen,
            &cfg.mock_soma_scenario,
            cfg.mock_soma_interval_ms,
            cfg.mock_soma_arm.clone(),
        )
        .await;
    }

    info!("connecting to atlas at {}", cfg.atlas_endpoint);
    let mut atlas =
        AtlasClient::connect_with_retry(&cfg.atlas_endpoint, 10, Duration::from_secs(2))
            .await
            .context("connect to atlas")?;

    atlas
        .register_service(&cfg.id, VITALS_NAMESPACE, "")
        .await?;
    info!("registered as '{}' under '{VITALS_NAMESPACE}'", cfg.id);

    let listen_addr: std::net::SocketAddr = cfg
        .listen
        .parse()
        .with_context(|| format!("invalid vitals listen address '{}'", cfg.listen))?;
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
        .context("declare Vitals shared lifecycle Driver")?;
    let lifecycle = SystemLifecycleDriver::new(atlas.clone(), cfg.id.clone());

    // Declare GetVitals RPC.
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/system/vitals/get",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/vitals/get.v1.toml",
                "robonix.contracts.RobonixSystemVitalsGet",
                "/robonix.contracts.RobonixSystemVitalsGet/GetVitals",
            ),
        )
        .await?;
    info!("declared GetVitals at {advertised}");

    // Declare StreamVitals server_stream.
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/system/vitals/stream",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/vitals/stream.v1.toml",
                "robonix.contracts.RobonixSystemVitalsStream",
                "/robonix.contracts.RobonixSystemVitalsStream/StreamVitals",
            ),
        )
        .await?;
    info!("declared StreamVitals at {advertised}");

    // Declare module health aggregate snapshot RPC.
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/system/vitals/modules/get",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/vitals/modules/get.toml",
                "robonix.contracts.RobonixSystemVitalsModulesGet",
                "/robonix.contracts.RobonixSystemVitalsModulesGet/GetModuleHealthSnapshot",
            ),
        )
        .await?;
    info!("declared ModuleHealthSnapshot at {advertised}");

    // Build the shared service state.
    let svc = VitalsServiceImpl::new();
    svc.update_self_module_health(&cfg.id)
        .await
        .context("initialize Vitals self module health")?;
    svc.apply_expected_module_config(&cfg.expected_modules)
        .await;

    let server_shutdown = lifecycle.subscribe_shutdown();
    let server_lifecycle = lifecycle.clone();
    let server_svc = svc.clone();
    let mut server_task = tokio::spawn(async move {
        tonic::transport::Server::builder()
            .add_service(RobonixLifecycleDriverServer::new(server_lifecycle))
            .add_service(RobonixSystemVitalsGetServer::new(server_svc.clone()))
            .add_service(RobonixSystemVitalsStreamServer::new(server_svc.clone()))
            .add_service(RobonixSystemVitalsModulesGetServer::new(server_svc))
            .serve_with_shutdown(listen_addr, wait_for_driver_shutdown(server_shutdown))
            .await
    });
    let startup_endpoint = startup_driver_endpoint(listen_addr);
    let mut startup_driver = tokio::select! {
        client = connect_startup_driver(&startup_endpoint) => client?,
        result = &mut server_task => {
            result.context("join Vitals gRPC server")?
                .context("Vitals gRPC server failed before readiness")?;
            anyhow::bail!("Vitals gRPC server stopped before readiness");
        }
    };
    call_startup_driver(&mut startup_driver, CMD_INIT)
        .await
        .context("initialize Vitals lifecycle")?;
    call_startup_driver(&mut startup_driver, CMD_ACTIVATE)
        .await
        .context("activate Vitals lifecycle")?;
    drop(startup_driver);

    // Heartbeat every 20s to prevent Atlas eviction (90s timeout).
    {
        let mut hb = atlas.clone();
        let provider_id = cfg.id.clone();
        let shutdown = lifecycle.subscribe_shutdown();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await;
            let shutdown = wait_for_driver_shutdown(shutdown);
            tokio::pin!(shutdown);
            loop {
                tokio::select! {
                    _ = &mut shutdown => break,
                    _ = tick.tick() => {
                        if let Err(e) = hb.heartbeat(&provider_id).await {
                            log::warn!("heartbeat failed: {e:#}");
                        }
                    }
                }
            }
        });
    }

    module_health_poll::spawn_module_health_poller(
        atlas.clone(),
        cfg.id.clone(),
        svc.clone(),
        cfg.expected_modules.clone(),
    );

    let soma_rules: Vec<soma_ingest::SomaThresholdRule> =
        match std::fs::read_to_string(&cfg.thresholds_path) {
            Ok(yaml_str) => match soma_ingest::load_soma_thresholds(&yaml_str) {
                Ok(r) => {
                    info!(
                        "loaded {} Soma threshold rules from {}",
                        r.len(),
                        cfg.thresholds_path.display()
                    );
                    r
                }
                Err(e) => {
                    log::error!(
                        "failed to parse Soma threshold file '{}': {e:#}; using defaults",
                        cfg.thresholds_path.display()
                    );
                    soma_ingest::default_thresholds()
                }
            },
            Err(_) => soma_ingest::default_thresholds(),
        };

    // ── SOMA stream consumer (background, retries until SOMA appears) ──
    // Start the gRPC server immediately so Vitals is ready before SOMA.
    // The SOMA connection retries in the background.
    {
        let svc_for_stream = svc.clone();
        let rules_for_stream = soma_rules.clone();
        let mut reconnect_atlas = atlas.clone();
        let reconnect_consumer_id = cfg.id.clone();
        let reconnect_soma_endpoint = cfg.soma_endpoint.clone();
        tokio::spawn(async move {
            // Initial connect: retry until SOMA appears.
            let mut stream = loop {
                match soma_ingest::open_soma_stream(
                    &mut reconnect_atlas,
                    &reconnect_consumer_id,
                    reconnect_soma_endpoint.as_deref(),
                )
                .await
                {
                    Ok(Some(s)) => break s,
                    Ok(None) => {
                        log::warn!("[vitals] waiting for Soma health stream...");
                    }
                    Err(e) => {
                        log::warn!("[vitals] Soma connect failed: {e:#} — retrying in 2s...");
                    }
                }
                tokio::time::sleep(std::time::Duration::from_secs(2)).await;
            };
            log::info!("[vitals] connected to Soma StreamHealth");

            // Outer loop: reconnect on stream end or error.
            loop {
                // Inner loop: process stream messages.
                loop {
                    match stream.message().await {
                        Ok(Some(snapshot)) => {
                            let vitals = soma_ingest::snapshot_to_vitals(
                                &snapshot,
                                &rules_for_stream,
                                monotonic_ns(),
                            );
                            svc_for_stream.update_snapshot(vitals).await;
                        }
                        Ok(None) => {
                            log::error!("[vitals] Soma StreamHealth ended — reconnecting...");
                            break;
                        }
                        Err(e) => {
                            log::error!(
                                "[vitals] Soma StreamHealth error: {e:#} — reconnecting..."
                            );
                            break;
                        }
                    }
                }

                // Reconnect with backoff.
                loop {
                    tokio::time::sleep(std::time::Duration::from_secs(2)).await;
                    match soma_ingest::open_soma_stream(
                        &mut reconnect_atlas,
                        &reconnect_consumer_id,
                        reconnect_soma_endpoint.as_deref(),
                    )
                    .await
                    {
                        Ok(Some(new_stream)) => {
                            log::info!("[vitals] reconnected to Soma StreamHealth");
                            stream = new_stream;
                            break;
                        }
                        Ok(None) => {
                            log::warn!("[vitals] Soma still unavailable — retrying in 2s...");
                        }
                        Err(e) => {
                            log::warn!("[vitals] reconnect failed: {e:#} — retrying in 2s...");
                        }
                    }
                }
            }
        });
    }

    info!("Vitals gRPC on {listen_addr} (Soma input)");
    eprintln!("robonix-vitals ready on {listen_addr} (Soma input)");

    server_task
        .await
        .context("join Vitals gRPC server")?
        .context("vitals gRPC server failed")?;

    Ok(())
}

fn monotonic_ns() -> u64 {
    static START: OnceLock<Instant> = OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_nanos() as u64
}

#[cfg(test)]
mod lifecycle_tests {
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
        let provider_id = "vitals-driver-rpc-test";
        atlas
            .register_service(provider_id, VITALS_NAMESPACE, "")
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
