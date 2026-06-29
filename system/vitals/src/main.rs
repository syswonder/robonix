// SPDX-License-Identifier: MulanPSL-2.0
//
// robonix-vitals — health monitoring: power state, component health, threshold alerts.
// On startup vitals:
//   1. Connects to atlas, registers as `vitals`.
//   2. Declares two gRPC capabilities:
//      - robonix/service/vitals/get    (rpc: GetVitals → VitalsSnapshot)
//      - robonix/service/vitals/stream (topic_out: StreamVitals → stream VitalsSnapshot)
//   3. Consumes Soma's StreamHealth gRPC stream (real or mock), normalizes
//      SomaHealthSnapshot → VitalsSnapshot via threshold rules.
//   4. Serves both gRPC services on `listen`.
//
// Vitals requires Soma.  Use --mock-soma to run an embedded mock, or point
// --soma-endpoint at a real Soma instance.

mod config;
mod mock_soma;
mod pb;
mod service;
mod soma_ingest;
mod subprocess;

use anyhow::{Context, Result};
use clap::Parser;
use config::{Args, VITALS_NAMESPACE, VitalsConfig};
use log::info;
use pb::contracts::robonix_service_vitals_get_server::RobonixServiceVitalsGetServer;
use pb::contracts::robonix_service_vitals_stream_server::RobonixServiceVitalsStreamServer;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use service::VitalsServiceImpl;
use std::sync::OnceLock;
use std::time::Duration;
use std::time::Instant;

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
        let piper_config =
            cfg.mock_soma_piper_can
                .as_ref()
                .map(|can| mock_soma::PiperBridgeConfig {
                    can_port: can.clone(),
                    python_bin: cfg.mock_soma_piper_python.clone(),
                    script: cfg.mock_soma_piper_script.to_string_lossy().to_string(),
                });
        return mock_soma::run_mock_soma(
            &cfg.atlas_endpoint,
            &cfg.mock_soma_id,
            &cfg.mock_soma_listen,
            &cfg.mock_soma_scenario,
            cfg.mock_soma_interval_ms,
            piper_config,
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
    let advertised = match listen_addr.ip() {
        std::net::IpAddr::V4(ip) if ip.is_unspecified() => {
            format!("127.0.0.1:{}", listen_addr.port())
        }
        _ => listen_addr.to_string(),
    };

    // Declare GetVitals RPC.
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/service/vitals/get",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/service/vitals/get.v1.toml",
                "robonix.contracts.RobonixServiceVitalsGet",
                "/robonix.contracts.RobonixServiceVitalsGet/GetVitals",
            ),
        )
        .await?;
    info!("declared GetVitals at {advertised}");

    // Declare StreamVitals server_stream.
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/service/vitals/stream",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/service/vitals/stream.v1.toml",
                "robonix.contracts.RobonixServiceVitalsStream",
                "/robonix.contracts.RobonixServiceVitalsStream/StreamVitals",
            ),
        )
        .await?;
    info!("declared StreamVitals at {advertised}");

    if let Err(e) = atlas
        .set_lifecycle_state(&cfg.id, atlas_pb::LifecycleState::StateActive, "")
        .await
    {
        log::warn!("SetLifecycleState(ACTIVE) failed: {e:#}");
    }

    // Heartbeat every 20s to prevent Atlas eviction (90s timeout).
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

    // Build the shared service state.
    let svc = VitalsServiceImpl::new();

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

    if let Some(mut stream) =
        soma_ingest::open_soma_stream(&mut atlas, &cfg.id, cfg.soma_endpoint.as_deref()).await?
    {
        let svc_for_stream = svc.clone();
        let rules_for_stream = soma_rules.clone();
        let mut reconnect_atlas = atlas.clone();
        let reconnect_consumer_id = cfg.id.clone();
        let reconnect_soma_endpoint = cfg.soma_endpoint.clone();
        tokio::spawn(async move {
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

        info!("Vitals gRPC on {listen_addr} (Soma input)");
        eprintln!("robonix-vitals ready on {listen_addr} (Soma input)");

        tonic::transport::Server::builder()
            .add_service(RobonixServiceVitalsGetServer::new(svc.clone()))
            .add_service(RobonixServiceVitalsStreamServer::new(svc))
            .serve(listen_addr)
            .await
            .context("vitals gRPC server failed")?;

        return Ok(());
    }

    log::error!("[vitals] no Soma stream available — Vitals requires Soma (real or mock)");
    eprintln!("robonix-vitals: no Soma stream available. Start real Soma or use --mock-soma.");
    std::process::exit(1);
}

fn monotonic_ns() -> u64 {
    static START: OnceLock<Instant> = OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_nanos() as u64
}
