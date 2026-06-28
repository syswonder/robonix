// SPDX-License-Identifier: MulanPSL-2.0
//
// Robonix Soma — body-data service AND runtime bring-up driver for
// primitive + skill packages.
//
// Two-stage bring-up (see `launcher.rs` for the full rationale):
//   * Stage 1, immediately after soma starts up: spawn every
//     primitive, run Driver(CMD_INIT, config) then Driver(CMD_ACTIVATE).
//     This happens BEFORE soma declares its own get_yaml/get_urdf
//     capabilities to atlas, so primitives that need urdf/yaml from
//     soma at INIT time should not exist — soma's data path comes
//     later. (In practice primitives are sensor drivers and don't
//     need soma data; the ordering just preserves the invariant
//     "soma is ACTIVE only after its primitives are.")
//   * Stage 2, gated on atlas's WatchProvider stream firing
//     `StageTrigger("stage2")`: spawn every skill, run only
//     Driver(CMD_INIT) (executor sends CMD_ACTIVATE on first MCP
//     call). rbnx boot sends the trigger after all non-builtin
//     system services (memory / speech / scene / mapping / pilot /
//     liaison) have finished registering, which is the earliest
//     skills can safely declare their MCP tools.

use anyhow::{Context, Result};
use clap::Parser;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{info, warn};
use robonix_soma::config::{Args, SomaConfig};
use robonix_soma::deployment::DeploymentStore;
use robonix_soma::launcher::PackageLauncher;
use robonix_soma::pb::contracts::{
    robonix_system_soma_get_urdf_server::RobonixSystemSomaGetUrdfServer,
    robonix_system_soma_get_yaml_server::RobonixSystemSomaGetYamlServer,
};
use robonix_soma::service::SomaService;
use robonix_soma::store::SomaStore;
use robonix_soma::{GET_URDF_CONTRACT, GET_YAML_CONTRACT, SOMA_NAMESPACE};
use std::sync::Arc;
use std::time::Duration;
use tokio::signal::unix::{SignalKind, signal};

const GET_YAML_TOML: &str = "capabilities/system/soma/get_yaml.v1.toml";
const GET_URDF_TOML: &str = "capabilities/system/soma/get_urdf.v1.toml";
/// `ProviderEvent::StageTrigger { stage_name }` value rbnx sends to
/// kick soma's stage 2 bring-up. Must match the literal in rbnx's
/// `cmd::deploy.rs` — they're a contract pair, not configurable per
/// deployment.
const STAGE2_TRIGGER: &str = "stage2";

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    // Apply the manifest's per-component `log:` level (delivered inside
    // --config-json) to scribe's file sink before the first log line, so it
    // controls what this component persists; `rbnx logs --level` still
    // filters at read time.
    robonix_scribe::init_from_config("soma", args.config_json.as_deref());
    info!("robonix-soma starting");

    let config = SomaConfig::resolve(args).context("resolve Soma config")?;
    let deployments = DeploymentStore::load(&config).context("load deployment manifests")?;
    let store = Arc::new(SomaStore::load(&config).context("load Soma YAML/URDF data")?);

    let log_dir = std::env::var("SCRIBE_LOG_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| std::path::PathBuf::from("./logs"));
    let mut launcher = PackageLauncher::new(
        log_dir,
        config.rbnx_bin.clone(),
        config.atlas_endpoint.clone(),
    )
    .context("create package process manager")?;

    let atlas_http = normalize_endpoint(&config.atlas_endpoint);
    let mut atlas = AtlasClient::connect_with_retry(&atlas_http, 10, Duration::from_secs(2))
        .await
        .context("connect to Atlas")?;

    // ── Stage 1: primitive bring-up ──────────────────────────────────
    // Primitives walk REGISTERED → INACTIVE → ACTIVE before we declare
    // soma's own capabilities, so a downstream consumer asking
    // "is the chassis primitive ready?" never sees soma ACTIVE
    // without its primitives ACTIVE.
    let stage1_report = launcher
        .spawn_primitives(&deployments, &mut atlas, config.start_packages)
        .await;
    stage1_report.print_to_terminal();
    if stage1_report.has_failures() {
        // A primitive boot failure is fatal for soma — pilot can't
        // drive a robot whose sensors / actuators didn't come up.
        // Tear down whatever did start before bailing.
        launcher.stop_all().await?;
        anyhow::bail!("Soma stage 1 (primitive bring-up) failed");
    }

    // ── Soma's own gRPC service registration ─────────────────────────
    let listen_addr: std::net::SocketAddr = config
        .listen
        .parse()
        .with_context(|| format!("invalid Soma listen address '{}'", config.listen))?;
    let advertised = format!("127.0.0.1:{}", listen_addr.port());
    atlas
        .register_service(&config.provider_id, SOMA_NAMESPACE, "")
        .await
        .context("register Soma service")?;
    atlas
        .declare_capability(
            &config.provider_id,
            GET_YAML_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                GET_YAML_TOML,
                "robonix.contracts.RobonixSystemSomaGetYaml",
                "/robonix.contracts.RobonixSystemSomaGetYaml/GetYaml",
            ),
        )
        .await
        .context("declare Soma get_yaml gRPC capability")?;
    atlas
        .declare_capability(
            &config.provider_id,
            GET_URDF_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                GET_URDF_TOML,
                "robonix.contracts.RobonixSystemSomaGetUrdf",
                "/robonix.contracts.RobonixSystemSomaGetUrdf/GetUrdf",
            ),
        )
        .await
        .context("declare Soma get_urdf gRPC capability")?;
    atlas
        .set_lifecycle_state(
            &config.provider_id,
            atlas_pb::LifecycleState::StateActive,
            "",
        )
        .await
        .context("set Soma lifecycle ACTIVE")?;
    let heartbeat_failure = {
        let mut hb = atlas.clone();
        let provider_id = config.provider_id.clone();
        let (tx, rx) = tokio::sync::oneshot::channel();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            let mut failures = 0usize;
            let mut tx = Some(tx);
            tick.tick().await;
            loop {
                tick.tick().await;
                if let Err(e) = hb.heartbeat(&provider_id).await {
                    failures += 1;
                    warn!("heartbeat failed ({failures}/3): {e:#}");
                    if failures >= 3 {
                        if let Some(tx) = tx.take() {
                            let _ = tx.send(());
                        }
                        break;
                    }
                } else {
                    failures = 0;
                }
            }
        });
        rx
    };

    // ── Stage 2 watcher ──────────────────────────────────────────────
    // Subscribe to atlas's reverse-notification stream BEFORE rbnx is
    // done with non-builtin system bring-up — atlas queues events
    // even if soma isn't watching yet, but only AFTER the subscriber
    // is registered is delivery guaranteed (see
    // `service::AtlasRegistry::notify_event`). Doing this right after
    // we go ACTIVE means we own the subscription slot for our
    // provider_id; any later subscriber would displace us (takeover
    // semantics). That's the right ordering: rbnx waits to send the
    // trigger until non-builtin system bring-up finishes, by which
    // point we've been ACTIVE+subscribed for tens of seconds.
    let stage2_launcher = Arc::new(tokio::sync::Mutex::new(launcher));
    let stage2_atlas = atlas.clone();
    let stage2_deployments = deployments.clone();
    let stage2_provider_id = config.provider_id.clone();
    let stage2_start_packages = config.start_packages;
    let stage2_task = tokio::spawn({
        let launcher = Arc::clone(&stage2_launcher);
        async move {
            if let Err(e) = run_stage2(
                stage2_atlas,
                &stage2_provider_id,
                stage2_deployments,
                launcher,
                stage2_start_packages,
            )
            .await
            {
                warn!("stage 2 (skill bring-up) failed: {e:#}");
            }
        }
    });

    let svc = Arc::new(SomaService::new(store));
    info!(
        "robonix-soma ready on {}  (robots loaded, provider_id={})",
        config.listen, config.provider_id
    );
    let shutdown = shutdown_signal(heartbeat_failure);
    let serve_result = tonic::transport::Server::builder()
        .add_service(RobonixSystemSomaGetYamlServer::from_arc(Arc::clone(&svc)))
        .add_service(RobonixSystemSomaGetUrdfServer::from_arc(svc))
        .serve_with_shutdown(listen_addr, shutdown)
        .await;
    // Always abort the stage-2 watcher before tearing down children,
    // so its `watch_provider` stream doesn't try to reconnect while
    // atlas is going away.
    stage2_task.abort();
    stage2_launcher.lock().await.stop_all().await?;
    serve_result?;
    Ok(())
}

/// Wait for `StageTrigger("stage2")` over the atlas WatchProvider
/// stream, then run skill bring-up exactly once. Returning early
/// (stream closed before trigger, RPC failure, …) means stage 2 is
/// skipped — log loudly so the operator knows, but don't kill soma:
/// primitives are up and pilot can still poke at them, which is
/// strictly better than tearing the whole deploy down.
async fn run_stage2(
    mut atlas: AtlasClient,
    provider_id: &str,
    deployments: robonix_soma::deployment::DeploymentStore,
    launcher: Arc<tokio::sync::Mutex<PackageLauncher>>,
    start_packages: bool,
) -> Result<()> {
    use robonix_atlas::pb::provider_event::Kind as EventKind;
    let mut stream = atlas
        .watch_provider(provider_id)
        .await
        .with_context(|| format!("watch_provider('{provider_id}') for stage 2 trigger"))?;
    info!("stage 2 watcher armed; waiting for StageTrigger('{STAGE2_TRIGGER}')");
    loop {
        let msg = match stream.message().await {
            Ok(Some(event)) => event,
            Ok(None) => {
                warn!(
                    "atlas closed the WatchProvider stream for '{provider_id}' before stage 2 trigger"
                );
                return Ok(());
            }
            Err(e) => {
                anyhow::bail!("WatchProvider stream error: {e}");
            }
        };
        match msg.kind {
            Some(EventKind::StageTrigger(t)) if t.stage_name == STAGE2_TRIGGER => {
                info!("stage 2 trigger received; starting skill bring-up");
                break;
            }
            Some(EventKind::StageTrigger(t)) => {
                // Unknown stage names are ignored, not fatal — keeps
                // soma forward-compatible with future stages rbnx
                // might add.
                warn!("ignoring unknown StageTrigger('{}')", t.stage_name);
            }
            None => {
                warn!("received empty ProviderEvent; ignoring");
            }
        }
    }
    let mut launcher = launcher.lock().await;
    let report = launcher
        .spawn_skills(&deployments, &mut atlas, start_packages)
        .await;
    report.print_to_terminal();
    if report.has_failures() {
        warn!("stage 2 finished with one or more skill failures (see report)");
    } else {
        info!("stage 2 (skill bring-up) complete");
    }
    Ok(())
}

fn normalize_endpoint(endpoint: &str) -> String {
    if endpoint.starts_with("http") {
        endpoint.to_string()
    } else {
        format!("http://{endpoint}")
    }
}

async fn shutdown_signal(mut heartbeat_failure: tokio::sync::oneshot::Receiver<()>) {
    let mut sigint = signal(SignalKind::interrupt()).expect("install SIGINT handler");
    let mut sigterm = signal(SignalKind::terminate()).expect("install SIGTERM handler");
    tokio::select! {
        _ = sigint.recv() => {
            info!("received SIGINT");
        }
        _ = sigterm.recv() => {
            info!("received SIGTERM");
        }
        _ = &mut heartbeat_failure => {
            warn!("heartbeat failed repeatedly; shutting down Soma");
        }
    }
}
