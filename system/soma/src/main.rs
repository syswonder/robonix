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
//   * Stage 2, gated on rbnx writing `stage2\n` down a private pipe
//     that rbnx inherited onto a known fd of soma at spawn time:
//     spawn every skill, run only Driver(CMD_INIT) (executor sends
//     CMD_ACTIVATE on first MCP call). rbnx boot sends the trigger
//     after all non-builtin system services (memory / speech / scene
//     / mapping / pilot / liaison) have finished registering, which
//     is the earliest skills can safely declare their MCP tools.
//
// The stage trigger travels as an unnamed pipe, NOT over atlas —
// atlas is a system-wide registry and shouldn't grow public RPCs
// (`WatchProvider` / `NotifyProvider`) that exist purely for rbnx ↔
// soma coordination. Keeping the trigger private also fixes the
// v1 race where soma had to subscribe to atlas BEFORE rbnx published
// the event, which was fragile in slow CI.

use anyhow::{Context, Result};
use clap::Parser;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{info, warn};
use robonix_soma::config::{Args, SomaConfig};
use robonix_soma::deployment::Deployment;
use robonix_soma::launcher::PackageLauncher;
use robonix_soma::pb::contracts::{
    robonix_system_soma_get_urdf_server::RobonixSystemSomaGetUrdfServer,
    robonix_system_soma_get_yaml_server::RobonixSystemSomaGetYamlServer,
};
use robonix_soma::service::SomaService;
use robonix_soma::store::SomaBody;
use robonix_soma::{GET_URDF_CONTRACT, GET_YAML_CONTRACT, SOMA_NAMESPACE};
use std::os::fd::{FromRawFd, RawFd};
use std::sync::Arc;
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::signal::unix::{SignalKind, signal};

const GET_YAML_TOML: &str = "capabilities/system/soma/get_yaml.v1.toml";
const GET_URDF_TOML: &str = "capabilities/system/soma/get_urdf.v1.toml";
/// Line rbnx writes to the stage-trigger pipe to release soma's
/// stage 2. Match this exactly in rbnx's `cmd::deploy.rs` — they're
/// a contract pair, not configurable per deployment.
const STAGE2_TRIGGER: &str = "stage2";
/// Environment variable rbnx sets on the soma child to communicate
/// which inherited fd carries the stage trigger. Absent = we skip
/// stage 2 (hand-launch / test scenario); we log loudly so an operator
/// running soma standalone knows skills won't come up automatically.
const STAGE_FD_ENV: &str = "ROBONIX_SOMA_STAGE_FD";

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
    let deployment = Deployment::load(config.manifest_dir()).context("load deployment manifest")?;
    let body = Arc::new(SomaBody::load(&config.robot_yaml).context("load Soma YAML/URDF data")?);

    // Take the stage-trigger fd BEFORE we spawn any children. `rbnx`
    // dup2's the pipe read-end onto a known fd on soma at fork time
    // and sets the env; if we forget to consume it here, our own
    // process fd inheritance would leak it into every rbnx grandchild
    // (each package spawn is a fresh fork+exec).
    let stage_fd = take_stage_fd_from_env();

    let log_dir = std::env::var("SCRIBE_LOG_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| std::path::PathBuf::from("./logs"));
    let mut launcher = PackageLauncher::new(log_dir, config.atlas_endpoint.clone())
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
    let stage1_report = launcher.spawn_primitives(&deployment, &mut atlas).await;
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
    // Consume the stage-trigger pipe on a background task. We started
    // reading BEFORE going ACTIVE so rbnx can't publish faster than we
    // subscribe (unnamed pipes buffer; a `stage2\n` write while we're
    // mid-registration just waits for us in the kernel).
    let stage2_launcher = Arc::new(tokio::sync::Mutex::new(launcher));
    let stage2_deployment = deployment.clone();
    let mut stage2_atlas = atlas.clone();
    let stage2_task = tokio::spawn({
        let launcher = Arc::clone(&stage2_launcher);
        async move {
            let Some(fd) = stage_fd else {
                warn!(
                    "{STAGE_FD_ENV} not set; stage 2 skill bring-up disabled \
                     (hand-launched? soma expects rbnx to inherit a pipe)"
                );
                return;
            };
            if let Err(e) = run_stage2(fd, stage2_deployment, launcher, &mut stage2_atlas).await {
                warn!("stage 2 (skill bring-up) failed: {e:#}");
            }
        }
    });

    let svc = Arc::new(SomaService::new(body));
    info!(
        "robonix-soma ready on {}  (robot loaded, provider_id={})",
        config.listen, config.provider_id
    );
    let shutdown = shutdown_signal(heartbeat_failure);
    let serve_result = tonic::transport::Server::builder()
        .add_service(RobonixSystemSomaGetYamlServer::from_arc(Arc::clone(&svc)))
        .add_service(RobonixSystemSomaGetUrdfServer::from_arc(svc))
        .serve_with_shutdown(listen_addr, shutdown)
        .await;
    // Always abort the stage-2 watcher before tearing down children,
    // so its pipe read doesn't come back and try to reuse a torn-down
    // launcher.
    stage2_task.abort();
    stage2_launcher.lock().await.stop_all().await?;
    serve_result?;
    Ok(())
}

/// Read `ROBONIX_SOMA_STAGE_FD` and REMOVE it so subsequent child
/// spawns don't inherit the (misleading) env. Returns `None` if the
/// var is unset or malformed — callers log and skip stage 2 in that
/// case.
fn take_stage_fd_from_env() -> Option<RawFd> {
    let raw = std::env::var(STAGE_FD_ENV).ok()?;
    // SAFETY: single-threaded startup path; we haven't spawned any
    // futures yet.
    // `remove_var` marked unsafe on some platforms; wrap for safety.
    unsafe {
        std::env::remove_var(STAGE_FD_ENV);
    }
    match raw.parse::<RawFd>() {
        Ok(fd) if fd >= 0 => Some(fd),
        _ => {
            warn!("{STAGE_FD_ENV}='{raw}' is not a valid non-negative fd");
            None
        }
    }
}

/// Block on the stage-trigger pipe until we read `stage2\n`, then run
/// stage-2 skill bring-up exactly once. A malformed / early-close
/// pipe is a "skip stage 2" event (log loudly, don't crash) — the
/// primitives are up and pilot can still poke at them, which is
/// strictly better than tearing the whole deploy down.
async fn run_stage2(
    fd: RawFd,
    deployment: Deployment,
    launcher: Arc<tokio::sync::Mutex<PackageLauncher>>,
    atlas: &mut AtlasClient,
) -> Result<()> {
    // Take ownership of the raw fd so tokio can close it on drop.
    // SAFETY: rbnx guarantees this fd is a live, owned pipe read-end
    // in this process (dup2'd at fork), no other code touches it.
    let file = unsafe { std::fs::File::from_raw_fd(fd) };
    // Non-blocking mode is required for tokio's AsyncFd wrapper.
    set_nonblocking(&file).context("mark stage-trigger fd non-blocking")?;
    let async_file = tokio::fs::File::from_std(file);
    let mut reader = BufReader::new(async_file);
    info!("stage 2 watcher armed on fd {fd}; waiting for '{STAGE2_TRIGGER}' line");
    let mut line = String::new();
    loop {
        line.clear();
        let n = reader
            .read_line(&mut line)
            .await
            .context("read stage-trigger pipe")?;
        if n == 0 {
            warn!("stage-trigger pipe closed by rbnx before '{STAGE2_TRIGGER}'");
            return Ok(());
        }
        let trimmed = line.trim();
        if trimmed == STAGE2_TRIGGER {
            info!("stage 2 trigger received; starting skill bring-up");
            break;
        }
        // Unknown lines are ignored, not fatal — keeps soma
        // forward-compatible with future stages rbnx might add.
        warn!("ignoring unknown stage-trigger line '{trimmed}'");
    }
    let mut launcher = launcher.lock().await;
    let report = launcher.spawn_skills(&deployment, atlas).await;
    report.print_to_terminal();
    if report.has_failures() {
        warn!("stage 2 finished with one or more skill failures (see report)");
    } else {
        info!("stage 2 (skill bring-up) complete");
    }
    Ok(())
}

/// Set `O_NONBLOCK` on the file. Needed so tokio can drive read
/// readiness via epoll instead of blocking a worker thread.
fn set_nonblocking(file: &std::fs::File) -> anyhow::Result<()> {
    use nix::fcntl::{FcntlArg, OFlag, fcntl};
    use std::os::fd::AsFd;
    let borrowed = file.as_fd();
    let bits = fcntl(borrowed, FcntlArg::F_GETFL).context("fcntl F_GETFL on stage-trigger fd")?;
    let mut flags = OFlag::from_bits_truncate(bits);
    flags.insert(OFlag::O_NONBLOCK);
    fcntl(borrowed, FcntlArg::F_SETFL(flags)).context("fcntl F_SETFL on stage-trigger fd")?;
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
