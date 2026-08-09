// SPDX-License-Identifier: MulanPSL-2.0
//
// Robonix Soma — body-data service AND runtime bring-up driver for
// primitive + skill packages.
//
// Two-stage bring-up (see `launcher.rs` for the full rationale):
//   * Soma first loads and validates its body files, then starts the
//     read-only get_yaml/get_urdf gRPC server. It is not registered or
//     ACTIVE in Atlas yet. Stage 1 primitives can therefore consume the
//     canonical body model without introducing another URDF source.
//   * Stage 1 then spawns every primitive and runs Driver(CMD_INIT,
//     config) followed by Driver(CMD_ACTIVATE). Soma only registers its
//     own capabilities and becomes ACTIVE after every primitive is ACTIVE.
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
    robonix_lifecycle_driver_client::RobonixLifecycleDriverClient,
    robonix_lifecycle_driver_server::{RobonixLifecycleDriver, RobonixLifecycleDriverServer},
    robonix_system_soma_footprint_server::RobonixSystemSomaFootprintServer,
    robonix_system_soma_get_health_server::RobonixSystemSomaGetHealthServer,
    robonix_system_soma_get_urdf_server::RobonixSystemSomaGetUrdfServer,
    robonix_system_soma_get_yaml_server::RobonixSystemSomaGetYamlServer,
    robonix_system_soma_health_server::RobonixSystemSomaHealthServer,
};
use robonix_soma::pb::lifecycle::{DriverRequest, DriverResponse};
use robonix_soma::service::SomaService;
use robonix_soma::store::SomaBody;
use robonix_soma::{
    GET_FOOTPRINT_CONTRACT, GET_HEALTH_CONTRACT, GET_URDF_CONTRACT, GET_YAML_CONTRACT,
    HEALTH_CONTRACT, SOMA_NAMESPACE,
};
use std::os::fd::{FromRawFd, OwnedFd, RawFd};
use std::sync::Arc;
use std::time::Duration;
use tokio::signal::unix::{SignalKind, signal};
use tonic::{Request, Response, Status};

const GET_YAML_TOML: &str = "capabilities/system/soma/get_yaml.v1.toml";
const GET_URDF_TOML: &str = "capabilities/system/soma/get_urdf.v1.toml";
const GET_FOOTPRINT_TOML: &str = "capabilities/system/soma/footprint.v1.toml";
const GET_HEALTH_TOML: &str = "capabilities/system/soma/get_health.v1.toml";
const HEALTH_TOML: &str = "capabilities/system/soma/health.v1.toml";
const SHARED_DRIVER_CONTRACT: &str = "robonix/lifecycle/driver";
const CMD_INIT: u32 = 0;
const CMD_ACTIVATE: u32 = 1;
const CMD_DEACTIVATE: u32 = 2;
const CMD_SHUTDOWN: u32 = 3;
/// Line rbnx writes to the stage-trigger pipe to release soma's
/// stage 2. Match this exactly in rbnx's `cmd::deploy.rs` — they're
/// a contract pair, not configurable per deployment.
const STAGE2_TRIGGER: &str = "stage2";
/// Environment variable rbnx sets on the soma child to communicate
/// which inherited fd carries the stage trigger. Absent = we skip
/// stage 2 (hand-launch / test scenario); we log loudly so an operator
/// running soma standalone knows skills won't come up automatically.
const STAGE_FD_ENV: &str = "ROBONIX_SOMA_STAGE_FD";

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

    fn shutdown_requested(&self) -> bool {
        *self.shutdown_tx.borrow()
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

/// Connect to this process's just-spawned Driver endpoint during its bounded
/// bind window, proving tonic is serving before startup publishes ACTIVE.
async fn connect_startup_driver(
    endpoint: &str,
) -> Result<RobonixLifecycleDriverClient<tonic::transport::Channel>> {
    let endpoint = normalize_endpoint(endpoint);
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

/// Invoke one startup command through the generated Driver RPC and reject a
/// callback-level failure instead of publishing readiness locally.
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
    let args = Args::parse();
    // Apply the manifest's per-component `log:` level (delivered inside
    // --config-json) to scribe's file sink before the first log line, so it
    // controls what this component persists; `rbnx logs --level` still
    // filters at read time.
    robonix_scribe::init_from_config("soma", args.config_json.as_deref());
    info!("robonix-soma starting");

    let config = SomaConfig::resolve(args).context("resolve Soma config")?;
    let deployment = Deployment::load_manifest(config.deployment_manifest())
        .context("load deployment manifest")?;
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
    let atlas_http = normalize_endpoint(&config.atlas_endpoint);
    let mut atlas = AtlasClient::connect_with_retry(&atlas_http, 10, Duration::from_secs(2))
        .await
        .context("connect to Atlas")?;

    let listen_addr: std::net::SocketAddr = config
        .listen
        .parse()
        .with_context(|| format!("invalid Soma listen address '{}'", config.listen))?;
    {
        let probe = std::net::TcpListener::bind(listen_addr)
            .with_context(|| format!("bind Soma listen address {}", config.listen))?;
        drop(probe);
    }

    // Body data is available before primitive bring-up, while Soma remains
    // unregistered and non-ACTIVE in Atlas. Robot-state primitives use this
    // single source to publish the URDF-defined ROS TF tree.
    let svc = Arc::new(SomaService::new(Arc::clone(&body)));
    let runtime_state = svc.runtime();
    let snapshot_service = Arc::clone(&svc);
    let snapshot_task = tokio::spawn(async move {
        let mut tick = tokio::time::interval(Duration::from_millis(500));
        let mut seq = 0_u64;
        loop {
            tick.tick().await;
            seq += 1;
            snapshot_service.publish_runtime_snapshot(seq).await;
        }
    });
    let (body_shutdown_tx, body_shutdown_rx) = tokio::sync::oneshot::channel();
    let lifecycle = SystemLifecycleDriver::new(atlas.clone(), config.provider_id.clone());
    let body_lifecycle = lifecycle.clone();
    let driver_shutdown = lifecycle.subscribe_shutdown();
    let mut body_server = tokio::spawn(async move {
        tonic::transport::Server::builder()
            .add_service(RobonixLifecycleDriverServer::new(body_lifecycle))
            .add_service(RobonixSystemSomaGetYamlServer::from_arc(Arc::clone(&svc)))
            .add_service(RobonixSystemSomaGetUrdfServer::from_arc(Arc::clone(&svc)))
            .add_service(RobonixSystemSomaFootprintServer::from_arc(Arc::clone(&svc)))
            .add_service(RobonixSystemSomaGetHealthServer::from_arc(Arc::clone(&svc)))
            .add_service(RobonixSystemSomaHealthServer::from_arc(svc))
            .serve_with_shutdown(listen_addr, async move {
                tokio::select! {
                    _ = body_shutdown_rx => {}
                    _ = wait_for_driver_shutdown(driver_shutdown) => {}
                }
            })
            .await
    });
    wait_for_body_api(listen_addr).await?;
    info!(
        "Soma body API ready on {} before primitive bring-up",
        config.listen
    );

    let mut launcher = PackageLauncher::new(
        log_dir.clone(),
        config.atlas_endpoint.clone(),
        config.listen.clone(),
    )
    .context("create package process manager")?;

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
        let _ = body_shutdown_tx.send(());
        let _ = body_server.await;
        anyhow::bail!("Soma stage 1 (primitive bring-up) failed");
    }

    // ── Soma's own gRPC service registration ─────────────────────────
    let advertised = format!("127.0.0.1:{}", listen_addr.port());
    atlas
        .register_service(&config.provider_id, SOMA_NAMESPACE, "")
        .await
        .context("register Soma service")?;
    atlas
        .declare_capability(
            &config.provider_id,
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
        .context("declare Soma shared lifecycle Driver")?;
    let startup_endpoint = startup_driver_endpoint(listen_addr);
    let mut startup_driver = tokio::select! {
        client = connect_startup_driver(&startup_endpoint) => client?,
        result = &mut body_server => {
            result.context("join Soma body API task")?
                .context("Soma body API failed before lifecycle readiness")?;
            anyhow::bail!("Soma body API stopped before lifecycle readiness");
        }
    };
    call_startup_driver(&mut startup_driver, CMD_INIT)
        .await
        .context("initialize Soma lifecycle")?;
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
        .declare_capability(
            &config.provider_id,
            GET_FOOTPRINT_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                GET_FOOTPRINT_TOML,
                "robonix.contracts.RobonixSystemSomaFootprint",
                "/robonix.contracts.RobonixSystemSomaFootprint/GetFootprint",
            ),
        )
        .await
        .context("declare Soma footprint gRPC capability")?;
    atlas
        .declare_capability(
            &config.provider_id,
            GET_HEALTH_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                GET_HEALTH_TOML,
                "robonix.contracts.RobonixSystemSomaGetHealth",
                "/robonix.contracts.RobonixSystemSomaGetHealth/GetHealth",
            ),
        )
        .await
        .context("declare Soma get_health gRPC capability")?;
    atlas
        .declare_capability(
            &config.provider_id,
            HEALTH_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                HEALTH_TOML,
                "robonix.contracts.RobonixSystemSomaHealth",
                "/robonix.contracts.RobonixSystemSomaHealth/StreamHealth",
            ),
        )
        .await
        .context("declare Soma health stream gRPC capability")?;
    call_startup_driver(&mut startup_driver, CMD_ACTIVATE)
        .await
        .context("activate Soma lifecycle")?;
    drop(startup_driver);
    let runtime_dir = log_dir.join("soma-runtime");
    let runtime_monitor = match robonix_soma::runtime_monitor::start(
        &mut atlas,
        &config.provider_id,
        runtime_state,
        &runtime_dir,
        &config.runtime_reader_command,
    )
    .await
    {
        Ok(monitor) => monitor,
        Err(error) => {
            warn!("[soma/state] runtime monitor unavailable: {error:#}");
            None
        }
    };
    let heartbeat_failure = {
        let mut hb = atlas.clone();
        let provider_id = config.provider_id.clone();
        let shutdown = lifecycle.subscribe_shutdown();
        let (tx, rx) = tokio::sync::oneshot::channel();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            let mut failures = 0usize;
            let mut tx = Some(tx);
            tick.tick().await;
            let shutdown = wait_for_driver_shutdown(shutdown);
            tokio::pin!(shutdown);
            loop {
                tokio::select! {
                    _ = &mut shutdown => break,
                    _ = tick.tick() => {
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

    info!(
        "robonix-soma ready on {}  (robot loaded, provider_id={})",
        config.listen, config.provider_id
    );
    let serve_result: Result<()> = tokio::select! {
        _ = shutdown_signal(heartbeat_failure) => {
            let _ = body_shutdown_tx.send(());
            body_server
                .await
                .context("join Soma body API task")?
                .context("serve Soma body API")
        }
        result = &mut body_server => {
            match result {
                Ok(Ok(())) if lifecycle.shutdown_requested() => Ok(()),
                Ok(Ok(())) => Err(anyhow::anyhow!("Soma body API stopped unexpectedly")),
                Ok(Err(e)) => Err(e).context("serve Soma body API"),
                Err(e) => Err(e).context("join Soma body API task"),
            }
        }
    };
    // Always abort the stage-2 watcher before tearing down children,
    // so its pipe read doesn't come back and try to reuse a torn-down
    // launcher.
    stage2_task.abort();
    let _ = stage2_task.await;
    snapshot_task.abort();
    if let Some(runtime_monitor) = runtime_monitor {
        runtime_monitor.shutdown(&mut atlas).await;
    }
    stage2_launcher.lock().await.stop_all().await?;
    serve_result?;
    Ok(())
}

async fn wait_for_body_api(listen_addr: std::net::SocketAddr) -> Result<()> {
    let dial_addr = startup_driver_endpoint(listen_addr);
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    loop {
        match tokio::net::TcpStream::connect(&dial_addr).await {
            Ok(_) => return Ok(()),
            Err(_) if tokio::time::Instant::now() < deadline => {
                tokio::time::sleep(Duration::from_millis(50)).await;
            }
            Err(e) => {
                return Err(e).with_context(|| format!("wait for Soma body API at {dial_addr}"));
            }
        }
    }
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
///
/// The inherited pipe is registered with Tokio as a nonblocking `AsyncFd`.
/// This preserves indefinite waiting across `EAGAIN` while making task abort
/// close the read end immediately; an open writer therefore cannot keep the
/// blocking pool, runtime, or process alive after Driver shutdown.
async fn run_stage2(
    fd: RawFd,
    deployment: Deployment,
    launcher: Arc<tokio::sync::Mutex<PackageLauncher>>,
    atlas: &mut AtlasClient,
) -> Result<()> {
    info!("stage 2 watcher armed on fd {fd}; waiting for '{STAGE2_TRIGGER}' line");

    let trigger = read_stage_trigger(fd).await?;

    match trigger {
        StageTrigger::Eof => {
            warn!("stage-trigger pipe closed by rbnx before '{STAGE2_TRIGGER}'");
            return Ok(());
        }
        StageTrigger::Fired => {
            info!("stage 2 trigger received; starting skill bring-up");
        }
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

/// Read complete trigger lines from an owned nonblocking pipe. Readiness is
/// retried after `WouldBlock`, and dropping this future closes the descriptor.
async fn read_stage_trigger(fd: RawFd) -> Result<StageTrigger> {
    use nix::fcntl::{FcntlArg, OFlag, fcntl};

    // SAFETY: rbnx gives Soma exclusive ownership of this inherited pipe
    // read-end. No other code accesses it after `take_stage_fd_from_env`.
    let fd = unsafe { OwnedFd::from_raw_fd(fd) };
    let flags = fcntl(&fd, FcntlArg::F_GETFL).context("read stage-trigger fd flags")?;
    fcntl(
        &fd,
        FcntlArg::F_SETFL(OFlag::from_bits_truncate(flags) | OFlag::O_NONBLOCK),
    )
    .context("set stage-trigger fd nonblocking")?;
    let fd = tokio::io::unix::AsyncFd::new(fd).context("register stage-trigger fd")?;
    let mut pending = Vec::new();

    loop {
        let mut ready = fd.readable().await.context("wait for stage-trigger pipe")?;
        let mut chunk = [0_u8; 256];
        let count = match ready.try_io(|inner| {
            nix::unistd::read(inner.get_ref(), &mut chunk)
                .map_err(|errno| std::io::Error::from_raw_os_error(errno as i32))
        }) {
            Err(_) => continue,
            Ok(result) => result.context("read stage-trigger pipe")?,
        };
        if count == 0 {
            return Ok(StageTrigger::Eof);
        }
        pending.extend_from_slice(&chunk[..count]);
        while let Some(newline) = pending.iter().position(|byte| *byte == b'\n') {
            let line = pending.drain(..=newline).collect::<Vec<_>>();
            let line = String::from_utf8_lossy(&line);
            let trimmed = line.trim();
            if trimmed == STAGE2_TRIGGER {
                return Ok(StageTrigger::Fired);
            }
            warn!("ignoring unknown stage-trigger line '{trimmed}'");
        }
    }
}

#[derive(Debug)]
enum StageTrigger {
    /// We saw `stage2\n` on the pipe — proceed with skill bring-up.
    Fired,
    /// rbnx closed its write-end without ever sending the trigger.
    /// Treated as "skip stage 2 gracefully".
    Eof,
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

#[cfg(test)]
mod tests {
    use super::*;
    use robonix_atlas::service::{AtlasRegistry, serve_atlas};
    use robonix_soma::pb::contracts::robonix_lifecycle_driver_client::RobonixLifecycleDriverClient;
    use std::net::SocketAddr;
    use std::os::fd::IntoRawFd;

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

    /// Cancelling an armed reader must close its pipe even while rbnx keeps
    /// the write end open, otherwise Tokio runtime shutdown can hang forever.
    #[tokio::test]
    async fn stage_trigger_reader_cancellation_closes_open_pipe() {
        let (read_end, write_end) = nix::unistd::pipe().expect("create stage-trigger pipe");
        let mut reader = tokio::spawn(read_stage_trigger(read_end.into_raw_fd()));
        assert!(
            tokio::time::timeout(Duration::from_millis(25), &mut reader)
                .await
                .is_err(),
            "reader should remain armed while the writer is open"
        );

        reader.abort();
        let join_error = tokio::time::timeout(Duration::from_secs(1), reader)
            .await
            .expect("cancelled reader did not stop")
            .expect_err("cancelled reader unexpectedly completed");
        assert!(join_error.is_cancelled());
        assert_eq!(
            nix::unistd::write(&write_end, b"stage2\n"),
            Err(nix::errno::Errno::EPIPE),
            "read end remained open after cancellation"
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
        let provider_id = "soma-driver-rpc-test";
        atlas
            .register_service(provider_id, SOMA_NAMESPACE, "")
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
