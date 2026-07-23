// SPDX-License-Identifier: MulanPSL-2.0
//
// Soma's runtime bring-up of primitive + skill packages, in two stages.
//
// Stage 1 (`spawn_primitives`, called at soma startup, BEFORE soma
// advertises its gRPC service to atlas):
//   For each `primitive:` entry in the deployment manifest:
//     1. snapshot atlas's current provider set
//     2. spawn `rbnx start -p <pkg>` as a child process
//     3. wait for the new provider to register on atlas
//     4. Driver(CMD_INIT, config_json)
//     5. Driver(CMD_ACTIVATE, config_json) — primitives drive sensors
//        / actuators that must be live before any skill or pilot
//        plan can touch them.
//
// Stage 2 (`spawn_skills`, called after rbnx signals soma over a
// private inherited pipe that `rbnx boot` has finished system
// bring-up):
//   Same dance, except we stop at INIT — skills park at INACTIVE and
//   the executor sends CMD_ACTIVATE on first MCP call (lazy-activate).
//   This matches the executor's existing FSM contract; the only
//   reason we wait for stage 2 is so that skills (which may MCP-call
//   into pilot/memory/scene from their tools) don't start declaring
//   capabilities while those system services are still booting.
//
// Why split the stages? `rbnx boot` runs primitives → services →
// skills in one sequential pass against a single atlas. Soma owns
// primitive + skill now; rbnx still owns the system services in
// between. If soma tried to boot skills inline at startup, the
// executor wouldn't be up yet to receive `CMD_ACTIVATE` calls into
// the skill's MCP server later. rbnx's private stage-trigger pipe (an
// os_pipe write-end kept in rbnx, read-end dup2'd onto a known fd in
// soma at spawn time) is the bridge.
//
// All the lifecycle plumbing (`wait_for_registration_core`,
// `call_driver_cmd`, the timeouts) is reused from `robonix_cli::launch`
// so soma and rbnx CAN'T drift on FSM semantics. Omitted Driver is the
// canonical shared selection. Only an exact legacy manifest may use a current
// shared runtime Driver; a provider without exactly one Driver fails startup.

use crate::deployment::{Deployment, PackageKind, PackageLaunchTarget};
use crate::report::{PackageStartupCheck, PackageStartupStatus, StartupReport};
use anyhow::{Context, Result};
use robonix_atlas::client::AtlasClient;
use robonix_cli::launch::{
    CMD_ACTIVATE, CMD_INIT, PackageRuntimeRecord, call_driver_cmd, resolve_runtime_driver_contract,
    shutdown_package_runtime, snapshot_provider_ids, terminate_process_group,
    wait_for_registration_core,
};
use robonix_cli::process::ProcessManager;
use robonix_scribe::{info, warn};
use std::path::PathBuf;
use std::process::Stdio;
use std::sync::Arc;
use std::time::Duration;
use tokio::io::AsyncBufReadExt;
use tokio::process::Command as TokioCommand;
use tokio::task::JoinHandle;

/// Binary name soma execs to spawn each primitive/skill package. Kept
/// as a constant (not a config field) because there is exactly one
/// rbnx co-installed with each soma — if that ever changes it's a
/// deployment-tooling problem, not a soma runtime knob.
const RBNX_BIN: &str = "rbnx";

pub struct PackageLauncher {
    manager: Arc<ProcessManager>,
    atlas_endpoint: String,
    soma_endpoint: String,
    log_dir: PathBuf,
    /// Track every async pipe-forwarding task we spawn so `stop_all`
    /// can abort them when soma shuts down. The ProcessManager kills
    /// child processes; these tasks just drain stdout/stderr.
    tasks: Vec<JoinHandle<()>>,
    /// Runtime records for `rbnx start -p` wrappers spawned directly by Soma.
    children: Vec<PackageRuntimeRecord>,
}

impl PackageLauncher {
    /// Build a launcher that spawns packages via `rbnx start -p` and
    /// drives their Driver(CMD_*) lifecycle through `atlas`.
    pub fn new(
        log_dir: PathBuf,
        atlas_endpoint: impl Into<String>,
        soma_endpoint: impl Into<String>,
    ) -> Result<Self> {
        Ok(Self {
            manager: Arc::new(ProcessManager::new(log_dir.clone())?),
            atlas_endpoint: atlas_endpoint.into(),
            soma_endpoint: soma_endpoint.into(),
            log_dir,
            tasks: Vec::new(),
            children: Vec::new(),
        })
    }

    /// Stage 1: spawn every primitive declared in the deployment,
    /// run Driver(CMD_INIT) then Driver(CMD_ACTIVATE), and return a
    /// report. Best-effort: a failure on one primitive is recorded
    /// but does NOT abort bring-up of the rest — pilot can still
    /// drive whatever did come up. Operators see the failed entries
    /// in the printed report.
    pub async fn spawn_primitives(
        &mut self,
        deployment: &Deployment,
        atlas: &mut AtlasClient,
    ) -> StartupReport {
        self.run_stage(deployment, atlas, PackageKind::Primitive)
            .await
    }

    /// Stage 2: spawn every skill declared in the deployment and
    /// run Driver(CMD_INIT) only — skills park at INACTIVE waiting
    /// for the executor to send CMD_ACTIVATE on first MCP call.
    pub async fn spawn_skills(
        &mut self,
        deployment: &Deployment,
        atlas: &mut AtlasClient,
    ) -> StartupReport {
        self.run_stage(deployment, atlas, PackageKind::Skill).await
    }

    async fn run_stage(
        &mut self,
        deployment: &Deployment,
        atlas: &mut AtlasClient,
        kind: PackageKind,
    ) -> StartupReport {
        let targets: &[PackageLaunchTarget] = match kind {
            PackageKind::Primitive => &deployment.primitives,
            PackageKind::Skill => &deployment.skills,
        };
        // Only carry per-stage targets in the per-deployment
        // report so the stage 1 / stage 2 prints don't conflate
        // primitives and skills. `skipped` belongs to the
        // deployment as a whole; emit it once with stage 1 (the
        // first call) and as an empty list with stage 2.
        let skipped = if kind == PackageKind::Primitive {
            deployment.skipped.clone()
        } else {
            Vec::new()
        };
        let mut report = StartupReport {
            deployment_path: deployment.deployment_path.clone(),
            manifest_path: deployment.manifest_path.clone(),
            packages: Vec::new(),
            skipped,
        };
        for target in targets {
            let status = self.bring_up_one(target, atlas).await;
            report.packages.push(PackageStartupCheck {
                kind: target.kind,
                name: target.name.clone(),
                package_dir: target.package_dir.clone(),
                package_manifest_path: target.package_manifest_path.clone(),
                status,
            });
        }
        report
    }

    async fn bring_up_one(
        &mut self,
        target: &PackageLaunchTarget,
        atlas: &mut AtlasClient,
    ) -> PackageStartupStatus {
        // Surface the most common "you forgot to run `rbnx build`"
        // failure with the actual missing path so the operator
        // doesn't have to spelunk through logs to figure out which
        // url-remote package didn't get cloned.
        if !target.package_manifest_path.is_file() {
            return PackageStartupStatus::MissingManifest;
        }

        let command = self.command_line(target);
        let (expected_driver_contract, allow_shared_driver_upgrade) =
            match robonix_cli::manifest::load_from_path(&target.package_manifest_path).and_then(
                |manifest| {
                    manifest.validate_and_summarize()?;
                    let allow_upgrade =
                        manifest
                            .explicit_lifecycle_driver_contract()?
                            .is_some_and(|contract| {
                                contract != robonix_cli::manifest::SHARED_LIFECYCLE_DRIVER_CONTRACT
                            });
                    Ok((
                        manifest.selected_lifecycle_driver_contract()?.to_string(),
                        allow_upgrade,
                    ))
                },
            ) {
                Ok(contract) => contract,
                Err(error) => {
                    return PackageStartupStatus::SpawnFailed {
                        command,
                        error: format!("selected package manifest: {error:#}"),
                    };
                }
            };

        // Snapshot the expected instance's registration generation before
        // spawn. Other providers may register concurrently; the shared waiter
        // ignores them and only accepts target.name with a new generation.
        let before = match snapshot_provider_ids(atlas).await {
            Ok(s) => s,
            Err(e) => {
                return PackageStartupStatus::SpawnFailed {
                    command,
                    error: format!("pre-spawn atlas snapshot: {e:#}"),
                };
            }
        };

        // Spawn the child. We piggyback on `rbnx start -p <pkg>`
        // (same entrypoint rbnx boot uses) so soma doesn't ship its
        // own start-script loader. Stdout/stderr stream into scribe
        // under the provider_id as tag — matches rbnx boot's logging
        // exactly, so `rbnx logs -t <provider_id>` works whether the
        // package was started by rbnx or soma.
        let (pid, mut child) = match self.spawn_child(target).await {
            Ok(v) => v,
            Err(e) => {
                return PackageStartupStatus::SpawnFailed {
                    command,
                    error: format!("spawn: {e:#}"),
                };
            }
        };

        // Wait for registration, then drive INIT [+ ACTIVATE].
        // Race the registration poll against child exit: if the
        // package crashes before it registers with atlas, surface
        // that immediately instead of sitting on the full
        // DRIVER_REGISTER_TIMEOUT (60s). Mirrors the try_wait
        // early-exit detection rbnx uses for soma itself (deploy.rs).
        let who = format!("{}/{}", target.kind, target.name);
        let outcome = tokio::select! {
            result = wait_for_registration_core(atlas, &before, &target.name, &who) => match result {
                Ok(o) => o,
                Err(e) => {
                    self.reap(pid).await;
                    return PackageStartupStatus::SpawnFailed {
                        command,
                        error: format!("{e:#}"),
                    };
                }
            },
            status = child.wait() => {
                self.reap(pid).await;
                return PackageStartupStatus::SpawnFailed {
                    command,
                    error: format!(
                        "package exited before registering with atlas (status={status:?})"
                    ),
                };
            }
        };
        if outcome.provider_id != target.name {
            self.reap(pid).await;
            return PackageStartupStatus::SpawnFailed {
                command,
                error: format!(
                    "provider_id mismatch: manifest name='{}' but Capability(id='{}') registered",
                    target.name, outcome.provider_id,
                ),
            };
        }
        let driver_contract = match resolve_runtime_driver_contract(
            &outcome.provider_id,
            &outcome.provider_namespace,
            &expected_driver_contract,
            &outcome.driver_contracts,
            allow_shared_driver_upgrade,
        ) {
            Ok(contract) => contract,
            Err(error) => {
                self.reap(pid).await;
                return PackageStartupStatus::SpawnFailed {
                    command,
                    error: format!("lifecycle: {error:#}"),
                };
            }
        };
        if driver_contract != expected_driver_contract {
            warn!(
                "{who}: provider publishes shared lifecycle Driver '{driver_contract}' for legacy manifest selection '{expected_driver_contract}'; remove the legacy Driver declaration to finish migration"
            );
        }

        let config_json = match serde_json::to_string(&target.config) {
            Ok(config_json) => config_json,
            Err(error) => {
                self.reap(pid).await;
                return PackageStartupStatus::SpawnFailed {
                    command,
                    error: format!(
                        "serialize config for deployment instance '{}': {error}",
                        target.name
                    ),
                };
            }
        };
        self.note_lifecycle(
            pid,
            outcome.provider_id.clone(),
            driver_contract.clone(),
            config_json.clone(),
        );

        if let Err(e) = call_driver_cmd(
            atlas,
            &outcome.provider_id,
            &driver_contract,
            CMD_INIT,
            config_json.clone(),
            &who,
        )
        .await
        {
            self.reap(pid).await;
            return PackageStartupStatus::SpawnFailed {
                command,
                error: format!("{e:#}"),
            };
        }

        if target.kind == PackageKind::Skill {
            // Skill: stop at INACTIVE; executor sends CMD_ACTIVATE on
            // first MCP call.
            info!("{who}: INIT ok (INACTIVE — awaits executor activate)");
            return PackageStartupStatus::Spawned { command };
        }

        if let Err(e) = call_driver_cmd(
            atlas,
            &outcome.provider_id,
            &driver_contract,
            CMD_ACTIVATE,
            config_json,
            &who,
        )
        .await
        {
            self.reap(pid).await;
            return PackageStartupStatus::SpawnFailed {
                command,
                error: format!("{e:#}"),
            };
        }
        info!("{who}: ACTIVE");
        PackageStartupStatus::Spawned { command }
    }

    /// Spawn one `rbnx start -p <pkg> [--manifest <m>]` child and
    /// pipe its stdout/stderr into scribe. Returns the child PID
    /// and handle so the caller can detect early exit (child exits
    /// before registering with atlas) and `reap` can SIGKILL it if
    /// a downstream step (INIT, ACTIVATE) fails after spawn but
    /// before we hand the child to ProcessManager for graceful
    /// shutdown.
    ///
    /// We use `tokio::process::Command` directly here instead of
    /// `ProcessManager::start_process` because the manager's API
    /// blocks-until-exit, which is the wrong shape for bring-up
    /// (we want to keep the child running and only wait for atlas
    /// registration). The manager still owns log-file orchestration
    /// via its log_dir, which we share through the constructor.
    async fn spawn_child(
        &mut self,
        target: &PackageLaunchTarget,
    ) -> Result<(u32, tokio::process::Child)> {
        let package_manifest = robonix_cli::manifest::load_from_path(&target.package_manifest_path)
            .with_context(|| format!("load {}", target.package_manifest_path.display()))?;
        let stop = package_manifest.stop.trim().to_string();

        // Match rbnx deploy's provider endpoint semantics: atlas may listen on
        // 0.0.0.0, but providers need a dialable address. In-container Webots
        // drivers can still override this in their start.sh via ROBONIX_SIM_ATLAS.
        let provider_atlas = self.provider_atlas_endpoint();

        let mut cmd = TokioCommand::new(RBNX_BIN);
        cmd.arg("start")
            .arg("-p")
            .arg(target.package_dir.as_os_str())
            .arg("--endpoint")
            .arg(&provider_atlas)
            .env("RBNX_INSTANCE_NAME", &target.name)
            .env("RBNX_INVOCATION_CWD", &target.package_dir)
            // This `rbnx start` is already the leader of a Soma-owned PGID.
            // Keep its package shell in that group so Soma's stop_all reaches
            // the wrapper and real driver together; standalone rbnx start
            // continues to create its own PGID.
            .env("RBNX_DEPLOY_MANAGED", "1")
            .env("SCRIBE_LOG_DIR", &self.log_dir)
            .env("ROBONIX_SOMA_ENDPOINT", self.provider_soma_endpoint())
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .process_group(0);
        if let Some(m) = target.manifest_override.as_deref() {
            cmd.arg("--manifest").arg(m);
        }
        let mut child = cmd.spawn().with_context(|| {
            format!(
                "spawn '{} start -p {}'",
                RBNX_BIN,
                target.package_dir.display()
            )
        })?;
        let pid = child.id().ok_or_else(|| {
            anyhow::anyhow!("spawned package '{}' but it had no pid", target.name)
        })?;
        self.children.push(PackageRuntimeRecord {
            name: target.name.clone(),
            kind: target.kind.to_string(),
            pid,
            pgid: pid,
            provider_id: None,
            driver_contract: None,
            config_json: None,
            package_dir: Some(target.package_dir.display().to_string()),
            stop: if stop.is_empty() { None } else { Some(stop) },
        });
        let stdout = child.stdout.take().expect("stdout piped");
        let stderr = child.stderr.take().expect("stderr piped");
        let tag_out = target.name.clone();
        let tag_err = target.name.clone();
        self.tasks.push(tokio::spawn(async move {
            let reader = tokio::io::BufReader::new(stdout);
            let mut lines = reader.lines();
            while let Ok(Some(line)) = lines.next_line().await {
                robonix_scribe::info(&tag_out, &line);
            }
        }));
        self.tasks.push(tokio::spawn(async move {
            // stderr is not always errors — Python logging defaults
            // to stderr for INFO too. Use info so we don't lie about
            // severity.
            let reader = tokio::io::BufReader::new(stderr);
            let mut lines = reader.lines();
            while let Ok(Some(line)) = lines.next_line().await {
                robonix_scribe::info(&tag_err, &line);
            }
        }));
        // Return the child handle alongside the pid so the caller
        // can detect early exit — a package that crashes before it
        // registers with atlas would otherwise cause soma to wait
        // the full DRIVER_REGISTER_TIMEOUT (60s) before moving on.
        // process_group(0) made the child its own PGID leader;
        // SIGKILL on `pid` (and `-pid` for the group) still works
        // whether or not the handle is alive. The caller drops the
        // handle after registration succeeds; tokio::process::Child::
        // drop detaches without killing, so the process runs on.
        Ok((pid, child))
    }

    fn note_lifecycle(
        &mut self,
        pid: u32,
        provider_id: String,
        driver_contract: String,
        config_json: String,
    ) {
        if let Some(record) = self.children.iter_mut().find(|record| record.pid == pid) {
            record.provider_id = Some(provider_id);
            record.driver_contract = Some(driver_contract);
            record.config_json = Some(config_json);
        }
    }

    /// Stop a failed package without bypassing provider cleanup.  Providers
    /// install SIGTERM handlers that run `on_shutdown`; an immediate SIGKILL
    /// leaves independently-sessioned ROS children behind under PID 1.
    async fn reap(&self, pid: u32) {
        terminate_process_group(pid, Duration::from_secs(8)).await;
    }

    /// Stop all packages launched by soma and abort their
    /// pipe-forwarding tasks. Called on SIGINT/SIGTERM in main.
    pub async fn stop_all(&mut self) -> Result<()> {
        let provider_atlas = self.provider_atlas_endpoint();
        for record in self.children.drain(..).rev() {
            shutdown_package_runtime(Some(&provider_atlas), &record, Duration::from_millis(500))
                .await;
        }
        if let Err(e) = self.manager.stop_all().await {
            warn!("stop ProcessManager-owned Soma packages: {e:#}");
        }
        for task in self.tasks.drain(..) {
            task.abort();
        }
        Ok(())
    }

    pub fn command_line(&self, target: &PackageLaunchTarget) -> String {
        format!(
            "{} start -p {} --endpoint {}",
            RBNX_BIN,
            target.package_dir.display(),
            self.provider_atlas_endpoint()
        )
    }

    fn provider_atlas_endpoint(&self) -> String {
        self.atlas_endpoint.replacen("0.0.0.0", "127.0.0.1", 1)
    }

    fn provider_soma_endpoint(&self) -> String {
        self.soma_endpoint.replacen("0.0.0.0", "127.0.0.1", 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deployment::{PackageKind, PackageLaunchTarget};

    #[test]
    /// The command passes `-p` directly and avoids rebasing through invocation cwd.
    fn command_line_uses_package_path_without_invocation_cwd_override() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let launcher = PackageLauncher::new(
            tmp.path().join("logs"),
            "127.0.0.1:50051",
            "127.0.0.1:50091",
        )
        .expect("launcher");
        let target = PackageLaunchTarget {
            kind: PackageKind::Primitive,
            name: "demo".into(),
            package_dir: PathBuf::from("/tmp/robonix/examples/test_ci/primitives/demo"),
            package_manifest_path: PathBuf::from(
                "/tmp/robonix/examples/test_ci/primitives/demo/package_manifest.yaml",
            ),
            manifest_override: None,
            config: serde_yaml::Value::Null,
        };

        assert_eq!(
            launcher.command_line(&target),
            "rbnx start -p /tmp/robonix/examples/test_ci/primitives/demo --endpoint 127.0.0.1:50051"
        );
    }

    #[test]
    fn command_line_rewrites_bind_all_atlas_to_loopback_for_packages() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let launcher =
            PackageLauncher::new(tmp.path().join("logs"), "0.0.0.0:50051", "0.0.0.0:50091")
                .expect("launcher");
        let target = PackageLaunchTarget {
            kind: PackageKind::Primitive,
            name: "demo".into(),
            package_dir: PathBuf::from("/tmp/robonix/examples/test_ci/primitives/demo"),
            package_manifest_path: PathBuf::from(
                "/tmp/robonix/examples/test_ci/primitives/demo/package_manifest.yaml",
            ),
            manifest_override: None,
            config: serde_yaml::Value::Null,
        };

        assert_eq!(
            launcher.command_line(&target),
            "rbnx start -p /tmp/robonix/examples/test_ci/primitives/demo --endpoint 127.0.0.1:50051"
        );
    }

    #[test]
    fn soma_bind_all_address_is_dialed_through_loopback() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let launcher =
            PackageLauncher::new(tmp.path().join("logs"), "127.0.0.1:50051", "0.0.0.0:50091")
                .expect("launcher");

        assert_eq!(launcher.provider_soma_endpoint(), "127.0.0.1:50091");
    }

    #[tokio::test]
    async fn failed_package_reap_runs_sigterm_cleanup_before_exit() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let marker = tmp.path().join("sigterm-cleanup-ran");
        let launcher = PackageLauncher::new(
            tmp.path().join("logs"),
            "127.0.0.1:50051",
            "127.0.0.1:50091",
        )
        .expect("launcher");
        let mut child = TokioCommand::new("bash");
        child
            .arg("-c")
            .arg(concat!(
                "trap 'printf cleaned > \"$MARKER\"; exit 0' TERM; ",
                "while :; do sleep 0.1; done"
            ))
            .env("MARKER", &marker)
            .process_group(0);
        let mut child = child.spawn().expect("spawn cleanup probe");
        let pid = child.id().expect("probe pid");
        tokio::time::sleep(Duration::from_millis(100)).await;

        launcher.reap(pid).await;
        let status = tokio::time::timeout(Duration::from_secs(2), child.wait())
            .await
            .expect("probe exit timeout")
            .expect("wait probe");

        assert!(status.success());
        assert_eq!(
            std::fs::read_to_string(marker).expect("cleanup marker"),
            "cleaned"
        );
    }
}
