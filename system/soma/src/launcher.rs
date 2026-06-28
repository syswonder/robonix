// SPDX-License-Identifier: MulanPSL-2.0
//
// Soma's runtime bring-up of primitive + skill packages, in two stages.
//
// Stage 1 (`spawn_primitives`, called at soma startup, BEFORE soma
// advertises its gRPC service to atlas):
//   For each `primitive:` entry in every deployment manifest:
//     1. snapshot atlas's current provider set
//     2. spawn `rbnx start -p <pkg>` as a child process
//     3. wait for the new provider to register on atlas
//     4. Driver(CMD_INIT, config_json)
//     5. Driver(CMD_ACTIVATE, config_json) — primitives drive sensors
//        / actuators that must be live before any skill or pilot
//        plan can touch them.
//
// Stage 2 (`spawn_skills`, called after atlas notifies soma via
// `WatchProvider` that `rbnx boot` has finished system bring-up):
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
// the skill's MCP server later. Atlas's reverse notification
// (`WatchProvider("soma")` + `NotifyProvider("soma", stage_trigger=
// "stage2")` from rbnx after non-builtin system bring-up completes)
// is the bridge.
//
// All the lifecycle plumbing (`wait_for_registration_core`,
// `call_driver_cmd`, the timeouts) is reused from `robonix_cli::launch`
// so soma and rbnx CAN'T drift on FSM semantics.

use crate::deployment::{DeploymentStore, PackageKind, PackageLaunchTarget};
use crate::report::{
    DeploymentStartupReport, PackageStartupCheck, PackageStartupStatus, StartupReport,
};
use anyhow::{Context, Result};
use robonix_atlas::client::AtlasClient;
use robonix_cli::launch::{
    CMD_ACTIVATE, CMD_INIT, call_driver_cmd, snapshot_provider_ids, wait_for_registration_core,
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

pub struct PackageLauncher {
    manager: Arc<ProcessManager>,
    rbnx_bin: String,
    atlas_endpoint: String,
    log_dir: PathBuf,
    /// Track every async pipe-forwarding task we spawn so `stop_all`
    /// can abort them when soma shuts down. The ProcessManager kills
    /// child processes; these tasks just drain stdout/stderr.
    tasks: Vec<JoinHandle<()>>,
}

impl PackageLauncher {
    /// Build a launcher that spawns packages via `rbnx start -p` and
    /// drives their Driver(CMD_*) lifecycle through `atlas`.
    pub fn new(
        log_dir: PathBuf,
        rbnx_bin: impl Into<String>,
        atlas_endpoint: impl Into<String>,
    ) -> Result<Self> {
        Ok(Self {
            manager: Arc::new(ProcessManager::new(log_dir.clone())?),
            rbnx_bin: rbnx_bin.into(),
            atlas_endpoint: atlas_endpoint.into(),
            log_dir,
            tasks: Vec::new(),
        })
    }

    /// Stage 1: spawn every primitive declared in every deployment,
    /// run Driver(CMD_INIT) then Driver(CMD_ACTIVATE), and return a
    /// report. Best-effort: a failure on one primitive is recorded
    /// but does NOT abort bring-up of the rest — pilot can still
    /// drive whatever did come up. Operators see the failed entries
    /// in the printed report.
    pub async fn spawn_primitives(
        &mut self,
        deployments: &DeploymentStore,
        atlas: &mut AtlasClient,
        start_packages: bool,
    ) -> StartupReport {
        self.run_stage(deployments, atlas, start_packages, PackageKind::Primitive)
            .await
    }

    /// Stage 2: spawn every skill declared in every deployment and
    /// run Driver(CMD_INIT) only — skills park at INACTIVE waiting
    /// for the executor to send CMD_ACTIVATE on first MCP call.
    pub async fn spawn_skills(
        &mut self,
        deployments: &DeploymentStore,
        atlas: &mut AtlasClient,
        start_packages: bool,
    ) -> StartupReport {
        self.run_stage(deployments, atlas, start_packages, PackageKind::Skill)
            .await
    }

    async fn run_stage(
        &mut self,
        deployments: &DeploymentStore,
        atlas: &mut AtlasClient,
        start_packages: bool,
        kind: PackageKind,
    ) -> StartupReport {
        let mut report = StartupReport::default();
        for deployment in deployments.records() {
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
            let mut deployment_report = DeploymentStartupReport {
                deployment_path: deployment.deployment_path.clone(),
                manifest_path: deployment.manifest_path.clone(),
                packages: Vec::new(),
                skipped,
            };
            for target in targets {
                let status = self.bring_up_one(target, atlas, start_packages).await;
                deployment_report.packages.push(PackageStartupCheck {
                    kind: target.kind,
                    name: target.name.clone(),
                    package_dir: target.package_dir.clone(),
                    package_manifest_path: target.package_manifest_path.clone(),
                    status,
                });
            }
            report.deployments.push(deployment_report);
        }
        report
    }

    async fn bring_up_one(
        &mut self,
        target: &PackageLaunchTarget,
        atlas: &mut AtlasClient,
        start_packages: bool,
    ) -> PackageStartupStatus {
        if !start_packages {
            return PackageStartupStatus::StartDisabled;
        }
        // Surface the most common "you forgot to run `rbnx build`"
        // failure with the actual missing path so the operator
        // doesn't have to spelunk through logs to figure out which
        // url-remote package didn't get cloned.
        if !target.package_manifest_path.is_file() {
            return PackageStartupStatus::MissingManifest;
        }

        let command = self.command_line(target);

        // Snapshot before spawn so wait_for_registration_core can
        // tell us which new provider is OUR child (not some
        // pre-existing one whose driver replied to a polling tick).
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
        let pid = match self.spawn_child(target).await {
            Ok(pid) => pid,
            Err(e) => {
                return PackageStartupStatus::SpawnFailed {
                    command,
                    error: format!("spawn: {e:#}"),
                };
            }
        };

        // Wait for registration, then drive INIT [+ ACTIVATE].
        let who = format!("{}/{}", target.kind, target.name);
        let outcome = match wait_for_registration_core(atlas, &before, &who).await {
            Ok(o) => o,
            Err(e) => {
                self.reap(pid).await;
                return PackageStartupStatus::SpawnFailed {
                    command,
                    error: format!("{e:#}"),
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
        let Some(driver_contract) = outcome.driver_contract else {
            // No driver contract = system-style auto-promotion. We
            // don't expect this for primitives or skills, but mirror
            // rbnx's "treat as ACTIVE" behaviour so a misclassified
            // entry doesn't bring boot down.
            warn!("{who}: registered without a */driver capability — skipping INIT/ACTIVATE",);
            return PackageStartupStatus::Spawned { command };
        };

        let config_json = serde_json::to_string(&target.config).unwrap_or_else(|_| "{}".into());

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
    /// pipe its stdout/stderr into scribe. Returns the child PID so
    /// `reap` can SIGKILL it if a downstream step (INIT, ACTIVATE)
    /// fails after spawn but before we hand the child to
    /// ProcessManager for graceful shutdown.
    ///
    /// We use `tokio::process::Command` directly here instead of
    /// `ProcessManager::start_process` because the manager's API
    /// blocks-until-exit, which is the wrong shape for bring-up
    /// (we want to keep the child running and only wait for atlas
    /// registration). The manager still owns log-file orchestration
    /// via its log_dir, which we share through the constructor.
    async fn spawn_child(&mut self, target: &PackageLaunchTarget) -> Result<u32> {
        let mut cmd = TokioCommand::new(&self.rbnx_bin);
        cmd.arg("start")
            .arg("-p")
            .arg(target.package_dir.as_os_str())
            .arg("--endpoint")
            .arg(&self.atlas_endpoint)
            .env("RBNX_INSTANCE_NAME", &target.name)
            .env("RBNX_INVOCATION_CWD", &target.package_dir)
            .env("SCRIBE_LOG_DIR", &self.log_dir)
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
                self.rbnx_bin,
                target.package_dir.display()
            )
        })?;
        let pid = child.id().ok_or_else(|| {
            anyhow::anyhow!("spawned package '{}' but it had no pid", target.name)
        })?;
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
        // We deliberately let `child` drop here. process_group(0)
        // made the child its own PGID leader; SIGKILL on `pid` (and
        // `-pid` for the group) still works after drop. ProcessManager
        // doesn't own this child — we manage its lifecycle directly.
        // tokio::process::Child::drop just detaches the handle; the
        // process keeps running until we reap it on shutdown.
        let _ = child;
        Ok(pid)
    }

    /// SIGKILL the package's process group. Called when a bring-up
    /// step after spawn fails (registration timeout, INIT/ACTIVATE
    /// error) so we don't leave orphaned children holding e.g.
    /// device locks or gRPC ports. Quiet best-effort — if the
    /// process already exited, killpg returns ESRCH and we move on.
    async fn reap(&self, pid: u32) {
        let _ = nix::sys::signal::killpg(
            nix::unistd::Pid::from_raw(pid as i32),
            nix::sys::signal::Signal::SIGKILL,
        );
        // Yield once so the kernel has a chance to deliver the
        // signal before the caller's report-building runs. Not
        // strictly required — the next atlas poll will still see
        // the provider drop out — but keeps timings tidy in tests.
        tokio::time::sleep(Duration::from_millis(50)).await;
    }

    /// Stop all packages launched by soma and abort their
    /// pipe-forwarding tasks. Called on SIGINT/SIGTERM in main.
    pub async fn stop_all(&mut self) -> Result<()> {
        self.manager
            .stop_all()
            .await
            .context("stop Soma packages")?;
        for task in self.tasks.drain(..) {
            task.abort();
        }
        Ok(())
    }

    pub fn command_line(&self, target: &PackageLaunchTarget) -> String {
        format!(
            "{} start -p {} --endpoint {}",
            self.rbnx_bin,
            target.package_dir.display(),
            self.atlas_endpoint
        )
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
        let launcher = PackageLauncher::new(tmp.path().join("logs"), "rbnx", "127.0.0.1:50051")
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
}
