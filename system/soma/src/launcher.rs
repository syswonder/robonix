// SPDX-License-Identifier: MulanPSL-2.0

use crate::deployment::DeploymentStore;
use crate::report::{
    DeploymentStartupReport, PackageStartupCheck, PackageStartupStatus, StartupReport,
};
use anyhow::{Context, Result};
use robonix_cli::process::ProcessManager;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;
use tokio::task::JoinHandle;

pub struct PackageLauncher {
    manager: Arc<ProcessManager>,
    rbnx_bin: String,
    atlas_endpoint: String,
    tasks: Vec<JoinHandle<()>>,
}

impl PackageLauncher {
    /// Build a launcher that starts packages through rbnx's process manager.
    pub fn new(
        log_dir: PathBuf,
        rbnx_bin: impl Into<String>,
        atlas_endpoint: impl Into<String>,
    ) -> Result<Self> {
        Ok(Self {
            manager: Arc::new(ProcessManager::new(log_dir)?),
            rbnx_bin: rbnx_bin.into(),
            atlas_endpoint: atlas_endpoint.into(),
            tasks: Vec::new(),
        })
    }

    /// Start primitive/skill packages and verify each child survives a short grace window.
    pub async fn start_from_deployments(
        &mut self,
        deployments: &DeploymentStore,
        start_packages: bool,
    ) -> StartupReport {
        let mut report = StartupReport::default();
        for deployment in deployments.records() {
            let mut deployment_report = DeploymentStartupReport {
                deployment_path: deployment.deployment_path.clone(),
                manifest_path: deployment.manifest_path.clone(),
                packages: Vec::new(),
                skipped: deployment.skipped.clone(),
            };
            for target in &deployment.packages {
                let status = if !start_packages {
                    PackageStartupStatus::StartDisabled
                } else if !target.package_manifest_path.is_file() {
                    PackageStartupStatus::MissingManifest
                } else {
                    let command = self.command_line(target);
                    let manager = Arc::clone(&self.manager);
                    let package_name = target.name.clone();
                    let package_kind = target.kind.to_string();
                    let package_dir = target.package_dir.clone();
                    let start_script = command.clone();
                    let task = tokio::spawn(async move {
                        if let Err(err) = manager
                            .start_process(
                                &package_name,
                                &package_name,
                                &package_kind,
                                &package_dir,
                                &start_script,
                            )
                            .await
                        {
                            robonix_scribe::warn!(
                                "package {} {} exited: {err:#}",
                                package_kind,
                                package_name
                            );
                        }
                    });
                    self.tasks.push(task);
                    tokio::time::sleep(Duration::from_millis(500)).await;
                    if self
                        .manager
                        .is_running(&target.name, &target.kind.to_string())
                    {
                        PackageStartupStatus::Spawned { command }
                    } else {
                        PackageStartupStatus::SpawnFailed {
                            command,
                            error: "process exited during startup grace window".into(),
                        }
                    }
                };
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

    /// Stop all packages launched by Soma and abort their wait tasks.
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

    pub fn command_line(&self, target: &crate::deployment::PackageLaunchTarget) -> String {
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
        };

        assert_eq!(
            launcher.command_line(&target),
            "rbnx start -p /tmp/robonix/examples/test_ci/primitives/demo --endpoint 127.0.0.1:50051"
        );
    }
}
