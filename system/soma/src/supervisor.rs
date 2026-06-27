// SPDX-License-Identifier: MulanPSL-2.0

use crate::deployment::DeploymentStore;
use crate::launcher::{LaunchHandle, PackageLauncher};
use crate::report::{
    DeploymentStartupReport, PackageStartupCheck, PackageStartupStatus, StartupReport,
};

#[derive(Debug, Default)]
pub struct PackageSupervisor {
    children: Vec<LaunchHandle>,
}

impl PackageSupervisor {
    pub fn new() -> Self {
        Self::default()
    }

    /// Start all local primitive/skill packages and return a human-readable report.
    pub fn start_from_deployments(
        &mut self,
        deployments: &DeploymentStore,
        launcher: &PackageLauncher,
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
                    let command = launcher.command_line(target);
                    match launcher.spawn(target) {
                        Ok(handle) => {
                            let pid = handle.child.id();
                            self.children.push(handle);
                            PackageStartupStatus::Spawned { command, pid }
                        }
                        Err(err) => PackageStartupStatus::SpawnFailed {
                            command,
                            error: format!("{err:#}"),
                        },
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

    pub fn child_count(&self) -> usize {
        self.children.len()
    }
}
