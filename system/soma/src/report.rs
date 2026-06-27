// SPDX-License-Identifier: MulanPSL-2.0

use crate::deployment::{PackageKind, SkippedPackage};
use std::path::PathBuf;

#[derive(Debug, Default)]
pub struct StartupReport {
    pub deployments: Vec<DeploymentStartupReport>,
}

#[derive(Debug)]
pub struct DeploymentStartupReport {
    pub deployment_path: PathBuf,
    pub manifest_path: PathBuf,
    pub packages: Vec<PackageStartupCheck>,
    pub skipped: Vec<SkippedPackage>,
}

#[derive(Debug)]
pub struct PackageStartupCheck {
    pub kind: PackageKind,
    pub name: String,
    pub package_dir: PathBuf,
    pub package_manifest_path: PathBuf,
    pub status: PackageStartupStatus,
}

#[derive(Debug)]
pub enum PackageStartupStatus {
    StartDisabled,
    MissingManifest,
    Spawned { command: String },
    SpawnFailed { command: String, error: String },
}

impl StartupReport {
    pub fn has_failures(&self) -> bool {
        self.deployments.iter().any(|deployment| {
            deployment.packages.iter().any(|package| {
                matches!(
                    package.status,
                    PackageStartupStatus::MissingManifest
                        | PackageStartupStatus::SpawnFailed { .. }
                )
            })
        })
    }

    /// Print a compact startup table to stderr so operators can inspect bring-up results.
    pub fn print_to_terminal(&self) {
        eprintln!();
        eprintln!("========== Soma startup report ==========");
        for deployment in &self.deployments {
            eprintln!(
                "deployment: {}  manifest: {}",
                deployment.deployment_path.display(),
                deployment.manifest_path.display()
            );
            if deployment.packages.is_empty() && deployment.skipped.is_empty() {
                eprintln!("  no primitive/skill packages declared");
            }
            for package in &deployment.packages {
                match &package.status {
                    PackageStartupStatus::StartDisabled => eprintln!(
                        "  [SKIP] {} {}: package startup disabled",
                        package.kind, package.name
                    ),
                    PackageStartupStatus::MissingManifest => eprintln!(
                        "  [FAIL] {} {}: missing {}",
                        package.kind,
                        package.name,
                        package.package_manifest_path.display()
                    ),
                    PackageStartupStatus::Spawned { command } => eprintln!(
                        "  [ OK ] {} {}: cmd={}",
                        package.kind, package.name, command
                    ),
                    PackageStartupStatus::SpawnFailed { command, error } => eprintln!(
                        "  [FAIL] {} {}: {} cmd={}",
                        package.kind, package.name, error, command
                    ),
                }
            }
            for skipped in &deployment.skipped {
                eprintln!(
                    "  [SKIP] {} {}: {}",
                    skipped.kind, skipped.name, skipped.reason
                );
            }
        }
        eprintln!("=========================================");
        eprintln!();
    }
}
