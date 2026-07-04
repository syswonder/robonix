// SPDX-License-Identifier: MulanPSL-2.0
//
// Startup report for one soma bring-up stage. Flat: soma serves one
// deployment, so the report is one deployment's worth of package
// checks plus the manifest's skipped list. The v1 shape carried a
// `Vec<DeploymentStartupReport>`; the reviewer flagged that as
// speculative generality, so we've collapsed it.

use crate::deployment::{PackageKind, SkippedPackage};
use std::path::PathBuf;

#[derive(Debug, Default)]
pub struct StartupReport {
    pub deployment_path: PathBuf,
    pub manifest_path: PathBuf,
    pub packages: Vec<PackageStartupCheck>,
    /// Manifest-level skipped entries (e.g. `service:` blocks soma
    /// hands off to rbnx). Populated once by stage 1; stage 2's
    /// report leaves this empty.
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
    MissingManifest,
    Spawned { command: String },
    SpawnFailed { command: String, error: String },
}

impl StartupReport {
    pub fn has_failures(&self) -> bool {
        self.packages.iter().any(|package| {
            matches!(
                package.status,
                PackageStartupStatus::MissingManifest | PackageStartupStatus::SpawnFailed { .. }
            )
        })
    }

    /// Print a compact startup table to stderr so operators can inspect bring-up results.
    pub fn print_to_terminal(&self) {
        eprintln!();
        eprintln!("========== Soma startup report ==========");
        eprintln!(
            "deployment: {}  manifest: {}",
            self.deployment_path.display(),
            self.manifest_path.display()
        );
        if self.packages.is_empty() && self.skipped.is_empty() {
            eprintln!("  no primitive/skill packages declared");
        }
        for package in &self.packages {
            match &package.status {
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
        for skipped in &self.skipped {
            eprintln!(
                "  [SKIP] {} {}: {}",
                skipped.kind, skipped.name, skipped.reason
            );
        }
        eprintln!("=========================================");
        eprintln!();
    }
}
