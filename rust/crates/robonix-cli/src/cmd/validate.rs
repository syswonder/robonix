// SPDX-License-Identifier: MulanPSL-2.0
// Validate command: check package manifest without building

use super::run_package::resolve_local_path_for_filesystem;
use anyhow::{Context, Result};
use robonix_cli::manifest;
use robonix_cli::output;
use std::path::PathBuf;

pub async fn execute(path: PathBuf) -> Result<()> {
    let path = resolve_local_path_for_filesystem(&path)?;
    let package_root = path
        .canonicalize()
        .with_context(|| format!("Failed to canonicalize: {}", path.display()))?;

    output::action(
        "Validating",
        &format!("package at {}", package_root.display()),
    );
    let detected = manifest::detect_and_load(&package_root)?;
    let summary = detected.manifest.validate_and_summarize()?;

    output::check(&format!("Manifest: {}", detected.path.display()));
    output::check(&format!("Package: {} {}", summary.name, summary.version));

    if !summary.nodes.is_empty() {
        output::sub_step(&format!("Nodes: {}", summary.nodes.join(", ")));
    }
    if !summary.provided_interfaces.is_empty() {
        output::sub_step(&format!(
            "Provides: {}",
            summary.provided_interfaces.join(", ")
        ));
    }
    if !summary.consumed_interfaces.is_empty() {
        output::sub_step(&format!(
            "Consumes: {}",
            summary.consumed_interfaces.join(", ")
        ));
    }

    output::success("Manifest validation passed");
    Ok(())
}
