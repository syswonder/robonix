// SPDX-License-Identifier: MulanPSL-2.0
// Validate command: check package manifest without building

use anyhow::Result;
use robonix_cli::manifest;
use robonix_cli::output;
use std::path::PathBuf;

pub async fn execute(path: PathBuf) -> Result<()> {
    let package_root = path
        .canonicalize()
        .map_err(anyhow::Error::from)
        .unwrap_or(path.clone());

    output::action(
        "Validating",
        &format!("package at {}", package_root.display()),
    );
    let detected = manifest::detect_and_load(&package_root)?;
    let script_rel = detected.manifest.build.script.trim();
    if !script_rel.is_empty() {
        let p = package_root.join(script_rel);
        if !p.is_file() {
            anyhow::bail!(
                "manifest.build.script not found: {} (declared as {:?})",
                p.display(),
                script_rel
            );
        }
    }
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
