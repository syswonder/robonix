// SPDX-License-Identifier: MulanPSL-2.0
// Validate command: check package manifest without building

use super::run_package::{find_package_from_cwd, resolve_local_path_for_filesystem};
use anyhow::{Context, Result};
use robonix_cli::manifest;
use robonix_cli::output;
use std::path::PathBuf;

pub async fn execute(path: Option<PathBuf>) -> Result<()> {
    let package_root = match path {
        Some(p) => {
            let p = resolve_local_path_for_filesystem(&p)?;
            p.canonicalize()
                .with_context(|| format!("Failed to canonicalize: {}", p.display()))?
        }
        None => find_package_from_cwd()?,
    };

    output::action(
        "Validating",
        &format!("package at {}", package_root.display()),
    );
    let detected = manifest::detect_and_load(&package_root)?;
    let summary = detected.manifest.validate_and_summarize()?;

    output::check(&format!("Manifest: {}", detected.path.display()));
    output::check(&format!("Package: {} {}", summary.name, summary.version));

    if !summary.capabilities.is_empty() {
        output::sub_step(&format!("Capabilities: {}", summary.capabilities.join(", ")));
    }
    if !summary.depends.is_empty() {
        output::sub_step(&format!("Depends: {}", summary.depends.join(", ")));
    }

    output::success("Manifest validation passed");
    Ok(())
}
