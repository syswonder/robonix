// SPDX-License-Identifier: MulanPSL-2.0
// Validate Command Module
//
// Manifest validation for robonix-cli

use robonix_cli::manifest;
use robonix_cli::output;
use anyhow::Result;
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

    let interface_check = manifest::validate_interface_references(&summary, &package_root)?;
    if let Some(catalog_root) = interface_check.catalog_root {
        output::check(&format!("Interface catalog: {}", catalog_root.display()));
        if !interface_check.checked_interfaces.is_empty() {
            output::sub_step(&format!(
                "Validated interfaces: {}",
                interface_check.checked_interfaces.join(", ")
            ));
        }
    } else if !interface_check.checked_interfaces.is_empty() {
        output::warning("Interface catalog not found nearby, skipping RIDL existence checks");
    }

    output::success("Manifest validation passed");
    Ok(())
}
