// SPDX-License-Identifier: MulanPSL-2.0
// PackageNew command: create a new package skeleton

use anyhow::Result;
use robonix_cli::output;
use std::path::Path;

pub async fn execute(name: &str, pkg_type: &str, path: Option<&Path>) -> Result<()> {
    let role_dir = match pkg_type {
        "primitive" => "primitives",
        "service" => "services",
        "skill" => "skills",
        other => anyhow::bail!(
            "unknown package type '{}': expected 'primitive', 'service', or 'skill'",
            other
        ),
    };
    let target = path
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|| std::path::PathBuf::from(role_dir).join(name));
    output::action(
        "PackageNew",
        &format!("creating {} '{}' at {}", pkg_type, name, target.display()),
    );
    output::warning("package-new not yet implemented (WIP)");
    Ok(())
}
