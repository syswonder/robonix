// SPDX-License-Identifier: MulanPSL-2.0
// Info command: show details of a system-installed package

use anyhow::{Context, Result};
use robonix_cli::output;
use robonix_cli::{Config, PackageDatabase, PackageSource};

pub async fn execute(config: Config, name: &str) -> Result<()> {
    config.ensure_storage_dir()?;
    let db = PackageDatabase::load(&config.package_storage_path)?;

    let pkg = db
        .get_package(name)
        .with_context(|| format!("Package '{}' not found", name))?;

    output::action("Package", &pkg.name);
    output::sub_step(&format!("Version: {}", pkg.version));
    output::sub_step(&format!("Path: {}", pkg.path.display()));
    output::sub_step(&format!("Installed: {}", pkg.installed_at));

    match &pkg.source {
        PackageSource::Local { path } => {
            output::sub_step(&format!("Source: local ({})", path.display()));
        }
        PackageSource::GitHub {
            repo,
            branch,
            commit,
        } => {
            let branch_str = branch
                .as_ref()
                .map(|b| format!(" branch={}", b))
                .unwrap_or_default();
            output::sub_step(&format!(
                "Source: GitHub {} (commit={}){}",
                repo, commit, branch_str
            ));
        }
    }

    if !pkg.capabilities.is_empty() {
        output::sub_step(&format!("Capabilities: {}", pkg.capabilities.join(", ")));
    }
    if !pkg.depends.is_empty() {
        output::sub_step(&format!("Depends: {}", pkg.depends.join(", ")));
    }

    Ok(())
}
