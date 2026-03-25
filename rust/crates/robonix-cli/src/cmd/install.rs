// SPDX-License-Identifier: MulanPSL-2.0
// Install command: install packages from GitHub or local path

use anyhow::{Context, Result};
use robonix_cli::output;
use robonix_cli::{Config, install::PackageInstaller};
use std::path::PathBuf;

pub async fn execute(config: Config, github: Option<String>, path: Option<PathBuf>) -> Result<()> {
    let installer = PackageInstaller::new(config.clone());
    config.ensure_storage_dir()?;

    if let Some(repo) = github {
        output::action("Installing", &format!("from GitHub: {}", repo));
        let branch = None::<&str>;
        let name = installer.install_from_github(&repo, branch)?;
        output::success(&format!("Installed '{}'", name));
        return Ok(());
    }

    if let Some(p) = path {
        let canonical = p
            .canonicalize()
            .with_context(|| format!("Failed to canonicalize: {}", p.display()))?;
        output::action("Installing", &format!("from path: {}", canonical.display()));
        let name = installer.install_from_path(&canonical)?;
        output::success(&format!("Installed '{}'", name));
        return Ok(());
    }

    anyhow::bail!("Specify --github <repo> or --path <dir> to install")
}
