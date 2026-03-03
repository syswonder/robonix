// SPDX-License-Identifier: MulanPSL-2.0
// Install Command Module
//
// Install command implementation for robonix-cli

use crate::{Config, PackageInstaller, output};
use anyhow::Result;
use std::path::PathBuf;

pub async fn execute(
    config: Config,
    github: Option<String>,
    path: Option<PathBuf>,
    branch: Option<String>,
) -> Result<()> {
    let installer = PackageInstaller::new(config);

    if let Some(repo) = github {
        output::action("Installing", &format!("package from {}", repo));
        let package_name = installer.install_from_github(&repo, branch.as_deref())?;
        output::success(&format!(
            "Package '{}' installed successfully",
            package_name
        ));
    } else if let Some(source_path) = path {
        output::action(
            "Installing",
            &format!("package from {}", source_path.display()),
        );
        let package_name = installer.install_from_path(&source_path)?;
        output::success(&format!(
            "Package '{}' installed successfully",
            package_name
        ));
    } else {
        output::error("Either --github or --path must be specified");
        std::process::exit(1);
    }

    Ok(())
}
