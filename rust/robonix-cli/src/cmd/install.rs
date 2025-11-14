use crate::{Config, PackageInstaller};
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
        let package_name = installer.install_from_github(&repo, branch.as_deref())?;
        println!("Successfully installed package: {}", package_name);
    } else if let Some(source_path) = path {
        let package_name = installer.install_from_path(&source_path)?;
        println!("Successfully installed package: {}", package_name);
    } else {
        eprintln!("Error: Either --github or --path must be specified");
        std::process::exit(1);
    }

    Ok(())
}
