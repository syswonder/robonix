// SPDX-License-Identifier: MulanPSL-2.0
// List command: list system-installed packages

use anyhow::Result;
use robonix_cli::output;
use robonix_cli::{Config, PackageDatabase};

pub async fn execute(config: Config) -> Result<()> {
    config.ensure_storage_dir()?;
    let db = PackageDatabase::load(&config.package_storage_path)?;
    let packages = db.list_packages();

    if packages.is_empty() {
        output::info("No packages installed.");
        output::sub_step(&format!(
            "Storage: {}",
            config.package_storage_path.display()
        ));
        return Ok(());
    }

    output::action("Installed", "packages");
    for pkg in packages {
        output::sub_step(&format!(
            "{} {}  ({})",
            pkg.name,
            pkg.version,
            pkg.path.display()
        ));
    }
    Ok(())
}
