use crate::{Config, PackageDatabase};
use anyhow::Result;

pub async fn execute(config: Config) -> Result<()> {
    let db = PackageDatabase::load(&config.package_storage_path)?;
    let packages = db.list_packages();

    if packages.is_empty() {
        println!("No packages installed.");
    } else {
        // Calculate column widths for alignment
        let max_name_len = packages
            .iter()
            .map(|p| p.name.len())
            .max()
            .unwrap_or(0)
            .max(7); // "Package" header
        let max_version_len = packages
            .iter()
            .map(|p| p.version.len())
            .max()
            .unwrap_or(0)
            .max(7); // "Version" header

        // Print header
        println!(
            "\x1b[1m{:<name_width$}  {:<ver_width$}  Prms  Srvs  Skills\x1b[0m",
            "Package",
            "Version",
            name_width = max_name_len,
            ver_width = max_version_len
        );
        println!(
            "{}  {}  {}  {}  {}",
            "─".repeat(max_name_len),
            "─".repeat(max_version_len),
            "────",
            "────",
            "──────"
        );

        // Print packages
        for pkg in packages {
            let prm_count = pkg.primitives.len();
            let srv_count = pkg.services.len();
            let skill_count = pkg.skills.len();
            let name_formatted = format!("\x1b[1;37m{}\x1b[0m", pkg.name);
            // Calculate padding for name (ANSI codes don't count toward width)
            let name_padding = if pkg.name.len() < max_name_len {
                max_name_len - pkg.name.len()
            } else {
                0
            };
            println!(
                "{}{}  {:<ver_width$}  {:4}  {:4}  {:6}",
                name_formatted,
                " ".repeat(name_padding),
                pkg.version,
                prm_count,
                srv_count,
                skill_count,
                ver_width = max_version_len
            );
        }
    }

    Ok(())
}
