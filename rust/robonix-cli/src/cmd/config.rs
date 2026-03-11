// SPDX-License-Identifier: MulanPSL-2.0
// Config Command Module
//
// Config command implementation for robonix-cli

use crate::Config;
use anyhow::Result;
use colored::*;
use std::path::PathBuf;

pub async fn execute(_config: Config, set_storage_path: Option<PathBuf>, show: bool) -> Result<()> {
    let mut config = Config::load()?;
    let mut updated = false;

    if let Some(new_path) = set_storage_path {
        config.package_storage_path = new_path;
        config.save()?;
        config.ensure_storage_dir()?;
        println!(
            "Package storage path updated to: {}",
            config.package_storage_path.display()
        );
        updated = true;
    }

    if show {
        // Get actual config file path and display real absolute path
        let config_path = Config::config_file_path()?;
        // Get canonical (real) path, resolving symlinks
        let real_path = std::fs::canonicalize(&config_path).unwrap_or_else(|_| config_path.clone());

        println!(
            "{} {}",
            "Current robonix system config are located at:",
            real_path.display().to_string().bold().cyan()
        );
        println!("\n{}", "Configuration:".bold().cyan());
        let label1 = format!("{:<25}", "Package storage path:");
        println!(
            "  {} {}",
            label1.bright_white(),
            config.package_storage_path.display().to_string().white()
        );
        println!();
    } else if !updated {
        eprintln!("Error: Either --set-storage-path or --show must be specified");
        std::process::exit(1);
    }

    Ok(())
}
