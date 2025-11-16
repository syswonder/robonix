use crate::Config;
use anyhow::{Context, Result};
use colored::*;
use std::path::PathBuf;

pub async fn execute(
    _config: Config,
    set_storage_path: Option<PathBuf>,
    set_msg_path: Option<PathBuf>,
    show: bool,
) -> Result<()> {
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

    if let Some(new_path) = set_msg_path {
        // Convert to absolute path
        let absolute_path = std::fs::canonicalize(&new_path)
            .with_context(|| format!("Failed to resolve path: {}", new_path.display()))?;
        
        // Verify it's a valid robonix-msg directory
        let setup_file = absolute_path.join("install").join("setup.bash");
        if !setup_file.exists() {
            anyhow::bail!(
                "Path does not appear to be a valid robonix-msg directory (missing install/setup.bash): {}",
                absolute_path.display()
            );
        }
        
        config.robonix_msg_path = Some(absolute_path.clone());
        config.save()?;
        println!(
            "Robonix-msg path updated to: {}",
            absolute_path.display()
        );
        updated = true;
    }

    if show {
        // Get actual config file path and display real absolute path
        let config_path = Config::config_file_path()?;
        // Get canonical (real) path, resolving symlinks
        let real_path = std::fs::canonicalize(&config_path)
            .unwrap_or_else(|_| config_path.clone());
        
        println!("{} {}", "Current robonix system config are located at:", real_path.display().to_string().bold().cyan());
        println!("\n{}", "Configuration:".bold().cyan());
        println!("  {} {}", "Package storage path:".bright_black(), config.package_storage_path.display().to_string().white());
        if let Some(ref msg_path) = config.robonix_msg_path {
            println!("  {} {}", "Robonix-msg path:".bright_black(), msg_path.display().to_string().white());
        } else {
            println!("  {} {}", "Robonix-msg path:".bright_black(), "(not set)".yellow());
        }
        println!();
    } else if !updated {
        eprintln!("Error: Either --set-storage-path, --set-msg-path, or --show must be specified");
        std::process::exit(1);
    }

    Ok(())
}
