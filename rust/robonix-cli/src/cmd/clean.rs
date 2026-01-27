// SPDX-License-Identifier: MulanPSL-2.0
// Clean Command Module
//
// Clean command implementation for robonix-cli

use crate::Config;
use crate::output;
use anyhow::{Context, Result};
use std::fs;
use std::path::PathBuf;

pub async fn execute(config: Config) -> Result<()> {
    let logs_dir = config.package_storage_path.join("logs");

    output::action(
        "Cleaning",
        &format!("logs directory: {}", logs_dir.display()),
    );

    // Check if logs directory exists
    if !logs_dir.exists() {
        output::info(&format!(
            "Logs directory does not exist: {}",
            logs_dir.display()
        ));
        return Ok(());
    }

    if !logs_dir.is_dir() {
        anyhow::bail!(
            "Logs path exists but is not a directory: {}",
            logs_dir.display()
        );
    }

    // Count files before deletion
    let file_count = count_files_in_dir(&logs_dir)?;

    if file_count == 0 {
        output::info("Logs directory is already empty");
        return Ok(());
    }

    output::step("Found", &format!("{} log file(s) to remove", file_count));

    // Remove all files in logs directory
    remove_files_in_dir(&logs_dir)
        .with_context(|| format!("Failed to clean logs directory: {}", logs_dir.display()))?;

    output::success(&format!("Cleaned {} log file(s)", file_count));

    Ok(())
}

fn count_files_in_dir(dir: &PathBuf) -> Result<usize> {
    let mut count = 0;
    let entries = fs::read_dir(dir)
        .with_context(|| format!("Failed to read directory: {}", dir.display()))?;

    for entry in entries {
        let entry = entry.context("Failed to read directory entry")?;
        let path = entry.path();
        if path.is_file() {
            count += 1;
        }
    }

    Ok(count)
}

fn remove_files_in_dir(dir: &PathBuf) -> Result<()> {
    let entries = fs::read_dir(dir)
        .with_context(|| format!("Failed to read directory: {}", dir.display()))?;

    for entry in entries {
        let entry = entry.context("Failed to read directory entry")?;
        let path = entry.path();
        if path.is_file() {
            fs::remove_file(&path)
                .with_context(|| format!("Failed to remove file: {}", path.display()))?;
        }
    }

    Ok(())
}
