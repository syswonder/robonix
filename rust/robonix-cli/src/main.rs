// SPDX-License-Identifier: MulanPSL-2.0
// Robonix CLI Main Entry
//
// Main entry point for robonix-cli command-line tool

use anyhow::{Context, Result};
use clap::Parser;
use robonix_cli::*;

mod cmd;

#[derive(Parser)]
#[command(name = "rbnx")]
#[command(about = "Robonix helper CLI for package validation, build, and local orchestration", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: cmd::Commands,
}

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .filter_module("rustdds", log::LevelFilter::Off)
        .filter_module("ros2_client", log::LevelFilter::Warn)
        .init();

    let cli = Cli::parse();

    let config = Config::load()?;
    config.ensure_storage_dir()?;

    // Sync database with filesystem on startup
    PackageDatabase::sync(&config.package_storage_path).context("Failed to sync database")?;

    cmd::execute(cli.command, config).await?;

    Ok(())
}
