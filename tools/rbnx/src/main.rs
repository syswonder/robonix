// SPDX-License-Identifier: MulanPSL-2.0
// Robonix CLI Main Entry
//
// Main entry point for robonix-cli command-line tool

use anyhow::Result;
use clap::Parser;
use robonix_cli::Config;
use robonix_scribe::{info, warn};

mod cmd;
mod pb;

#[derive(Parser)]
#[command(name = "rbnx")]
#[command(version)]
#[command(about = "Robonix helper CLI for package validation, build, and local orchestration", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: cmd::Commands,
}

#[tokio::main]
async fn main() -> Result<()> {
    // Set Scribe console threshold to Error BEFORE the first scribe call
    // triggers LazyLock init — otherwise CONSOLE_MIN defaults to Warn and
    // Warn messages leak to the terminal during boot.
    // Safety: no other threads exist yet (tokio runtime not started).
    unsafe {
        std::env::set_var("SCRIBE_CONSOLE_LEVEL", "error");
    }

    robonix_scribe::init("rbnx");
    info!("rbnx starting");

    let cli = Cli::parse();

    let config = Config::load()?;
    config.ensure_storage_dir()?;

    // Enforce robonix_source_path migration: subcommands that build / start packages
    // need the new ~/.robonix/config.yaml field. Exempt commands that bootstrap the
    // config itself (Setup), read it (Config, Path), or operate purely on atlas (Nodes,
    // Describe, Tools, Channels, Inspect, Chat, Graph, List, Info).
    let needs_source = matches!(
        &cli.command,
        cmd::Commands::Build { .. }
            | cmd::Commands::Start { .. }
            | cmd::Commands::Validate { .. }
            | cmd::Commands::Install { .. }
            | cmd::Commands::Codegen { .. }
    );
    if needs_source {
        let _ = config.require_source_path();
    }

    if let Err(e) = robonix_cli::PackageDatabase::sync(&config.package_storage_path) {
        warn!("Package database sync failed: {}", e);
    }

    cmd::execute(cli.command, config).await?;

    Ok(())
}
