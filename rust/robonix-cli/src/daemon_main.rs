// SPDX-License-Identifier: MulanPSL-2.0
// Daemon Main Entry
//
// Main entry point for robonix daemon process

use anyhow::Result;
use robonix_cli::*;

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .filter_module("rustdds", log::LevelFilter::Off)
        .filter_module("ros2_client", log::LevelFilter::Warn)
        .init();

    let config = Config::load()?;
    config.ensure_storage_dir()?;

    let daemon = daemon::Daemon::new(config).await?;
    daemon.run().await?;

    Ok(())
}
