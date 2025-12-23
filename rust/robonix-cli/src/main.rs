use anyhow::{Context, Result};
use clap::Parser;
use robonix_cli::*;

mod cmd;

#[derive(Parser)]
#[command(name = "rbnx")]
#[command(about = "Robonix Package Manager CLI", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: cmd::Commands,
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| {
                tracing_subscriber::EnvFilter::new("info")
                    .add_directive("rustdds=off".parse().unwrap())
                    .add_directive("ros2_client=warn".parse().unwrap())
            }),
        )
        .init();

    let cli = Cli::parse();

    let config = Config::load()?;
    config.ensure_storage_dir()?;

    // Sync database with filesystem on startup
    PackageDatabase::sync(&config.package_storage_path).context("Failed to sync database")?;

    cmd::execute(cli.command, config).await?;

    Ok(())
}
