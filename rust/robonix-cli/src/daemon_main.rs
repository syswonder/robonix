use anyhow::Result;
use robonix_cli::*;

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

    let config = Config::load()?;
    config.ensure_storage_dir()?;

    let daemon = daemon::Daemon::new(config).await?;
    daemon.run().await?;

    Ok(())
}
