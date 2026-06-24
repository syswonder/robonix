// SPDX-License-Identifier: MulanPSL-2.0
// Robonix CLI Main Entry
//
// Main entry point for robonix-cli command-line tool

use anyhow::Result;
use clap::Parser;
use robonix_cli::Config;

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

/// Ensure the loopback control plane is never tunneled through an HTTP proxy.
///
/// Robonix's atlas (gRPC), the executor↔package MCP calls (HTTP), and the CLI
/// itself all talk over `127.0.0.1` / `localhost`. If the environment has
/// `http_proxy` set without those hosts in `no_proxy`, every local call gets
/// routed through the proxy and fails (gRPC "Socket closed", MCP "502 Bad
/// Gateway"). Merge the loopback hosts into `no_proxy`/`NO_PROXY` (preserving
/// any existing entries, case-insensitively) so this process and every child
/// it spawns — atlas, executor, pilot, the Python packages — bypass the proxy
/// for the local control plane.
fn ensure_loopback_bypasses_proxy() {
    const LOOPBACK: [&str; 3] = ["127.0.0.1", "localhost", "::1"];
    for var in ["no_proxy", "NO_PROXY"] {
        let mut entries: Vec<String> = std::env::var(var)
            .unwrap_or_default()
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();
        let have: std::collections::HashSet<String> =
            entries.iter().map(|s| s.to_ascii_lowercase()).collect();
        for host in LOOPBACK {
            if !have.contains(host) {
                entries.push(host.to_string());
            }
        }
        // SAFETY: called once at the very top of main, before any worker
        // thread or child process reads proxy configuration from the env.
        unsafe { std::env::set_var(var, entries.join(",")) };
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    ensure_loopback_bypasses_proxy();

    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .filter_module("rustdds", log::LevelFilter::Off)
        .filter_module("ros2_client", log::LevelFilter::Warn)
        .init();

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
        log::warn!("Package database sync failed: {}", e);
    }

    cmd::execute(cli.command, config).await?;

    Ok(())
}
