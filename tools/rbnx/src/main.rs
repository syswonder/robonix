// SPDX-License-Identifier: MulanPSL-2.0
// Robonix CLI Main Entry
//
// Main entry point for robonix-cli command-line tool

use anyhow::Result;
use clap::Parser;
use robonix_cli::Config;
use robonix_scribe::warn;
use std::path::{Path, PathBuf};

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

fn boot_log_dir(command: &cmd::Commands) -> Option<PathBuf> {
    let cmd::Commands::Boot { file, log_dir, .. } = command else {
        return None;
    };
    if let Some(log_dir) = log_dir {
        return Some(log_dir.clone());
    }
    let manifest = if file.is_absolute() {
        file.clone()
    } else {
        std::env::current_dir().ok()?.join(file)
    };
    Some(
        manifest
            .parent()
            .unwrap_or_else(|| Path::new("."))
            .join("rbnx-boot")
            .join("logs"),
    )
}

fn prepare_boot_log_dir(command: &cmd::Commands) -> Result<()> {
    let Some(log_dir) = boot_log_dir(command) else {
        return Ok(());
    };
    std::fs::create_dir_all(&log_dir)?;
    for entry in std::fs::read_dir(&log_dir)?.flatten() {
        let path = entry.path();
        if path.extension().and_then(|s| s.to_str()) == Some("log") {
            let _ = std::fs::remove_file(path);
        }
    }
    // Safety: startup is still single-threaded and Scribe has not been used.
    unsafe { std::env::set_var("SCRIBE_LOG_DIR", &log_dir) };
    Ok(())
}

#[tokio::main]
async fn main() -> Result<()> {
    ensure_loopback_bypasses_proxy();

    let cli = Cli::parse();
    // The detached boot watchdog must not depend on user config, package DB
    // sync, a terminal, or Scribe initialization. It only reads the persisted
    // boot state and performs teardown if the exact parent process vanishes.
    if let cmd::Commands::WatchBoot {
        state,
        boot_pid,
        boot_start_time_ticks,
        boot_id,
    } = &cli.command
    {
        return cmd::boot_watchdog::execute(
            state.clone(),
            *boot_pid,
            *boot_start_time_ticks,
            boot_id.clone(),
        )
        .await;
    }
    let verbose_boot = matches!(&cli.command, cmd::Commands::Boot { verbose: true, .. });
    prepare_boot_log_dir(&cli.command)?;

    // Select the Scribe console threshold before its LazyLock is initialized.
    // Normal boot keeps package logs in files; verbose boot streams the same
    // Scribe records to the terminal without introducing a second log path.
    // Safety: no other threads exist yet (tokio runtime not started).
    unsafe {
        std::env::set_var(
            "SCRIBE_CONSOLE_LEVEL",
            if verbose_boot { "info" } else { "error" },
        );
    }

    robonix_scribe::init("rbnx");

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_accepts_explicit_deploy_manifest() {
        let cli = Cli::try_parse_from(["rbnx", "build", "-f", "robot/robonix_manifest.yaml"])
            .expect("build -f should parse");
        match cli.command {
            cmd::Commands::Build {
                file, path, global, ..
            } => {
                assert_eq!(file, Some("robot/robonix_manifest.yaml".into()));
                assert!(path.is_none());
                assert!(global.is_none());
            }
            _ => panic!("expected build command"),
        }
    }

    #[test]
    fn build_manifest_conflicts_with_single_package_selectors() {
        assert!(Cli::try_parse_from(["rbnx", "build", "-f", "deploy.yaml", "-p", "."]).is_err());
        assert!(Cli::try_parse_from(["rbnx", "build", "-f", "deploy.yaml", "-g", "pkg"]).is_err());
    }

    #[test]
    fn build_accepts_no_update_check() {
        let cli = Cli::try_parse_from([
            "rbnx",
            "build",
            "-f",
            "robot/robonix_manifest.yaml",
            "--no-update-check",
        ])
        .expect("build --no-update-check should parse");
        assert!(matches!(
            cli.command,
            cmd::Commands::Build {
                no_update_check: true,
                ..
            }
        ));
    }

    #[test]
    fn boot_accepts_verbose_mode() {
        let cli = Cli::try_parse_from(["rbnx", "boot", "-v"]).expect("boot -v should parse");
        assert!(matches!(
            cli.command,
            cmd::Commands::Boot { verbose: true, .. }
        ));
    }

    #[test]
    fn boot_uses_manifest_local_scribe_directory() {
        let cli = Cli::try_parse_from(["rbnx", "boot", "-f", "robot/robonix_manifest.yaml"])
            .expect("boot manifest should parse");
        assert_eq!(
            boot_log_dir(&cli.command),
            std::env::current_dir()
                .ok()
                .map(|cwd| cwd.join("robot/rbnx-boot/logs"))
        );
    }
}
