use crate::daemon_client::DaemonClient;
use crate::output;
use anyhow::{Context, Result};
use std::path::PathBuf;
use std::process::Stdio;

pub async fn start() -> Result<()> {
    let client = DaemonClient::new()?;

    if client.is_daemon_running().await {
        output::info("Daemon is already running");
        return Ok(());
    }

    output::action("Starting", "daemon");

    // Try to find daemon executable
    let daemon_path = find_daemon_executable()?;

    // Start daemon in background
    #[cfg(unix)]
    {
        let mut cmd = std::process::Command::new(&daemon_path);
        cmd.stdout(Stdio::null());
        cmd.stderr(Stdio::null());
        cmd.spawn()
            .with_context(|| format!("Failed to start daemon: {}", daemon_path.display()))?;
    }

    // Wait a bit for daemon to start
    tokio::time::sleep(tokio::time::Duration::from_millis(1000)).await;

    // Check if daemon is now running
    if client.is_daemon_running().await {
        output::check("Daemon started successfully");
    } else {
        anyhow::bail!("Failed to start daemon. Check logs for details.");
    }

    Ok(())
}

pub async fn stop() -> Result<()> {
    let client = DaemonClient::new()?;

    if !client.is_daemon_running().await {
        output::info("Daemon is not running");
        return Ok(());
    }

    output::action("Stopping", "daemon");

    // Try to ping daemon to get PID or use socket to signal shutdown
    // For now, we'll use a simple approach: find the process and kill it
    #[cfg(unix)]
    {
        use nix::sys::signal::{kill, Signal};
        use nix::unistd::Pid;

        // Try to find daemon process by checking socket file's owner or by name
        // For simplicity, we'll use pkill or find the process
        let output = std::process::Command::new("pgrep")
            .arg("-f")
            .arg("rbnx-daemon")
            .output()?;

        if output.status.success() {
            let pid_str = String::from_utf8_lossy(&output.stdout).trim().to_string();
            if let Ok(pid) = pid_str.parse::<i32>() {
                let pid = Pid::from_raw(pid);
                if let Err(e) = kill(pid, Signal::SIGTERM) {
                    anyhow::bail!("Failed to send SIGTERM to daemon (PID {}): {}", pid, e);
                }

                // Wait for process to exit
                for _ in 0..10 {
                    tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;
                    if !client.is_daemon_running().await {
                        output::check("Daemon stopped successfully");
                        return Ok(());
                    }
                }

                // If still running, try SIGKILL
                let _ = kill(pid, Signal::SIGKILL);
                tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;

                if !client.is_daemon_running().await {
                    output::check("Daemon stopped successfully (force kill)");
                } else {
                    anyhow::bail!("Failed to stop daemon");
                }
            } else {
                anyhow::bail!("Failed to parse daemon PID");
            }
        } else {
            anyhow::bail!("Daemon process not found");
        }
    }
    #[cfg(not(unix))]
    {
        anyhow::bail!("Daemon stop not implemented for this platform");
    }

    Ok(())
}

pub async fn status() -> Result<()> {
    let client = DaemonClient::new()?;

    if client.is_daemon_running().await {
        output::info("Daemon status: Running");

        // Try to ping daemon
        match client
            .send_command(crate::daemon_client::DaemonCommand::Ping)
            .await
        {
            Ok(crate::daemon_client::DaemonResponse::Ok(_)) => {
                output::info("Daemon is responsive");
            }
            Ok(crate::daemon_client::DaemonResponse::Error(e)) => {
                output::warning(&format!("Daemon responded with error: {}", e));
            }
            Err(e) => {
                output::warning(&format!("Failed to ping daemon: {}", e));
            }
            _ => {}
        }
    } else {
        output::info("Daemon status: Stopped");
    }

    Ok(())
}

pub async fn restart() -> Result<()> {
    output::action("Restarting", "daemon");

    stop().await?;
    tokio::time::sleep(tokio::time::Duration::from_millis(1000)).await;
    start().await?;

    Ok(())
}

fn find_daemon_executable() -> Result<PathBuf> {
    // Try 1: Same directory as current executable
    if let Ok(current_exe) = std::env::current_exe() {
        if let Some(parent) = current_exe.parent() {
            let daemon_path = parent.join("rbnx-daemon");
            if daemon_path.exists() {
                return Ok(daemon_path);
            }
        }
    }

    // Try 2: In PATH
    if let Ok(path) = std::env::var("PATH") {
        for dir in path.split(':') {
            let daemon_path = PathBuf::from(dir).join("rbnx-daemon");
            if daemon_path.exists() {
                return Ok(daemon_path);
            }
        }
    }

    // Try 3: Common installation paths
    let common_paths = vec![
        PathBuf::from("/usr/local/bin/rbnx-daemon"),
        PathBuf::from("/usr/bin/rbnx-daemon"),
        PathBuf::from("/opt/robonix/bin/rbnx-daemon"),
    ];

    for path in common_paths {
        if path.exists() {
            return Ok(path);
        }
    }

    anyhow::bail!(
        "Daemon executable (rbnx-daemon) not found. Please build the project with 'cargo build' or install it."
    )
}
