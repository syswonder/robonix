// SPDX-License-Identifier: MulanPSL-2.0
// Detached `rbnx boot` watchdog.
//
// The foreground boot process owns the terminal and can disappear without a
// catchable signal (terminal close, SIGKILL, IDE cancellation). This helper is
// re-execed in a new session, watches the exact parent PID identity, and runs
// the same persisted teardown path if that parent disappears unexpectedly.

use anyhow::{Context, Result};
use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::time::Duration;

use super::teardown;

pub fn spawn(state_path: &Path, boot_pid: u32, boot_start_time_ticks: Option<u64>) -> Result<()> {
    use std::os::unix::process::CommandExt;

    let exe = std::env::current_exe().context("resolve rbnx binary for boot watchdog")?;
    let mut command = std::process::Command::new(exe);
    command
        .arg("__watch-boot")
        .arg("--state")
        .arg(state_path)
        .arg("--boot-pid")
        .arg(boot_pid.to_string())
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null());
    if let Some(ticks) = boot_start_time_ticks {
        command
            .arg("--boot-start-time-ticks")
            .arg(ticks.to_string());
    }
    // Safety: pre_exec runs after fork and before exec. setsid(2) is
    // async-signal-safe and detaches the watcher from the boot PTY/session.
    unsafe {
        command.pre_exec(|| {
            nix::unistd::setsid()
                .map(|_| ())
                .map_err(std::io::Error::other)
        });
    }
    command.spawn().context("spawn detached boot watchdog")?;
    Ok(())
}

pub async fn execute(
    state_path: PathBuf,
    boot_pid: u32,
    boot_start_time_ticks: Option<u64>,
) -> Result<()> {
    loop {
        if !state_path.exists() {
            return Ok(());
        }
        let alive = match boot_start_time_ticks {
            Some(expected) => {
                robonix_cli::launch::proc_start_time_ticks(boot_pid) == Some(expected)
            }
            None => robonix_cli::launch::proc_start_time_ticks(boot_pid).is_some(),
        };
        if !alive {
            break;
        }
        tokio::time::sleep(Duration::from_millis(250)).await;
    }

    let state = teardown::read_state(&state_path)?;
    if !matches_watched_boot(&state, boot_pid, boot_start_time_ticks) {
        return Ok(());
    }
    let boot_id = (!state.boot_id.is_empty()).then_some(state.boot_id.as_str());
    let complete =
        teardown::teardown(Some(&state.atlas_endpoint), &state.components, boot_id).await;
    if !complete {
        return Ok(());
    }

    // A new boot may have replaced state.json while cleanup was running. Only
    // remove the file if it still belongs to the parent we watched.
    if teardown::read_state(&state_path)
        .ok()
        .is_some_and(|current| matches_watched_boot(&current, boot_pid, boot_start_time_ticks))
    {
        let _ = std::fs::remove_file(&state_path);
    }
    Ok(())
}

fn matches_watched_boot(
    state: &teardown::BootState,
    boot_pid: u32,
    boot_start_time_ticks: Option<u64>,
) -> bool {
    state.boot_pid == boot_pid
        && match (boot_start_time_ticks, state.boot_start_time_ticks) {
            (Some(expected), Some(actual)) => expected == actual,
            (Some(_), None) => false,
            (None, _) => true,
        }
}
