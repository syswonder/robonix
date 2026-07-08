// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx shutdown` — tear down a stack previously brought up by `rbnx boot`.
//
// Reads `<manifest-dir>/rbnx-boot/state.json` (written incrementally by
// boot at every successful spawn), kills each component's process group,
// and sweeps any docker container boot recorded as a driver host. After
// teardown the state file is deleted so a stale state can't confuse the
// next `rbnx shutdown`.

use anyhow::{Context, Result};
use robonix_cli::output;
use std::path::PathBuf;

use super::teardown;

pub async fn execute(file: PathBuf) -> Result<()> {
    let manifest_path = file.canonicalize().with_context(|| {
        format!(
            "manifest not found: {} (rbnx shutdown defaults to ./robonix_manifest.yaml; \
             pass -f to point elsewhere)",
            file.display()
        )
    })?;
    let manifest_dir = manifest_path
        .parent()
        .context("manifest has no parent directory")?
        .to_path_buf();

    let state_path = teardown::state_path(&manifest_dir);
    if !state_path.exists() {
        anyhow::bail!(
            "no boot state at {} — has `rbnx boot` been run from this manifest? \
             (state is written when boot starts spawning components and removed on shutdown.)",
            state_path.display()
        );
    }

    let state = teardown::read_state(&state_path)?;
    output::action(
        "Shutting down",
        &format!(
            "{} ({} component(s))",
            state.manifest_path,
            state.components.len()
        ),
    );

    teardown::teardown(Some(&state.atlas_endpoint), &state.components).await;

    // Best-effort: also signal the boot process itself (which is probably
    // already dead because we just killed all its children, but if the
    // user ran `rbnx boot` in the foreground and `rbnx shutdown` from
    // another shell, this lets boot exit its signal-wait loop cleanly).
    if state.boot_pid != 0 && state.boot_pid != std::process::id() {
        let pid = nix::unistd::Pid::from_raw(state.boot_pid as i32);
        let _ = nix::sys::signal::kill(pid, nix::sys::signal::Signal::SIGTERM);
    }

    let _ = std::fs::remove_file(&state_path);
    output::success(&format!("torn down — removed {}", state_path.display()));
    Ok(())
}
