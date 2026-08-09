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

    // `rbnx boot` applies the deploy's top-level env before it spawns package
    // wrappers. Do the same for a separately invoked `rbnx shutdown`, so a
    // package's manifest `stop:` hook sees the same native/docker/profile
    // selection as its corresponding `start:` hook.
    let raw_manifest = std::fs::read_to_string(&manifest_path)
        .with_context(|| format!("read {}", manifest_path.display()))?;
    let manifest_value: serde_yaml::Value = serde_yaml::from_str(&raw_manifest)
        .with_context(|| format!("parse {}", manifest_path.display()))?;
    super::deploy::prepare_manifest(manifest_value, None)
        .context("apply top-level deploy env for shutdown hooks")?;

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

    let boot_id = (!state.boot_id.is_empty()).then_some(state.boot_id.as_str());
    let complete =
        teardown::teardown(Some(&state.atlas_endpoint), &state.components, boot_id).await;
    if !complete {
        anyhow::bail!(
            "shutdown refused one or more stale/mismatched process groups; preserving {}",
            state_path.display()
        );
    }

    // Best-effort: also signal the boot process itself (which is probably
    // already dead because we just killed all its children, but if the
    // user ran `rbnx boot` in the foreground and `rbnx shutdown` from
    // another shell, this lets boot exit its signal-wait loop cleanly).
    let boot_identity_matches = state.boot_start_time_ticks.is_none_or(|expected| {
        robonix_cli::launch::proc_start_time_ticks(state.boot_pid) == Some(expected)
    });
    if state.boot_pid != 0 && state.boot_pid != std::process::id() && boot_identity_matches {
        let pid = nix::unistd::Pid::from_raw(state.boot_pid as i32);
        let _ = nix::sys::signal::kill(pid, nix::sys::signal::Signal::SIGTERM);
    }

    let _ = std::fs::remove_file(&state_path);
    output::success(&format!("torn down — removed {}", state_path.display()));
    Ok(())
}
