// SPDX-License-Identifier: MulanPSL-2.0
// Shared boot-state persistence + tear-down logic for `rbnx boot` and
// `rbnx shutdown`. Boot writes `state.json`; shutdown reads it.
//
// Scope: host-side process groups only. Each child boot spawns is given
// its own PGID via `process_group(0)`, so a single `kill -TERM -<PGID>`
// takes down the wrapper plus everything it fork+exec'd. CLI does NOT
// reach into container runtimes, network proxies, etc — that's the
// individual package's job. The convention (already followed by the
// webots nav2 start.sh) is for any package whose start body spawns
// out-of-process children — `docker exec`, `kubectl exec`, ssh,
// systemd-run, … — to install a `trap` that cleans them up on EXIT/
// INT/TERM. When that trap fires, our SIGTERM-the-PGID arrives at the
// wrapper script, the trap runs, the side-channel children die, and
// then the wrapper exits.
//
// TODO: add a top-level `stop:` field to package_manifest.yaml — a
// shell snippet rbnx runs *before* SIGTERMing the PGID. Right now we
// rely entirely on each package's `trap` discipline; an explicit stop
// hook makes the cleanup contract part of the manifest schema and is
// easier to write correctly than nested traps. (Mirror what `build:`
// and `start:` already are: just a string body executed in the package
// root.)
//
// Boot also doesn't currently kill its children on the `?`-error path,
// leaking atlas/pilot/executor as orphans. Persisted state + a separate
// `rbnx shutdown` command lets the user (or boot's own error path)
// always reach the same teardown helper.

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};
use std::time::Duration;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BootState {
    pub manifest_path: String,
    pub boot_pid: u32,
    pub started_at_ms: u64,
    pub atlas_endpoint: String,
    pub components: Vec<ComponentRecord>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComponentRecord {
    pub name: String,
    /// "system_builtin" | "system_package" | "primitive" | "service"
    pub kind: String,
    pub pid: u32,
    /// Process group id. Boot starts each child with `process_group(0)`
    /// so PGID == PID at spawn time; tear-down sends SIGTERM/SIGKILL to
    /// `-PGID` to take down the wrapper plus every direct fork+exec
    /// descendant. Side-channel children (`docker exec`, ssh, …) must
    /// be cleaned up by the package's own start-body trap — see this
    /// module's header comment.
    pub pgid: u32,
}

pub fn state_path(manifest_dir: &Path) -> PathBuf {
    manifest_dir.join("rbnx-boot").join("state.json")
}

pub fn write_state(path: &Path, state: &BootState) -> Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).with_context(|| format!("create {}", parent.display()))?;
    }
    let text = serde_json::to_string_pretty(state)?;
    std::fs::write(path, text).with_context(|| format!("write {}", path.display()))?;
    Ok(())
}

pub fn read_state(path: &Path) -> Result<BootState> {
    let raw = std::fs::read_to_string(path).with_context(|| format!("read {}", path.display()))?;
    let state: BootState =
        serde_json::from_str(&raw).with_context(|| format!("parse {}", path.display()))?;
    Ok(state)
}

/// SIGTERM each PGID, wait for graceful exit, SIGKILL stragglers.
/// Idempotent: missing PGIDs are ok.
pub async fn teardown(components: &[ComponentRecord]) {
    use nix::sys::signal::{Signal, killpg};
    use nix::unistd::Pid;

    // Reverse order so primitives/services die before pilot/atlas — keeps
    // log noise about "atlas unreachable" from drowning out the real
    // shutdown trace.
    let ordered: Vec<&ComponentRecord> = components.iter().rev().collect();

    for c in &ordered {
        let pgid = Pid::from_raw(c.pgid as i32);
        match killpg(pgid, Signal::SIGTERM) {
            Ok(()) => robonix_cli::output::sub_step(&format!(
                "[shutdown] {} TERM (pgid={})",
                c.name, c.pgid
            )),
            Err(nix::errno::Errno::ESRCH) => { /* already dead */ }
            Err(e) => {
                robonix_cli::output::sub_step(&format!("[shutdown] {} TERM failed: {e}", c.name))
            }
        }
    }

    tokio::time::sleep(Duration::from_secs(3)).await;

    for c in &ordered {
        let pgid = Pid::from_raw(c.pgid as i32);
        // SIGKILL is best-effort; ESRCH means already gone.
        let _ = killpg(pgid, Signal::SIGKILL);
    }
}
