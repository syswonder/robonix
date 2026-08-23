// SPDX-License-Identifier: MulanPSL-2.0
// Shared boot-state persistence + tear-down logic for `rbnx boot` and
// `rbnx shutdown`. Boot writes `state.json`; shutdown reads it.
//
// Scope: package runtime shutdown. Each component record carries provider
// lifecycle metadata, optional package `stop`, and wrapper PGID. Teardown uses
// the same ordered contract for Ctrl-C, `rbnx shutdown`, boot failure cleanup,
// and restart recovery: Driver(CMD_SHUTDOWN) if reachable, then manifest stop,
// then TERM/KILL of the wrapper process group.
//
// Boot also doesn't currently kill its children on the `?`-error path,
// leaking atlas/pilot/executor as orphans. Persisted state + a separate
// `rbnx shutdown` command lets the user (or boot's own error path)
// always reach the same teardown helper.

use anyhow::{Context, Result};
use robonix_cli::launch::{PackageRuntimeRecord, shutdown_package_runtime_checked};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};
use std::time::Duration;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BootState {
    pub manifest_path: String,
    pub boot_pid: u32,
    #[serde(default)]
    pub boot_start_time_ticks: Option<u64>,
    #[serde(default)]
    pub boot_id: String,
    pub started_at_ms: u64,
    pub atlas_endpoint: String,
    pub components: Vec<ComponentRecord>,
}

/// Backward-compatible name for persisted boot components. New fields are
/// optional in JSON so old state files still deserialize.
pub type ComponentRecord = PackageRuntimeRecord;

pub fn state_path(manifest_dir: &Path) -> PathBuf {
    manifest_dir.join("rbnx-boot").join("state.json")
}

pub fn write_state(path: &Path, state: &BootState) -> Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).with_context(|| format!("create {}", parent.display()))?;
    }
    let text = serde_json::to_string_pretty(state)?;
    let temp = path.with_extension(format!("json.{}.tmp", std::process::id()));
    std::fs::write(&temp, text).with_context(|| format!("write {}", temp.display()))?;
    std::fs::rename(&temp, path)
        .with_context(|| format!("replace {} with {}", path.display(), temp.display()))?;
    Ok(())
}

pub fn read_state(path: &Path) -> Result<BootState> {
    let raw = std::fs::read_to_string(path).with_context(|| format!("read {}", path.display()))?;
    let state: BootState =
        serde_json::from_str(&raw).with_context(|| format!("parse {}", path.display()))?;
    Ok(state)
}

/// Stop each component with the canonical runtime order. Idempotent: missing
/// providers, stop hooks, or PGIDs are treated as best-effort shutdown noise.
pub async fn teardown(
    atlas_endpoint: Option<&str>,
    components: &[ComponentRecord],
    boot_id: Option<&str>,
) -> bool {
    let mut complete = true;
    // Reverse order so services/skills/primitives die before pilot/atlas.
    for c in components.iter().rev() {
        robonix_cli::output::sub_step(&format!(
            "[shutdown] {} stopping (pid={}, pgid={})",
            c.name, c.pid, c.pgid
        ));
        complete &=
            shutdown_package_runtime_checked(atlas_endpoint, c, Duration::from_secs(30), boot_id)
                .await;
    }
    complete
}
