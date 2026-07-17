// SPDX-License-Identifier: MulanPSL-2.0
//
// Cross-component package bring-up primitives.
//
// `rbnx boot` and Soma both need to: spawn a package process, wait for it
// to register a CapabilityProvider on Atlas, optionally find that
// provider's `*/driver` capability, and drive `Driver(CMD_INIT)` /
// `Driver(CMD_ACTIVATE)` over gRPC. Historically this lived entirely
// inside `cmd::deploy.rs` and was tangled with terminal UI (`output::*`).
//
// This module exposes the **pure** parts of that flow as crate-public
// helpers so Soma (which orchestrates primitive/skill bring-up at
// runtime) can do exactly the same lifecycle dance without copy-pasting
// the timeout / channel-hygiene / FSM-step logic. The UI side stays
// inside `cmd::deploy` — Soma logs via `scribe` instead.
//
// What's here vs what's not:
//   * `wait_for_registration_core` — atlas-side poll loop, no UI. Accepts
//     a `before` snapshot so callers can reason about "which provider
//     appeared because of this spawn." Returns the provider plus every
//     lifecycle Driver observed during declaration settling.
//   * `call_driver_cmd` — one Driver(cmd) RPC against a freshly-connected
//     channel, no UI. Returns the response's `state` string.
//   * `CMD_*` constants and the `DRIVER_*_TIMEOUT` values — single source
//     of truth so rbnx + soma agree on the deadlines.
//   * `contract_id_to_service_name` — mirrors codegen's PascalCase rule,
//     needed by both the rbnx CLI and Soma to assemble the
//     `/robonix.contracts.<Name>/Driver` gRPC path.
//
// What's NOT here:
//   * `spawn_package` — depends on `cmd::install` / `cmd::run` / the
//     manifest-driven start.sh wrapper, which is a heavy subtree we
//     don't want to drag into Soma. Soma spawns packages through its
//     own `ProcessManager` (existing) and only borrows the *post-spawn*
//     bookkeeping from here.
//   * Terminal output (spinners, ok/fail lines) — these are
//     `cmd::deploy::output::*` and are deliberately left out so this
//     module compiles in any caller that doesn't want a TTY.
//
// Stability: this is an *internal* crate boundary between rbnx-cli and
// Soma; both are workspace members and bumped together. The API is
// free to evolve as long as we keep the two callers in sync.

use crate::pb::lifecycle::{DriverRequest, DriverResponse};
use anyhow::{Context, Result};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{info, warn};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::PathBuf;
use std::process::Stdio;
use std::time::{Duration, Instant};
use tokio::process::Command;
use tonic::Request;
use tonic::transport::Endpoint;

// ── Driver.srv command discriminators (mirrors capabilities/lib/lifecycle/srv/Driver.srv) ──

pub const CMD_INIT: u32 = 0;
pub const CMD_ACTIVATE: u32 = 1;
#[allow(dead_code)]
pub const CMD_DEACTIVATE: u32 = 2;
#[allow(dead_code)]
pub const CMD_SHUTDOWN: u32 = 3;
/// Positive proof published by the Python SDK only when a fallback-marked
/// package's generated module contains neither shared nor namespace Driver
/// stubs. Launchers require this marker before accepting zero Drivers.
pub const OLD_ARTIFACT_NO_DRIVER_STATE_DETAIL: &str = "robonix.lifecycle.old_artifact_no_driver.v1";

/// Max time we'll wait for a freshly spawned package to register its
/// driver capability with atlas before giving up.
pub const DRIVER_REGISTER_TIMEOUT: Duration = Duration::from_secs(60);

/// Max time we'll wait for one `Driver(CMD_*)` RPC to return. 90s gives
/// generous slack for slow-warming sensors (Webots's camera can take
/// 30–50s on cold boot). Primitive driver-side waits should stay
/// strictly below this so they own their own timeout semantics rather
/// than racing the CLI deadline.
pub const DRIVER_INIT_TIMEOUT: Duration = Duration::from_secs(90);

/// Consumer id used by both rbnx-cli (deploy) and Soma when opening
/// channels to call `Driver(...)` during boot. Atlas just records the
/// edge for bookkeeping — the id is only ever read in `inspect_atlas`
/// output, so we keep one stable string instead of inventing per-caller
/// ones (would only add noise).
pub const BOOT_CONSUMER_ID: &str = "rbnx-cli/deploy";

/// Runtime-owned process and lifecycle metadata for one package wrapper.
///
/// Both `rbnx boot` and Soma spawn long-lived `rbnx start -p` wrappers.
/// Shutdown must be ordered the same way in both callers: provider lifecycle,
/// package stop hook, process-group fallback. Keeping the record here prevents
/// Soma and rbnx from growing divergent cleanup semantics.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PackageRuntimeRecord {
    pub name: String,
    pub kind: String,
    pub pid: u32,
    pub pgid: u32,
    #[serde(default)]
    pub provider_id: Option<String>,
    #[serde(default)]
    pub driver_contract: Option<String>,
    #[serde(default)]
    pub config_json: Option<String>,
    #[serde(default)]
    pub package_dir: Option<String>,
    #[serde(default)]
    pub stop: Option<String>,
}

/// Stop one package runtime using the canonical shutdown order:
/// Driver(CMD_SHUTDOWN) -> manifest stop -> TERM PGID -> KILL PGID.
pub async fn shutdown_package_runtime(
    atlas_endpoint: Option<&str>,
    record: &PackageRuntimeRecord,
    term_wait: Duration,
) {
    let _ = shutdown_package_runtime_checked(atlas_endpoint, record, term_wait, None).await;
}

/// Checked variant used by persisted boot-state teardown. A boot id is
/// inherited by every wrapper and provider through `RBNX_BOOT_ID`; requiring a
/// matching process in the recorded PGID prevents a stale state file from
/// killing an unrelated process group after PID/PGID reuse.
pub async fn shutdown_package_runtime_checked(
    atlas_endpoint: Option<&str>,
    record: &PackageRuntimeRecord,
    term_wait: Duration,
    boot_id: Option<&str>,
) -> bool {
    if let Some(boot_id) = boot_id {
        if !process_group_has_members(record.pgid).await {
            return true;
        }
        if !process_group_has_boot_id(record.pgid, boot_id).await {
            warn!(
                "refusing to stop {} pgid {}: no process carries RBNX_BOOT_ID={}",
                record.name, record.pgid, boot_id
            );
            return false;
        }
    }

    if let (Some(endpoint), Some(provider_id), Some(driver_contract)) = (
        atlas_endpoint,
        record.provider_id.as_deref(),
        record.driver_contract.as_deref(),
    ) {
        let normalized = if endpoint.starts_with("http") {
            endpoint.to_string()
        } else {
            format!("http://{endpoint}")
        };
        match AtlasClient::connect(&normalized).await {
            Ok(mut atlas) => {
                let config_json = record.config_json.clone().unwrap_or_else(|| "{}".into());
                match call_driver_cmd(
                    &mut atlas,
                    provider_id,
                    driver_contract,
                    CMD_SHUTDOWN,
                    config_json,
                    &record.name,
                )
                .await
                {
                    Ok(state) => info!(
                        "shutdown {} via Driver(CMD_SHUTDOWN): {}",
                        record.name, state
                    ),
                    Err(e) => warn!(
                        "shutdown {} Driver(CMD_SHUTDOWN) failed: {e:#}",
                        record.name
                    ),
                }
            }
            Err(e) => warn!(
                "shutdown {} could not connect atlas at {}: {e:#}",
                record.name, normalized
            ),
        }
    }

    if let Some(stop) = record
        .stop
        .as_deref()
        .map(str::trim)
        .filter(|s| !s.is_empty())
    {
        run_package_stop_hook(record, stop, atlas_endpoint).await;
    }

    if !terminate_process_group_guarded(record.pgid, term_wait, boot_id).await {
        warn!(
            "refusing to finish shutdown of {} pgid {}: boot ownership changed before signaling",
            record.name, record.pgid
        );
        return false;
    }
    !process_group_has_members(record.pgid).await
}

/// Prove that at least one process in a recorded group still carries the exact
/// inherited boot id. Enumeration or environment-inspection failure is an
/// ownership failure so checked teardown cannot signal a stale numeric PGID.
async fn process_group_has_boot_id(pgid: u32, boot_id: &str) -> bool {
    let output = match Command::new("pgrep")
        .arg("-g")
        .arg(pgid.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .output()
        .await
    {
        Ok(output) if output.status.success() => output,
        _ => return false,
    };
    let expected = format!("RBNX_BOOT_ID={boot_id}");
    String::from_utf8_lossy(&output.stdout)
        .lines()
        .filter_map(|line| line.trim().parse::<u32>().ok())
        .any(|pid| process_has_environment_entry(pid, expected.as_bytes()))
}

/// Linux exposes the immutable exec environment as NUL-delimited procfs
/// bytes. Failure to read it means ownership is unproven, so stale-PGID
/// protection refuses teardown rather than guessing.
#[cfg(target_os = "linux")]
fn process_has_environment_entry(pid: u32, expected: &[u8]) -> bool {
    std::fs::read(format!("/proc/{pid}/environ"))
        .ok()
        .is_some_and(|env| env.split(|byte| *byte == 0).any(|entry| entry == expected))
}

/// Parse a Darwin `KERN_PROCARGS2` buffer and look for one exact environment
/// entry after the executable path and argc-counted argv strings. `None`
/// denotes malformed/incomplete kernel data rather than a proven mismatch.
#[cfg(any(target_os = "macos", test))]
fn macos_procargs_has_environment_entry(buffer: &[u8], expected: &[u8]) -> Option<bool> {
    let argc_bytes: [u8; std::mem::size_of::<i32>()] =
        buffer.get(..std::mem::size_of::<i32>())?.try_into().ok()?;
    let argc = usize::try_from(i32::from_ne_bytes(argc_bytes)).ok()?;
    let mut cursor = std::mem::size_of::<i32>();

    cursor += buffer.get(cursor..)?.iter().position(|byte| *byte == 0)? + 1;
    while buffer.get(cursor) == Some(&0) {
        cursor += 1;
    }
    for _ in 0..argc {
        cursor += buffer.get(cursor..)?.iter().position(|byte| *byte == 0)? + 1;
    }

    while cursor < buffer.len() {
        while buffer.get(cursor) == Some(&0) {
            cursor += 1;
        }
        if cursor == buffer.len() {
            break;
        }
        let tail = buffer.get(cursor..)?;
        let end = tail.iter().position(|byte| *byte == 0)?;
        if &tail[..end] == expected {
            return Some(true);
        }
        cursor += end + 1;
    }
    Some(false)
}

/// macOS has no procfs. Read `KERN_PROCARGS2` directly so persisted shutdown
/// can prove that a live process group belongs to its recorded boot without
/// parsing human-formatted `ps` output.
#[cfg(target_os = "macos")]
fn process_has_environment_entry(pid: u32, expected: &[u8]) -> bool {
    use std::ptr::null_mut;

    let Ok(pid) = i32::try_from(pid) else {
        return false;
    };
    let mut mib = [nix::libc::CTL_KERN, nix::libc::KERN_PROCARGS2, pid];
    let mut buffer_size = 0;
    // SAFETY: the MIB is initialized and its length is correct. A null output
    // pointer asks sysctl for the required byte count only.
    if unsafe {
        nix::libc::sysctl(
            mib.as_mut_ptr(),
            mib.len() as u32,
            null_mut(),
            &mut buffer_size,
            null_mut(),
            0,
        )
    } != 0
        || buffer_size == 0
    {
        return false;
    }

    let mut buffer = vec![0_u8; buffer_size];
    // SAFETY: `buffer` owns `buffer_size` writable bytes and sysctl receives
    // that exact capacity. The kernel updates `buffer_size` to bytes written.
    if unsafe {
        nix::libc::sysctl(
            mib.as_mut_ptr(),
            mib.len() as u32,
            buffer.as_mut_ptr().cast(),
            &mut buffer_size,
            null_mut(),
            0,
        )
    } != 0
        || buffer_size > buffer.len()
    {
        return false;
    }
    buffer.truncate(buffer_size);
    macos_procargs_has_environment_entry(&buffer, expected) == Some(true)
}

/// Unknown targets cannot prove boot ownership yet, so checked teardown must
/// refuse the process group instead of risking a PID/PGID-reuse kill.
#[cfg(not(any(target_os = "linux", target_os = "macos")))]
fn process_has_environment_entry(_pid: u32, _expected: &[u8]) -> bool {
    false
}

/// Linux `/proc/<pid>/stat` field 22. The value is stable for the lifetime of
/// a process and lets the watchdog distinguish its boot parent from a reused
/// PID. Returns `None` when procfs is unavailable or the process exited.
pub fn proc_start_time_ticks(pid: u32) -> Option<u64> {
    let stat = std::fs::read_to_string(format!("/proc/{pid}/stat")).ok()?;
    let after_comm = stat.get(stat.rfind(')')? + 1..)?.trim_start();
    after_comm.split_whitespace().nth(19)?.parse().ok()
}

async fn run_package_stop_hook(
    record: &PackageRuntimeRecord,
    stop: &str,
    atlas_endpoint: Option<&str>,
) {
    let Some(package_dir) = record.package_dir.as_deref() else {
        warn!("shutdown {} has stop hook but no package_dir", record.name);
        return;
    };
    let mut cmd = Command::new("bash");
    cmd.arg("-c")
        .arg(stop)
        .current_dir(PathBuf::from(package_dir))
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null());
    if let Some(endpoint) = atlas_endpoint {
        cmd.env("ROBONIX_ATLAS", endpoint);
    }
    match tokio::time::timeout(Duration::from_secs(20), cmd.status()).await {
        Ok(Ok(status)) if status.success() => {
            info!("shutdown {} manifest stop hook completed", record.name);
        }
        Ok(Ok(status)) => warn!(
            "shutdown {} manifest stop hook exited with {}",
            record.name, status
        ),
        Ok(Err(e)) => warn!(
            "shutdown {} manifest stop hook failed to start: {e:#}",
            record.name
        ),
        Err(_) => warn!(
            "shutdown {} manifest stop hook timed out after 20s",
            record.name
        ),
    }
}

/// TERM one wrapper process group, wait until it exits, then KILL stragglers.
pub async fn terminate_process_group(pgid: u32, term_wait: Duration) {
    let _ = terminate_process_group_guarded(pgid, term_wait, None).await;
}

/// Signal one process group while optionally preserving its recorded boot
/// identity across slow Driver/stop cleanup and the TERM grace period. Returns
/// false only when a live group no longer proves the expected boot id.
async fn terminate_process_group_guarded(
    pgid: u32,
    term_wait: Duration,
    boot_id: Option<&str>,
) -> bool {
    use nix::sys::signal::{Signal, killpg};
    use nix::unistd::Pid;
    if let Some(boot_id) = boot_id {
        if !process_group_has_members(pgid).await {
            return true;
        }
        if !process_group_has_boot_id(pgid, boot_id).await {
            // The group may have finished between the liveness and ownership
            // probes (common after Driver(SHUTDOWN)). Only refuse if a live
            // group still exists after the failed identity lookup.
            return !process_group_has_members(pgid).await;
        }
    }

    let pgid_raw = pgid;
    let pgid = Pid::from_raw(pgid as i32);
    match killpg(pgid, Signal::SIGTERM) {
        Ok(()) => {}
        Err(nix::errno::Errno::ESRCH) => return true,
        Err(e) => warn!("SIGTERM pgid {} failed: {e}", pgid),
    }

    let deadline = Instant::now() + term_wait;
    while Instant::now() < deadline {
        if !process_group_has_members(pgid_raw).await {
            return true;
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
    }

    if let Some(boot_id) = boot_id {
        if !process_group_has_members(pgid_raw).await {
            return true;
        }
        if !process_group_has_boot_id(pgid_raw, boot_id).await {
            return !process_group_has_members(pgid_raw).await;
        }
    }
    let _ = killpg(pgid, Signal::SIGKILL);
    true
}

/// Return whether a group contains at least one non-zombie process. Tooling or
/// metadata failures count as non-empty so shutdown never skips Driver/TERM or
/// removes persisted state based on inconclusive process inspection.
async fn process_group_has_members(pgid: u32) -> bool {
    let output = match Command::new("pgrep")
        .arg("-g")
        .arg(pgid.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .output()
        .await
    {
        Ok(output) => output,
        Err(_) => return true,
    };
    if !output.status.success() {
        // pgrep reserves exit 1 for "no processes matched". Treat every
        // execution/tooling failure conservatively so shutdown cannot erase
        // state or skip cleanup merely because process inspection failed.
        return output.status.code() != Some(1);
    }

    let stdout = String::from_utf8_lossy(&output.stdout);
    for line in stdout.lines() {
        let Ok(pid) = line.trim().parse::<u32>() else {
            return true;
        };
        if process_is_not_zombie(pid).await {
            return true;
        }
    }
    false
}

/// Linux reports zombie state through procfs. Missing or malformed process
/// metadata is treated as live: the caller must not skip Driver/TERM cleanup
/// based on an inconclusive inspection.
#[cfg(target_os = "linux")]
async fn process_is_not_zombie(pid: u32) -> bool {
    let status_path = format!("/proc/{pid}/status");
    let Ok(status) = std::fs::read_to_string(status_path) else {
        return true;
    };
    linux_proc_status_is_not_zombie(&status)
}

#[cfg(any(target_os = "linux", test))]
/// Parse Linux's `State:` record, treating absent/malformed state as live.
fn linux_proc_status_is_not_zombie(status: &str) -> bool {
    !status.lines().any(|line| {
        line.strip_prefix("State:")
            .is_some_and(|state| state.trim_start().starts_with('Z'))
    })
}

/// Parse the one-letter Darwin `ps` state field. Missing output is
/// inconclusive; `Z` (with optional suffix flags) is the only zombie state.
#[cfg(any(target_os = "macos", test))]
fn macos_ps_state_is_not_zombie(status: &str) -> Option<bool> {
    status
        .lines()
        .find_map(|line| line.trim().chars().next())
        .map(|state| state != 'Z')
}

/// Darwin has no procfs, and libproc omits already-exited zombie processes.
/// Query the machine-readable `ps` state field; inspection failures count as
/// live so cleanup is never skipped on inconclusive evidence.
#[cfg(target_os = "macos")]
async fn process_is_not_zombie(pid: u32) -> bool {
    let output = match Command::new("ps")
        .arg("-o")
        .arg("state=")
        .arg("-p")
        .arg(pid.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .output()
        .await
    {
        Ok(output) => output,
        Err(_) => return true,
    };
    if output.status.success() {
        return macos_ps_state_is_not_zombie(&String::from_utf8_lossy(&output.stdout))
            .unwrap_or(true);
    }
    true
}

/// Other Unix targets still get safe shutdown semantics even when no native
/// zombie-state backend has been added: a PID reported by `pgrep` counts as a
/// live member instead of risking a false empty-group result.
#[cfg(not(any(target_os = "linux", target_os = "macos")))]
async fn process_is_not_zombie(_pid: u32) -> bool {
    true
}

// ── helpers ────────────────────────────────────────────────────────────────

/// Mirrors `robonix_codegen::contract_gen::contract_id_to_service_name`.
/// Uniform PascalCase: `robonix/primitive/chassis/driver` →
/// `RobonixPrimitiveChassisDriver`. No prefix stripping. Full gRPC
/// service path: `/robonix.contracts.<this>/Driver`.
///
/// Lifted out of `cmd::deploy` so Soma can build the same gRPC URLs
/// without re-implementing the rule (which would drift).
pub fn contract_id_to_service_name(id: &str) -> String {
    id.split('/')
        .filter(|x| !x.is_empty())
        .map(|seg| {
            seg.split('_')
                .filter(|p| !p.is_empty())
                .map(|p| {
                    let mut c = p.chars();
                    match c.next() {
                        None => String::new(),
                        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
                    }
                })
                .collect::<String>()
        })
        .collect::<String>()
}

/// Snapshot provider ids and their opaque registration generations. Stable
/// package ids may be taken over on restart, so ids alone cannot correlate a
/// spawn; `registration_id` changes on every successful Atlas Register but not
/// on heartbeat/state/capability updates.
pub type ProviderRegistrationSnapshot = HashMap<String, String>;

pub async fn snapshot_provider_ids(
    atlas: &mut AtlasClient,
) -> Result<ProviderRegistrationSnapshot> {
    Ok(atlas
        .query_capabilities("", "", atlas_pb::Transport::Unspecified)
        .await
        .context("pre-spawn atlas snapshot")?
        .into_iter()
        .map(|provider| (provider.id, provider.registration_id))
        .collect())
}

/// True when `provider` did not exist in the snapshot or has since taken over
/// its stable id with a new Atlas registration generation.
pub fn is_new_provider_registration(
    provider: &atlas_pb::CapabilityProvider,
    before: &ProviderRegistrationSnapshot,
) -> bool {
    before
        .get(&provider.id)
        .is_none_or(|registration_id| registration_id != &provider.registration_id)
}

/// Outcome of `wait_for_registration_core`:
///   * `provider_id` — the new provider that appeared after `before`.
///   * `provider_kind` — Atlas provider kind, used to keep skills INACTIVE.
///   * `provider_state` — settled Atlas lifecycle state for fallback reporting.
///   * `provider_namespace` — primary namespace used to validate the exact
///     namespace Driver allowed for old-artifact fallback.
///   * `provider_state_detail` — carries positive no-Driver fallback proof.
///   * `driver_contracts` — every distinct `*/driver` gRPC capability
///     observed after the declaration settle window. The launch owner must
///     validate this list against the exact selected package manifest before
///     sending lifecycle commands.
#[derive(Debug, Clone)]
pub struct RegistrationOutcome {
    pub provider_id: String,
    pub provider_kind: i32,
    pub provider_state: i32,
    pub provider_namespace: String,
    pub provider_state_detail: String,
    pub registration_id: String,
    pub driver_contracts: Vec<String>,
}

/// Resolve the runtime Driver while optionally tolerating old generated code.
///
/// Exact manifest selections stay strict. Only callers whose manifest omitted
/// Driver pass `allow_old_artifact_fallback=true`: the shared Driver remains
/// preferred, one namespace Driver may substitute when the old stubs lack the
/// shared service, and no Driver is the last-resort compatibility result.
/// Multiple declarations are always ambiguous and rejected.
pub fn resolve_runtime_driver_contract(
    provider_id: &str,
    provider_namespace: &str,
    expected: &str,
    observed: &[String],
    allow_old_artifact_fallback: bool,
    old_artifact_no_driver_proof: bool,
) -> Result<Option<String>> {
    if observed.len() > 1 {
        anyhow::bail!(
            "provider '{provider_id}' registered multiple lifecycle Drivers: {}; expected exactly '{expected}'",
            observed.join(", ")
        );
    }
    if observed.is_empty() {
        if allow_old_artifact_fallback
            && expected == "robonix/lifecycle/driver"
            && old_artifact_no_driver_proof
        {
            return Ok(None);
        }
        if allow_old_artifact_fallback && expected == "robonix/lifecycle/driver" {
            anyhow::bail!(
                "provider '{provider_id}' registered without the shared lifecycle Driver and did not publish old-artifact no-Driver proof"
            );
        }
        anyhow::bail!(
            "provider '{provider_id}' registered without required lifecycle Driver '{expected}'"
        );
    }
    if observed[0] == expected {
        return Ok(Some(observed[0].clone()));
    }
    let legacy_fallback = format!("{}/driver", provider_namespace.trim_end_matches('/'));
    if allow_old_artifact_fallback
        && expected == "robonix/lifecycle/driver"
        && observed[0] == legacy_fallback
    {
        return Ok(Some(observed[0].clone()));
    }
    anyhow::bail!(
        "provider '{provider_id}' registered lifecycle Driver '{}', expected '{expected}' from the selected package manifest",
        observed[0]
    )
}

/// Enforce that Atlas exposes exactly the lifecycle Driver selected by the
/// package manifest. This check runs before INIT, so config can never be sent
/// to a missing, mismatched, or ambiguous runtime service.
pub fn validate_runtime_driver_contracts(
    provider_id: &str,
    expected: &str,
    observed: &[String],
) -> Result<String> {
    resolve_runtime_driver_contract(provider_id, "", expected, observed, false, false)?.ok_or_else(
        || anyhow::anyhow!("strict lifecycle validation unexpectedly resolved no Driver"),
    )
}

fn runtime_driver_contracts(provider: &atlas_pb::CapabilityProvider) -> Vec<String> {
    let mut contracts = provider
        .capabilities
        .iter()
        .filter(|capability| {
            capability.transport == atlas_pb::Transport::Grpc as i32
                && capability.contract_id.ends_with("/driver")
        })
        .map(|capability| capability.contract_id.clone())
        .collect::<Vec<_>>();
    contracts.sort();
    contracts.dedup();
    contracts
}

/// Poll atlas until a provider NOT in `before` appears, then briefly
/// settle to see if it also declares a `*/driver` gRPC capability.
///
/// No terminal UI — pure RPC + sleep loop. Callers that want spinner /
/// boot_progress output (rbnx CLI) wrap this with their own decorator.
/// Soma calls it directly and logs through scribe.
///
/// Errors:
///   * timeout (no new provider before `DRIVER_REGISTER_TIMEOUT`),
///   * multiple new providers from a single spawn (a deploy bug — the
///     spec is one package start = one Capability(id=...)),
///   * the new provider vanished between matching and settling (crashed
///     or was evicted by heartbeat lapse).
///
/// `who` is a free-form label included in error messages so the caller
/// can tell which spawn this poll belonged to.
pub async fn wait_for_registration_core(
    atlas: &mut AtlasClient,
    before: &ProviderRegistrationSnapshot,
    who: &str,
) -> Result<RegistrationOutcome> {
    // Poll cadence is decoupled from any spinner refresh: we just sleep
    // 200ms between Atlas queries. That's the same cadence the CLI used
    // (one query per 2 spinner ticks at 100ms/tick).
    const POLL_INTERVAL: Duration = Duration::from_millis(200);
    let started = Instant::now();
    let deadline = started + DRIVER_REGISTER_TIMEOUT;
    loop {
        let providers = atlas
            .query_capabilities("", "", atlas_pb::Transport::Unspecified)
            .await
            .with_context(|| format!("[{who}] poll atlas"))?;
        let matches: Vec<&atlas_pb::CapabilityProvider> = providers
            .iter()
            .filter(|provider| is_new_provider_registration(provider, before))
            .collect();
        if matches.len() > 1 {
            let cap_ids: Vec<&str> = matches.iter().map(|r| r.id.as_str()).collect();
            anyhow::bail!(
                "[{who}] multiple new providers appeared from one spawn: {} \
                 — spec is one package start = one Capability(id=...)",
                cap_ids.join(", ")
            );
        }
        if let Some(first) = matches.first() {
            let provider_id = first.id.clone();
            let registration_id = first.registration_id.clone();
            // RegisterPrimitive/Service/Skill and DeclareCapability are
            // two separate RPCs from the package side — Register lands
            // first, declares follow within a few hundred ms. Give it
            // up to a 1s settle window so we don't false-fire a missing
            // Driver error on a fast poll. Capped by the outer
            // deadline so we never exceed the user-facing timeout.
            let settle_until = Instant::now()
                .checked_add(Duration::from_millis(1000))
                .map(|t| t.min(deadline))
                .unwrap_or(deadline);
            let mut current: atlas_pb::CapabilityProvider = (*first).clone();
            // Always consume the whole settle window. RegisterProvider and
            // DeclareCapability are separate RPCs; returning after the first
            // Driver would hide a second shared/legacy declaration and make
            // lifecycle ownership ambiguous.
            loop {
                if Instant::now() >= settle_until {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
                let providers = atlas
                    .query_capabilities(&provider_id, "", atlas_pb::Transport::Unspecified)
                    .await
                    .with_context(|| format!("[{who}] re-poll for driver"))?;
                match providers.into_iter().find(|p| p.id == provider_id) {
                    Some(p) if p.registration_id == registration_id => current = p,
                    Some(p) => {
                        anyhow::bail!(
                            "[{who}] provider '{provider_id}' registration changed during settle ('{registration_id}' -> '{}')",
                            p.registration_id,
                        );
                    }
                    None => {
                        // Provider vanished between the original match
                        // and now (crashed mid-settle, atlas evicted,
                        // heartbeat lapsed). Caller's downstream logic must
                        // not march on against a dead process; instead fail
                        // here so the caller can log + reap.
                        anyhow::bail!(
                            "[{who}] provider '{provider_id}' unregistered during settle window",
                        );
                    }
                }
            }
            let driver_contracts = runtime_driver_contracts(&current);
            return Ok(RegistrationOutcome {
                provider_id,
                provider_kind: current.kind,
                provider_state: current.state,
                provider_namespace: current.namespace,
                provider_state_detail: current.state_detail,
                registration_id: current.registration_id,
                driver_contracts,
            });
        }
        if Instant::now() >= deadline {
            anyhow::bail!(
                "[{who}] timed out after {DRIVER_REGISTER_TIMEOUT:?} — package never \
                 registered a provider with atlas",
            );
        }
        tokio::time::sleep(POLL_INTERVAL).await;
    }
}

/// Issue one `Driver(cmd)` RPC against a freshly-connected channel, then
/// release the channel. Returns the response's `state` string on success;
/// bails when ok=false or the RPC itself fails. Mirrors rbnx-cli's boot
/// behaviour for both CMD_INIT and CMD_ACTIVATE so Soma drives the same
/// lifecycle transitions.
///
/// `who` shows up in every error message so callers can attribute
/// failures to a specific package without re-wrapping each return.
pub async fn call_driver_cmd(
    atlas: &mut AtlasClient,
    provider_id: &str,
    driver_contract: &str,
    cmd: u32,
    config_json: String,
    who: &str,
) -> Result<String> {
    let cmd_name = match cmd {
        CMD_INIT => "INIT",
        CMD_ACTIVATE => "ACTIVATE",
        CMD_DEACTIVATE => "DEACTIVATE",
        CMD_SHUTDOWN => "SHUTDOWN",
        _ => "?",
    };
    let (channel_id, endpoint, _params) = atlas
        .connect_capability(
            BOOT_CONSUMER_ID,
            provider_id,
            driver_contract,
            atlas_pb::Transport::Grpc,
        )
        .await
        .with_context(|| format!("[{who}] ConnectCapability for {driver_contract}"))?;
    let normalized = if endpoint.starts_with("http") {
        endpoint
    } else {
        format!("http://{endpoint}")
    };
    let result = async {
        let channel = Endpoint::new(normalized.clone())
            .with_context(|| format!("invalid driver endpoint '{normalized}'"))?
            .connect()
            .await
            .with_context(|| format!("dial driver at '{normalized}'"))?;
        let svc_name = contract_id_to_service_name(driver_contract);
        let path: tonic::codegen::http::uri::PathAndQuery =
            format!("/robonix.contracts.{svc_name}/Driver")
                .parse()
                .with_context(|| format!("build gRPC path for '{driver_contract}'"))?;
        let mut grpc = tonic::client::Grpc::new(channel);
        grpc.ready().await.with_context(|| "gRPC ready")?;
        let codec: tonic_prost::ProstCodec<DriverRequest, DriverResponse> = Default::default();
        let resp = tokio::time::timeout(
            DRIVER_INIT_TIMEOUT,
            grpc.unary(
                Request::new(DriverRequest {
                    command: cmd,
                    config_json,
                }),
                path,
                codec,
            ),
        )
        .await
        .map_err(|_| {
            anyhow::anyhow!(
                "Driver(CMD_{cmd_name}) timed out after {:?}",
                DRIVER_INIT_TIMEOUT
            )
        })?
        .with_context(|| format!("Driver(CMD_{cmd_name}) RPC failed"))?;
        Ok::<_, anyhow::Error>(resp.into_inner())
    }
    .await;
    // Always disconnect, even on error — Atlas counts open channels and
    // we don't want them to leak when boot races a half-up provider.
    let _ = atlas.disconnect_capability(&channel_id).await;
    let r = result.map_err(|e| anyhow::anyhow!("[{who}] Driver(CMD_{cmd_name}): {e:#}"))?;
    if !r.ok {
        anyhow::bail!(
            "[{who}] Driver(CMD_{cmd_name}) returned ok=false (state={}, error={})",
            r.state,
            r.error
        );
    }
    Ok(r.state)
}

#[cfg(test)]
mod tests {
    use super::{
        PackageRuntimeRecord, atlas_pb, is_new_provider_registration,
        linux_proc_status_is_not_zombie, macos_procargs_has_environment_entry,
        macos_ps_state_is_not_zombie, process_group_has_members, resolve_runtime_driver_contract,
        shutdown_package_runtime_checked, terminate_process_group_guarded,
        validate_runtime_driver_contracts,
    };

    const SHARED: &str = "robonix/lifecycle/driver";
    const LEGACY: &str = "robonix/primitive/camera/driver";

    #[test]
    fn runtime_driver_must_exactly_match_shared_or_legacy_selection() {
        assert_eq!(
            validate_runtime_driver_contracts("camera", SHARED, &[SHARED.to_string()]).unwrap(),
            SHARED
        );
        assert_eq!(
            validate_runtime_driver_contracts("camera", LEGACY, &[LEGACY.to_string()]).unwrap(),
            LEGACY
        );
    }

    #[test]
    fn runtime_driver_rejects_missing_mismatched_and_dual_declarations() {
        let missing = validate_runtime_driver_contracts("camera", SHARED, &[])
            .unwrap_err()
            .to_string();
        assert!(missing.contains("without required"));

        let mismatch = validate_runtime_driver_contracts("camera", SHARED, &[LEGACY.to_string()])
            .unwrap_err()
            .to_string();
        assert!(mismatch.contains("expected 'robonix/lifecycle/driver'"));

        let dual = validate_runtime_driver_contracts(
            "camera",
            SHARED,
            &[SHARED.to_string(), LEGACY.to_string()],
        )
        .unwrap_err()
        .to_string();
        assert!(dual.contains("multiple lifecycle Drivers"));
    }

    #[test]
    fn omitted_manifest_allows_only_old_artifact_driver_fallbacks() {
        assert_eq!(
            resolve_runtime_driver_contract(
                "camera",
                "robonix/primitive/camera",
                SHARED,
                &[SHARED.to_string()],
                true,
                false,
            )
            .unwrap(),
            Some(SHARED.to_string())
        );
        assert_eq!(
            resolve_runtime_driver_contract(
                "camera",
                "robonix/primitive/camera",
                SHARED,
                &[LEGACY.to_string()],
                true,
                false,
            )
            .unwrap(),
            Some(LEGACY.to_string())
        );
        assert_eq!(
            resolve_runtime_driver_contract(
                "camera",
                "robonix/primitive/camera",
                SHARED,
                &[],
                true,
                true,
            )
            .unwrap(),
            None
        );
        assert!(
            resolve_runtime_driver_contract(
                "camera",
                "robonix/primitive/camera",
                SHARED,
                &[],
                true,
                false,
            )
            .unwrap_err()
            .to_string()
            .contains("did not publish old-artifact")
        );
        assert!(
            resolve_runtime_driver_contract(
                "camera",
                "robonix/primitive/camera",
                SHARED,
                &["robonix/primitive/lidar/driver".to_string()],
                true,
                false,
            )
            .unwrap_err()
            .to_string()
            .contains("expected 'robonix/lifecycle/driver'")
        );
        assert!(
            resolve_runtime_driver_contract(
                "camera",
                "robonix/primitive/camera",
                LEGACY,
                &[],
                true,
                true,
            )
            .unwrap_err()
            .to_string()
            .contains("without required")
        );
    }

    #[test]
    fn registration_snapshot_detects_same_id_takeover_but_not_same_generation() {
        let before =
            std::collections::HashMap::from([("camera".to_string(), "registration-a".to_string())]);
        let provider = |id: &str, registration_id: &str| atlas_pb::CapabilityProvider {
            id: id.to_string(),
            registration_id: registration_id.to_string(),
            ..Default::default()
        };

        assert!(!is_new_provider_registration(
            &provider("camera", "registration-a"),
            &before,
        ));
        assert!(is_new_provider_registration(
            &provider("camera", "registration-b"),
            &before,
        ));
        assert!(is_new_provider_registration(
            &provider("lidar", "registration-a"),
            &before,
        ));
    }

    #[test]
    fn linux_proc_status_zombie_detection_is_conservative() {
        assert!(!linux_proc_status_is_not_zombie(
            "Name:\tworker\nState:\tZ (zombie)\n"
        ));
        assert!(linux_proc_status_is_not_zombie(
            "Name:\tworker\nState:\tS (sleeping)\n"
        ));
        assert!(linux_proc_status_is_not_zombie("Name:\tworker\n"));
    }

    #[test]
    fn macos_procargs_parser_matches_exact_environment_entry() {
        let mut procargs = 2_i32.to_ne_bytes().to_vec();
        procargs.extend_from_slice(
            b"/usr/local/bin/worker\0\0worker\0--serve\0MODE=test\0RBNX_BOOT_ID=boot-123\0\0",
        );

        assert_eq!(
            macos_procargs_has_environment_entry(&procargs, b"RBNX_BOOT_ID=boot-123"),
            Some(true)
        );
        assert_eq!(
            macos_procargs_has_environment_entry(&procargs, b"RBNX_BOOT_ID=boot"),
            Some(false)
        );
        assert_eq!(
            macos_procargs_has_environment_entry(&2_i32.to_ne_bytes(), b"MODE=test"),
            None
        );

        let mut truncated = procargs;
        truncated.pop();
        truncated.pop();
        assert_eq!(
            macos_procargs_has_environment_entry(&truncated, b"RBNX_BOOT_ID=boot-123"),
            None
        );
    }

    #[test]
    fn macos_ps_state_parser_only_excludes_zombies() {
        assert_eq!(macos_ps_state_is_not_zombie("Z+  \n"), Some(false));
        assert_eq!(macos_ps_state_is_not_zombie("S   \n"), Some(true));
        assert_eq!(macos_ps_state_is_not_zombie(" \n"), None);
    }

    #[tokio::test]
    async fn live_process_group_is_not_reported_empty() {
        use std::os::unix::process::CommandExt;

        let mut child = std::process::Command::new("sleep")
            .arg("30")
            .process_group(0)
            .spawn()
            .unwrap();
        let pgid = child.id();
        let detected = process_group_has_members(pgid).await;
        let _ = nix::sys::signal::killpg(
            nix::unistd::Pid::from_raw(pgid as i32),
            nix::sys::signal::Signal::SIGKILL,
        );
        let _ = child.wait();

        assert!(detected);
    }

    #[tokio::test]
    async fn checked_shutdown_accepts_matching_boot_group_and_terminates_it() {
        use std::os::unix::process::CommandExt;

        let boot_id = format!("shutdown-test-{}", std::process::id());
        let mut child = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "launch::tests::boot_identity_fixture_process",
                "--ignored",
                "--test-threads=1",
            ])
            .env("RBNX_BOOT_ID", &boot_id)
            .process_group(0)
            .spawn()
            .unwrap();
        let pgid = child.id();
        let record = PackageRuntimeRecord {
            name: "shutdown-test".into(),
            pid: pgid,
            pgid,
            ..PackageRuntimeRecord::default()
        };

        let complete = shutdown_package_runtime_checked(
            None,
            &record,
            std::time::Duration::from_secs(2),
            Some(&boot_id),
        )
        .await;
        let still_running = child.try_wait().unwrap().is_none();
        if still_running {
            let _ = nix::sys::signal::killpg(
                nix::unistd::Pid::from_raw(pgid as i32),
                nix::sys::signal::Signal::SIGKILL,
            );
        }
        let _ = child.wait();

        assert!(complete);
        assert!(!still_running);
    }

    #[tokio::test]
    async fn mismatched_boot_group_is_rejected_without_signaling() {
        use std::os::unix::process::CommandExt;

        let boot_id = format!("mismatch-test-{}", std::process::id());
        let mut child = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "launch::tests::boot_identity_fixture_process",
                "--ignored",
                "--test-threads=1",
            ])
            .env("RBNX_BOOT_ID", &boot_id)
            .process_group(0)
            .spawn()
            .unwrap();
        let pgid = child.id();

        let accepted = terminate_process_group_guarded(
            pgid,
            std::time::Duration::from_millis(10),
            Some("different-boot-id"),
        )
        .await;
        let still_running = child.try_wait().unwrap().is_none();
        let _ = nix::sys::signal::killpg(
            nix::unistd::Pid::from_raw(pgid as i32),
            nix::sys::signal::Signal::SIGKILL,
        );
        let _ = child.wait();

        assert!(!accepted);
        assert!(still_running);
    }

    #[tokio::test]
    async fn zombie_only_process_group_is_reported_empty() {
        use super::process_is_not_zombie;
        use std::os::unix::process::CommandExt;

        let mut child = std::process::Command::new("sh")
            .arg("-c")
            .arg("exit 0")
            .process_group(0)
            .spawn()
            .unwrap();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while process_is_not_zombie(child.id()).await && std::time::Instant::now() < deadline {
            tokio::time::sleep(std::time::Duration::from_millis(10)).await;
        }
        let zombie_detected = !process_is_not_zombie(child.id()).await;
        let group_has_members = process_group_has_members(child.id()).await;
        let _ = child.wait();

        assert!(zombie_detected);
        assert!(!group_has_members);
    }

    #[test]
    #[ignore = "spawned by checked_shutdown_accepts_matching_boot_group_and_terminates_it"]
    fn boot_identity_fixture_process() {
        std::thread::sleep(std::time::Duration::from_secs(30));
    }
}
