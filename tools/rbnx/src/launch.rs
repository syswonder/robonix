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
//     the expected provider's pre-spawn registration fingerprint so callers
//     can recognize either a new id or a same-id endpoint takeover. Returns
//     `(provider_id, driver_contract)`.
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
use std::collections::BTreeSet;
use std::path::PathBuf;
use std::process::Stdio;
use std::time::{Duration, Instant};
use tokio::process::Command;
use tonic::transport::Endpoint;
use tonic::{Code, Request};

// ── Driver.srv command discriminators (mirrors capabilities/lib/lifecycle/srv/Driver.srv) ──

pub const CMD_INIT: u32 = 0;
pub const CMD_ACTIVATE: u32 = 1;
#[allow(dead_code)]
pub const CMD_DEACTIVATE: u32 = 2;
#[allow(dead_code)]
pub const CMD_SHUTDOWN: u32 = 3;

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

    terminate_process_group(record.pgid, term_wait).await;
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
    use nix::sys::signal::{Signal, killpg};
    use nix::unistd::Pid;
    let pgid_raw = pgid;
    let pgid = Pid::from_raw(pgid as i32);
    match killpg(pgid, Signal::SIGTERM) {
        Ok(()) => {}
        Err(nix::errno::Errno::ESRCH) => return,
        Err(e) => warn!("SIGTERM pgid {} failed: {e}", pgid),
    }

    let deadline = Instant::now() + term_wait;
    while Instant::now() < deadline {
        if !process_group_has_members(pgid_raw).await {
            return;
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
    }

    let _ = killpg(pgid, Signal::SIGKILL);
}

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
        return false;
    }

    let stdout = String::from_utf8_lossy(&output.stdout);
    stdout.lines().any(|line| {
        line.trim()
            .parse::<u32>()
            .map(process_is_not_zombie)
            .unwrap_or(true)
    })
}

fn process_is_not_zombie(pid: u32) -> bool {
    let status_path = format!("/proc/{pid}/status");
    let Ok(status) = std::fs::read_to_string(status_path) else {
        return false;
    };
    !status
        .lines()
        .any(|line| line.starts_with("State:") && line.contains("Z"))
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

/// One declared capability's stable registration identity.
///
/// Atlas Query intentionally omits resolved endpoints, so fingerprint capture
/// opens a metadata-only ConnectCapability probe and immediately disconnects
/// it. The probe never dials the provider process itself.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct CapabilityEndpointFingerprint {
    contract_id: String,
    transport: i32,
    endpoint: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ProviderRegistrationFingerprint {
    provider_id: String,
    capabilities: BTreeSet<CapabilityEndpointFingerprint>,
}

/// Opaque pre-spawn state for exactly one manifest-expected provider id.
///
/// Comparing endpoint fingerprints distinguishes the dynamic-port takeover
/// used by package wrappers without relying on heartbeat or lifecycle state,
/// both of which change during ordinary operation. A takeover that reuses the
/// exact same fixed endpoint cannot be distinguished if polling misses Atlas's
/// brief endpoint-clear interval; that is an explicit boundary of the current
/// Atlas read protocol.
#[derive(Debug, Clone)]
pub struct ProviderRegistrationSnapshot {
    expected_provider_id: String,
    fingerprint: Option<ProviderRegistrationFingerprint>,
}

enum FingerprintRead {
    Stable(Option<ProviderRegistrationFingerprint>),
    Retry,
}

fn atlas_not_found(error: &anyhow::Error) -> bool {
    error
        .chain()
        .filter_map(|cause| cause.downcast_ref::<tonic::Status>())
        .any(|status| status.code() == Code::NotFound)
}

/// Read one expected provider's capability endpoint fingerprint.
///
/// Register takeover clears the old capability set before the new process
/// redeclares it. If that happens between Query and a probe ConnectCapability,
/// report a transient retry instead of failing package startup. Every probe
/// that successfully opens a channel disconnects it before this function can
/// return, including when a later probe encounters an error.
async fn read_provider_fingerprint(
    atlas: &mut AtlasClient,
    expected_provider_id: &str,
) -> Result<FingerprintRead> {
    let providers = atlas
        .query_capabilities(expected_provider_id, "", atlas_pb::Transport::Unspecified)
        .await
        .with_context(|| format!("query expected provider '{expected_provider_id}'"))?;
    let Some(provider) = providers
        .into_iter()
        .find(|provider| provider.id == expected_provider_id)
    else {
        return Ok(FingerprintRead::Stable(None));
    };

    let mut capabilities = BTreeSet::new();
    for capability in provider.capabilities {
        let transport = atlas_pb::Transport::try_from(capability.transport)
            .unwrap_or(atlas_pb::Transport::Unspecified);
        let connected = atlas
            .connect_capability(
                BOOT_CONSUMER_ID,
                expected_provider_id,
                &capability.contract_id,
                transport,
            )
            .await;
        let (channel_id, endpoint, _params) = match connected {
            Ok(binding) => binding,
            Err(error) if atlas_not_found(&error) => return Ok(FingerprintRead::Retry),
            Err(error) => {
                return Err(error).with_context(|| {
                    format!(
                        "probe expected provider '{expected_provider_id}' capability '{}'",
                        capability.contract_id
                    )
                });
            }
        };
        let disconnected = atlas
            .disconnect_capability(&channel_id)
            .await
            .with_context(|| format!("disconnect registration probe '{channel_id}'"))?;
        if !disconnected {
            // A same-id takeover auto-drops old channels. The probe has no
            // remaining Atlas state, but its endpoint sample is no longer a
            // stable view of the current registration.
            return Ok(FingerprintRead::Retry);
        }
        capabilities.insert(CapabilityEndpointFingerprint {
            contract_id: capability.contract_id,
            transport: capability.transport,
            endpoint,
        });
    }

    Ok(FingerprintRead::Stable(Some(
        ProviderRegistrationFingerprint {
            provider_id: expected_provider_id.to_string(),
            capabilities,
        },
    )))
}

/// Snapshot one manifest-expected provider before spawning its package.
pub async fn snapshot_provider_registration(
    atlas: &mut AtlasClient,
    expected_provider_id: &str,
) -> Result<ProviderRegistrationSnapshot> {
    // A concurrent takeover can invalidate Query before the endpoint probes.
    // Re-query without a fixed sleep; ordinary package spawn has not started
    // yet, so a small bounded retry is sufficient to obtain a stable baseline.
    const SNAPSHOT_ATTEMPTS: usize = 4;
    for _ in 0..SNAPSHOT_ATTEMPTS {
        match read_provider_fingerprint(atlas, expected_provider_id).await? {
            FingerprintRead::Stable(fingerprint) => {
                return Ok(ProviderRegistrationSnapshot {
                    expected_provider_id: expected_provider_id.to_string(),
                    fingerprint,
                });
            }
            FingerprintRead::Retry => tokio::task::yield_now().await,
        }
    }
    anyhow::bail!(
        "pre-spawn atlas snapshot for '{expected_provider_id}' changed during endpoint probes"
    )
}

fn changed_expected_registration<'a>(
    before: &ProviderRegistrationSnapshot,
    current: &'a [ProviderRegistrationFingerprint],
) -> Option<&'a ProviderRegistrationFingerprint> {
    current.iter().find(|provider| {
        provider.provider_id == before.expected_provider_id
            && before.fingerprint.as_ref() != Some(*provider)
    })
}

/// Outcome of `wait_for_registration_core`:
///   * `provider_id` — the expected provider whose registration appeared or
///     changed endpoint fingerprint after `before`.
///   * `driver_contract` — if that registration also declared a
///     `*/driver` gRPC capability within the settle window, its
///     contract_id; otherwise `None` (no Driver(CMD_*) lifecycle).
#[derive(Debug, Clone)]
pub struct RegistrationOutcome {
    pub provider_id: String,
    pub driver_contract: Option<String>,
}

/// Poll Atlas until the expected provider appears or changes its endpoint
/// fingerprint, then briefly settle for its `*/driver` gRPC capability.
///
/// No terminal UI — pure RPC + sleep loop. Callers that want spinner /
/// boot_progress output (rbnx CLI) wrap this with their own decorator.
/// Soma calls it directly and logs through scribe.
///
/// Errors:
///   * timeout (the expected registration did not change before
///     `DRIVER_REGISTER_TIMEOUT`),
///   * the expected provider vanished between matching and settling (crashed
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
    'poll: loop {
        if Instant::now() >= deadline {
            anyhow::bail!(
                "[{who}] timed out after {DRIVER_REGISTER_TIMEOUT:?} — package never \
                 registered expected provider '{}' with a new endpoint fingerprint",
                before.expected_provider_id,
            );
        }
        let current = match read_provider_fingerprint(atlas, &before.expected_provider_id)
            .await
            .with_context(|| format!("[{who}] poll atlas"))?
        {
            FingerprintRead::Stable(current) => current,
            FingerprintRead::Retry => {
                tokio::time::sleep(POLL_INTERVAL).await;
                continue;
            }
        };
        if let Some(first) = current.as_ref().and_then(|provider| {
            changed_expected_registration(before, std::slice::from_ref(provider))
        }) {
            let provider_id = first.provider_id.clone();
            // RegisterPrimitive/Service/Skill and DeclareCapability are
            // two separate RPCs from the package side — Register lands
            // first, declares follow within a few hundred ms. Give it
            // up to a 1s settle window so we don't false-fire the
            // "no driver" path on a fast poll. Capped by the outer
            // deadline so we never exceed the user-facing timeout.
            let settle_until = Instant::now()
                .checked_add(Duration::from_millis(1000))
                .map(|t| t.min(deadline))
                .unwrap_or(deadline);
            let mut current = first.clone();
            let driver_contract = loop {
                let driver = current.capabilities.iter().find(|cap| {
                    cap.transport == atlas_pb::Transport::Grpc as i32
                        && cap.contract_id.ends_with("/driver")
                });
                if driver.is_some() {
                    break driver.map(|c| c.contract_id.clone());
                }
                if Instant::now() >= settle_until {
                    break None;
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
                match read_provider_fingerprint(atlas, &provider_id)
                    .await
                    .with_context(|| format!("[{who}] re-poll for driver"))?
                {
                    FingerprintRead::Retry => continue,
                    FingerprintRead::Stable(Some(next)) => {
                        if changed_expected_registration(before, std::slice::from_ref(&next))
                            .is_none()
                        {
                            // The endpoint set returned to its pre-spawn
                            // fingerprint. We cannot prove this registration
                            // belongs to the spawned process, so resume the
                            // outer wait instead of sending CMD_INIT.
                            continue 'poll;
                        }
                        current = next;
                    }
                    FingerprintRead::Stable(None) => {
                        // Provider vanished between the original match
                        // and now (crashed mid-settle, atlas evicted,
                        // heartbeat lapsed). Caller's downstream logic
                        // would silently treat "no driver" as fine and
                        // march on against a dead process; instead fail
                        // here so the caller can log + reap.
                        anyhow::bail!(
                            "[{who}] provider '{provider_id}' unregistered during settle window",
                        );
                    }
                }
            };
            return Ok(RegistrationOutcome {
                provider_id,
                driver_contract,
            });
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
mod registration_tests {
    use super::{
        CapabilityEndpointFingerprint, ProviderRegistrationFingerprint,
        ProviderRegistrationSnapshot, changed_expected_registration,
        snapshot_provider_registration, wait_for_registration_core,
    };
    use robonix_atlas::client::{AtlasClient, grpc_params};
    use robonix_atlas::pb;
    use robonix_atlas::service::{AtlasRegistry, serve_atlas};
    use std::collections::BTreeSet;
    use std::sync::Arc;
    use std::time::Duration;

    const EXPECTED_ID: &str = "health_piper";
    const DRIVER_CONTRACT: &str = "robonix/primitive/health/driver";

    fn grpc_capability(contract_id: &str, endpoint: &str) -> CapabilityEndpointFingerprint {
        CapabilityEndpointFingerprint {
            contract_id: contract_id.to_string(),
            transport: pb::Transport::Grpc as i32,
            endpoint: endpoint.to_string(),
        }
    }

    fn provider(
        provider_id: &str,
        capabilities: impl IntoIterator<Item = CapabilityEndpointFingerprint>,
    ) -> ProviderRegistrationFingerprint {
        ProviderRegistrationFingerprint {
            provider_id: provider_id.to_string(),
            capabilities: capabilities.into_iter().collect::<BTreeSet<_>>(),
        }
    }

    fn snapshot(
        expected_provider_id: &str,
        fingerprint: Option<ProviderRegistrationFingerprint>,
    ) -> ProviderRegistrationSnapshot {
        ProviderRegistrationSnapshot {
            expected_provider_id: expected_provider_id.to_string(),
            fingerprint,
        }
    }

    /// A provider absent from the snapshot is accepted when it first appears.
    #[test]
    fn new_expected_provider_matches_registration() {
        let before = snapshot(EXPECTED_ID, None);
        let current = provider(
            EXPECTED_ID,
            [grpc_capability(DRIVER_CONTRACT, "127.0.0.1:34001")],
        );

        assert_eq!(
            changed_expected_registration(&before, std::slice::from_ref(&current)),
            Some(&current)
        );
    }

    /// Reusing an identity with a new endpoint is treated as provider takeover.
    #[test]
    fn same_id_new_endpoint_matches_takeover() {
        let before = snapshot(
            EXPECTED_ID,
            Some(provider(
                EXPECTED_ID,
                [grpc_capability(DRIVER_CONTRACT, "127.0.0.1:34001")],
            )),
        );
        let takeover = provider(
            EXPECTED_ID,
            [grpc_capability(DRIVER_CONTRACT, "127.0.0.1:34513")],
        );

        assert_eq!(
            changed_expected_registration(&before, std::slice::from_ref(&takeover)),
            Some(&takeover)
        );
    }

    /// An unchanged endpoint fingerprint cannot satisfy a new launch attempt.
    #[test]
    fn same_id_unchanged_fingerprint_does_not_match() {
        let unchanged = provider(
            EXPECTED_ID,
            [grpc_capability(DRIVER_CONTRACT, "127.0.0.1:34001")],
        );
        let before = snapshot(EXPECTED_ID, Some(unchanged.clone()));

        assert_eq!(changed_expected_registration(&before, &[unchanged]), None);
    }

    /// Changes from unrelated providers cannot satisfy the expected registration.
    #[test]
    fn unrelated_provider_change_does_not_match_expected_provider() {
        let expected = provider(
            EXPECTED_ID,
            [grpc_capability(DRIVER_CONTRACT, "127.0.0.1:34001")],
        );
        let before = snapshot(EXPECTED_ID, Some(expected.clone()));
        let unrelated = provider(
            "other_provider",
            [grpc_capability(
                "robonix/primitive/other/driver",
                "127.0.0.1:35001",
            )],
        );

        assert_eq!(
            changed_expected_registration(&before, &[expected, unrelated]),
            None
        );
    }

    /// Takeover waits through Atlas' capability gap until the new driver appears.
    #[tokio::test]
    async fn same_id_takeover_waits_for_driver_declared_during_settle() {
        let probe = std::net::TcpListener::bind("127.0.0.1:0").expect("bind Atlas probe");
        let atlas_addr = probe.local_addr().expect("Atlas probe address");
        drop(probe);

        let registry = Arc::new(AtlasRegistry::default());
        let server = tokio::spawn(serve_atlas(Arc::clone(&registry), atlas_addr));
        let endpoint = format!("http://{atlas_addr}");
        let mut atlas = AtlasClient::connect_with_retry(&endpoint, 20, Duration::from_millis(25))
            .await
            .expect("connect test Atlas");

        atlas
            .register_primitive(EXPECTED_ID, "robonix/primitive/health", "")
            .await
            .expect("register old provider");
        atlas
            .declare_capability(
                EXPECTED_ID,
                DRIVER_CONTRACT,
                pb::Transport::Grpc,
                "127.0.0.1:34001",
                grpc_params("", "", ""),
            )
            .await
            .expect("declare old driver");
        let before = snapshot_provider_registration(&mut atlas, EXPECTED_ID)
            .await
            .expect("snapshot old provider");
        let inspect: serde_json::Value = serde_json::from_str(
            &registry
                .inspect_json()
                .await
                .expect("inspect Atlas after snapshot"),
        )
        .expect("parse Atlas inspection");
        assert_eq!(
            inspect["channels"]
                .as_object()
                .map(|channels| channels.len()),
            Some(0),
            "registration fingerprint probes must disconnect every Atlas channel",
        );

        let mut takeover_client = atlas.clone();
        let takeover = tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(50)).await;
            takeover_client
                .register_primitive(EXPECTED_ID, "robonix/primitive/health", "")
                .await
                .expect("take over provider id");
            tokio::time::sleep(Duration::from_millis(150)).await;
            takeover_client
                .declare_capability(
                    EXPECTED_ID,
                    DRIVER_CONTRACT,
                    pb::Transport::Grpc,
                    "127.0.0.1:34513",
                    grpc_params("", "", ""),
                )
                .await
                .expect("declare takeover driver");
        });

        let outcome = tokio::time::timeout(
            Duration::from_secs(3),
            wait_for_registration_core(&mut atlas, &before, "primitive/health_piper"),
        )
        .await
        .expect("registration wait must not reach the 60s production timeout")
        .expect("recognize same-id takeover");

        assert_eq!(outcome.provider_id, EXPECTED_ID);
        assert_eq!(outcome.driver_contract.as_deref(), Some(DRIVER_CONTRACT));
        takeover.await.expect("takeover task");
        server.abort();
    }
}
