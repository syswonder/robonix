// SPDX-License-Identifier: MulanPSL-2.0
//
// Poll system modules that expose ModuleHealthReport and feed their reports
// into Vitals' aggregate ModuleHealthStore.

use crate::config::{
    EXECUTOR_GET_HEALTH_CONTRACT, ExpectedModuleConfig, ExpectedModulePolicy,
    PILOT_GET_HEALTH_CONTRACT,
};
use crate::module_health::{HEALTH_ERROR, HEALTH_OK, HEALTH_WARN, SOURCE_VITALS_SYNTHESIZED_STALE};
use crate::pb::contracts::{
    robonix_system_executor_get_health_client::RobonixSystemExecutorGetHealthClient,
    robonix_system_pilot_get_health_client::RobonixSystemPilotGetHealthClient,
};
use crate::pb::module_health::{
    GetModuleHealthRequest, ModuleHealth, ModuleHealthEvent, ModuleHealthReport,
};
use crate::service::VitalsServiceImpl;
use anyhow::{Context, Result};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tonic::transport::Channel;

const MODULE_HEALTH_POLL_INTERVAL: Duration = Duration::from_secs(2);
const MODULE_HEALTH_RPC_TIMEOUT: Duration = Duration::from_secs(1);

#[derive(Clone)]
struct ModuleHealthPollTarget {
    expected: ExpectedModuleConfig,
    label: String,
    contract_id: String,
    client_kind: ModuleHealthClientKind,
}

#[derive(Clone, Copy)]
enum ModuleHealthClientKind {
    Executor,
    Pilot,
}

/// Poll supported module health endpoints and synthesize stale required modules.
///
/// This spawns a background task that runs until the Vitals process exits.
pub fn spawn_module_health_poller(
    mut atlas: AtlasClient,
    consumer_id: String,
    svc: VitalsServiceImpl,
    expected_modules: Vec<ExpectedModuleConfig>,
) {
    let targets = build_poll_targets(&expected_modules);

    tokio::spawn(async move {
        let mut tick = tokio::time::interval(MODULE_HEALTH_POLL_INTERVAL);
        tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        tick.tick().await; // consume interval's immediate first tick

        let started_at = Instant::now();
        let mut available = HashMap::<String, bool>::new();
        let mut module_keys = HashMap::<String, String>::new();
        let mut missing_since = HashMap::<String, Instant>::new();

        loop {
            for target in &targets {
                let was_available = available.get(&target.label).copied().unwrap_or(false);
                match poll_target(&mut atlas, &consumer_id, &svc, target).await {
                    Ok(success) => {
                        if !was_available {
                            log::info!(
                                "[vitals] module health poll connected: {} ({})",
                                target.label,
                                target.contract_id
                            );
                        }
                        module_keys.insert(target.label.clone(), success.module_key);
                        missing_since.remove(&target.label);
                        if let Some(event) = success.event {
                            log_module_event(&event, target.expected.policy);
                        }
                        available.insert(target.label.clone(), true);
                    }
                    Err(e) => {
                        if was_available {
                            log::warn!("[vitals] module health poll lost {}: {e:#}", target.label);
                        } else {
                            log::debug!(
                                "[vitals] module health poll waiting for {}: {e:#}",
                                target.label
                            );
                        }

                        if let Some(module_key) = module_keys.get(&target.label) {
                            if let Some(event) = svc
                                .synthesize_stale_module_if_expired(
                                    module_key,
                                    target.expected.policy,
                                )
                                .await
                            {
                                log_module_event(&event, target.expected.policy);
                            }
                        } else if target.expected.policy == ExpectedModulePolicy::Required {
                            let missing_from = missing_since
                                .entry(target.label.clone())
                                .or_insert(started_at);
                            if missing_from.elapsed()
                                >= Duration::from_millis(u64::from(target.expected.ttl_ms))
                                && let Some(event) = svc
                                    .synthesize_expected_module_unavailable(&target.expected)
                                    .await
                            {
                                log_module_event(&event, target.expected.policy);
                            }
                        }

                        available.insert(target.label.clone(), false);
                    }
                }
            }

            tick.tick().await;
        }
    });
}

/// Convert enabled expected-module entries into clients supported by this poller.
fn build_poll_targets(expected_modules: &[ExpectedModuleConfig]) -> Vec<ModuleHealthPollTarget> {
    let mut targets = Vec::new();

    for expected in expected_modules {
        if expected.policy == ExpectedModulePolicy::Disabled {
            continue;
        }

        let Some((client_kind, contract_id)) = known_client(expected) else {
            if expected.policy == ExpectedModulePolicy::Required && expected.module_id != "vitals" {
                log::warn!(
                    "[vitals] required module '{}' has no module-health client adapter yet",
                    expected.module_id
                );
            }
            continue;
        };

        targets.push(ModuleHealthPollTarget {
            expected: expected.clone(),
            label: expected.module_key(),
            contract_id,
            client_kind,
        });
    }

    targets
}

/// Select the typed gRPC client for one known module or capability contract.
fn known_client(expected: &ExpectedModuleConfig) -> Option<(ModuleHealthClientKind, String)> {
    let capability = expected.capability.as_deref().unwrap_or_default();
    match (expected.module_id.as_str(), capability) {
        ("executor", _) | (_, EXECUTOR_GET_HEALTH_CONTRACT) => Some((
            ModuleHealthClientKind::Executor,
            EXECUTOR_GET_HEALTH_CONTRACT.to_string(),
        )),
        ("pilot", _) | (_, PILOT_GET_HEALTH_CONTRACT) => Some((
            ModuleHealthClientKind::Pilot,
            PILOT_GET_HEALTH_CONTRACT.to_string(),
        )),
        _ => None,
    }
}

struct ModuleHealthPollSuccess {
    module_key: String,
    event: Option<ModuleHealthEvent>,
}

/// Connect, fetch, ingest, and disconnect one module-health target per poll cycle.
async fn poll_target(
    atlas: &mut AtlasClient,
    consumer_id: &str,
    svc: &VitalsServiceImpl,
    target: &ModuleHealthPollTarget,
) -> Result<ModuleHealthPollSuccess> {
    let (channel_id, provider_id, channel) =
        atlas_client::connect_to_capability(atlas, consumer_id, &target.contract_id)
            .await
            .with_context(|| format!("connect module health target '{}'", target.label))?;

    let result = async {
        let report =
            call_get_health_with_timeout(target.client_kind, channel, MODULE_HEALTH_RPC_TIMEOUT)
                .await
                .with_context(|| format!("call module health target '{}'", target.label))?;
        let module_key = report
            .module
            .as_ref()
            .map(module_key_for)
            .unwrap_or_else(|| provider_id.clone());

        let event = svc.ingest_module_health_report(report).await.map_err(|e| {
            anyhow::anyhow!("ingest module health from provider '{provider_id}': {e}")
        })?;

        Ok::<_, anyhow::Error>(ModuleHealthPollSuccess { module_key, event })
    }
    .await;

    let _ = atlas.disconnect_capability(&channel_id).await;
    result
}

fn module_key_for(module: &ModuleHealth) -> String {
    let provider_id = module.provider_id.trim();
    if !provider_id.is_empty() {
        provider_id.to_string()
    } else {
        module.module_id.trim().to_string()
    }
}

/// Log every transition and elevate actionable non-optional failures to warnings.
fn log_module_event(event: &ModuleHealthEvent, policy: ExpectedModulePolicy) {
    log::info!(
        "[vitals] module {} health: {} -> {} ({})",
        event.module_key,
        health_label(event.previous_health),
        health_label(event.current_health),
        event.reason_code
    );

    let synthesized_optional_stale =
        policy == ExpectedModulePolicy::Optional && event.source == SOURCE_VITALS_SYNTHESIZED_STALE;
    if event.current_health != HEALTH_OK && !synthesized_optional_stale {
        log::warn!(
            "[vitals] ALERT: module {} — {}",
            event.module_key,
            event.detail
        );
    }
}

/// Call the module-specific health RPC and reject an empty report payload.
async fn call_get_health(
    client_kind: ModuleHealthClientKind,
    channel: Channel,
) -> Result<ModuleHealthReport> {
    match client_kind {
        ModuleHealthClientKind::Executor => {
            let mut client = RobonixSystemExecutorGetHealthClient::new(channel);
            let response = client
                .get_module_health(GetModuleHealthRequest {})
                .await
                .context("executor get_module_health")?
                .into_inner();
            response
                .report
                .ok_or_else(|| anyhow::anyhow!("executor returned empty ModuleHealthReport"))
        }
        ModuleHealthClientKind::Pilot => {
            let mut client = RobonixSystemPilotGetHealthClient::new(channel);
            let response = client
                .get_module_health(GetModuleHealthRequest {})
                .await
                .context("pilot get_module_health")?
                .into_inner();
            response
                .report
                .ok_or_else(|| anyhow::anyhow!("pilot returned empty ModuleHealthReport"))
        }
    }
}

/// Bound one module-health RPC so a stalled provider cannot block later targets.
async fn call_get_health_with_timeout(
    client_kind: ModuleHealthClientKind,
    channel: Channel,
    timeout: Duration,
) -> Result<ModuleHealthReport> {
    tokio::time::timeout(timeout, call_get_health(client_kind, channel))
        .await
        .with_context(|| {
            format!(
                "module health RPC timed out after {}ms",
                timeout.as_millis()
            )
        })?
}

fn health_label(health: u32) -> &'static str {
    match health {
        HEALTH_OK => "OK",
        HEALTH_WARN => "WARN",
        HEALTH_ERROR => "ERROR",
        _ => "UNKNOWN",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::net::TcpListener;
    use tonic::transport::Endpoint;

    /// A connected peer that never answers must fail within the configured timeout.
    #[tokio::test]
    async fn hanging_module_health_rpc_times_out() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (_socket, _) = listener.accept().await.unwrap();
            std::future::pending::<()>().await;
        });
        let channel = Endpoint::from_shared(format!("http://{address}"))
            .unwrap()
            .connect_lazy();

        let result = call_get_health_with_timeout(
            ModuleHealthClientKind::Pilot,
            channel,
            Duration::from_millis(25),
        )
        .await;

        server.abort();
        assert!(
            result
                .unwrap_err()
                .to_string()
                .contains("module health RPC timed out after 25ms")
        );
    }
}
