// SPDX-License-Identifier: MulanPSL-2.0
//
// Poll system modules that expose ModuleHealthReport and feed their reports
// into Vitals' aggregate ModuleHealthStore.

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
use std::time::Duration;
use tonic::transport::Channel;

const MODULE_HEALTH_POLL_INTERVAL: Duration = Duration::from_secs(2);
const EXECUTOR_GET_HEALTH_CONTRACT: &str = "robonix/system/executor/get_health";
const PILOT_GET_HEALTH_CONTRACT: &str = "robonix/system/pilot/get_health";

const TARGETS: &[ModuleHealthPollTarget] = &[
    ModuleHealthPollTarget {
        label: "executor",
        contract_id: EXECUTOR_GET_HEALTH_CONTRACT,
        client_kind: ModuleHealthClientKind::Executor,
    },
    ModuleHealthPollTarget {
        label: "pilot",
        contract_id: PILOT_GET_HEALTH_CONTRACT,
        client_kind: ModuleHealthClientKind::Pilot,
    },
];

#[derive(Clone, Copy)]
struct ModuleHealthPollTarget {
    label: &'static str,
    contract_id: &'static str,
    client_kind: ModuleHealthClientKind,
}

#[derive(Clone, Copy)]
enum ModuleHealthClientKind {
    Executor,
    Pilot,
}

pub fn spawn_module_health_poller(
    mut atlas: AtlasClient,
    consumer_id: String,
    svc: VitalsServiceImpl,
) {
    tokio::spawn(async move {
        let mut tick = tokio::time::interval(MODULE_HEALTH_POLL_INTERVAL);
        tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        tick.tick().await; // consume interval's immediate first tick

        let mut available = HashMap::<&'static str, bool>::new();
        let mut module_keys = HashMap::<&'static str, String>::new();

        loop {
            for target in TARGETS {
                let was_available = available.get(target.label).copied().unwrap_or(false);
                match poll_target(&mut atlas, &consumer_id, &svc, *target).await {
                    Ok(success) => {
                        if !was_available {
                            log::info!(
                                "[vitals] module health poll connected: {} ({})",
                                target.label,
                                target.contract_id
                            );
                        }
                        module_keys.insert(target.label, success.module_key);
                        if let Some(event) = success.event {
                            log_module_event(&event);
                        }
                        available.insert(target.label, true);
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
                        if let Some(module_key) = module_keys.get(target.label)
                            && let Some(event) =
                                svc.synthesize_stale_module_if_expired(module_key).await
                        {
                            log_module_event(&event);
                        }
                        available.insert(target.label, false);
                    }
                }
            }

            tick.tick().await;
        }
    });
}

struct ModuleHealthPollSuccess {
    module_key: String,
    event: Option<ModuleHealthEvent>,
}

async fn poll_target(
    atlas: &mut AtlasClient,
    consumer_id: &str,
    svc: &VitalsServiceImpl,
    target: ModuleHealthPollTarget,
) -> Result<ModuleHealthPollSuccess> {
    let (channel_id, provider_id, channel) =
        atlas_client::connect_to_capability(atlas, consumer_id, target.contract_id)
            .await
            .with_context(|| format!("connect module health target '{}'", target.label))?;

    let result = async {
        let report = call_get_health(target.client_kind, channel)
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

fn log_module_event(event: &ModuleHealthEvent) {
    log::info!(
        "[vitals] module {} health: {} -> {} ({})",
        event.module_key,
        health_label(event.previous_health),
        health_label(event.current_health),
        event.reason_code
    );
    if event.current_health != crate::module_health::HEALTH_OK {
        log::warn!(
            "[vitals] ALERT: module {} — {}",
            event.module_key,
            event.detail
        );
    }
}

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

fn health_label(health: u32) -> &'static str {
    match health {
        crate::module_health::HEALTH_OK => "OK",
        crate::module_health::HEALTH_WARN => "WARN",
        crate::module_health::HEALTH_ERROR => "ERROR",
        _ => "UNKNOWN",
    }
}
