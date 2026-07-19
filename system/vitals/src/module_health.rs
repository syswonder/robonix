// SPDX-License-Identifier: MulanPSL-2.0
//
// Module health aggregation for Vitals.
//
// V1 scope: normalize self-reported ModuleHealthReport frames, keep the latest
// state per module_key, and emit ModuleHealthEvent only when top-level health
// changes.

use crate::pb::module_health::{
    ModuleHealth, ModuleHealthEvent, ModuleHealthReport, ModuleHealthSnapshot,
};
use std::collections::{BTreeMap, VecDeque};
use std::fmt;

pub const MODULE_HEALTH_SCHEMA_VERSION: u32 = 1;
pub const HEALTH_OK: u32 = 0;
pub const HEALTH_WARN: u32 = 1;
pub const HEALTH_ERROR: u32 = 2;
pub const SOURCE_SELF_REPORTED: &str = "SELF_REPORTED";
pub const SOURCE_VITALS_SYNTHESIZED_STALE: &str = "VITALS_SYNTHESIZED_STALE";
pub const SOURCE_CONFIG_DISABLED: &str = "CONFIG_DISABLED";

const MAX_EVENTS: usize = 256;
const STALE_REASON_CODE: &str = "STALE";
const STALE_DETAIL: &str = "no health report received within ttl";
const DISABLED_REASON_CODE: &str = "DISABLED";
const DISABLED_DETAIL: &str = "disabled by deployment config";
const NS_PER_MS: u64 = 1_000_000;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ModuleHealthError {
    UnsupportedSchemaVersion { got: u32, expected: u32 },
    MissingModule,
    MissingModuleId,
}

impl fmt::Display for ModuleHealthError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSchemaVersion { got, expected } => {
                write!(
                    f,
                    "unsupported module health schema_version={got}, expected {expected}"
                )
            }
            Self::MissingModule => write!(f, "module health report missing module"),
            Self::MissingModuleId => write!(f, "module health report missing module_id"),
        }
    }
}

impl std::error::Error for ModuleHealthError {}

#[derive(Debug, Default)]
pub struct ModuleHealthStore {
    latest: BTreeMap<String, ModuleHealth>,
    events: VecDeque<ModuleHealthEvent>,
    snapshot_seq: u64,
    event_seq: u64,
}

impl ModuleHealthStore {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn ingest_report(
        &mut self,
        report: ModuleHealthReport,
        received_ts_ns: u64,
    ) -> Result<Option<ModuleHealthEvent>, ModuleHealthError> {
        if report.schema_version != MODULE_HEALTH_SCHEMA_VERSION {
            return Err(ModuleHealthError::UnsupportedSchemaVersion {
                got: report.schema_version,
                expected: MODULE_HEALTH_SCHEMA_VERSION,
            });
        }

        let mut module = report.module.ok_or(ModuleHealthError::MissingModule)?;
        if module.module_id.trim().is_empty() {
            return Err(ModuleHealthError::MissingModuleId);
        }

        let module_key = module_key_for(&module);
        module.module_key = module_key.clone();
        module.source = SOURCE_SELF_REPORTED.to_string();
        module.received_ts_ns = received_ts_ns;

        let previous = self.latest.get(&module_key).cloned();
        let event = previous
            .as_ref()
            .and_then(|previous| self.health_transition_event(previous, &module, received_ts_ns));

        self.latest.insert(module_key, module);

        if let Some(event) = event.clone() {
            self.push_event(event);
        }

        Ok(event)
    }

    pub fn synthesize_stale_if_expired(
        &mut self,
        module_key: &str,
        stale_health: u32,
        now_ns: u64,
    ) -> Option<ModuleHealthEvent> {
        let previous = self.latest.get(module_key).cloned()?;
        if previous.ttl_ms == 0 || !is_expired(&previous, now_ns) {
            return None;
        }
        if previous.source == SOURCE_VITALS_SYNTHESIZED_STALE
            && previous.reason_code == STALE_REASON_CODE
            && previous.health == stale_health
        {
            return None;
        }

        let mut current = previous.clone();
        current.health = stale_health;
        current.state = "stale".to_string();
        current.reason_code = STALE_REASON_CODE.to_string();
        current.detail = STALE_DETAIL.to_string();
        current.source = SOURCE_VITALS_SYNTHESIZED_STALE.to_string();
        current.received_ts_ns = now_ns;

        let event = self.health_transition_event(&previous, &current, now_ns);
        self.latest.insert(module_key.to_string(), current);

        if let Some(event) = event.clone() {
            self.push_event(event);
        }

        event
    }

    pub fn synthesize_expected_unavailable(
        &mut self,
        module_id: &str,
        provider_id: &str,
        health: u32,
        ttl_ms: u32,
        now_ns: u64,
    ) -> Option<ModuleHealthEvent> {
        self.upsert_synthesized(
            ModuleHealth {
                module_id: module_id.trim().to_string(),
                provider_id: provider_id.trim().to_string(),
                health,
                state: "stale".to_string(),
                reason_code: STALE_REASON_CODE.to_string(),
                detail: STALE_DETAIL.to_string(),
                source: SOURCE_VITALS_SYNTHESIZED_STALE.to_string(),
                ttl_ms,
                ..Default::default()
            },
            now_ns,
            true,
        )
    }

    pub fn synthesize_config_disabled(
        &mut self,
        module_id: &str,
        provider_id: &str,
        now_ns: u64,
    ) -> Option<ModuleHealthEvent> {
        self.upsert_synthesized(
            ModuleHealth {
                module_id: module_id.trim().to_string(),
                provider_id: provider_id.trim().to_string(),
                health: HEALTH_OK,
                state: "disabled".to_string(),
                reason_code: DISABLED_REASON_CODE.to_string(),
                detail: DISABLED_DETAIL.to_string(),
                source: SOURCE_CONFIG_DISABLED.to_string(),
                ttl_ms: 0,
                ..Default::default()
            },
            now_ns,
            false,
        )
    }

    pub fn snapshot(&mut self, ts_ns: u64) -> ModuleHealthSnapshot {
        self.snapshot_seq += 1;
        ModuleHealthSnapshot {
            schema_version: MODULE_HEALTH_SCHEMA_VERSION,
            ts_ns,
            seq: self.snapshot_seq,
            modules: self.latest.values().cloned().collect(),
        }
    }

    pub fn latest(&self, module_key: &str) -> Option<&ModuleHealth> {
        self.latest.get(module_key)
    }

    pub fn events(&self) -> impl Iterator<Item = &ModuleHealthEvent> {
        self.events.iter()
    }

    fn health_transition_event(
        &mut self,
        previous: &ModuleHealth,
        current: &ModuleHealth,
        ts_ns: u64,
    ) -> Option<ModuleHealthEvent> {
        if previous.health == current.health {
            return None;
        }

        self.event_seq += 1;
        Some(ModuleHealthEvent {
            ts_ns,
            seq: self.event_seq,
            module_key: current.module_key.clone(),
            previous_health: previous.health,
            current_health: current.health,
            reason_code: current.reason_code.clone(),
            detail: current.detail.clone(),
            source: current.source.clone(),
        })
    }

    fn push_event(&mut self, event: ModuleHealthEvent) {
        if self.events.len() == MAX_EVENTS {
            self.events.pop_front();
        }
        self.events.push_back(event);
    }

    fn upsert_synthesized(
        &mut self,
        mut current: ModuleHealth,
        now_ns: u64,
        emit_initial_non_ok: bool,
    ) -> Option<ModuleHealthEvent> {
        let module_key = module_key_for(&current);
        current.module_key = module_key.clone();
        current.received_ts_ns = now_ns;

        let previous = self.latest.get(&module_key).cloned();
        let event = match previous.as_ref() {
            Some(previous) => self.health_transition_event(previous, &current, now_ns),
            None if emit_initial_non_ok && current.health != HEALTH_OK => {
                self.event_seq += 1;
                Some(ModuleHealthEvent {
                    ts_ns: now_ns,
                    seq: self.event_seq,
                    module_key: current.module_key.clone(),
                    previous_health: HEALTH_OK,
                    current_health: current.health,
                    reason_code: current.reason_code.clone(),
                    detail: current.detail.clone(),
                    source: current.source.clone(),
                })
            }
            None => None,
        };

        self.latest.insert(module_key, current);

        if let Some(event) = event.clone() {
            self.push_event(event);
        }

        event
    }
}

fn is_expired(module: &ModuleHealth, now_ns: u64) -> bool {
    let ttl_ns = u64::from(module.ttl_ms).saturating_mul(NS_PER_MS);
    module.received_ts_ns.saturating_add(ttl_ns) <= now_ns
}

fn module_key_for(module: &ModuleHealth) -> String {
    module_key_from_parts(&module.module_id, &module.provider_id)
}

fn module_key_from_parts(module_id: &str, provider_id: &str) -> String {
    let provider_id = provider_id.trim();
    if !provider_id.is_empty() {
        provider_id.to_string()
    } else {
        module_id.trim().to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn report(
        module_id: &str,
        provider_id: &str,
        health: u32,
        reason_code: &str,
    ) -> ModuleHealthReport {
        ModuleHealthReport {
            schema_version: MODULE_HEALTH_SCHEMA_VERSION,
            module: Some(ModuleHealth {
                module_key: String::new(),
                module_id: module_id.to_string(),
                provider_id: provider_id.to_string(),
                health,
                state: if health == HEALTH_OK {
                    "active".to_string()
                } else {
                    "degraded".to_string()
                },
                reason_code: reason_code.to_string(),
                detail: reason_code.to_string(),
                source: String::new(),
                received_ts_ns: 0,
                ttl_ms: 5000,
            }),
        }
    }

    #[test]
    fn ingest_report_normalizes_vitals_owned_fields() {
        let mut store = ModuleHealthStore::new();
        let event = store
            .ingest_report(report("executor", "executor", HEALTH_OK, "OK"), 100)
            .unwrap();
        assert!(event.is_none());

        let module = store.latest("executor").unwrap();
        assert_eq!(module.module_key, "executor");
        assert_eq!(module.source, SOURCE_SELF_REPORTED);
        assert_eq!(module.received_ts_ns, 100);
    }

    #[test]
    fn provider_id_is_preferred_for_module_key() {
        let mut store = ModuleHealthStore::new();
        store
            .ingest_report(report("pilot", "pilot-main", HEALTH_OK, "OK"), 100)
            .unwrap();

        assert!(store.latest("pilot").is_none());
        assert!(store.latest("pilot-main").is_some());
    }

    #[test]
    fn module_id_is_module_key_when_provider_id_is_empty() {
        let mut store = ModuleHealthStore::new();
        store
            .ingest_report(report("pilot", "", HEALTH_OK, "OK"), 100)
            .unwrap();

        assert!(store.latest("pilot").is_some());
    }

    #[test]
    fn health_transition_emits_event() {
        let mut store = ModuleHealthStore::new();
        store
            .ingest_report(report("pilot", "pilot", HEALTH_OK, "OK"), 100)
            .unwrap();
        let event = store
            .ingest_report(report("pilot", "pilot", HEALTH_WARN, "VLM_RETRYING"), 200)
            .unwrap()
            .expect("health transition event");

        assert_eq!(event.seq, 1);
        assert_eq!(event.ts_ns, 200);
        assert_eq!(event.module_key, "pilot");
        assert_eq!(event.previous_health, HEALTH_OK);
        assert_eq!(event.current_health, HEALTH_WARN);
        assert_eq!(event.reason_code, "VLM_RETRYING");
        assert_eq!(event.source, SOURCE_SELF_REPORTED);
        assert_eq!(store.events().count(), 1);
    }

    #[test]
    fn same_health_updates_snapshot_without_event() {
        let mut store = ModuleHealthStore::new();
        store
            .ingest_report(report("executor", "executor", HEALTH_OK, "OK"), 100)
            .unwrap();
        let event = store
            .ingest_report(report("executor", "executor", HEALTH_OK, "OK"), 200)
            .unwrap();

        assert!(event.is_none());
        assert_eq!(store.latest("executor").unwrap().received_ts_ns, 200);
        assert_eq!(store.events().count(), 0);
    }

    #[test]
    fn stale_is_synthesized_after_ttl_expires() {
        let mut store = ModuleHealthStore::new();
        store
            .ingest_report(report("executor", "executor", HEALTH_OK, "OK"), 100)
            .unwrap();

        assert!(
            store
                .synthesize_stale_if_expired("executor", HEALTH_ERROR, 100 + 4_999_000_000)
                .is_none()
        );
        assert_eq!(store.latest("executor").unwrap().health, HEALTH_OK);

        let event = store
            .synthesize_stale_if_expired("executor", HEALTH_ERROR, 100 + 5_000_000_000)
            .expect("stale event");

        let module = store.latest("executor").unwrap();
        assert_eq!(module.health, HEALTH_ERROR);
        assert_eq!(module.state, "stale");
        assert_eq!(module.reason_code, STALE_REASON_CODE);
        assert_eq!(module.detail, STALE_DETAIL);
        assert_eq!(module.source, SOURCE_VITALS_SYNTHESIZED_STALE);
        assert_eq!(module.received_ts_ns, 100 + 5_000_000_000);
        assert_eq!(event.previous_health, HEALTH_OK);
        assert_eq!(event.current_health, HEALTH_ERROR);
        assert_eq!(event.reason_code, STALE_REASON_CODE);
        assert_eq!(event.source, SOURCE_VITALS_SYNTHESIZED_STALE);
        assert_eq!(store.events().count(), 1);
    }

    #[test]
    fn stale_synthesis_is_not_repeated_until_self_report_recovers() {
        let mut store = ModuleHealthStore::new();
        store
            .ingest_report(report("pilot", "pilot", HEALTH_OK, "OK"), 100)
            .unwrap();
        assert!(
            store
                .synthesize_stale_if_expired("pilot", HEALTH_ERROR, 100 + 5_000_000_000)
                .is_some()
        );
        assert!(
            store
                .synthesize_stale_if_expired("pilot", HEALTH_ERROR, 100 + 20_000_000_000)
                .is_none()
        );

        let recovered = store
            .ingest_report(
                report("pilot", "pilot", HEALTH_OK, "OK"),
                100 + 21_000_000_000,
            )
            .unwrap()
            .expect("recovery event");
        assert_eq!(recovered.previous_health, HEALTH_ERROR);
        assert_eq!(recovered.current_health, HEALTH_OK);
        assert_eq!(store.latest("pilot").unwrap().source, SOURCE_SELF_REPORTED);
        assert_eq!(store.events().count(), 2);
    }

    #[test]
    fn expected_unavailable_creates_initial_non_ok_event() {
        let mut store = ModuleHealthStore::new();
        let event = store
            .synthesize_expected_unavailable("executor", "executor", HEALTH_ERROR, 5000, 500)
            .expect("initial missing event");

        let module = store.latest("executor").unwrap();
        assert_eq!(module.health, HEALTH_ERROR);
        assert_eq!(module.state, "stale");
        assert_eq!(module.reason_code, STALE_REASON_CODE);
        assert_eq!(module.source, SOURCE_VITALS_SYNTHESIZED_STALE);
        assert_eq!(event.previous_health, HEALTH_OK);
        assert_eq!(event.current_health, HEALTH_ERROR);
        assert_eq!(store.events().count(), 1);

        assert!(
            store
                .synthesize_expected_unavailable("executor", "executor", HEALTH_ERROR, 5000, 800)
                .is_none()
        );
        assert_eq!(store.events().count(), 1);
    }

    #[test]
    fn config_disabled_is_stored_without_event() {
        let mut store = ModuleHealthStore::new();
        let event = store.synthesize_config_disabled("speech", "", 100);
        assert!(event.is_none());

        let module = store.latest("speech").unwrap();
        assert_eq!(module.health, HEALTH_OK);
        assert_eq!(module.state, "disabled");
        assert_eq!(module.reason_code, DISABLED_REASON_CODE);
        assert_eq!(module.detail, DISABLED_DETAIL);
        assert_eq!(module.source, SOURCE_CONFIG_DISABLED);
        assert_eq!(module.ttl_ms, 0);
        assert_eq!(store.events().count(), 0);
    }

    #[test]
    fn snapshot_is_deterministic_and_increments_seq() {
        let mut store = ModuleHealthStore::new();
        store
            .ingest_report(report("pilot", "pilot", HEALTH_OK, "OK"), 100)
            .unwrap();
        store
            .ingest_report(report("executor", "executor", HEALTH_OK, "OK"), 100)
            .unwrap();

        let first = store.snapshot(1000);
        let second = store.snapshot(2000);

        assert_eq!(first.schema_version, MODULE_HEALTH_SCHEMA_VERSION);
        assert_eq!(first.seq, 1);
        assert_eq!(second.seq, 2);
        assert_eq!(first.ts_ns, 1000);
        assert_eq!(first.modules.len(), 2);
        assert_eq!(first.modules[0].module_key, "executor");
        assert_eq!(first.modules[1].module_key, "pilot");
    }

    #[test]
    fn invalid_report_is_rejected() {
        let mut store = ModuleHealthStore::new();
        let err = store
            .ingest_report(
                ModuleHealthReport {
                    schema_version: 99,
                    module: None,
                },
                100,
            )
            .unwrap_err();

        assert_eq!(
            err,
            ModuleHealthError::UnsupportedSchemaVersion {
                got: 99,
                expected: MODULE_HEALTH_SCHEMA_VERSION,
            }
        );

        let err = store
            .ingest_report(
                ModuleHealthReport {
                    schema_version: MODULE_HEALTH_SCHEMA_VERSION,
                    module: None,
                },
                100,
            )
            .unwrap_err();
        assert_eq!(err, ModuleHealthError::MissingModule);

        let err = store
            .ingest_report(report("", "", HEALTH_OK, "OK"), 100)
            .unwrap_err();
        assert_eq!(err, ModuleHealthError::MissingModuleId);
    }
}
