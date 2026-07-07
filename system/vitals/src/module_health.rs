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
}

fn module_key_for(module: &ModuleHealth) -> String {
    let provider_id = module.provider_id.trim();
    if !provider_id.is_empty() {
        provider_id.to_string()
    } else {
        module.module_id.trim().to_string()
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
