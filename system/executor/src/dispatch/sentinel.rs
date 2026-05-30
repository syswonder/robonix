// SPDX-License-Identifier: MulanPSL-2.0
//
// dispatch/sentinel.rs — POC: in-executor safety gate for capability dispatch.
//
// This is a CONCEPT VALIDATION embedded in robonix-executor. The real sentinel
// will land as its own crate `robonix-sentinel` with a `Check` gRPC contract,
// discoverable via atlas, hot-reloadable, and consumable from pilot/liaison
// too. For the 2026-05-25 demo we only need to prove the dispatch-time
// intercept story end-to-end, so we keep it inline.
//
// Rule shape (YAML, one file referenced via `SENTINEL_RULES_FILE` env):
//
//     version: 1
//     rules:
//       - id: night_no_speak
//         contract_id: robonix/service/speech/speak
//         deny_between: ["22:00", "07:00"]   # local time, HH:MM, wraps overnight
//         reason: "安静时段,不允许语音播报"
//
// Behaviour:
//   - File missing / env not set → empty ruleset → every call Allow (fail-open).
//   - Rule matches contract_id AND current local time is inside deny_between
//     → CapabilityCallResult{success=false, error="sentinel:<id>: <reason>"}
//
// The dispatch path checks this before any other work (builtin / activate /
// MCP). Planner/TUI sees the structured error string and surfaces it to the
// user. We also log each deny so an `audit` view can tail the executor log.

use std::fs;
use std::sync::OnceLock;

use chrono::{Local, NaiveTime, Timelike};
use serde::Deserialize;

use crate::pb::pilot::CapabilityCall;

/// Outcome of a sentinel check. `Allow` is the happy path; `Deny` carries the
/// rule id, the human reason, and the configured deny window so callers
/// (chat + pilot's LLM) can understand WHEN the rule applies, not just
/// the rejection itself.
#[derive(Debug)]
pub enum Decision {
    Allow,
    Deny {
        rule_id: String,
        reason: String,
        /// Formatted "HH:MM–HH:MM" of the configured deny window.
        deny_window: String,
    },
}

#[derive(Debug, Deserialize)]
struct RulesFile {
    #[allow(dead_code)]
    version: Option<u32>,
    #[serde(default)]
    rules: Vec<RuleSpec>,
}

#[derive(Debug, Deserialize)]
struct RuleSpec {
    id: String,
    contract_id: String,
    /// `["22:00", "07:00"]` — if the right end is earlier than the left, the
    /// window wraps midnight.
    deny_between: Vec<String>,
    #[serde(default)]
    reason: String,
}

/// Parsed + validated rule. Compared every dispatch, so keep it cheap.
#[derive(Debug)]
struct Rule {
    id: String,
    contract_id: String,
    /// Minutes since midnight (0..1440).
    deny_start: u32,
    deny_end: u32,
    reason: String,
}

static RULES: OnceLock<Vec<Rule>> = OnceLock::new();

/// Load rules from `SENTINEL_RULES_FILE` (set by the deploy manifest). Idempotent
/// — only the first call populates the static. Called once from executor `main`.
pub fn init_from_env() {
    let path = match std::env::var("SENTINEL_RULES_FILE") {
        Ok(p) if !p.trim().is_empty() => p,
        _ => {
            let _ = RULES.set(Vec::new());
            log::info!(
                "[sentinel] no SENTINEL_RULES_FILE set — running with empty ruleset (allow-all)"
            );
            return;
        }
    };
    let rules = match load_file(&path) {
        Ok(r) => r,
        Err(e) => {
            log::warn!(
                "[sentinel] failed to load rules from '{path}': {e:#} — running with empty ruleset (allow-all)"
            );
            Vec::new()
        }
    };
    log::info!(
        "[sentinel] loaded {} rule(s) from {path}: {:?}",
        rules.len(),
        rules.iter().map(|r| &r.id).collect::<Vec<_>>()
    );
    let _ = RULES.set(rules);
}

fn load_file(path: &str) -> anyhow::Result<Vec<Rule>> {
    let body = fs::read_to_string(path).map_err(|e| anyhow::anyhow!("read {path}: {e}"))?;
    let parsed: RulesFile =
        serde_yaml::from_str(&body).map_err(|e| anyhow::anyhow!("parse YAML: {e}"))?;
    let mut out = Vec::with_capacity(parsed.rules.len());
    for spec in parsed.rules {
        if spec.deny_between.len() != 2 {
            anyhow::bail!("rule '{}': deny_between must be [start, end]", spec.id);
        }
        let start = parse_hhmm(&spec.deny_between[0])
            .map_err(|e| anyhow::anyhow!("rule '{}': bad start time: {e}", spec.id))?;
        let end = parse_hhmm(&spec.deny_between[1])
            .map_err(|e| anyhow::anyhow!("rule '{}': bad end time: {e}", spec.id))?;
        out.push(Rule {
            id: spec.id,
            contract_id: spec.contract_id,
            deny_start: start,
            deny_end: end,
            reason: spec.reason,
        });
    }
    Ok(out)
}

fn parse_hhmm(s: &str) -> anyhow::Result<u32> {
    let t = NaiveTime::parse_from_str(s.trim(), "%H:%M")
        .map_err(|e| anyhow::anyhow!("'{s}' is not HH:MM: {e}"))?;
    Ok(t.hour() * 60 + t.minute())
}

/// Check a single dispatch. Called from `dispatch::dispatch` before any other
/// work. The hot path is one slice scan + one comparison; rule counts in the
/// demo are O(1) so we don't index.
pub fn check(call: &CapabilityCall) -> Decision {
    let rules = match RULES.get() {
        Some(r) => r,
        None => return Decision::Allow, // init_from_env never called
    };
    if rules.is_empty() {
        return Decision::Allow;
    }
    let now = Local::now().time();
    let now_min = now.hour() * 60 + now.minute();
    for rule in rules {
        if rule.contract_id != call.contract_id {
            continue;
        }
        if in_window(now_min, rule.deny_start, rule.deny_end) {
            log::warn!(
                "[sentinel] DENY rule='{}' contract='{}' provider='{}' call_id='{}' reason='{}'",
                rule.id,
                call.contract_id,
                call.provider_id,
                call.call_id,
                rule.reason
            );
            return Decision::Deny {
                rule_id: rule.id.clone(),
                reason: rule.reason.clone(),
                deny_window: format!(
                    "{:02}:{:02}–{:02}:{:02}",
                    rule.deny_start / 60,
                    rule.deny_start % 60,
                    rule.deny_end / 60,
                    rule.deny_end % 60,
                ),
            };
        }
    }
    Decision::Allow
}

/// `[start, end)` window on a 24-hour clock. When `end < start` the window
/// wraps midnight (e.g. 22:00–07:00 spans the night).
fn in_window(now: u32, start: u32, end: u32) -> bool {
    if start == end {
        return false; // empty window
    }
    if start < end {
        now >= start && now < end
    } else {
        now >= start || now < end
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn window_non_wrap() {
        assert!(!in_window(0, 60, 120));
        assert!(in_window(60, 60, 120));
        assert!(in_window(90, 60, 120));
        assert!(!in_window(120, 60, 120));
    }

    #[test]
    fn window_wraps_midnight() {
        // 22:00 (1320) → 07:00 (420)
        assert!(in_window(1320, 1320, 420));
        assert!(in_window(1400, 1320, 420));
        assert!(in_window(0, 1320, 420));
        assert!(in_window(419, 1320, 420));
        assert!(!in_window(420, 1320, 420));
        assert!(!in_window(720, 1320, 420));
    }

    #[test]
    fn window_empty() {
        assert!(!in_window(100, 200, 200));
    }

    #[test]
    fn parses_hhmm() {
        assert_eq!(parse_hhmm("00:00").unwrap(), 0);
        assert_eq!(parse_hhmm("07:30").unwrap(), 450);
        assert_eq!(parse_hhmm("23:59").unwrap(), 23 * 60 + 59);
        assert!(parse_hhmm("24:00").is_err());
        assert!(parse_hhmm("garbage").is_err());
    }
}
