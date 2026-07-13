// SPDX-License-Identifier: MulanPSL-2.0

//! Policy evaluation core for Robonix capability calls.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};
use serde_json::Value;
use thiserror::Error;

/// Effect applied when a rule matches.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Effect {
    Allow,
    Deny,
}

/// Local-time window. Weekdays use ISO values: Monday=1 through Sunday=7.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TimeWindow {
    pub days: Vec<u8>,
    pub start: String,
    pub end: String,
}

/// Conditions in one rule. Every non-empty field must match.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct Cond {
    pub contract: String,
    pub time: Vec<TimeWindow>,
    pub args: String,
    pub user: String,
}

/// One Sentinel policy rule.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Rule {
    pub id: String,
    pub effect: Effect,
    pub priority: i32,
    pub cond: Cond,
    pub reason: String,
}

/// Result of checking one capability call.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Decision {
    pub allow: bool,
    pub reason: String,
}

/// Invalid rule-set errors returned before rules become active.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum RuleError {
    #[error("rule ID must not be empty")]
    EmptyRuleId,
    #[error("duplicate rule ID '{0}'")]
    DuplicateRuleId(String),
    #[error("weekday must be between 1 and 7, got {0}")]
    InvalidWeekday(u8),
    #[error("invalid time '{0}', expected HH:MM")]
    InvalidTime(String),
    #[error("time window start and end must differ")]
    EmptyTimeWindow,
    #[error("invalid argument condition '{0}'")]
    InvalidArgumentCondition(String),
}

/// Ordered in-memory Sentinel rule set.
#[derive(Debug, Default)]
pub struct Sentinel {
    rules: Vec<Rule>,
}

impl Sentinel {
    /// Construct Sentinel with a validated initial rule set.
    pub fn new(rules: Vec<Rule>) -> Result<Self, RuleError> {
        validate_rules(&rules)?;
        Ok(Self { rules })
    }

    /// Return active rules in their configured order.
    pub fn list_rules(&self) -> &[Rule] {
        &self.rules
    }

    /// Atomically replace all active rules after validating every entry.
    pub fn set_rules(&mut self, rules: Vec<Rule>) -> Result<(), RuleError> {
        validate_rules(&rules)?;
        self.rules = rules;
        Ok(())
    }

    /// Check one capability call against all active rules.
    ///
    /// `weekday` uses ISO Monday=1 through Sunday=7 and `minute_of_day` is
    /// local time in the range 0..1440. Integration code will obtain these
    /// values from Chronos and user roles from Keystone. The highest-priority
    /// matching rule wins; equal priorities preserve configured order.
    pub fn check(
        &self,
        contract_id: &str,
        args_json: &Value,
        user_id: Option<&str>,
        roles: &[String],
        weekday: u8,
        minute_of_day: u16,
    ) -> Decision {
        let mut selected: Option<&Rule> = None;
        for rule in &self.rules {
            if rule_matches(
                rule,
                contract_id,
                args_json,
                user_id,
                roles,
                weekday,
                minute_of_day,
            ) && selected.is_none_or(|current| rule.priority > current.priority)
            {
                selected = Some(rule);
            }
        }
        let Some(rule) = selected else {
            return Decision {
                allow: true,
                reason: "no Sentinel rule matched".to_owned(),
            };
        };
        let allow = rule.effect == Effect::Allow;
        let reason = if rule.reason.trim().is_empty() {
            format!(
                "{} by Sentinel rule '{}'",
                if allow { "allowed" } else { "denied" },
                rule.id
            )
        } else {
            rule.reason.clone()
        };
        Decision { allow, reason }
    }
}

/// Validate a complete rule set without mutating active policy.
fn validate_rules(rules: &[Rule]) -> Result<(), RuleError> {
    let mut ids = BTreeSet::new();
    for rule in rules {
        if rule.id.trim().is_empty() {
            return Err(RuleError::EmptyRuleId);
        }
        if !ids.insert(rule.id.as_str()) {
            return Err(RuleError::DuplicateRuleId(rule.id.clone()));
        }
        for window in &rule.cond.time {
            validate_time_window(window)?;
        }
        if !rule.cond.args.trim().is_empty() {
            parse_argument_condition(&rule.cond.args)?;
        }
    }
    Ok(())
}

/// Validate weekday values and parse both time endpoints.
fn validate_time_window(window: &TimeWindow) -> Result<(), RuleError> {
    for day in &window.days {
        if !(1..=7).contains(day) {
            return Err(RuleError::InvalidWeekday(*day));
        }
    }
    let start = parse_time(&window.start)?;
    let end = parse_time(&window.end)?;
    if start == end {
        return Err(RuleError::EmptyTimeWindow);
    }
    Ok(())
}

/// Return whether every non-empty condition in a rule matches the call.
fn rule_matches(
    rule: &Rule,
    contract_id: &str,
    args_json: &Value,
    user_id: Option<&str>,
    roles: &[String],
    weekday: u8,
    minute_of_day: u16,
) -> bool {
    let cond = &rule.cond;
    (cond.contract.trim().is_empty() || wildcard_matches(&cond.contract, contract_id))
        && (cond.time.is_empty()
            || cond
                .time
                .iter()
                .any(|window| time_matches(window, weekday, minute_of_day)))
        && (cond.args.trim().is_empty() || argument_matches(&cond.args, args_json).unwrap_or(false))
        && (cond.user.trim().is_empty()
            || user_id == Some(cond.user.as_str())
            || roles.iter().any(|role| role == &cond.user))
}

/// Match `*` wildcards without treating any other character specially.
fn wildcard_matches(pattern: &str, value: &str) -> bool {
    let (mut pattern_index, mut value_index) = (0usize, 0usize);
    let (mut star_index, mut star_value_index) = (None, 0usize);
    let pattern = pattern.as_bytes();
    let value = value.as_bytes();
    while value_index < value.len() {
        if pattern_index < pattern.len() && pattern[pattern_index] == value[value_index] {
            pattern_index += 1;
            value_index += 1;
        } else if pattern_index < pattern.len() && pattern[pattern_index] == b'*' {
            star_index = Some(pattern_index);
            pattern_index += 1;
            star_value_index = value_index;
        } else if let Some(star) = star_index {
            pattern_index = star + 1;
            star_value_index += 1;
            value_index = star_value_index;
        } else {
            return false;
        }
    }
    while pattern_index < pattern.len() && pattern[pattern_index] == b'*' {
        pattern_index += 1;
    }
    pattern_index == pattern.len()
}

/// Match a local time window, treating listed days as start days.
///
/// For a Monday 22:00-08:00 window, Monday 23:00 and Tuesday 07:00 match.
/// An empty day list means every day. The integration boundary must supply a
/// valid ISO weekday and minute; invalid values simply do not match.
fn time_matches(window: &TimeWindow, weekday: u8, minute_of_day: u16) -> bool {
    if !(1..=7).contains(&weekday) || minute_of_day >= 24 * 60 {
        return false;
    }
    let (Ok(start), Ok(end)) = (parse_time(&window.start), parse_time(&window.end)) else {
        return false;
    };
    if start < end {
        day_matches(&window.days, weekday) && (start..end).contains(&minute_of_day)
    } else if minute_of_day >= start {
        day_matches(&window.days, weekday)
    } else if minute_of_day < end {
        day_matches(&window.days, previous_weekday(weekday))
    } else {
        false
    }
}

fn day_matches(days: &[u8], weekday: u8) -> bool {
    days.is_empty() || days.contains(&weekday)
}

fn previous_weekday(weekday: u8) -> u8 {
    if weekday == 1 { 7 } else { weekday - 1 }
}

/// Parse `HH:MM` into minutes since local midnight.
fn parse_time(value: &str) -> Result<u16, RuleError> {
    let Some((hour, minute)) = value.split_once(':') else {
        return Err(RuleError::InvalidTime(value.to_owned()));
    };
    if hour.len() != 2 || minute.len() != 2 {
        return Err(RuleError::InvalidTime(value.to_owned()));
    }
    let (Ok(hour), Ok(minute)) = (hour.parse::<u16>(), minute.parse::<u16>()) else {
        return Err(RuleError::InvalidTime(value.to_owned()));
    };
    if hour >= 24 || minute >= 60 {
        return Err(RuleError::InvalidTime(value.to_owned()));
    }
    Ok(hour * 60 + minute)
}

#[derive(Debug, Clone, Copy)]
enum NumericOperator {
    Equal,
    NotEqual,
    Greater,
    GreaterOrEqual,
    Less,
    LessOrEqual,
}

/// Parse a simple numeric condition: `<json.path> <operator> <number>`.
fn parse_argument_condition(expression: &str) -> Result<(&str, NumericOperator, f64), RuleError> {
    let parts: Vec<&str> = expression.split_whitespace().collect();
    if parts.len() != 3 || parts[0].is_empty() {
        return Err(RuleError::InvalidArgumentCondition(expression.to_owned()));
    }
    let operator = match parts[1] {
        "==" => NumericOperator::Equal,
        "!=" => NumericOperator::NotEqual,
        ">" => NumericOperator::Greater,
        ">=" => NumericOperator::GreaterOrEqual,
        "<" => NumericOperator::Less,
        "<=" => NumericOperator::LessOrEqual,
        _ => return Err(RuleError::InvalidArgumentCondition(expression.to_owned())),
    };
    let expected = parts[2]
        .parse::<f64>()
        .map_err(|_| RuleError::InvalidArgumentCondition(expression.to_owned()))?;
    if !expected.is_finite() {
        return Err(RuleError::InvalidArgumentCondition(expression.to_owned()));
    }
    Ok((parts[0], operator, expected))
}

/// Evaluate one validated numeric condition against a JSON object.
fn argument_matches(expression: &str, args_json: &Value) -> Result<bool, RuleError> {
    let (path, operator, expected) = parse_argument_condition(expression)?;
    let Some(actual) = json_path(args_json, path).and_then(Value::as_f64) else {
        return Ok(false);
    };
    Ok(match operator {
        NumericOperator::Equal => actual == expected,
        NumericOperator::NotEqual => actual != expected,
        NumericOperator::Greater => actual > expected,
        NumericOperator::GreaterOrEqual => actual >= expected,
        NumericOperator::Less => actual < expected,
        NumericOperator::LessOrEqual => actual <= expected,
    })
}

fn json_path<'a>(value: &'a Value, path: &str) -> Option<&'a Value> {
    path.split('.')
        .try_fold(value, |current, key| current.get(key))
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    fn rule(id: &str, effect: Effect, priority: i32, cond: Cond) -> Rule {
        Rule {
            id: id.to_owned(),
            effect,
            priority,
            cond,
            reason: format!("matched {id}"),
        }
    }

    #[test]
    fn defaults_to_allow_when_no_rule_matches() {
        let sentinel = Sentinel::default();

        let decision = sentinel.check(
            "robonix/primitive/chassis/move",
            &json!({}),
            None,
            &[],
            1,
            0,
        );

        assert!(decision.allow);
    }

    #[test]
    fn contract_wildcard_can_deny_chassis_calls() {
        let sentinel = Sentinel::new(vec![rule(
            "deny-chassis",
            Effect::Deny,
            10,
            Cond {
                contract: "robonix/primitive/chassis/*".to_owned(),
                ..Cond::default()
            },
        )])
        .unwrap();

        let decision = sentinel.check(
            "robonix/primitive/chassis/move",
            &json!({}),
            None,
            &[],
            1,
            0,
        );

        assert!(!decision.allow);
        assert_eq!(decision.reason, "matched deny-chassis");
    }

    #[test]
    fn highest_priority_matching_rule_wins() {
        let sentinel = Sentinel::new(vec![
            rule("general-deny", Effect::Deny, 10, Cond::default()),
            rule("operator-allow", Effect::Allow, 20, Cond::default()),
        ])
        .unwrap();

        let decision = sentinel.check("any", &json!({}), None, &[], 1, 0);

        assert!(decision.allow);
        assert_eq!(decision.reason, "matched operator-allow");
    }

    #[test]
    fn numeric_argument_condition_reads_nested_json() {
        let sentinel = Sentinel::new(vec![rule(
            "speed-limit",
            Effect::Deny,
            10,
            Cond {
                args: "motion.linear_x > 0.5".to_owned(),
                ..Cond::default()
            },
        )])
        .unwrap();

        assert!(
            sentinel
                .check(
                    "move",
                    &json!({"motion": {"linear_x": 0.4}}),
                    None,
                    &[],
                    1,
                    0,
                )
                .allow
        );
        assert!(
            !sentinel
                .check(
                    "move",
                    &json!({"motion": {"linear_x": 0.6}}),
                    None,
                    &[],
                    1,
                    0,
                )
                .allow
        );
    }

    #[test]
    fn user_condition_matches_user_id_or_role() {
        let sentinel = Sentinel::new(vec![rule(
            "admin-only",
            Effect::Allow,
            10,
            Cond {
                user: "admin".to_owned(),
                ..Cond::default()
            },
        )])
        .unwrap();
        let roles = vec!["admin".to_owned()];

        let role_decision = sentinel.check("shutdown", &json!({}), None, &roles, 1, 0);
        let user_decision = sentinel.check("shutdown", &json!({}), Some("admin"), &[], 1, 0);

        assert_eq!(role_decision.reason, "matched admin-only");
        assert_eq!(user_decision.reason, "matched admin-only");
    }

    #[test]
    fn cross_midnight_window_uses_the_start_day() {
        let sentinel = Sentinel::new(vec![rule(
            "monday-night",
            Effect::Deny,
            10,
            Cond {
                time: vec![TimeWindow {
                    days: vec![1],
                    start: "22:00".to_owned(),
                    end: "08:00".to_owned(),
                }],
                ..Cond::default()
            },
        )])
        .unwrap();

        assert!(
            !sentinel
                .check("move", &json!({}), None, &[], 1, 23 * 60)
                .allow
        );
        assert!(
            !sentinel
                .check("move", &json!({}), None, &[], 2, 7 * 60)
                .allow
        );
        assert!(
            sentinel
                .check("move", &json!({}), None, &[], 2, 9 * 60)
                .allow
        );
    }

    #[test]
    fn invalid_rule_replacement_leaves_existing_rules_active() {
        let mut sentinel =
            Sentinel::new(vec![rule("existing", Effect::Deny, 1, Cond::default())]).unwrap();
        let invalid = rule(
            "invalid",
            Effect::Allow,
            2,
            Cond {
                args: "linear_x much_faster 0.5".to_owned(),
                ..Cond::default()
            },
        );

        assert!(sentinel.set_rules(vec![invalid]).is_err());
        assert_eq!(sentinel.list_rules()[0].id, "existing");
    }
}
