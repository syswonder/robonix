// SPDX-License-Identifier: MulanPSL-2.0

//! Transport-independent policy evaluation for capability calls.

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

/// Local-time window. Weekdays use ISO Monday=1 through Sunday=7.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TimeWindow {
    #[serde(default)]
    pub days: Vec<u8>,
    pub start: String,
    pub end: String,
}

/// The exact JSON type expected at a predicate path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ValueKind {
    Boolean,
    String,
    Integer,
    Float,
}

/// Operators supported by the deliberately small v0.1 predicate language.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Operator {
    Eq,
    Ne,
    Lt,
    Le,
    Gt,
    Ge,
    Range,
}

const fn default_true() -> bool {
    true
}

/// One typed comparison against an RFC 6901 JSON Pointer.
///
/// `range` uses `min` and `max`; every other operator uses `value`. Boolean and
/// string predicates only accept `eq` and `ne`. No type coercion is performed.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Predicate {
    pub path: String,
    pub kind: ValueKind,
    pub operator: Operator,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub value: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max: Option<Value>,
    #[serde(default = "default_true")]
    pub min_inclusive: bool,
    #[serde(default = "default_true")]
    pub max_inclusive: bool,
}

/// Conditions in one rule. Every configured category and predicate must match.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(default)]
pub struct Conditions {
    /// Capability contract pattern. `*` is the only wildcard.
    pub contract: String,
    /// Exact Keystone user IDs. Empty means any user.
    pub users: Vec<String>,
    /// At least one exact Keystone role must match. Empty means any role.
    pub roles: Vec<String>,
    /// Any one time window may match. Empty means any time.
    pub time: Vec<TimeWindow>,
    /// Predicates over Executor-owned call arguments.
    pub args: Vec<Predicate>,
    /// Predicates over trusted robot snapshots collected by Sentinel.
    pub context: Vec<Predicate>,
}

/// One robot-wide Sentinel policy rule.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Rule {
    pub id: String,
    pub effect: Effect,
    #[serde(default)]
    pub priority: i32,
    #[serde(default)]
    pub conditions: Conditions,
    #[serde(default)]
    pub reason: String,
}

/// Trusted inputs to a policy decision.
#[derive(Debug, Clone, PartialEq)]
pub struct EvaluationContext {
    /// Distinguishes a known anonymous/role-less identity from unavailable data.
    pub identity_known: bool,
    pub user_id: Option<String>,
    pub roles: Vec<String>,
    /// Root object populated by trusted Sentinel state adapters only.
    pub state: Value,
    pub weekday: u8,
    pub minute_of_day: u16,
}

impl Default for EvaluationContext {
    fn default() -> Self {
        Self {
            identity_known: false,
            user_id: None,
            roles: Vec::new(),
            state: Value::Object(Default::default()),
            weekday: 0,
            minute_of_day: 0,
        }
    }
}

/// Result of checking one capability call.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Decision {
    pub allow: bool,
    pub indeterminate: bool,
    pub rule_id: String,
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
    #[error("invalid predicate path '{0}', expected an RFC 6901 JSON Pointer")]
    InvalidPredicatePath(String),
    #[error("operator '{operator:?}' is invalid for {kind:?} predicate '{path}'")]
    InvalidPredicateOperator {
        path: String,
        kind: ValueKind,
        operator: Operator,
    },
    #[error("predicate '{path}' has an invalid {field}; expected {kind:?}")]
    InvalidPredicateValue {
        path: String,
        field: &'static str,
        kind: ValueKind,
    },
    #[error("predicate '{0}' must use value or min/max exclusively")]
    InvalidPredicateShape(String),
    #[error("predicate '{0}' has an empty or reversed range")]
    InvalidPredicateRange(String),
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum MatchOutcome {
    Match,
    NoMatch,
    Indeterminate(String),
}

/// Ordered in-memory Sentinel rule set.
#[derive(Debug, Clone, Default)]
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

    /// Return trusted snapshot roots referenced by active rules.
    pub fn required_context_roots(&self) -> BTreeSet<String> {
        self.rules
            .iter()
            .flat_map(|rule| &rule.conditions.context)
            .filter_map(|predicate| {
                predicate
                    .path
                    .strip_prefix('/')
                    .and_then(|rest| rest.split('/').next())
                    .filter(|root| !root.is_empty())
                    .map(str::to_owned)
            })
            .collect()
    }

    /// Evaluate a call. Highest priority wins; configured order breaks ties.
    ///
    /// Sentinel is an overlay policy and therefore allows calls for which no
    /// rule applies. A candidate rule whose trusted inputs are unavailable is
    /// fail-closed and produces an indeterminate denial.
    pub fn check(
        &self,
        contract_id: &str,
        args_json: &Value,
        context: &EvaluationContext,
    ) -> Decision {
        let selected = self
            .rules
            .iter()
            .filter_map(
                |rule| match rule_matches(rule, contract_id, args_json, context) {
                    MatchOutcome::NoMatch => None,
                    outcome => Some((rule, outcome)),
                },
            )
            .fold(
                None,
                |selected: Option<(&Rule, MatchOutcome)>, candidate| match selected {
                    Some((current, _)) if current.priority >= candidate.0.priority => selected,
                    _ => Some(candidate),
                },
            );

        let Some((rule, outcome)) = selected else {
            return Decision {
                allow: true,
                indeterminate: false,
                rule_id: String::new(),
                reason: "no Sentinel rule matched".to_owned(),
            };
        };
        if let MatchOutcome::Indeterminate(reason) = outcome {
            return Decision {
                allow: false,
                indeterminate: true,
                rule_id: rule.id.clone(),
                reason: format!(
                    "Sentinel rule '{}' could not be evaluated: {reason}",
                    rule.id
                ),
            };
        }

        let allow = rule.effect == Effect::Allow;
        Decision {
            allow,
            indeterminate: false,
            rule_id: rule.id.clone(),
            reason: if rule.reason.trim().is_empty() {
                format!(
                    "{} by Sentinel rule '{}'",
                    if allow { "allowed" } else { "denied" },
                    rule.id
                )
            } else {
                rule.reason.clone()
            },
        }
    }
}

/// Validate all rules before a replacement becomes visible to dispatch.
fn validate_rules(rules: &[Rule]) -> Result<(), RuleError> {
    let mut ids = BTreeSet::new();
    for rule in rules {
        if rule.id.trim().is_empty() {
            return Err(RuleError::EmptyRuleId);
        }
        if !ids.insert(rule.id.as_str()) {
            return Err(RuleError::DuplicateRuleId(rule.id.clone()));
        }
        for window in &rule.conditions.time {
            validate_time_window(window)?;
        }
        for predicate in rule.conditions.args.iter().chain(&rule.conditions.context) {
            validate_predicate(predicate)?;
        }
    }
    Ok(())
}

fn validate_predicate(predicate: &Predicate) -> Result<(), RuleError> {
    if predicate.path.is_empty()
        || !predicate.path.starts_with('/')
        || predicate.path.contains("//")
    {
        return Err(RuleError::InvalidPredicatePath(predicate.path.clone()));
    }
    if matches!(predicate.kind, ValueKind::Boolean | ValueKind::String)
        && !matches!(predicate.operator, Operator::Eq | Operator::Ne)
    {
        return Err(RuleError::InvalidPredicateOperator {
            path: predicate.path.clone(),
            kind: predicate.kind,
            operator: predicate.operator,
        });
    }

    if predicate.operator == Operator::Range {
        if predicate.value.is_some() || predicate.min.is_none() || predicate.max.is_none() {
            return Err(RuleError::InvalidPredicateShape(predicate.path.clone()));
        }
        validate_typed_value(predicate, predicate.min.as_ref(), "min")?;
        validate_typed_value(predicate, predicate.max.as_ref(), "max")?;
        if !range_is_nonempty(predicate) {
            return Err(RuleError::InvalidPredicateRange(predicate.path.clone()));
        }
    } else {
        if predicate.value.is_none() || predicate.min.is_some() || predicate.max.is_some() {
            return Err(RuleError::InvalidPredicateShape(predicate.path.clone()));
        }
        validate_typed_value(predicate, predicate.value.as_ref(), "value")?;
    }
    Ok(())
}

fn validate_typed_value(
    predicate: &Predicate,
    value: Option<&Value>,
    field: &'static str,
) -> Result<(), RuleError> {
    if value.is_some_and(|value| expected_matches_kind(value, predicate.kind)) {
        Ok(())
    } else {
        Err(RuleError::InvalidPredicateValue {
            path: predicate.path.clone(),
            field,
            kind: predicate.kind,
        })
    }
}

fn expected_matches_kind(value: &Value, kind: ValueKind) -> bool {
    match kind {
        // `kind` supplies the numeric semantic type. JSON writers are allowed
        // to serialize an integral float bound as `1` rather than `1.0`.
        ValueKind::Float => value.is_number(),
        _ => value_matches_kind(value, kind),
    }
}

fn value_matches_kind(value: &Value, kind: ValueKind) -> bool {
    match kind {
        ValueKind::Boolean => value.is_boolean(),
        ValueKind::String => value.is_string(),
        ValueKind::Integer => value.as_i64().is_some(),
        ValueKind::Float => {
            value.as_f64().is_some() && value.as_i64().is_none() && value.as_u64().is_none()
        }
    }
}

fn range_is_nonempty(predicate: &Predicate) -> bool {
    let order = match predicate.kind {
        ValueKind::Integer => predicate
            .min
            .as_ref()
            .and_then(Value::as_i64)
            .zip(predicate.max.as_ref().and_then(Value::as_i64))
            .map(|(min, max)| min.partial_cmp(&max)),
        ValueKind::Float => predicate
            .min
            .as_ref()
            .and_then(Value::as_f64)
            .zip(predicate.max.as_ref().and_then(Value::as_f64))
            .map(|(min, max)| min.partial_cmp(&max)),
        _ => None,
    }
    .flatten();
    match order {
        Some(std::cmp::Ordering::Less) => true,
        Some(std::cmp::Ordering::Equal) => predicate.min_inclusive && predicate.max_inclusive,
        _ => false,
    }
}

/// Return whether every configured condition matches the current call.
fn rule_matches(
    rule: &Rule,
    contract_id: &str,
    args_json: &Value,
    context: &EvaluationContext,
) -> MatchOutcome {
    let conditions = &rule.conditions;
    let mut outcomes = vec![if conditions.contract.trim().is_empty()
        || wildcard_matches(&conditions.contract, contract_id)
    {
        MatchOutcome::Match
    } else {
        MatchOutcome::NoMatch
    }];

    outcomes.push(identity_matches(conditions, context));
    outcomes.push(
        if conditions.time.is_empty()
            || conditions
                .time
                .iter()
                .any(|window| time_matches(window, context.weekday, context.minute_of_day))
        {
            MatchOutcome::Match
        } else {
            MatchOutcome::NoMatch
        },
    );
    outcomes.extend(
        conditions
            .args
            .iter()
            .map(|predicate| predicate_matches(predicate, args_json, "argument")),
    );
    outcomes.extend(
        conditions
            .context
            .iter()
            .map(|predicate| predicate_matches(predicate, &context.state, "context")),
    );

    combine_outcomes(outcomes)
}

fn identity_matches(conditions: &Conditions, context: &EvaluationContext) -> MatchOutcome {
    if conditions.users.is_empty() && conditions.roles.is_empty() {
        return MatchOutcome::Match;
    }
    if !context.identity_known {
        return MatchOutcome::Indeterminate("Keystone identity is unavailable".to_owned());
    }
    if !conditions.users.is_empty()
        && !context
            .user_id
            .as_ref()
            .is_some_and(|user| conditions.users.contains(user))
    {
        return MatchOutcome::NoMatch;
    }
    if !conditions.roles.is_empty()
        && !context
            .roles
            .iter()
            .any(|role| conditions.roles.contains(role))
    {
        return MatchOutcome::NoMatch;
    }
    MatchOutcome::Match
}

fn combine_outcomes(outcomes: Vec<MatchOutcome>) -> MatchOutcome {
    let mut indeterminate = None;
    for outcome in outcomes {
        match outcome {
            MatchOutcome::NoMatch => return MatchOutcome::NoMatch,
            MatchOutcome::Indeterminate(reason) if indeterminate.is_none() => {
                indeterminate = Some(reason);
            }
            MatchOutcome::Match | MatchOutcome::Indeterminate(_) => {}
        }
    }
    indeterminate.map_or(MatchOutcome::Match, MatchOutcome::Indeterminate)
}

fn predicate_matches(predicate: &Predicate, root: &Value, source: &str) -> MatchOutcome {
    let Some(actual) = root.pointer(&predicate.path) else {
        return MatchOutcome::Indeterminate(format!(
            "{source} path '{}' is unavailable",
            predicate.path
        ));
    };
    if !value_matches_kind(actual, predicate.kind) {
        return MatchOutcome::Indeterminate(format!(
            "{source} path '{}' is not a {:?}",
            predicate.path, predicate.kind
        ));
    }
    if compare_predicate(predicate, actual) {
        MatchOutcome::Match
    } else {
        MatchOutcome::NoMatch
    }
}

fn compare_predicate(predicate: &Predicate, actual: &Value) -> bool {
    if predicate.operator == Operator::Range {
        return match predicate.kind {
            ValueKind::Integer => in_range(
                actual.as_i64().unwrap(),
                predicate.min.as_ref().and_then(Value::as_i64).unwrap(),
                predicate.max.as_ref().and_then(Value::as_i64).unwrap(),
                predicate.min_inclusive,
                predicate.max_inclusive,
            ),
            ValueKind::Float => in_range(
                actual.as_f64().unwrap(),
                predicate.min.as_ref().and_then(Value::as_f64).unwrap(),
                predicate.max.as_ref().and_then(Value::as_f64).unwrap(),
                predicate.min_inclusive,
                predicate.max_inclusive,
            ),
            _ => false,
        };
    }

    let expected = predicate.value.as_ref().unwrap();
    match predicate.kind {
        ValueKind::Boolean => compare_ordered(
            actual.as_bool().unwrap(),
            expected.as_bool().unwrap(),
            predicate.operator,
        ),
        ValueKind::String => compare_ordered(
            actual.as_str().unwrap(),
            expected.as_str().unwrap(),
            predicate.operator,
        ),
        ValueKind::Integer => compare_ordered(
            actual.as_i64().unwrap(),
            expected.as_i64().unwrap(),
            predicate.operator,
        ),
        ValueKind::Float => compare_ordered(
            actual.as_f64().unwrap(),
            expected.as_f64().unwrap(),
            predicate.operator,
        ),
    }
}

fn compare_ordered<T: PartialOrd + PartialEq>(actual: T, expected: T, operator: Operator) -> bool {
    match operator {
        Operator::Eq => actual == expected,
        Operator::Ne => actual != expected,
        Operator::Lt => actual < expected,
        Operator::Le => actual <= expected,
        Operator::Gt => actual > expected,
        Operator::Ge => actual >= expected,
        Operator::Range => false,
    }
}

fn in_range<T: PartialOrd>(
    actual: T,
    min: T,
    max: T,
    min_inclusive: bool,
    max_inclusive: bool,
) -> bool {
    (if min_inclusive {
        actual >= min
    } else {
        actual > min
    }) && (if max_inclusive {
        actual <= max
    } else {
        actual < max
    })
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

/// Match a local time window, including windows that cross midnight.
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
        day_matches(&window.days, if weekday == 1 { 7 } else { weekday - 1 })
    } else {
        false
    }
}

fn day_matches(days: &[u8], weekday: u8) -> bool {
    days.is_empty() || days.contains(&weekday)
}

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

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    fn rule(id: &str, effect: Effect, priority: i32, conditions: Conditions) -> Rule {
        Rule {
            id: id.to_owned(),
            effect,
            priority,
            conditions,
            reason: format!("matched {id}"),
        }
    }

    fn predicate(path: &str, kind: ValueKind, operator: Operator, value: Value) -> Predicate {
        Predicate {
            path: path.to_owned(),
            kind,
            operator,
            value: Some(value),
            min: None,
            max: None,
            min_inclusive: true,
            max_inclusive: true,
        }
    }

    fn range(path: &str, kind: ValueKind, min: Value, max: Value) -> Predicate {
        Predicate {
            path: path.to_owned(),
            kind,
            operator: Operator::Range,
            value: None,
            min: Some(min),
            max: Some(max),
            min_inclusive: true,
            max_inclusive: true,
        }
    }

    fn known_context(state: Value) -> EvaluationContext {
        EvaluationContext {
            identity_known: true,
            user_id: Some("u1".to_owned()),
            roles: vec!["user".to_owned()],
            state,
            weekday: 1,
            minute_of_day: 12 * 60,
        }
    }

    #[test]
    fn contract_and_role_rule_denies_matching_call() {
        let sentinel = Sentinel::new(vec![rule(
            "deny-guest-motion",
            Effect::Deny,
            10,
            Conditions {
                contract: "robonix/*/navigation/*".to_owned(),
                roles: vec!["guest".to_owned()],
                ..Conditions::default()
            },
        )])
        .unwrap();
        let mut context = known_context(json!({}));
        context.roles = vec!["guest".to_owned()];

        let decision = sentinel.check("robonix/service/navigation/navigate", &json!({}), &context);

        assert!(!decision.allow);
        assert!(!decision.indeterminate);
        assert_eq!(decision.rule_id, "deny-guest-motion");
    }

    #[test]
    fn boolean_false_is_a_real_condition_not_missing_state() {
        let sentinel = Sentinel::new(vec![rule(
            "deny-when-motion-disabled",
            Effect::Deny,
            10,
            Conditions {
                context: vec![predicate(
                    "/soma/safety/motion_allowed",
                    ValueKind::Boolean,
                    Operator::Eq,
                    json!(false),
                )],
                ..Conditions::default()
            },
        )])
        .unwrap();

        let decision = sentinel.check(
            "navigate",
            &json!({}),
            &known_context(json!({"soma":{"safety":{"motion_allowed":false}}})),
        );

        assert!(!decision.allow);
        assert!(!decision.indeterminate);
    }

    #[test]
    fn integer_and_float_ranges_are_strictly_typed() {
        let sentinel = Sentinel::new(vec![rule(
            "deny-risk-envelope",
            Effect::Deny,
            10,
            Conditions {
                args: vec![range(
                    "/waypoints",
                    ValueKind::Integer,
                    json!(33),
                    json!(64),
                )],
                context: vec![range(
                    "/soma/grippers/left/position_m",
                    ValueKind::Float,
                    json!(0.0),
                    json!(0.01),
                )],
                ..Conditions::default()
            },
        )])
        .unwrap();
        let context = known_context(json!({
            "soma":{"grippers":{"left":{"position_m":0.005}}}
        }));

        assert!(
            !sentinel
                .check("trajectory", &json!({"waypoints":40}), &context)
                .allow
        );
        assert!(
            sentinel
                .check("trajectory", &json!({"waypoints":10}), &context)
                .allow
        );
    }

    #[test]
    fn explicit_float_kind_accepts_integral_json_bounds_from_web_clients() {
        let sentinel = Sentinel::new(vec![rule(
            "float-range",
            Effect::Deny,
            10,
            Conditions {
                context: vec![range("/temperature", ValueKind::Float, json!(0), json!(1))],
                ..Conditions::default()
            },
        )])
        .unwrap();

        assert!(
            !sentinel
                .check(
                    "thermal",
                    &json!({}),
                    &known_context(json!({"temperature":0.5})),
                )
                .allow
        );
        assert!(
            sentinel
                .check(
                    "thermal",
                    &json!({}),
                    &known_context(json!({"temperature":0})),
                )
                .indeterminate
        );
    }

    #[test]
    fn missing_or_wrongly_typed_candidate_context_fails_closed() {
        let sentinel = Sentinel::new(vec![rule(
            "deny-closed-gripper-operation",
            Effect::Deny,
            10,
            Conditions {
                contract: "gripper/*".to_owned(),
                context: vec![range(
                    "/soma/grippers/left/position_m",
                    ValueKind::Float,
                    json!(0.0),
                    json!(0.01),
                )],
                ..Conditions::default()
            },
        )])
        .unwrap();

        for state in [
            json!({}),
            json!({"soma":{"grippers":{"left":{"position_m":"closed"}}}}),
        ] {
            let decision = sentinel.check("gripper/move", &json!({}), &known_context(state));
            assert!(!decision.allow);
            assert!(decision.indeterminate);
        }
        assert!(
            sentinel
                .check("navigation/move", &json!({}), &known_context(json!({})))
                .allow
        );
    }

    #[test]
    fn higher_priority_allow_overrides_general_deny() {
        let sentinel = Sentinel::new(vec![
            rule("deny", Effect::Deny, 10, Conditions::default()),
            rule("allow-admin", Effect::Allow, 20, Conditions::default()),
        ])
        .unwrap();

        assert!(
            sentinel
                .check("any", &json!({}), &known_context(json!({})))
                .allow
        );
    }

    #[test]
    fn invalid_replacement_leaves_existing_rules_active() {
        let mut sentinel = Sentinel::new(vec![rule(
            "existing",
            Effect::Deny,
            1,
            Conditions::default(),
        )])
        .unwrap();
        let invalid = rule(
            "invalid",
            Effect::Allow,
            2,
            Conditions {
                context: vec![predicate(
                    "not-a-pointer",
                    ValueKind::Boolean,
                    Operator::Eq,
                    json!(true),
                )],
                ..Conditions::default()
            },
        );

        assert!(sentinel.set_rules(vec![invalid]).is_err());
        assert_eq!(sentinel.list_rules()[0].id, "existing");
    }
}
