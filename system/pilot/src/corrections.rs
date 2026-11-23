// SPDX-License-Identifier: MulanPSL-2.0
//
// Correction rules: operator-written files that tell Pilot to add a short
// hint to the next planning request when the conversation matches a condition.
//
// A rule is one YAML file in the directory named by `system.pilot.corrections_dir`.
// Pilot evaluates every rule before each planning request. A rule that matches
// adds its text to that request only; it is never written to the session
// history, so the hint disappears as soon as the condition stops holding.
//
// This module only loads, matches and renders. It performs no I/O during
// evaluation and does not depend on the executor or the model client.

use anyhow::{Context, Result, bail};
use regex::Regex;
use robonix_scribe::warn;
use serde::Deserialize;
use std::collections::{BTreeMap, HashSet};
use std::path::Path;

use crate::vlm::Message;

/// Prefix of the history message Pilot writes for every executor leaf result.
/// `planner::rtdl_result_to_messages` produces it; rules read it back.
pub(crate) const LEAF_FEEDBACK_PREFIX: &str =
    "Executor feedback for the current RTDL leaf (not a new user request): ";

/// Prefix of the history message Pilot writes for every user message.
/// `tasks::UserMessage::line` produces it.
pub(crate) const USER_MESSAGE_PREFIX: &str = "User message ";

const API_VERSION: &str = "robonix.pilot/correction/v1";
/// Longest injected text per rule, in bytes. Keeps a mistaken rule from
/// putting a large block into every round.
const MAX_TEMPLATE_BYTES: usize = 1200;
const INJECTION_HEADER: &str = "Pilot corrections for this step:\n";

// ── File format ──────────────────────────────────────────────────────────────

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RuleFile {
    #[serde(rename = "apiVersion")]
    api_version: String,
    id: String,
    #[serde(default)]
    #[allow(dead_code)] // for humans reading the file
    description: String,
    #[serde(default = "default_true")]
    enabled: bool,
    #[serde(default)]
    scope: ScopeFile,
    when: WhenFile,
    #[serde(default)]
    window: WindowFile,
    actions: Vec<serde_yaml::Value>,
}

fn default_true() -> bool {
    true
}

#[derive(Deserialize, Default, Clone, Copy, PartialEq, Eq, Debug)]
#[serde(rename_all = "snake_case")]
enum ScopeFile {
    /// Only history after the latest user message.
    #[default]
    CurrentMessage,
    /// Only history added since this turn began.
    Turn,
    /// The whole session history.
    Session,
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct WindowFile {
    /// Planning rounds, counted from the first match, during which the rule
    /// keeps applying. Absent means until its scope ends.
    #[serde(default)]
    max_rounds: Option<u32>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct WhenFile {
    #[serde(default)]
    any: Vec<MatcherFile>,
    #[serde(default)]
    all: Vec<MatcherFile>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct MatcherFile {
    leaf: LeafFile,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct LeafFile {
    contract: String,
    #[serde(default)]
    success: Option<bool>,
    #[serde(default)]
    error: Option<TextTestFile>,
    #[serde(default)]
    output: Option<TextTestFile>,
}

/// `error: "text"` means contains; `error: {regex: "..."}` is a pattern.
#[derive(Deserialize)]
#[serde(untagged)]
enum TextTestFile {
    Contains(String),
    Pattern { regex: String },
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct InjectFile {
    template: String,
}

// ── Loaded rules ─────────────────────────────────────────────────────────────

enum TextTest {
    Contains(String),
    Pattern(Regex),
}

impl TextTest {
    fn from_file(test: TextTestFile, what: &str) -> Result<Self> {
        Ok(match test {
            TextTestFile::Contains(text) => Self::Contains(text),
            TextTestFile::Pattern { regex } => Self::Pattern(
                Regex::new(&regex).with_context(|| format!("{what}: invalid regex {regex:?}"))?,
            ),
        })
    }

    /// Named captures of the match, or `None` when the text does not match.
    fn captures(&self, text: &str) -> Option<BTreeMap<String, String>> {
        match self {
            Self::Contains(needle) => text.contains(needle.as_str()).then(BTreeMap::new),
            Self::Pattern(re) => {
                let caps = re.captures(text)?;
                let mut vars = BTreeMap::new();
                for name in re.capture_names().flatten() {
                    if let Some(m) = caps.name(name) {
                        vars.insert(name.to_string(), m.as_str().to_string());
                    }
                }
                Some(vars)
            }
        }
    }

    fn variable_names(&self) -> Vec<String> {
        match self {
            Self::Contains(_) => Vec::new(),
            Self::Pattern(re) => re.capture_names().flatten().map(str::to_string).collect(),
        }
    }
}

struct LeafMatcher {
    contract: String,
    success: Option<bool>,
    error: Option<TextTest>,
    output: Option<TextTest>,
}

impl LeafMatcher {
    fn from_file(leaf: LeafFile, rule: &str) -> Result<Self> {
        let what = |field: &str| format!("rule '{rule}' leaf {}: {field}", leaf.contract);
        let error = leaf
            .error
            .map(|t| TextTest::from_file(t, &what("error")))
            .transpose()?;
        let output = leaf
            .output
            .map(|t| TextTest::from_file(t, &what("output")))
            .transpose()?;
        Ok(Self {
            contract: leaf.contract,
            success: leaf.success,
            error,
            output,
        })
    }

    fn variable_names(&self) -> Vec<String> {
        let mut names = Vec::new();
        for test in [&self.error, &self.output].into_iter().flatten() {
            names.extend(test.variable_names());
        }
        names
    }

    /// The variables this leaf provides, if it satisfies every field.
    fn matches(&self, leaf: &LeafResult) -> Option<BTreeMap<String, String>> {
        if leaf.contract_id != self.contract {
            return None;
        }
        if self.success.is_some_and(|want| want != leaf.success) {
            return None;
        }
        let mut vars = BTreeMap::new();
        if let Some(test) = &self.error {
            vars.extend(test.captures(&leaf.error)?);
        }
        if let Some(test) = &self.output {
            vars.extend(test.captures(&leaf.output)?);
        }
        Some(vars)
    }
}

enum Condition {
    Any(Vec<LeafMatcher>),
    All(Vec<LeafMatcher>),
}

struct Rule {
    id: String,
    scope: ScopeFile,
    when: Condition,
    max_rounds: Option<u32>,
    template: String,
}

/// Every rule loaded from the corrections directory.
#[derive(Default)]
pub struct CorrectionSet {
    rules: Vec<Rule>,
}

/// One rule that fired for a request, for the log.
pub struct FiredRule {
    pub id: String,
    /// 1 on the round the rule first matched.
    pub window_round: u32,
    pub vars: BTreeMap<String, String>,
}

/// What to add to a request.
pub struct Injection {
    pub message: Message,
    pub fired: Vec<FiredRule>,
}

/// Per-turn bookkeeping: how many consecutive rounds each rule has applied.
#[derive(Default)]
pub struct CorrectionState {
    rounds: BTreeMap<String, u32>,
}

// ── Loading ──────────────────────────────────────────────────────────────────

impl CorrectionSet {
    pub fn ids(&self) -> Vec<&str> {
        self.rules.iter().map(|r| r.id.as_str()).collect()
    }

    /// Read every `*.yaml` / `*.yml` in `dir`, in file-name order.
    ///
    /// A file that cannot be used is skipped with a warning, so one bad rule
    /// does not disable the others. A missing directory or a duplicate rule id
    /// is an error: both mean the deployment is misconfigured.
    pub fn load(dir: &Path) -> Result<Self> {
        let entries = std::fs::read_dir(dir)
            .with_context(|| format!("read corrections_dir '{}'", dir.display()))?;
        let mut paths: Vec<_> = entries
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| {
                p.is_file()
                    && p.extension()
                        .and_then(|e| e.to_str())
                        .is_some_and(|e| matches!(e, "yaml" | "yml"))
            })
            .collect();
        paths.sort();

        let mut rules = Vec::new();
        let mut seen = HashSet::new();
        for path in paths {
            let text = match std::fs::read_to_string(&path) {
                Ok(text) => text,
                Err(error) => {
                    warn!("[pilot/correction] skip {}: {error}", path.display());
                    continue;
                }
            };
            let rule = match parse_rule(&text) {
                Ok(Some(rule)) => rule,
                Ok(None) => continue, // disabled
                Err(error) => {
                    warn!("[pilot/correction] skip {}: {error:#}", path.display());
                    continue;
                }
            };
            if !seen.insert(rule.id.clone()) {
                bail!(
                    "corrections_dir '{}': rule id '{}' is defined more than once",
                    dir.display(),
                    rule.id
                );
            }
            rules.push(rule);
        }
        Ok(Self { rules })
    }
}

/// `Ok(None)` is a rule that is switched off.
fn parse_rule(text: &str) -> Result<Option<Rule>> {
    let file: RuleFile = serde_yaml::from_str(text).context("parse rule")?;
    if file.api_version != API_VERSION {
        bail!(
            "apiVersion '{}' is not supported (expected '{API_VERSION}')",
            file.api_version
        );
    }
    if file.id.is_empty()
        || !file
            .id
            .chars()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
    {
        bail!("id '{}' must be non-empty and use only [a-z0-9_]", file.id);
    }
    if !file.enabled {
        return Ok(None);
    }

    let when = match (file.when.any.is_empty(), file.when.all.is_empty()) {
        (false, true) => Condition::Any(leaf_matchers(file.when.any, &file.id)?),
        (true, false) => Condition::All(leaf_matchers(file.when.all, &file.id)?),
        (true, true) => bail!("rule '{}': `when` needs `any` or `all`", file.id),
        (false, false) => bail!("rule '{}': use `any` or `all` in `when`, not both", file.id),
    };

    // Only `inject_message` exists. A later action kind in the same file must
    // not stop this version from using the rest of the rule.
    let mut template = None;
    for action in file.actions {
        let kind = action
            .get("kind")
            .and_then(serde_yaml::Value::as_str)
            .unwrap_or("")
            .to_string();
        if kind != "inject_message" {
            warn!(
                "[pilot/correction] rule '{}': action kind '{kind}' is not supported, skipped",
                file.id
            );
            continue;
        }
        let mut body = action;
        if let Some(map) = body.as_mapping_mut() {
            map.remove("kind");
        }
        let inject: InjectFile = serde_yaml::from_value(body)
            .with_context(|| format!("rule '{}': inject_message", file.id))?;
        if template.is_some() {
            bail!(
                "rule '{}': only one inject_message action is allowed",
                file.id
            );
        }
        template = Some(inject.template);
    }
    let Some(template) = template else {
        bail!("rule '{}': no supported action", file.id);
    };
    let template = template.trim().to_string();
    if template.is_empty() {
        bail!("rule '{}': empty template", file.id);
    }
    if template.len() > MAX_TEMPLATE_BYTES {
        bail!(
            "rule '{}': template is {} bytes, the limit is {MAX_TEMPLATE_BYTES}",
            file.id,
            template.len()
        );
    }

    // Every {{name}} must come from a capture in the condition: otherwise the
    // placeholder would reach the model unreplaced.
    let provided = match &when {
        Condition::Any(ms) | Condition::All(ms) => {
            let mut sets = ms
                .iter()
                .map(|m| m.variable_names().into_iter().collect::<HashSet<_>>());
            // `any` guarantees only what every alternative provides; `all`
            // provides the union.
            match &when {
                Condition::Any(_) => {
                    let first = sets.next().unwrap_or_default();
                    sets.fold(first, |acc, s| acc.intersection(&s).cloned().collect())
                }
                Condition::All(_) => sets.flatten().collect(),
            }
        }
    };
    for name in placeholders(&template) {
        if !provided.contains(&name) {
            bail!(
                "rule '{}': {{{{{name}}}}} is not captured by every way the condition can match",
                file.id
            );
        }
    }

    Ok(Some(Rule {
        id: file.id,
        scope: file.scope,
        when,
        max_rounds: file.window.max_rounds,
        template,
    }))
}

fn leaf_matchers(items: Vec<MatcherFile>, rule: &str) -> Result<Vec<LeafMatcher>> {
    items
        .into_iter()
        .map(|m| LeafMatcher::from_file(m.leaf, rule))
        .collect()
}

/// Names used as `{{name}}` in a template.
fn placeholders(template: &str) -> Vec<String> {
    let mut names = Vec::new();
    let mut rest = template;
    while let Some(start) = rest.find("{{") {
        let after = &rest[start + 2..];
        let Some(end) = after.find("}}") else { break };
        names.push(after[..end].trim().to_string());
        rest = &after[end + 2..];
    }
    names
}

fn render(template: &str, vars: &BTreeMap<String, String>) -> String {
    let mut out = String::with_capacity(template.len());
    let mut rest = template;
    while let Some(start) = rest.find("{{") {
        out.push_str(&rest[..start]);
        let after = &rest[start + 2..];
        let Some(end) = after.find("}}") else {
            out.push_str(&rest[start..]);
            return out;
        };
        let name = after[..end].trim();
        out.push_str(vars.get(name).map_or("", String::as_str));
        rest = &after[end + 2..];
    }
    out.push_str(rest);
    out
}

// ── Reading the history ──────────────────────────────────────────────────────

/// One executor leaf result as Pilot wrote it into the history.
struct LeafResult {
    contract_id: String,
    success: bool,
    output: String,
    error: String,
}

fn parse_leaf_feedback(message: &Message) -> Option<LeafResult> {
    if message.role != "user" {
        return None;
    }
    let body = message
        .content
        .as_deref()?
        .strip_prefix(LEAF_FEEDBACK_PREFIX)?;
    let value: serde_json::Value = serde_json::from_str(body).ok()?;
    let leaf = value.get("leaf_result")?;
    Some(LeafResult {
        contract_id: leaf.get("contract_id")?.as_str()?.to_string(),
        success: leaf.get("success")?.as_bool()?,
        output: leaf
            .get("output")
            .and_then(|v| v.as_str())
            .unwrap_or("")
            .to_string(),
        error: leaf
            .get("error")
            .and_then(|v| v.as_str())
            .unwrap_or("")
            .to_string(),
    })
}

fn is_user_message(message: &Message) -> bool {
    message.role == "user"
        && message
            .content
            .as_deref()
            .is_some_and(|c| c.starts_with(USER_MESSAGE_PREFIX))
}

/// Index of the first history message a rule with this scope may look at.
fn scope_start(history: &[Message], scope: ScopeFile, turn_start: usize) -> usize {
    match scope {
        ScopeFile::Session => 0,
        ScopeFile::Turn => turn_start.min(history.len()),
        ScopeFile::CurrentMessage => history
            .iter()
            .rposition(is_user_message)
            .map_or(0, |i| i + 1),
    }
}

impl Condition {
    /// The variables when the condition holds over `leaves` (oldest first);
    /// for a repeated match the most recent one wins.
    fn evaluate(&self, leaves: &[LeafResult]) -> Option<BTreeMap<String, String>> {
        match self {
            Self::Any(matchers) => {
                let mut best: Option<(usize, BTreeMap<String, String>)> = None;
                for matcher in matchers {
                    for (i, leaf) in leaves.iter().enumerate() {
                        if let Some(vars) = matcher.matches(leaf)
                            && best.as_ref().is_none_or(|(at, _)| i >= *at)
                        {
                            best = Some((i, vars));
                        }
                    }
                }
                best.map(|(_, vars)| vars)
            }
            Self::All(matchers) => {
                let mut merged = BTreeMap::new();
                for matcher in matchers {
                    let vars = leaves.iter().rev().find_map(|leaf| matcher.matches(leaf))?;
                    merged.extend(vars);
                }
                Some(merged)
            }
        }
    }
}

// ── Evaluation ───────────────────────────────────────────────────────────────

impl CorrectionSet {
    /// Decide what, if anything, to add to the request about to be sent.
    ///
    /// `history` is the session history as it stands; it is only read.
    /// `turn_start` is the history length when the current turn began.
    pub fn evaluate(
        &self,
        history: &[Message],
        turn_start: usize,
        state: &mut CorrectionState,
    ) -> Option<Injection> {
        if self.rules.is_empty() {
            return None;
        }
        let mut fired = Vec::new();
        let mut sections = Vec::new();
        let mut still_firing = HashSet::new();

        for rule in &self.rules {
            let start = scope_start(history, rule.scope, turn_start);
            let leaves: Vec<LeafResult> = history[start..]
                .iter()
                .filter_map(parse_leaf_feedback)
                .collect();
            let Some(vars) = rule.when.evaluate(&leaves) else {
                continue;
            };
            let round = state.rounds.get(&rule.id).copied().unwrap_or(0) + 1;
            if rule.max_rounds.is_some_and(|max| round > max) {
                // Out of window: keep counting so it stays out until the
                // condition has gone away and comes back.
                still_firing.insert(rule.id.clone());
                state.rounds.insert(rule.id.clone(), round);
                continue;
            }
            still_firing.insert(rule.id.clone());
            state.rounds.insert(rule.id.clone(), round);
            sections.push(render(&rule.template, &vars));
            fired.push(FiredRule {
                id: rule.id.clone(),
                window_round: round,
                vars,
            });
        }
        // A rule whose condition no longer holds starts over next time.
        state.rounds.retain(|id, _| still_firing.contains(id));

        if sections.is_empty() {
            return None;
        }
        let mut text = String::from(INJECTION_HEADER);
        text.push_str(&sections.join("\n\n"));
        Some(Injection {
            message: Message::user(&text),
            fired,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOAL_NEAR_RULE: &str = r#"
apiVersion: robonix.pilot/correction/v1
id: goal_near_unreachable
description: test
scope: current_message
when:
  any:
    - leaf:
        contract: robonix/system/scene/goal_near
        success: false
        error:
          regex: 'no free cell within [\d.]+ m of (?P<object_id>[\w.]+)'
    - leaf:
        contract: robonix/system/scene/goal_near
        success: true
        output:
          regex: 'no footprint-safe approach cell for ''(?P<object_id>[^'']+)'''
actions:
  - kind: inject_message
    template: |
      Rule goal_near_unreachable: {{object_id}} cannot be reached; do not navigate.
"#;

    fn set_of(rules: &[&str]) -> CorrectionSet {
        CorrectionSet {
            rules: rules
                .iter()
                .map(|text| parse_rule(text).unwrap().unwrap())
                .collect(),
        }
    }

    fn user_message(n: u32, text: &str) -> Message {
        Message::user(&format!("User message m{n} (filed under T1): {text}"))
    }

    /// Written through the real producer so a format change breaks these tests.
    fn feedback(contract: &str, success: bool, output: &str, error: &str) -> Message {
        let result = crate::pb::pilot::CapabilityCallResult {
            call_id: "p-1:0".into(),
            contract_id: contract.into(),
            success,
            output: output.into(),
            error: error.into(),
            ..Default::default()
        };
        crate::planner::rtdl_result_to_messages_for_test(&result)
            .tool_messages
            .into_iter()
            .next()
            .unwrap()
    }

    fn no_free_cell(object: &str) -> Message {
        feedback(
            "robonix/system/scene/goal_near",
            false,
            "",
            &format!(
                "mcp tool error: Error executing tool goal_near: no free cell within 1.5 m of {object}"
            ),
        )
    }

    fn text(injection: &Injection) -> String {
        injection.message.content.clone().unwrap()
    }

    #[test]
    fn failed_goal_near_injects_the_object_id() {
        let set = set_of(&[GOAL_NEAR_RULE]);
        let history = vec![
            user_message(1, "go next to the plant"),
            no_free_cell("scene.object.0002"),
        ];
        let injection = set
            .evaluate(&history, 0, &mut CorrectionState::default())
            .expect("rule should fire");
        assert!(text(&injection).contains("scene.object.0002 cannot be reached"));
        assert_eq!(injection.fired[0].id, "goal_near_unreachable");
        assert_eq!(injection.fired[0].window_round, 1);
        assert_eq!(injection.message.role, "user");
    }

    #[test]
    fn unreachable_reply_from_the_real_scene_service_also_fires() {
        let set = set_of(&[GOAL_NEAR_RULE]);
        let history = vec![
            user_message(1, "go next to the sink"),
            feedback(
                "robonix/system/scene/goal_near",
                true,
                r#"{"reachable":false,"reason":"no footprint-safe approach cell for 'scene.object.0026'"}"#,
                "",
            ),
        ];
        let injection = set
            .evaluate(&history, 0, &mut CorrectionState::default())
            .expect("rule should fire");
        assert!(text(&injection).contains("scene.object.0026"));
    }

    #[test]
    fn other_failures_and_other_contracts_do_not_fire() {
        let set = set_of(&[GOAL_NEAR_RULE]);
        for leaf in [
            // Wrong object id: the model should retry with another.
            feedback(
                "robonix/system/scene/goal_near",
                false,
                "",
                "mcp tool error: unknown physical object_id 'plant'",
            ),
            // A different capability failing the same way.
            feedback(
                "robonix/system/scene/goal_region",
                false,
                "",
                "no free cell within 1.5 m of scene.object.0002",
            ),
            // A reachable result.
            feedback(
                "robonix/system/scene/goal_near",
                true,
                r#"{"reachable":true,"x":1.0,"y":2.0,"yaw":0.0}"#,
                "",
            ),
        ] {
            let history = vec![user_message(1, "go"), leaf];
            assert!(
                set.evaluate(&history, 0, &mut CorrectionState::default())
                    .is_none()
            );
        }
    }

    #[test]
    fn a_new_user_message_ends_the_rule() {
        let set = set_of(&[GOAL_NEAR_RULE]);
        let history = vec![
            user_message(1, "go next to the plant"),
            no_free_cell("scene.object.0002"),
            user_message(2, "forget it, take a photo"),
        ];
        assert!(
            set.evaluate(&history, 0, &mut CorrectionState::default())
                .is_none()
        );
    }

    #[test]
    fn the_most_recent_match_supplies_the_variables() {
        let set = set_of(&[GOAL_NEAR_RULE]);
        let history = vec![
            user_message(1, "visit the plant then the sink"),
            no_free_cell("scene.object.0002"),
            no_free_cell("scene.object.0026"),
        ];
        let injection = set
            .evaluate(&history, 0, &mut CorrectionState::default())
            .unwrap();
        assert!(text(&injection).contains("scene.object.0026"));
        assert!(!text(&injection).contains("scene.object.0002"));
    }

    #[test]
    fn window_limits_the_rounds_and_a_returning_condition_restarts_it() {
        let rule = GOAL_NEAR_RULE.replace("actions:", "window:\n  max_rounds: 2\nactions:");
        let set = set_of(&[&rule]);
        let mut state = CorrectionState::default();
        let failing = vec![user_message(1, "go"), no_free_cell("scene.object.0002")];
        let rounds: Vec<bool> = (0..4)
            .map(|_| set.evaluate(&failing, 0, &mut state).is_some())
            .collect();
        assert_eq!(rounds, [true, true, false, false]);

        // The condition goes away (new message), then comes back.
        let clear = vec![user_message(2, "something else")];
        assert!(set.evaluate(&clear, 0, &mut state).is_none());
        assert!(set.evaluate(&failing, 0, &mut state).is_some());
    }

    #[test]
    fn evaluation_leaves_the_history_untouched() {
        let set = set_of(&[GOAL_NEAR_RULE]);
        let history = vec![user_message(1, "go"), no_free_cell("scene.object.0002")];
        let before: Vec<_> = history.iter().map(|m| m.content.clone()).collect();
        let _ = set.evaluate(&history, 0, &mut CorrectionState::default());
        let after: Vec<_> = history.iter().map(|m| m.content.clone()).collect();
        assert_eq!(before, after);
    }

    #[test]
    fn turn_scope_ignores_history_from_before_the_turn() {
        let rule = GOAL_NEAR_RULE.replace("scope: current_message", "scope: turn");
        let set = set_of(&[&rule]);
        let history = vec![user_message(1, "go"), no_free_cell("scene.object.0002")];
        assert!(
            set.evaluate(&history, 2, &mut CorrectionState::default())
                .is_none()
        );
        assert!(
            set.evaluate(&history, 0, &mut CorrectionState::default())
                .is_some()
        );
    }

    #[test]
    fn several_rules_are_merged_in_file_order_under_one_header() {
        let second = GOAL_NEAR_RULE
            .replace("goal_near_unreachable", "second_rule")
            .replace("Rule second_rule", "Second");
        let set = set_of(&[GOAL_NEAR_RULE, &second]);
        let history = vec![user_message(1, "go"), no_free_cell("scene.object.0002")];
        let injection = set
            .evaluate(&history, 0, &mut CorrectionState::default())
            .unwrap();
        let body = text(&injection);
        assert!(body.starts_with(INJECTION_HEADER));
        assert!(body.find("Rule goal_near_unreachable").unwrap() < body.find("Second").unwrap());
        assert_eq!(injection.fired.len(), 2);
    }

    #[test]
    fn all_requires_every_matcher_and_merges_variables() {
        let rule = r#"
apiVersion: robonix.pilot/correction/v1
id: both
when:
  all:
    - leaf: {contract: a/one, success: false, error: {regex: 'bad (?P<x>\w+)'}}
    - leaf: {contract: a/two, success: true, output: {regex: 'ok (?P<y>\w+)'}}
actions:
  - kind: inject_message
    template: "x={{x}} y={{y}}"
"#;
        let set = set_of(&[rule]);
        let one = feedback("a/one", false, "", "bad thing");
        let two = feedback("a/two", true, "ok fine", "");
        let only_one = vec![user_message(1, "go"), one.clone()];
        assert!(
            set.evaluate(&only_one, 0, &mut CorrectionState::default())
                .is_none()
        );
        let both = vec![user_message(1, "go"), one, two];
        let injection = set
            .evaluate(&both, 0, &mut CorrectionState::default())
            .unwrap();
        assert!(text(&injection).contains("x=thing y=fine"));
    }

    #[test]
    fn a_plain_string_test_means_contains() {
        let rule = r#"
apiVersion: robonix.pilot/correction/v1
id: plain
when:
  any:
    - leaf: {contract: a/one, error: "timed out"}
actions:
  - kind: inject_message
    template: retry later
"#;
        let set = set_of(&[rule]);
        let history = vec![
            user_message(1, "go"),
            feedback("a/one", false, "", "call timed out"),
        ];
        assert!(
            set.evaluate(&history, 0, &mut CorrectionState::default())
                .is_some()
        );
    }

    // ── Loading ──

    fn expect_err(text: &str, needle: &str) {
        let error = format!("{:#}", parse_rule(text).err().expect("should be rejected"));
        assert!(error.contains(needle), "error {error:?} lacks {needle:?}");
    }

    #[test]
    fn bad_rule_files_are_rejected_with_a_reason() {
        expect_err(
            &GOAL_NEAR_RULE.replace("correction/v1", "correction/v9"),
            "apiVersion",
        );
        expect_err(
            &GOAL_NEAR_RULE.replace("id: goal_near_unreachable", "id: Bad-Id"),
            "id",
        );
        expect_err(
            &GOAL_NEAR_RULE.replace(r"(?P<object_id>[\w.]+)", r"(?P<object_id>[\w.]+"),
            "regex",
        );
        expect_err(
            &GOAL_NEAR_RULE.replace("{{object_id}}", "{{unknown}}"),
            "not captured",
        );
        let long = format!("{GOAL_NEAR_RULE}\n      {}", "x".repeat(1300));
        expect_err(&long, "limit");
        expect_err(
            &GOAL_NEAR_RULE.replace("  any:", "  neither:"),
            "parse rule",
        );
    }

    #[test]
    fn a_placeholder_only_one_alternative_provides_is_rejected() {
        let rule = r#"
apiVersion: robonix.pilot/correction/v1
id: partial
when:
  any:
    - leaf: {contract: a/one, error: {regex: '(?P<x>\w+)'}}
    - leaf: {contract: a/two, error: "boom"}
actions:
  - kind: inject_message
    template: "{{x}}"
"#;
        expect_err(rule, "not captured");
    }

    #[test]
    fn disabled_rules_are_not_loaded() {
        let rule = GOAL_NEAR_RULE.replace("description: test", "enabled: false");
        assert!(parse_rule(&rule).unwrap().is_none());
    }

    #[test]
    fn an_unknown_action_kind_is_skipped_but_the_rest_of_the_rule_works() {
        let rule = GOAL_NEAR_RULE.replace(
            "actions:\n",
            "actions:\n  - kind: rewrite_rtdl\n    drop_calls: {contract: x}\n",
        );
        let set = set_of(&[&rule]);
        let history = vec![user_message(1, "go"), no_free_cell("scene.object.0002")];
        assert!(
            set.evaluate(&history, 0, &mut CorrectionState::default())
                .is_some()
        );
    }

    #[test]
    fn a_rule_with_only_unknown_actions_is_rejected() {
        let rule = GOAL_NEAR_RULE.replace("inject_message", "rewrite_rtdl");
        expect_err(&rule, "no supported action");
    }

    #[test]
    fn load_reads_a_directory_skips_bad_files_and_rejects_duplicate_ids() {
        let dir = std::env::temp_dir().join(format!("pilot-corrections-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("a.yaml"), GOAL_NEAR_RULE).unwrap();
        std::fs::write(dir.join("b.yml"), "not: [valid").unwrap();
        std::fs::write(dir.join("notes.txt"), "ignored").unwrap();
        let set = CorrectionSet::load(&dir).unwrap();
        assert_eq!(set.ids(), ["goal_near_unreachable"]);

        std::fs::write(dir.join("c.yaml"), GOAL_NEAR_RULE).unwrap();
        let error = format!("{:#}", CorrectionSet::load(&dir).err().unwrap());
        assert!(error.contains("more than once"), "{error}");
        std::fs::remove_dir_all(&dir).unwrap();

        assert!(
            CorrectionSet::load(&dir).is_err(),
            "a missing directory is an error"
        );
    }

    #[test]
    fn the_shipped_example_rule_loads() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("corrections/examples");
        let set = CorrectionSet::load(&dir).unwrap();
        assert!(set.ids().contains(&"goal_near_unreachable"));
    }
}
