// SPDX-License-Identifier: MulanPSL-2.0
//
// User-feedback correction learning — phase 1 of the plan in
// docs/agent_fix/pilot_self_correction_skill_plan.md.
//
// Pilot sends user messages and bounded history to a model through a side
// channel that cannot reach the Executor. Validated drafts remain disabled
// until an operator reviews and enables them.
//
// Diagnosis runs separately so it does not delay stop or steer handling.

use crate::config::LearningConfig;
use crate::corrections::{LEAF_FEEDBACK_PREFIX, USER_MESSAGE_PREFIX, validate_rule_text};
use crate::planner::{RESPONSE_MODE_TEXT, RESPONSE_MODE_VOICE};
use crate::vlm::{Message, VlmClient};
use anyhow::{Context, Result, anyhow, bail};
use robonix_scribe::{info, warn};
use serde::Deserialize;
use serde_json::{Value as Json, json};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

/// The prefix of the history message that records a dispatched plan.
/// Mirrors the string in `planner::record_dispatched_plan`.
const DISPATCH_RECORD_PREFIX: &str = "Pilot harness dispatch record";

/// Hard ceiling on the evidence handed to the reflection model.
const MAX_EVIDENCE_ITEMS: usize = 24;
const MAX_EVIDENCE_CHARS: usize = 6_000;
/// At most this many diagnoses may be in flight process-wide; a correction
/// storm must not turn into a model-call storm.
const MAX_IN_FLIGHT: usize = 2;
const MAX_PENDING: usize = 32;
/// One revision attempt when the first draft fails strict validation.
const MAX_REVISIONS: usize = 1;

// ── Runtime ──────────────────────────────────────────────────────────────────

/// Everything the side channel may use. Built once at startup; the active
/// rule ids stay accurate because phase 1 has no hot reload.
pub struct LearningRuntime {
    /// Locations and model selection for correction learning.
    cfg: LearningConfig,
    /// Directory read for active correction rules, when configured.
    corrections_dir: Option<PathBuf>,
    /// Client for the side-channel model calls.
    client: VlmClient,
    /// The model that answers the reflection call.
    model: String,
    /// Active rule ids, for new-vs-revise decisions and id collisions.
    active_rule_ids: Vec<String>,
    /// Number of queued and running diagnoses.
    in_flight: AtomicUsize,
    /// Limits simultaneous side-channel calls.
    slots: tokio::sync::Semaphore,
    /// Serializes candidate deduplication and file creation.
    persist_lock: tokio::sync::Mutex<()>,
}

impl LearningRuntime {
    pub fn new(
        cfg: LearningConfig,
        vlm: &crate::config::VlmConfig,
        active_rule_ids: Vec<String>,
        corrections_dir: Option<PathBuf>,
    ) -> Self {
        let model = cfg
            .reflection_model
            .clone()
            .unwrap_or_else(|| vlm.model.clone());
        let mut reflection_vlm = vlm.clone();
        if let Some(upstream) = &cfg.reflection_upstream {
            reflection_vlm.upstream.clone_from(upstream);
        }
        if let Some(api_key) = &cfg.reflection_api_key {
            reflection_vlm.api_key.clone_from(api_key);
        }
        reflection_vlm.model.clone_from(&model);
        Self {
            client: VlmClient::new(&reflection_vlm),
            cfg,
            corrections_dir,
            model,
            active_rule_ids,
            in_flight: AtomicUsize::new(0),
            slots: tokio::sync::Semaphore::new(MAX_IN_FLIGHT),
            persist_lock: tokio::sync::Mutex::new(()),
        }
    }

    /// Client shared by correction drafting and rule applicability checks.
    pub(crate) fn client(&self) -> &VlmClient {
        &self.client
    }

    /// Describe the configured locations without claiming a candidate exists.
    pub(crate) fn guidance(&self) -> String {
        let candidates = self.cfg.root_dir.join("candidates");
        let active = self.corrections_dir.as_ref().map_or_else(
            || "No active corrections directory is configured. Tell the user to configure corrections_dir and copy the reviewed candidate there before enabling it.".to_string(),
            |dir| format!("New validated rules are saved disabled directly in {}. Tell the user to review the specified file, set enabled: true in it, and restart Pilot.", dir.display()),
        );
        format!(
            "Candidate records are created asynchronously under {} and rules remain disabled. {} \
             When the user requests a correction, remind them to review it. The exact rule \
             file will be reported when available, possibly on the next turn. Do not claim that \
             a candidate was generated or name a specific file unless its creation is confirmed.",
            candidates.display(),
            active
        )
    }

    /// Return unreported candidates produced for this session.
    pub(crate) fn pending_notices(&self, session_id: &str) -> Vec<(PathBuf, String)> {
        let Ok(entries) = std::fs::read_dir(self.cfg.root_dir.join("candidates")) else {
            return Vec::new();
        };
        let mut notices = Vec::new();
        for entry in entries.flatten() {
            if !entry.file_type().is_ok_and(|kind| kind.is_dir()) {
                continue;
            }
            let dir = entry.path();
            let marker = dir.join(".notified");
            if marker.exists() {
                continue;
            }
            let Ok(bytes) = std::fs::read(dir.join("metadata.json")) else {
                continue;
            };
            let Ok(metadata) = serde_json::from_slice::<Json>(&bytes) else {
                continue;
            };
            if metadata["status"] != "drafted" || metadata["episode"]["session_id"] != session_id {
                continue;
            }
            let Some(id) = metadata["rule_id"].as_str() else {
                continue;
            };
            let candidate = dir.join("rule.yaml");
            if !std::fs::symlink_metadata(&candidate).is_ok_and(|meta| meta.file_type().is_file()) {
                continue;
            }
            let active = metadata["active_rule_file"]
                .as_str()
                .and_then(|name| {
                    let valid = name.strip_suffix(".yaml").is_some_and(|stem| {
                        !stem.is_empty()
                            && stem
                                .chars()
                                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
                    });
                    valid
                        .then(|| self.corrections_dir.as_ref().map(|dir| dir.join(name)))
                        .flatten()
                })
                .filter(|path| {
                    std::fs::symlink_metadata(path).is_ok_and(|meta| meta.file_type().is_file())
                });
            let notice = if let Some(active) = active {
                if self.active_rule_ids.iter().any(|loaded| loaded == id) {
                    format!(
                        "Correction {id} was loaded from {} when Pilot started.",
                        active.display()
                    )
                } else {
                    format!(
                        "Correction candidate {id}: {}. Review this file, set enabled: true, and restart Pilot for it to take effect.",
                        active.display()
                    )
                }
            } else {
                let destination = self.corrections_dir.as_ref().map_or_else(
                    || "Configure corrections_dir first.".to_string(),
                    |path| format!("Copy the reviewed file into {}, set enabled: true in the copy, and restart Pilot.", path.display()),
                );
                format!(
                    "Correction candidate {id}: {}. {destination}",
                    candidate.display()
                )
            };
            notices.push((marker, notice));
        }
        notices.sort_by(|a, b| a.0.cmp(&b.0));
        notices
    }
}

// ── Evidence collection ──────────────────────────────────────────────────────

/// One correction event with the evidence Pilot could associate with it.
pub(crate) struct Episode {
    pub session_id: String,
    pub turn_task_id: String,
    pub correction_text: String,
    /// The user message before the correction: the request that went wrong.
    pub original_request: Option<String>,
    /// Rendered dispatch records and executor feedback between the two
    /// requests, oldest first, bounded.
    pub evidence: Vec<String>,
    pub captured_at_unix: u64,
}

fn is_user_labelled(message: &Message) -> bool {
    message.role == "user"
        && message
            .content
            .as_deref()
            .is_some_and(|c| c.starts_with(USER_MESSAGE_PREFIX))
}

/// The body of a labelled user message, without the `User message mN (filed
/// under Tn): ` label and without the generated response-mode suffix.
fn user_body(content: &str) -> Option<&str> {
    let labelled = content.strip_prefix(USER_MESSAGE_PREFIX)?;
    let (_ids, rest) = labelled.split_once(" (filed under ")?;
    let (_task, body) = rest.split_once("): ")?;
    for suffix in [RESPONSE_MODE_TEXT, RESPONSE_MODE_VOICE] {
        if let Some(stripped) = body.strip_suffix(suffix) {
            return stripped.strip_suffix('\n').or(Some(stripped));
        }
    }
    Some(body)
}

fn truncate_chars(text: &str, max_chars: usize) -> String {
    if text.chars().count() <= max_chars {
        text.to_string()
    } else {
        let mut cut: String = text.chars().take(max_chars).collect();
        cut.push('…');
        cut
    }
}

/// One bounded evidence line from a history message, or `None` for messages
/// that carry no execution evidence (assistant text, boards, memories).
fn evidence_line(message: &Message) -> Option<String> {
    let content = message.content.as_deref()?;
    if let Some(body) = content.strip_prefix(LEAF_FEEDBACK_PREFIX) {
        let value: Json = serde_json::from_str(body).ok()?;
        let leaf = value.get("leaf_result")?;
        let contract = leaf
            .get("contract_id")
            .and_then(Json::as_str)
            .unwrap_or("?");
        let success = leaf.get("success").and_then(Json::as_bool).unwrap_or(false);
        let error = leaf.get("error").and_then(Json::as_str).unwrap_or("");
        let output = leaf.get("output").and_then(Json::as_str).unwrap_or("");
        let line = format!(
            "leaf {contract} success={success} error={:?} output={:?}",
            truncate_chars(error, 120),
            truncate_chars(output, 240),
        );
        Some(truncate_chars(&line, 420))
    } else if let Some(body) = content.strip_prefix(DISPATCH_RECORD_PREFIX) {
        let (_note, record) = body.split_once("): ")?;
        let value: Json = serde_json::from_str(record).ok()?;
        let plan_id = value.get("plan_id").and_then(Json::as_str).unwrap_or("?");
        let calls: Vec<String> = value
            .get("calls")
            .and_then(Json::as_array)
            .map(|calls| {
                calls
                    .iter()
                    .filter_map(|call| {
                        let contract = call.get("contract_id").and_then(Json::as_str)?;
                        let args = call.get("args").map(|a| a.to_string()).unwrap_or_default();
                        Some(format!("{contract}({})", truncate_chars(&args, 80)))
                    })
                    .collect()
            })
            .unwrap_or_default();
        let line = format!("dispatch plan {plan_id}: {}", calls.join(", "));
        Some(truncate_chars(&line, 420))
    } else {
        None
    }
}

/// Build the episode for a correction that was just pushed to `history` (the
/// last labelled user message). A standing instruction may be the first turn.
pub(crate) fn build_episode(
    history: &[Message],
    session_id: &str,
    task_id: &str,
    correction_text: &str,
) -> Option<Episode> {
    let last = history.iter().rposition(is_user_labelled)?;
    let previous = history[..last].iter().rposition(is_user_labelled);
    let original_request = previous
        .and_then(|index| history[index].content.as_deref())
        .and_then(user_body)
        .map(str::to_string);
    let mut evidence: Vec<String> = history[previous.map_or(0, |index| index + 1)..last]
        .iter()
        .filter_map(evidence_line)
        .collect();
    while evidence.len() > MAX_EVIDENCE_ITEMS {
        evidence.remove(0);
    }
    while evidence.len() > 1
        && evidence.iter().map(|line| line.len()).sum::<usize>() > MAX_EVIDENCE_CHARS
    {
        evidence.remove(0);
    }
    Some(Episode {
        session_id: session_id.to_string(),
        turn_task_id: task_id.to_string(),
        correction_text: truncate_chars(correction_text, 4_000),
        original_request,
        evidence,
        captured_at_unix: now_unix(),
    })
}

// ── Hook (called from the planner) ───────────────────────────────────────────

/// Inspect each message on a detached task without delaying the turn.
pub(crate) fn maybe_start_learning(
    learning: Option<&Arc<LearningRuntime>>,
    history: &[Message],
    session_id: &str,
    task_id: &str,
    user_text: &str,
) {
    let Some(runtime) = learning else { return };
    if user_text.trim().is_empty() {
        return;
    }
    let Some(episode) = build_episode(history, session_id, task_id, user_text) else {
        return;
    };
    if runtime
        .in_flight
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |n| {
            (n < MAX_PENDING).then_some(n + 1)
        })
        .is_err()
    {
        warn!(
            "[pilot/learning] {MAX_PENDING} messages already pending; skipping message in session {session_id}"
        );
        return;
    }
    let runtime = Arc::clone(runtime);
    tokio::spawn(async move {
        let result = match runtime.slots.acquire().await {
            Ok(_permit) => tokio::time::timeout(
                std::time::Duration::from_secs(90),
                run_episode(&runtime, episode),
            )
            .await
            .context("correction learning timed out")
            .and_then(|result| result),
            Err(error) => Err(error.into()),
        };
        runtime.in_flight.fetch_sub(1, Ordering::Relaxed);
        if let Err(error) = result {
            warn!("[pilot/learning] draft failed: {error:#}");
        }
    });
}

// ── Diagnosis (side channel; no Executor access) ─────────────────────────────

const ANALYST_SYSTEM: &str = "You are the Robonix Pilot correction analyst. You receive a user \
    message, the evidence Pilot captured, and the schema of the \
    operator's correction rules. Reply with exactly one JSON object and nothing else. Your output \
    is parsed by a program and is never dispatched to the robot.";

fn build_prompt(episode: &Episode, active_rule_ids: &[String]) -> String {
    let active = if active_rule_ids.is_empty() {
        "(none)".to_string()
    } else {
        active_rule_ids.join(", ")
    };
    let original = episode
        .original_request
        .as_deref()
        .unwrap_or("(none captured)");
    let evidence = if episode.evidence.is_empty() {
        "(none captured)".to_string()
    } else {
        episode.evidence.join("\n")
    };
    format!(
        "## Task\n\
Determine whether the latest user message reports a mistake or gives a clear instruction for future tasks. \
Ordinary requests, questions and hypothetical examples need no rule. Use needs_clarification if \
the user's intended correction is unclear. Use the evidence where relevant, and draft a persistent \
rule only when the user's intended future behavior is clear. A validated candidate is saved disabled; \
the application will tell the user the exact candidate file and active directory for review and enabling. \
Do not invent a file path or claim the rule is already active.\n\n\
## Correction rule format (robonix.pilot/correction/v1)\n\
- apiVersion: robonix.pilot/correction/v1\n\
- id: [a-z0-9_] only\n\
- enabled: must be false (drafts are never auto-enabled)\n\
- scope: current_message | turn | session (prefer current_message)\n\
- when:\n\
  - latest_user_request (optional): {{regex: \"...\"}} matched against the latest user request \
text only; it can be used alone or as an additional AND gate and provides no template variables\n\
  - any: (or all:) an optional non-empty list of leaf matchers {{contract: \"<exact contract id>\", \
success: true|false (optional), error: \"substring\" or {{regex: \"...\"}}, output: same}}; a leaf \
matcher tests executor feedback already present in the conversation history\n\
- window: {{max_rounds: N or null}}\n\
- actions: exactly one {{kind: inject_message, template: \"...\"}}; the template is at most 1200 \
bytes; {{name}} placeholders must be captured by named groups in EVERY branch of the condition. \
For an explicit instruction to require the user's coordinates before navigating to a named place, \
also add {{kind: require_user_coordinates}}. This blocks navigation unless its goal matches the \
latest request's x=<number>, y=<number>. It requires a request-only condition. Use \
latest_user_request regex \"(?s).+\" for this action: a model checks the destination against \
the template before the coordinate restriction is applied. State the protected destination \
and the requested behavior clearly in the template.\n\
- the regex dialect is Rust regex: no lookahead, no lookbehind, no backreferences\n\n\
## Hard constraints\n\
- For corrections of a past failure, trigger on a specific error signal whenever possible. \
For clear standing instructions, match relevant future requests before execution and avoid \
matching unrelated requests.\n\
- Use only contract ids that appear in the evidence or in the active rule list.\n\
- Never hardcode object ids, coordinates, counts or passwords from this single incident into a rule.\n\
- If the mistake needs call ordering or anything the format above cannot express, \
answer \"unsupported\" and say what would be needed.\n\
- If the root cause looks environmental (a missing file, a provider outage), answer \"no_rule\" \
unless the user also gives a clear instruction for future behavior.\n\
- If the active rules already cover this mistake, prefer \"no_rule\" with a diagnosis that names \
the covering rule over drafting a near-duplicate.\n\n\
## Active rule ids\n{active}\n\n\
## Original task request\n{original}\n\n\
## User correction\n{correction}\n\n\
## Evidence (dispatch records and executor feedback between the two requests, oldest first; \
listed calls may not have run)\n{evidence}\n\n\
## Reply\n\
Return exactly one JSON object, no prose outside it:\n\
{{\"decision\":\"new_rule|revise_rule|no_rule|needs_clarification|unsupported\",\
\"diagnosis\":\"what went wrong and the evidence for it\",\
\"rule\":{{...}} or null,\
\"positive_tests\":[\"requests plus feedback that should trigger the rule\"],\
\"negative_tests\":[\"nearby tasks that must NOT trigger it\"],\
\"behavior_expectations\":\"what correct execution looks like\",\
\"open_questions\":\"what is still uncertain\"}}\n\
\"rule\" must be null unless decision is new_rule or revise_rule, and must follow the format above.",
        correction = truncate_chars(&episode.correction_text, 600),
    )
}

/// The model's reply, as a validated proposal.
#[derive(Deserialize)]
struct Proposal {
    decision: String,
    #[serde(default)]
    diagnosis: String,
    #[serde(default)]
    rule: Option<Json>,
    #[serde(default)]
    positive_tests: Vec<String>,
    #[serde(default)]
    negative_tests: Vec<String>,
    #[serde(default)]
    behavior_expectations: String,
    #[serde(default)]
    open_questions: String,
}

fn parse_proposal(mut reply: &str) -> Result<Proposal> {
    // Tolerate a fenced ```json block, since models often add one even when
    // told not to.
    let mut body = reply.trim();
    if let Some(start) = body.find("```")
        && let Some(end) = body[start + 3..].find("```")
    {
        let inner = &body[start + 3..start + 3 + end];
        body = inner.trim_start_matches("json").trim();
        // Keep the fenced content as the candidate body only when the fence
        // actually wrapped something.
        if !body.is_empty() {
            reply = body;
        }
    }
    serde_json::from_str(reply).context("model reply is not one JSON object")
}

/// What happened to one correction event.
struct DraftOutcome {
    status: &'static str,
    decision: String,
    diagnosis: String,
    rule_yaml: Option<String>,
    rule_id: Option<String>,
    validation_error: Option<String>,
    positive_tests: Vec<String>,
    negative_tests: Vec<String>,
    behavior_expectations: String,
    open_questions: String,
}

impl DraftOutcome {
    fn no_rule(proposal: Proposal, status: &'static str) -> Self {
        Self {
            status,
            decision: proposal.decision,
            diagnosis: proposal.diagnosis,
            rule_yaml: None,
            rule_id: None,
            validation_error: None,
            positive_tests: proposal.positive_tests,
            negative_tests: proposal.negative_tests,
            behavior_expectations: proposal.behavior_expectations,
            open_questions: proposal.open_questions,
        }
    }

    fn invalid(proposal: Proposal, error: String) -> Self {
        Self {
            status: "draft_invalid",
            decision: proposal.decision,
            diagnosis: proposal.diagnosis,
            rule_yaml: None,
            rule_id: None,
            validation_error: Some(error),
            positive_tests: proposal.positive_tests,
            negative_tests: proposal.negative_tests,
            behavior_expectations: proposal.behavior_expectations,
            open_questions: proposal.open_questions,
        }
    }
}

/// Turn the model's `rule` object into candidate YAML: the id is sanitized and
/// de-duplicated against the active rules, `enabled` is forced to false, and
/// the result must pass the strict validator.
fn materialize_rule(rule: &Json, active_rule_ids: &[String]) -> Result<(String, String)> {
    let mut value = serde_yaml::to_value(rule).context("serialize the drafted rule")?;
    let map = value
        .as_mapping_mut()
        .ok_or_else(|| anyhow!("the drafted rule must be a mapping"))?;
    let raw_id = map
        .get(serde_yaml::Value::String("id".into()))
        .and_then(|v| v.as_str())
        .unwrap_or("")
        .to_string();
    let sanitized: String = raw_id
        .to_ascii_lowercase()
        .chars()
        .map(|c| {
            if c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect();
    let mut id = sanitized.trim_matches('_').to_string();
    if id.is_empty() {
        id = "learned_rule".to_string();
    }
    if active_rule_ids.contains(&id) {
        let mut version = 2;
        while active_rule_ids.contains(&format!("{id}_v{version}")) {
            version += 1;
        }
        id = format!("{id}_v{version}");
    }
    map.insert("id".into(), serde_yaml::Value::String(id.clone()));
    map.insert("enabled".into(), serde_yaml::Value::Bool(false));
    let yaml = serde_yaml::to_string(&value).context("render the candidate YAML")?;
    let validated = validate_rule_text(&yaml)?;
    if validated != id {
        bail!("validated id {validated:?} differs from the assigned id {id:?}");
    }
    Ok((yaml, id))
}

/// Pure decision step: parse the reply, materialize and strictly validate the
/// rule. No I/O, so every branch is unit-testable without a model.
fn draft_outcome(active_rule_ids: &[String], reply: &str) -> DraftOutcome {
    let proposal = match parse_proposal(reply) {
        Ok(proposal) => proposal,
        Err(error) => {
            return DraftOutcome::invalid(
                Proposal {
                    decision: "unparsed".into(),
                    diagnosis: String::new(),
                    rule: None,
                    positive_tests: vec![],
                    negative_tests: vec![],
                    behavior_expectations: String::new(),
                    open_questions: String::new(),
                },
                format!("{error:#}"),
            );
        }
    };
    match proposal.decision.as_str() {
        "new_rule" | "revise_rule" => {}
        "no_rule" => return DraftOutcome::no_rule(proposal, "no_rule"),
        "needs_clarification" => return DraftOutcome::no_rule(proposal, "needs_clarification"),
        "unsupported" => return DraftOutcome::no_rule(proposal, "unsupported"),
        other => {
            let error = format!(
                "unknown decision {other:?}; expected new_rule, revise_rule, no_rule, \
                 needs_clarification or unsupported"
            );
            return DraftOutcome::invalid(proposal, error);
        }
    }
    let Some(rule) = proposal.rule.as_ref() else {
        return DraftOutcome::invalid(
            proposal,
            "decision asks for a rule but \"rule\" is null".to_string(),
        );
    };
    match materialize_rule(rule, active_rule_ids) {
        Ok((yaml, id)) => {
            let mut outcome = DraftOutcome::no_rule(proposal, "drafted");
            outcome.rule_yaml = Some(yaml);
            outcome.rule_id = Some(id);
            outcome
        }
        Err(error) => DraftOutcome::invalid(proposal, format!("{error:#}")),
    }
}

// ── Persistence ──────────────────────────────────────────────────────────────

fn now_unix() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

/// FNV-1a: deterministic across processes, so duplicate detection survives a
/// restart (unlike the std hasher).
fn hash_hex(text: &str) -> String {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for byte in text.as_bytes() {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("{hash:016x}")
}

fn append_audit(root: &Path, line: &Json) -> Result<()> {
    use std::io::Write;
    let mut file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(root.join("audit.jsonl"))
        .context("open the correction-learning audit log")?;
    writeln!(file, "{line}").context("append to the correction-learning audit log")?;
    Ok(())
}

/// The proposal id of an earlier `drafted` candidate with the same content
/// key, if one exists.
fn find_duplicate(
    candidates_dir: &Path,
    content_key: &str,
    corrections_dir: Option<&Path>,
) -> Option<String> {
    let entries = std::fs::read_dir(candidates_dir).ok()?;
    for entry in entries.flatten() {
        let Ok(text) = std::fs::read_to_string(entry.path().join("metadata.json")) else {
            continue;
        };
        let Ok(metadata) = serde_json::from_str::<Json>(&text) else {
            continue;
        };
        if metadata.get("content_key").and_then(Json::as_str) == Some(content_key)
            && metadata.get("status").and_then(Json::as_str) == Some("drafted")
        {
            if let Some(active_dir) = corrections_dir {
                let Some(name) = metadata["active_rule_file"].as_str() else {
                    continue;
                };
                if !name.strip_suffix(".yaml").is_some_and(|stem| {
                    !stem.is_empty()
                        && stem
                            .chars()
                            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
                }) || !active_dir.join(name).is_file()
                {
                    continue;
                }
            }
            return metadata
                .get("proposal_id")
                .and_then(Json::as_str)
                .map(str::to_string);
        }
    }
    None
}

fn unique_dir(dir: PathBuf) -> PathBuf {
    if !dir.exists() {
        return dir;
    }
    for n in 2.. {
        let candidate = dir.with_file_name(format!(
            "{}-{n}",
            dir.file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("proposal")
        ));
        if !candidate.exists() {
            return candidate;
        }
    }
    dir
}

fn save_disabled_rule(dir: &Path, outcome: &mut DraftOutcome) -> Result<String> {
    use std::io::Write;

    let yaml = outcome
        .rule_yaml
        .as_deref()
        .context("missing candidate rule")?;
    let base: String = outcome
        .rule_id
        .as_deref()
        .context("missing rule id")?
        .chars()
        .take(96)
        .collect();
    let mut ids = std::collections::HashSet::new();
    for entry in
        std::fs::read_dir(dir).with_context(|| format!("read corrections_dir {}", dir.display()))?
    {
        let path = entry?.path();
        if !path.is_file()
            || !path
                .extension()
                .and_then(|s| s.to_str())
                .is_some_and(|s| matches!(s, "yaml" | "yml"))
        {
            continue;
        }
        let text =
            std::fs::read_to_string(&path).with_context(|| format!("read {}", path.display()))?;
        if let Ok(id) = validate_rule_text(&text) {
            ids.insert(id);
        }
    }
    let mut version = 1u32;
    loop {
        let id = if version == 1 {
            base.clone()
        } else {
            format!("{base}_v{version}")
        };
        let filename = format!("{id}.yaml");
        let target = dir.join(&filename);
        if ids.contains(&id) || target.exists() {
            version = version
                .checked_add(1)
                .context("too many rule id collisions")?;
            continue;
        }
        let mut value: serde_yaml::Value =
            serde_yaml::from_str(yaml).context("read candidate rule")?;
        value
            .as_mapping_mut()
            .context("candidate rule is not a mapping")?
            .insert("id".into(), id.clone().into());
        let text = serde_yaml::to_string(&value).context("render candidate rule")?;
        validate_rule_text(&text)?;
        let temp = dir.join(format!(".{}.tmp", uuid::Uuid::new_v4()));
        let result = (|| -> Result<()> {
            let mut file = std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&temp)
                .with_context(|| format!("create {}", temp.display()))?;
            file.write_all(text.as_bytes())
                .with_context(|| format!("write {}", temp.display()))?;
            std::fs::hard_link(&temp, &target)
                .with_context(|| format!("install {}", target.display()))?;
            Ok(())
        })();
        if let Err(error) = std::fs::remove_file(&temp)
            && error.kind() != std::io::ErrorKind::NotFound
        {
            warn!(
                "[pilot/learning] cannot remove temporary file {}: {error}",
                temp.display()
            );
        }
        result?;
        outcome.rule_id = Some(id);
        outcome.rule_yaml = Some(text);
        return Ok(filename);
    }
}

/// Persist one outcome under the learning root. A `drafted` outcome whose
/// content key matches an earlier drafted candidate is marked `duplicate` and
/// not written again. Returns the proposal id.
fn save_outcome(
    root: &Path,
    corrections_dir: Option<&Path>,
    episode: &Episode,
    model: &str,
    active_rule_ids: &[String],
    outcome: &mut DraftOutcome,
) -> Result<String> {
    let content_key = hash_hex(&format!(
        "{}\n{}",
        outcome.decision,
        outcome
            .rule_yaml
            .clone()
            .unwrap_or_else(|| outcome.diagnosis.clone())
    ));
    let candidates_dir = root.join("candidates");
    let existing = find_duplicate(&candidates_dir, &content_key, corrections_dir);
    if let Some(existing) = existing {
        outcome.status = "duplicate";
        append_audit(
            root,
            &json!({
                "ts": now_unix(), "session_id": episode.session_id,
                "status": "duplicate", "proposal_id": existing,
                "rule_id": outcome.rule_id,
            }),
        )?;
        return Ok(existing);
    }
    let proposal_id = format!("p{}-{}", now_unix(), &content_key[..8]);
    let dir = unique_dir(candidates_dir.join(&proposal_id));
    let proposal_id = dir
        .file_name()
        .and_then(|name| name.to_str())
        .context("invalid candidate directory name")?
        .to_string();
    std::fs::create_dir_all(&dir)
        .with_context(|| format!("create the candidate dir {}", dir.display()))?;
    let active_rule_file = if outcome.rule_yaml.is_some() {
        corrections_dir
            .map(|active| save_disabled_rule(active, outcome))
            .transpose()?
    } else {
        None
    };
    if let Some(yaml) = &outcome.rule_yaml {
        std::fs::write(dir.join("rule.yaml"), yaml).context("write the candidate rule.yaml")?;
    }
    let metadata = json!({
        "proposal_id": proposal_id,
        "status": outcome.status,
        "decision": outcome.decision,
        "created_unix": now_unix(),
        "model": model,
        "rule_id": outcome.rule_id,
        "active_rule_file": active_rule_file,
        "content_key": content_key,
        "validation_error": outcome.validation_error,
        "episode": {
            "session_id": episode.session_id,
            "turn_task_id": episode.turn_task_id,
            "correction_text": episode.correction_text,
            "original_request": episode.original_request,
            "evidence_lines": episode.evidence.len(),
            "captured_at_unix": episode.captured_at_unix,
        },
        "diagnosis": outcome.diagnosis,
        "active_rules_at_draft": active_rule_ids,
        "note": if active_rule_file.is_some() {
            "Disabled draft is in corrections_dir. Review it, enable it, and restart Pilot."
        } else {
            "Disabled draft requires a configured corrections_dir; review, copy, enable, and restart Pilot."
        },
    });
    std::fs::write(
        dir.join("metadata.json"),
        serde_json::to_vec_pretty(&metadata).context("serialize the candidate metadata")?,
    )
    .context("write the candidate metadata.json")?;
    let tests = json!({
        "positive_tests": outcome.positive_tests,
        "negative_tests": outcome.negative_tests,
        "behavior_expectations": outcome.behavior_expectations,
        "open_questions": outcome.open_questions,
    });
    std::fs::write(
        dir.join("tests.json"),
        serde_json::to_vec_pretty(&tests).context("serialize the candidate tests")?,
    )
    .context("write the candidate tests.json")?;
    append_audit(
        root,
        &json!({
            "ts": now_unix(), "session_id": episode.session_id, "status": outcome.status,
            "proposal_id": proposal_id, "rule_id": outcome.rule_id,
        }),
    )?;
    Ok(proposal_id)
}

// ── Episode runner ───────────────────────────────────────────────────────────

async fn run_episode(runtime: &LearningRuntime, episode: Episode) -> Result<()> {
    let prompt = build_prompt(&episode, &runtime.active_rule_ids);
    let mut reply = runtime
        .client
        .chat_once(Some(&runtime.model), ANALYST_SYSTEM, &prompt)
        .await
        .context("the reflection call failed")?;
    let mut outcome = draft_outcome(&runtime.active_rule_ids, &reply);
    let mut attempts = 0;
    while outcome.status == "draft_invalid" && attempts < MAX_REVISIONS {
        attempts += 1;
        let error = outcome
            .validation_error
            .clone()
            .unwrap_or_else(|| "the draft was rejected".into());
        warn!(
            "[pilot/learning] draft rejected (attempt {attempts}): {error}; asking for a revision"
        );
        let revision = format!(
            "{prompt}\n\n## Validation error\nThe previous draft was rejected:\n{error}\n\n\
             Return the corrected JSON object in the same format. Keep the diagnosis, fix the rule."
        );
        match runtime
            .client
            .chat_once(Some(&runtime.model), ANALYST_SYSTEM, &revision)
            .await
        {
            Ok(next_reply) => {
                reply = next_reply;
                outcome = draft_outcome(&runtime.active_rule_ids, &reply);
            }
            Err(error) => {
                warn!("[pilot/learning] revision call failed: {error:#}");
                break;
            }
        }
    }
    if outcome.status == "no_rule" {
        return Ok(());
    }
    let _lock = runtime.persist_lock.lock().await;
    let proposal_id = save_outcome(
        &runtime.cfg.root_dir,
        runtime.corrections_dir.as_deref(),
        &episode,
        &runtime.model,
        &runtime.active_rule_ids,
        &mut outcome,
    )?;
    match outcome.status {
        "drafted" => info!(
            "[pilot/learning] candidate {} (rule {}) saved disabled; records under {}, \
             corrections directory: {}; review it before enabling",
            proposal_id,
            outcome.rule_id.as_deref().unwrap_or("?"),
            runtime.cfg.root_dir.display(),
            runtime.corrections_dir.as_ref().map_or_else(
                || "not configured".to_string(),
                |path| path.display().to_string()
            )
        ),
        "duplicate" => info!(
            "[pilot/learning] episode matches existing candidate {proposal_id}; not duplicated"
        ),
        other => info!(
            "[pilot/learning] episode recorded as {other} (proposal {proposal_id}); no rule drafted"
        ),
    }
    Ok(())
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    #[ignore = "requires a configured live model"]
    async fn live_model_understands_standing_navigation_instruction() {
        let root = tempfile::tempdir().unwrap();
        std::fs::create_dir(root.path().join("active")).unwrap();
        let vlm = crate::config::VlmConfig {
            upstream: std::env::var("ROBONIX_TEST_VLM_BASE_URL").unwrap(),
            api_key: std::env::var("ROBONIX_TEST_VLM_API_KEY").unwrap(),
            model: std::env::var("ROBONIX_TEST_VLM_MODEL").unwrap(),
            reasoning_effort: None,
            catalog_id: None,
            context_window_tokens: None,
            api_format: "openai".into(),
        };
        let runtime = LearningRuntime::new(
            LearningConfig {
                root_dir: root.path().into(),
                reflection_upstream: None,
                reflection_api_key: None,
                reflection_model: None,
            },
            &vlm,
            vec![],
            Some(root.path().join("active")),
        );
        let episode = Episode {
            session_id: "test".into(), turn_task_id: "test".into(),
            correction_text: "客厅的默认坐标会导致你导航出错，以后我让你去客厅的时候请你拒绝并提醒我给出具体的可达坐标".into(), // i18n-ok
            original_request: Some("去客厅".into()), evidence: vec![], captured_at_unix: 1, // i18n-ok
        };
        run_episode(&runtime, episode).await.unwrap();
        let candidates = std::fs::read_dir(root.path().join("candidates"))
            .unwrap()
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        assert_eq!(candidates.len(), 1);
        let rule = std::fs::read_to_string(candidates[0].path().join("rule.yaml")).unwrap();
        validate_rule_text(&rule).unwrap();
        assert!(rule.contains("require_user_coordinates"), "{rule}");
        let notices = runtime.pending_notices("test");
        assert_eq!(notices.len(), 1);
        assert!(
            notices[0]
                .1
                .contains(root.path().join("active").to_str().unwrap())
        );
        assert!(notices[0].1.contains("restart Pilot"));
        let client = VlmClient::new(&vlm);
        let instruction = "When asked to navigate to the living room, request explicit map coordinates; do not use its default destination.";
        assert!(
            crate::corrections::coordinate_restriction_applies(&client, "请回客厅", instruction) // i18n-ok
                .await
        );
        assert!(
            !crate::corrections::coordinate_restriction_applies(
                &client,
                "我在客厅，请去厨房", // i18n-ok
                instruction
            )
            .await
        );
    }

    fn user_message(n: u32, text: &str) -> Message {
        Message::user(&format!(
            "User message m{n} (filed under T1): {text}\n{RESPONSE_MODE_TEXT}"
        ))
    }

    fn leaf_feedback(contract: &str, success: bool, output: &str, error: &str) -> Message {
        let feedback = json!({
            "leaf_result": {
                "call_id": "p-1:0", "contract_id": contract, "success": success,
                "output": output, "error": error,
            }
        });
        Message::user(&format!("{LEAF_FEEDBACK_PREFIX}{feedback}"))
    }

    fn dispatch_record() -> Message {
        Message::user(&format!(
            "{DISPATCH_RECORD_PREFIX} (plan node definitions sent to Executor; listed calls may not have run): {}",
            json!({
                "plan_id": "9fd9-1", "description": "go near the plant",
                "calls": [
                    {"call_id": "9fd9-1:0", "contract_id": "robonix/system/scene/goal_near",
                     "args": {"object_id": "scene.object.0002"}},
                    {"call_id": "9fd9-1:1", "contract_id": "robonix/service/navigation/navigate",
                     "args": {"goal": {"pose": {"position": {"x": 2.0, "y": 0.0}}}}},
                ],
            })
        ))
    }

    #[test]
    fn episode_captures_the_original_request_and_bounded_evidence() {
        let mut history = vec![
            user_message(1, "去那盆植物旁边"), // i18n-ok
            dispatch_record(),
            leaf_feedback(
                "robonix/system/scene/goal_near",
                false,
                "",
                "no free cell within 1.5 m of scene.object.0002",
            ),
            leaf_feedback("robonix/service/navigation/navigate", true, "done", ""),
            user_message(2, "你去错了，我说的是植物旁边"), // i18n-ok
        ];
        let episode = build_episode(&history, "s1", "t1", "你去错了，我说的是植物旁边").unwrap(); // i18n-ok
        assert_eq!(episode.original_request.as_deref(), Some("去那盆植物旁边")); // i18n-ok
        assert_eq!(episode.evidence.len(), 3);
        assert!(episode.evidence[0].starts_with("dispatch plan 9fd9-1"));
        assert!(episode.evidence[1].contains("goal_near success=false"));
        assert!(episode.evidence[1].contains("no free cell"));
        assert!(episode.evidence[2].contains("navigate success=true"));

        // A standing instruction can be given before any task.
        history.remove(0);
        assert!(
            build_episode(&history, "s1", "t1", "你去错了") // i18n-ok
                .unwrap()
                .original_request
                .is_none()
        );

        // Boards and assistant text carry no evidence.
        let history = vec![
            user_message(1, "去那盆植物旁边"), // i18n-ok
            Message::user("Tasks: T1 in_progress"),
            Message::assistant("going"),
            user_message(2, "你去错了"), // i18n-ok
        ];
        let episode = build_episode(&history, "s1", "t1", "你去错了").unwrap(); // i18n-ok
        assert!(episode.evidence.is_empty());
    }

    fn new_rule_reply(id: &str, contract: &str) -> String {
        json!({
            "decision": "new_rule",
            "diagnosis": "navigated after goal_near reported no free cell",
            "rule": {
                "apiVersion": "robonix.pilot/correction/v1",
                "id": id,
                "description": "draft",
                "enabled": true,
                "scope": "current_message",
                "when": {"any": [{"leaf": {
                    "contract": contract, "success": false,
                    "error": {"regex": "no free cell within [\\d.]+ m of (?P<object_id>[\\w.]+)"},
                }}]},
                "window": {"max_rounds": 3},
                "actions": [{"kind": "inject_message", "template":
                    "goal_near found no free cell near {{object_id}}; do not navigate."}],
            },
            "positive_tests": ["goal_near fails with no free cell"],
            "negative_tests": ["goal_near returns a pose"],
            "behavior_expectations": "report the blocker",
            "open_questions": "",
        })
        .to_string()
    }

    #[test]
    fn drafted_rules_are_forced_disabled_sanitized_and_validated() {
        let active = vec!["goal_near_unreachable".to_string()];
        let outcome = draft_outcome(
            &active,
            &new_rule_reply("goal_near_unreachable", "robonix/system/scene/goal_near"),
        );
        assert_eq!(outcome.status, "drafted");
        // The id collided with an active rule and was de-duplicated.
        assert_eq!(outcome.rule_id.as_deref(), Some("goal_near_unreachable_v2"));
        let yaml = outcome.rule_yaml.as_deref().unwrap();
        assert!(yaml.contains("enabled: false"));

        let outcome = draft_outcome(
            &active,
            &new_rule_reply("Weird ID!!", "robonix/system/scene/goal_near"),
        );
        assert_eq!(outcome.status, "drafted");
        assert_eq!(outcome.rule_id.as_deref(), Some("weird_id"));

        // A broken regex must not survive strict validation, and one revision
        // is allowed before the episode is recorded as invalid. A lookahead
        // is invalid in Rust's regex dialect.
        let broken = new_rule_reply("bad_regex", "robonix/system/scene/goal_near")
            .replace("no free cell", "no free cell (?=");
        let outcome = draft_outcome(&active, &broken);
        assert_eq!(outcome.status, "draft_invalid");
        assert!(
            outcome
                .validation_error
                .as_deref()
                .unwrap()
                .contains("regex")
        );

        // Decisions that draft nothing.
        for (decision, status) in [
            ("no_rule", "no_rule"),
            ("needs_clarification", "needs_clarification"),
            ("unsupported", "unsupported"),
        ] {
            let reply = json!({"decision": decision, "diagnosis": "environmental"}).to_string();
            assert_eq!(draft_outcome(&active, &reply).status, status);
        }
        // new_rule without a rule object is invalid, not silently accepted.
        let reply = json!({"decision": "new_rule", "diagnosis": "x"}).to_string();
        assert_eq!(draft_outcome(&active, &reply).status, "draft_invalid");
        // Not JSON at all.
        assert_eq!(
            draft_outcome(&active, "I think it went wrong").status,
            "draft_invalid"
        );
        // A fenced reply still parses.
        let fenced = format!(
            "```json\n{}\n```",
            new_rule_reply("fenced", "robonix/system/scene/goal_near")
        );
        assert_eq!(draft_outcome(&active, &fenced).status, "drafted");
    }

    #[test]
    fn outcomes_are_saved_as_disabled_candidates_with_audit_and_dedup() {
        let root = std::env::temp_dir().join(format!("pilot-learning-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&root).unwrap();
        let episode = Episode {
            session_id: "s1".into(),
            turn_task_id: "t1".into(),
            correction_text: "你去错了".into(), // i18n-ok
            original_request: Some("去那盆植物旁边".into()), // i18n-ok
            evidence: vec!["leaf goal_near success=false".into()],
            captured_at_unix: 1,
        };
        let active = vec!["goal_near_unreachable".to_string()];
        let mut outcome = draft_outcome(
            &active,
            &new_rule_reply("plant_no_free_cell", "robonix/system/scene/goal_near"),
        );
        let first =
            save_outcome(&root, None, &episode, "test-model", &active, &mut outcome).unwrap();
        let candidate = root.join("candidates").join(&first);
        assert!(candidate.join("rule.yaml").is_file());
        let yaml = std::fs::read_to_string(candidate.join("rule.yaml")).unwrap();
        assert!(yaml.contains("enabled: false"));
        let metadata: Json = serde_json::from_str(
            &std::fs::read_to_string(candidate.join("metadata.json")).unwrap(),
        )
        .unwrap();
        assert_eq!(metadata["status"], "drafted");
        assert_eq!(metadata["model"], "test-model");
        assert!(candidate.join("tests.json").is_file());
        assert!(root.join("audit.jsonl").is_file());

        // The same draft again is a duplicate, not a second candidate.
        let mut repeat = draft_outcome(
            &active,
            &new_rule_reply("plant_no_free_cell", "robonix/system/scene/goal_near"),
        );
        assert_eq!(
            save_outcome(&root, None, &episode, "test-model", &active, &mut repeat).unwrap(),
            first
        );
        assert_eq!(repeat.status, "duplicate");
        let count = std::fs::read_dir(root.join("candidates")).unwrap().count();
        assert_eq!(count, 1);

        // A no_rule outcome records the episode without a rule file.
        let mut none = draft_outcome(
            &active,
            r#"{"decision":"no_rule","diagnosis":"file missing"}"#,
        );
        let second = save_outcome(&root, None, &episode, "test-model", &active, &mut none).unwrap();
        assert!(
            !root
                .join("candidates")
                .join(&second)
                .join("rule.yaml")
                .exists()
        );

        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn draft_is_saved_disabled_in_active_directory_without_overwriting_existing_rules() {
        let root = tempfile::tempdir().unwrap();
        let active_dir = root.path().join("active");
        std::fs::create_dir(&active_dir).unwrap();
        let reply = new_rule_reply("living_room", "robonix/system/scene/goal_near");
        let existing = draft_outcome(&[], &reply)
            .rule_yaml
            .unwrap()
            .replace("enabled: false", "enabled: true");
        std::fs::write(active_dir.join("user_rule.yaml"), &existing).unwrap();
        let episode = Episode {
            session_id: "s1".into(),
            turn_task_id: "t1".into(),
            correction_text: "test".into(),
            original_request: None,
            evidence: vec![],
            captured_at_unix: 1,
        };
        let mut outcome = draft_outcome(&[], &reply);
        let proposal = save_outcome(
            root.path(),
            Some(&active_dir),
            &episode,
            "test",
            &[],
            &mut outcome,
        )
        .unwrap();
        let id = outcome.rule_id.as_deref().unwrap();
        assert_eq!(id, "living_room_v2");
        assert_eq!(
            std::fs::read_to_string(active_dir.join("user_rule.yaml")).unwrap(),
            existing
        );
        let active_file = active_dir.join(format!("{id}.yaml"));
        let content = std::fs::read_to_string(&active_file).unwrap();
        assert!(content.contains("enabled: false"));
        assert_eq!(
            content,
            std::fs::read_to_string(
                root.path()
                    .join("candidates")
                    .join(proposal)
                    .join("rule.yaml")
            )
            .unwrap()
        );
        let runtime = LearningRuntime::new(
            LearningConfig {
                root_dir: root.path().into(),
                reflection_upstream: None,
                reflection_api_key: None,
                reflection_model: None,
            },
            &crate::config::VlmConfig {
                upstream: "http://127.0.0.1:1".into(),
                api_key: String::new(),
                model: "test".into(),
                reasoning_effort: None,
                catalog_id: None,
                context_window_tokens: None,
                api_format: "openai".into(),
            },
            vec![],
            Some(active_dir.clone()),
        );
        let notice = &runtime.pending_notices("s1")[0].1;
        assert!(notice.contains(active_file.to_str().unwrap()));
        assert!(notice.contains("restart Pilot"));
        assert!(!notice.contains("Copy"));
        let loaded = crate::corrections::CorrectionSet::load(&active_dir).unwrap();
        assert_eq!(loaded.ids(), vec!["living_room"]);
        std::fs::write(
            &active_file,
            content.replace("enabled: false", "enabled: true"),
        )
        .unwrap();
        let reloaded = crate::corrections::CorrectionSet::load(&active_dir).unwrap();
        assert!(reloaded.ids().contains(&"living_room_v2"));
        let mut duplicate = draft_outcome(&[], &reply);
        save_outcome(
            root.path(),
            Some(&active_dir),
            &episode,
            "test",
            &[],
            &mut duplicate,
        )
        .unwrap();
        assert_eq!(duplicate.status, "duplicate");
        assert_eq!(std::fs::read_dir(&active_dir).unwrap().count(), 2);
    }

    #[test]
    fn a_legacy_duplicate_without_an_active_file_gets_an_active_copy() {
        let root = tempfile::tempdir().unwrap();
        let active_dir = root.path().join("active");
        std::fs::create_dir(&active_dir).unwrap();
        let reply = new_rule_reply("legacy", "robonix/system/scene/goal_near");
        let episode = Episode {
            session_id: "s".into(),
            turn_task_id: "t".into(),
            correction_text: "test".into(),
            original_request: None,
            evidence: vec![],
            captured_at_unix: 1,
        };
        let mut old = draft_outcome(&[], &reply);
        let old_id = save_outcome(root.path(), None, &episode, "test", &[], &mut old).unwrap();
        let mut fresh = draft_outcome(&[], &reply);
        let new_id = save_outcome(
            root.path(),
            Some(&active_dir),
            &episode,
            "test",
            &[],
            &mut fresh,
        )
        .unwrap();
        assert_ne!(old_id, new_id);
        assert!(active_dir.join("legacy.yaml").is_file());
        let mut again = draft_outcome(&[], &reply);
        let duplicate = save_outcome(
            root.path(),
            Some(&active_dir),
            &episode,
            "test",
            &[],
            &mut again,
        )
        .unwrap();
        assert_eq!(duplicate, new_id);
        assert_eq!(again.status, "duplicate");
        assert_eq!(std::fs::read_dir(&active_dir).unwrap().count(), 1);
    }

    #[test]
    fn the_prompt_carries_the_schema_constraints_and_evidence() {
        let episode = Episode {
            session_id: "s1".into(),
            turn_task_id: "t1".into(),
            correction_text: "你去错了".into(), // i18n-ok
            original_request: Some("去那盆植物旁边".into()), // i18n-ok
            evidence: vec!["leaf goal_near success=false".into()],
            captured_at_unix: 1,
        };
        let prompt = build_prompt(&episode, &["goal_near_unreachable".to_string()]);
        for needle in [
            "robonix.pilot/correction/v1",
            "must be false",
            "Rust regex",
            "goal_near_unreachable",
            "去那盆植物旁边", // i18n-ok
            "leaf goal_near success=false",
            "no lookahead",
            "never auto-enabled",
        ] {
            assert!(prompt.contains(needle), "prompt lacks {needle:?}");
        }
    }
}
