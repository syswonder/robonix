// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// RTDL: the protocol text, parsing the model's reply, and expanding it into an
// Executor plan.

use super::*;

pub(super) const RTDL_SEQUENCE: u32 = 0;
pub(super) const RTDL_PARALLEL: u32 = 1;
pub(super) const RTDL_DO: u32 = 2;

/// Compact contract reminder for rounds after the first. The complete frozen
/// protocol is sent on round zero; later requests keep only the wire grammar
/// and harness invariants that are required to parse and admit the next plan.
pub(super) const RTDL_PROTOCOL_REMINDER: &str = r#"## RTDL output (same frozen contract as round 0)
Reply with exactly one JSON object and no surrounding prose:
{"content":"...","rtdl_description":"...","rtdl":<node>,"task_update":null|{"goal":"...","success_criterion":"...","status":"in_progress"|"done"}}

Nodes are exactly one of:
- {"op":"sequence","op_id":0,"description":"...","children":[...]}
- {"op":"parallel","op_id":0,"description":"...","children":[...]}
- {"op":"do","op_id":0,"description":"...","cap":"<exact capability_name>","args":{...}}
Write op_id=0; Pilot assigns the real ID. Do not add node fields. Compose every currently-known dependent step into one sequence and every independent step into one parallel tree; do not drip one known call per round. With no new call, return an empty sequence.

Plan control is the entire rtdl value, never a nested node or capability: cancel_plan, cancel_all, or stop_plan_at using an exact listed plan_id/op_id. Never repeat a completed/cancelled control operation and never cancel an unrelated tree after another tree fails.

Resolve a named room through current Scene regions before navigation; never use a remembered grasp or observation pose as a room goal. A navigation SUCCEEDED result proves only the resolved requested destination; a zero-distance result does not prove that the robot moved. Never call a skill's cancel capability; Executor propagates RTDL cancellation.

Copy each cap exactly from the current catalog. Executor feedback and dispatch records are scoped by plan_id/call_id and prove what was already sent; do not repeat a successful call. task_update.goal must exactly copy Current user interaction. Use status=done only when its success criterion is verified and its relevant tree is no longer running.
"#;

pub(super) fn rtdl_protocol(full: bool) -> &'static str {
    if full {
        include_str!("../../rtdl_protocol.md")
    } else {
        RTDL_PROTOCOL_REMINDER
    }
}

/// One parsed RTDL envelope from the VLM.
#[derive(Debug)]
pub(super) struct RtdlEnvelope {
    /// User-facing narration.
    pub(super) content: String,
    /// Short label for the dispatched tree (sub-task name); may be empty only
    /// when `rtdl` is an empty sequence.
    pub(super) rtdl_description: String,
    /// The declarative ops tree to dispatch this round.
    pub(super) rtdl: serde_json::Value,
    /// Overall-task update. `None` means "keep the current task unchanged"
    /// (envelope `task_update: null`).
    pub(super) task_update: Option<TaskState>,
}

/// Parses one VLM reply in RTDL envelope form.
///
/// The model must emit a JSON **object** whose only keys are exactly
/// `content`, `rtdl_description`, `rtdl`, and `task_update`. See
/// `rtdl_protocol.md` for the field contract.
///
/// Extract the first balanced top-level JSON object from `raw`, ignoring any
/// prose before or after it (e.g. a narration line the model emitted before the
/// JSON, or a trailing comment). Returns the `{...}` slice, or `None` if there
/// is no `{` or no matching close brace.
///
/// Brace depth is counted only outside JSON string literals, so braces inside
/// strings don't affect it. Scanning by bytes is UTF-8-safe here because `{`,
/// `}`, `"`, and `\` are all ASCII and never collide with multibyte
/// continuation bytes (which are all >= 0x80).
pub(super) fn extract_json_object(raw: &str) -> Option<&str> {
    let bytes = raw.as_bytes();
    let start = bytes.iter().position(|&b| b == b'{')?;
    let mut depth = 0usize;
    let mut in_str = false;
    let mut escaped = false;
    for (i, &b) in bytes.iter().enumerate().skip(start) {
        if in_str {
            if escaped {
                escaped = false;
            } else if b == b'\\' {
                escaped = true;
            } else if b == b'"' {
                in_str = false;
            }
            continue;
        }
        match b {
            b'"' => in_str = true,
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(&raw[start..=i]);
                }
            }
            _ => {}
        }
    }
    None
}

/// Tolerates a prose preamble or trailing commentary around the JSON object
/// (a common model habit, e.g. a narration line then the JSON on the next
/// line) by extracting the first balanced `{...}` before parsing; the raw
/// string is used unchanged when no object is found, so a genuinely
/// JSON-less reply still surfaces the original parse error.
///
/// Fails if `raw` is not valid JSON, the root is not an object, the key set is
/// not exactly those four, `content` / `rtdl_description` are not strings,
/// `rtdl` is not an object, or `task_update` is neither `null` nor a valid task
/// object.
pub(super) fn parse_rtdl_assistant_response(raw: &str) -> Result<RtdlEnvelope> {
    let candidate = extract_json_object(raw).unwrap_or(raw);
    let v: serde_json::Value = serde_json::from_str(candidate)?;
    let obj = v
        .as_object()
        .ok_or_else(|| anyhow::anyhow!("assistant response must be a JSON object"))?;
    const KEYS: [&str; 4] = ["content", "rtdl_description", "rtdl", "task_update"];
    if obj.len() != KEYS.len() || !KEYS.iter().all(|k| obj.contains_key(*k)) {
        anyhow::bail!(
            "assistant response must contain exactly `content`, `rtdl_description`, `rtdl`, and `task_update`"
        );
    }
    let content = obj
        .get("content")
        .and_then(|x| x.as_str())
        .ok_or_else(|| anyhow::anyhow!("assistant `content` must be a string"))?
        .to_string();
    let rtdl_description = obj
        .get("rtdl_description")
        .and_then(|x| x.as_str())
        .ok_or_else(|| anyhow::anyhow!("assistant `rtdl_description` must be a string"))?
        .to_string();
    let rtdl = obj
        .get("rtdl")
        .filter(|x| x.is_object())
        .ok_or_else(|| anyhow::anyhow!("assistant `rtdl` must be an object"))?
        .clone();
    let task_update = match obj.get("task_update") {
        None | Some(serde_json::Value::Null) => None,
        Some(v) => Some(parse_task_update(v)?),
    };
    Ok(RtdlEnvelope {
        content,
        rtdl_description,
        rtdl,
        task_update,
    })
}

pub(super) fn raw_preview(raw: &str) -> String {
    if raw.is_empty() {
        return "assistant content was empty".to_string();
    }
    let preview: String = raw.chars().take(240).collect();
    let ellipsis = if raw.chars().count() > 240 { "..." } else { "" };
    format!("assistant content preview: {preview:?}{ellipsis}")
}

// ── RTDL recovery (merged from dev #88) ────────────────────────────────────────
// When the VLM emits an RTDL that fails to parse or expand, feed the error back
// once and let it self-correct; a second failure ends the turn gracefully.

/// Corrective prompt appended to the next VLM round after a parse/expand failure.
pub(super) fn build_rtdl_retry_prompt(
    err: &anyhow::Error,
    raw_content: &str,
    display_caps: &[DisplayCapability<'_>],
) -> String {
    let mut p = format!(
        "Your previous RTDL response could not be parsed or expanded by Pilot.\n\
         Error: {err:#}\n\
         Previous response preview: {}\n\n\
         Fix the RTDL error and retry the same user request exactly once. If the error \
         mentions an unknown capability, do not repeat that capability. Return ONLY a JSON object \
         with exactly `content`, `rtdl_description`, `rtdl`, and `task_update`. The reply MUST begin \
         with `{{` and end with `}}`: no prose, narration, or markdown fences before or after it (put \
         any user-facing text inside `content`). Use only \
         capability_name values from this list; do not invent provider names, method names, or \
         aliases:\n",
        raw_preview(raw_content)
    );
    for cap in display_caps {
        p.push_str("- ");
        p.push_str(&cap.display_name);
        p.push('\n');
    }
    // A truncated reply and a malformed one read the same to a parser, but the
    // model can only act on the first if it is told which it was. deepseek-v3.2
    // copied the task text back into `task_update.goal` every round, hit the
    // output ceiling and had its JSON cut mid-string; "fix the RTDL error" gave
    // it nothing to change, and it repeated the same reply.
    let truncated = format!("{err:#}").contains("EOF while parsing");
    if truncated {
        p.push_str(
            "\nYour reply was cut off because it ran past the output limit. Do not \
             restate the task, the action catalogue, or any other prompt text: \
             `task_update.goal` and `success_criterion` are one-line summaries, \
             a few dozen characters each. Keep the whole reply short.\n",
        );
    }
    p.push_str(
        "\nIf no further capability call is needed, use \
         {\"op\":\"sequence\",\"children\":[]} as `rtdl`. If the user's requested action cannot \
         be performed using the listed capabilities, explain the missing capability in `content` \
         and return an empty RTDL sequence instead of inventing a capability.\n",
    );
    p
}

/// A single empty-sequence root plan, used as the no-op plan when a turn ends in
/// RTDL recovery. Carries non-empty `op_id`/`description` so executor's
/// `validate_plan` accepts it.
pub(super) fn empty_sequence_plan(plan_id: String, session_id: String, round: u32) -> Plan {
    Plan {
        plan_id,
        session_id,
        round,
        nodes: vec![RtdlNode {
            node_kind: RTDL_SEQUENCE,
            children: Vec::new(),
            call: None,
            op_id: "recovery".to_string(),
            description: "recovery: no valid plan produced".to_string(),
        }],
        root_index: 0,
    }
}

/// User-facing message when RTDL recovery gives up — never leaks the internal error.
pub(super) fn rtdl_recovery_final_text() -> String {
    "I couldn't produce a valid robot plan after retrying once. Please try again or rephrase the request."
        .to_string()
}

/// Process-wide monotonic source of node `op_id`s. The LLM-emitted RTDL does
/// not carry a usable op_id (it defaults to 0), so pilot assigns one itself
/// while parsing: a globally-unique, auto-incrementing id starting at 1.
/// "Global" = across every plan/round in this pilot process, so each node in
/// the live task-graph forest is uniquely addressable for steering and result
/// correlation — not merely unique within one plan.
pub(super) static OP_ID_SEQ: AtomicU64 = AtomicU64::new(0);

/// Allocate the next global op_id (1, 2, 3, …) as a decimal string.
pub(super) fn next_op_id() -> String {
    (OP_ID_SEQ.fetch_add(1, Ordering::Relaxed) + 1).to_string()
}

pub(super) fn expand_rtdl_to_plan(
    rtdl: &serde_json::Value,
    target_map: &CapabilityTargetMap,
    plan_id: String,
    session_id: String,
    round: u32,
    root_description: &str,
) -> Result<Plan> {
    let mut nodes = Vec::new();
    let mut next_call = 0usize;
    let root_index = expand_rtdl_node(
        rtdl,
        "$",
        target_map,
        plan_id.as_str(),
        root_description,
        &mut next_call,
        &mut nodes,
    )?;
    Ok(Plan {
        plan_id,
        session_id,
        round,
        nodes,
        root_index,
    })
}

/// Pick a node's `description`, in priority order:
/// 1. the LLM's own per-node `description` field when present and non-empty;
/// 2. the LLM's tree label (`rtdl_description`) for an otherwise-unlabelled root;
/// 3. a synthesized fallback (e.g. `call camera_snapshot`).
///
/// The model is asked to author a node-level `description` for every node (see
/// `rtdl_protocol.md`); the fallbacks keep a sloppy or older reply from failing
/// the turn, since executor's `validate_plan` requires a non-empty description.
pub(super) fn pick_description(
    obj: &serde_json::Map<String, serde_json::Value>,
    path: &str,
    root_description: &str,
    synthesized: String,
) -> String {
    if let Some(d) = obj.get("description").and_then(|x| x.as_str()) {
        let d = d.trim();
        if !d.is_empty() {
            return d.to_string();
        }
    }
    if path == "$" && !root_description.is_empty() {
        return root_description.to_string();
    }
    synthesized
}

/// Reject node fields outside the allowed set and require the structural ones.
///
/// `required` lists the keys an operator must carry beyond `op` (e.g.
/// `children` for sequence/parallel; `cap` + `args` for do). `op_id` and
/// `description` are always optional — the model emits them (op_id defaults to
/// 0, which pilot ignores and reassigns), but a reply that omits them still
/// parses. Any other key (`out`, `id`, `plan_id`, …) is an error.
pub(super) fn reject_unknown_node_keys(
    obj: &serde_json::Map<String, serde_json::Value>,
    path: &str,
    op: &str,
    required: &[&str],
) -> Result<()> {
    const OPTIONAL: [&str; 2] = ["op_id", "description"];
    for key in obj.keys() {
        let known =
            key == "op" || required.contains(&key.as_str()) || OPTIONAL.contains(&key.as_str());
        if !known {
            anyhow::bail!("{path}: {op} node has unexpected field `{key}`");
        }
    }
    for req in required {
        if !obj.contains_key(*req) {
            anyhow::bail!("{path}: {op} node must contain `{req}`");
        }
    }
    Ok(())
}

pub(super) fn expand_rtdl_node(
    node: &serde_json::Value,
    path: &str,
    target_map: &CapabilityTargetMap,
    plan_id: &str,
    root_description: &str,
    next_call: &mut usize,
    nodes: &mut Vec<RtdlNode>,
) -> Result<u32> {
    let obj = node
        .as_object()
        .ok_or_else(|| anyhow::anyhow!("{path}: RTDL node must be an object"))?;
    let op = obj
        .get("op")
        .and_then(|x| x.as_str())
        .ok_or_else(|| anyhow::anyhow!("{path}.op must be a string"))?;

    match op {
        "sequence" | "parallel" => {
            reject_unknown_node_keys(obj, path, op, &["children"])?;
            let children = obj
                .get("children")
                .and_then(|x| x.as_array())
                .ok_or_else(|| anyhow::anyhow!("{path}.children must be an array"))?;
            let node_index = nodes.len() as u32;
            let node_kind = if op == "sequence" {
                RTDL_SEQUENCE
            } else {
                RTDL_PARALLEL
            };
            let description = pick_description(
                obj,
                path,
                root_description,
                format!("{op} of {} step(s)", children.len()),
            );
            nodes.push(RtdlNode {
                node_kind,
                children: Vec::new(),
                call: None,
                op_id: next_op_id(),
                description,
            });
            let mut child_indices = Vec::with_capacity(children.len());
            for (idx, child) in children.iter().enumerate() {
                let child_index = expand_rtdl_node(
                    child,
                    &format!("{path}.children[{idx}]"),
                    target_map,
                    plan_id,
                    root_description,
                    next_call,
                    nodes,
                )?;
                child_indices.push(child_index);
            }
            nodes[node_index as usize].children = child_indices;
            Ok(node_index)
        }
        "do" => {
            reject_unknown_node_keys(obj, path, op, &["cap", "args"])?;
            let cap = obj
                .get("cap")
                .and_then(|x| x.as_str())
                .ok_or_else(|| anyhow::anyhow!("{path}.cap must be a string"))?;
            let args = obj
                .get("args")
                .filter(|x| x.is_object())
                .ok_or_else(|| anyhow::anyhow!("{path}.args must be an object"))?;
            let (provider_id, contract_id) = target_map
                .get(cap)
                .cloned()
                .ok_or_else(|| anyhow::anyhow!("{path}.cap unknown capability `{cap}`"))?;
            let call_index = *next_call;
            *next_call += 1;
            let node_index = nodes.len() as u32;
            let description = pick_description(obj, path, root_description, format!("call {cap}"));
            nodes.push(RtdlNode {
                node_kind: RTDL_DO,
                children: Vec::new(),
                call: Some(CapabilityCall {
                    call_id: format!("{plan_id}:{call_index}"),
                    provider_id,
                    contract_id,
                    args_json: serde_json::to_string(args)?,
                }),
                op_id: next_op_id(),
                description,
            });
            Ok(node_index)
        }
        other => anyhow::bail!("{path}.op unknown operator `{other}`"),
    }
}

pub(super) fn plan_call_count(plan: &Plan) -> usize {
    plan.nodes
        .iter()
        .filter(|node| node.node_kind == RTDL_DO && node.call.is_some())
        .count()
}
