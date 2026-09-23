// SPDX-License-Identifier: MulanPSL-2.0
//! Independent complete-response expansion retained as a test oracle.
use super::*;
use crate::planner::{RtdlEnvelope, parse_task_update};

/// Expand a validated complete RTDL tree into the established preorder Plan arena.
pub(crate) fn expand_rtdl_to_plan(
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
pub(crate) fn pick_description(
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
pub(crate) fn reject_unknown_node_keys(
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

/// Validate one complete node and recursively append its descendants and call identities.
pub(crate) fn expand_rtdl_node(
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
pub(crate) fn extract_json_object(raw: &str) -> Option<&str> {
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
pub(crate) fn parse_rtdl_assistant_response(raw: &str) -> Result<RtdlEnvelope> {
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
