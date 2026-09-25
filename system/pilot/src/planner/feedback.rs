// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Execution feedback: writing results back into history, and the plan and node
// logs.

use super::*;

/// Feed finalized leaf results into LLM history.
pub(super) fn feed_results_into_history(
    history: &mut Vec<Message>,
    plan_id: &str,
    plan_description: &str,
    results: &[CapabilityCallResult],
) {
    history.push(Message::user(&format!(
        "Executor feedback scope: plan_id={plan_id}, independent RTDL tree={plan_description:?}. \
         Attribute the following results only to this tree. A failure here blocks dependent \
         steps in this tree, but does not cancel or invalidate other in-flight trees."
    )));
    let mut deferred_followups = Vec::new();
    for r in results {
        let mut bounded = r.clone();
        if !history::is_image_output(&bounded.output) {
            bounded.output = compact_tool_result(&bounded.contract_id, &bounded.output, 4096);
        }
        let mapped = rtdl_result_to_messages(&bounded);
        history.extend(mapped.tool_messages);
        deferred_followups.extend(mapped.followup_messages);
    }
    history.extend(deferred_followups);
}

/// Persist the exact capability calls handed to Executor so a later planning
/// round can correlate terminal results with work it already dispatched.
///
/// RTDL is a custom planning protocol rather than an OpenAI tool call, so the
/// model's structured plan is otherwise lost when only `content` is appended to
/// chat history. That made a successful physical step look unexecuted on the
/// mandatory post-PlanDone round and allowed the same user step to be planned a
/// second time from the robot's new state.
pub(super) fn record_dispatched_plan(history: &mut Vec<Message>, plan: &Plan, description: &str) {
    let calls: Vec<serde_json::Value> = plan
        .nodes
        .iter()
        .filter_map(|node| {
            let call = node.call.as_ref()?;
            let args = serde_json::from_str::<serde_json::Value>(&call.args_json)
                .unwrap_or_else(|_| serde_json::Value::String(call.args_json.clone()));
            Some(serde_json::json!({
                "call_id": call.call_id,
                "op_id": node.op_id,
                "step": node.description,
                "provider_id": call.provider_id,
                "contract_id": call.contract_id,
                "args": args,
            }))
        })
        .collect();
    if calls.is_empty() {
        return;
    }
    let record = serde_json::json!({
        "plan_id": plan.plan_id,
        "description": description,
        "calls": calls,
    });
    history.push(Message::user(&format!(
        "Pilot harness dispatch record (already sent to Executor; not a new user request): {record}"
    )));
}

/// Render an RTDL node state as a stable human-readable name for logs.
pub(super) fn rtdl_state_name(state: u32) -> String {
    match RtdlNodeStateEnum::try_from(state as i32) {
        Ok(RtdlNodeStateEnum::Pending) => "Pending".to_string(),
        Ok(RtdlNodeStateEnum::Running) => "Running".to_string(),
        Ok(RtdlNodeStateEnum::Succeeded) => "Succeeded".to_string(),
        Ok(RtdlNodeStateEnum::Failed) => "Failed".to_string(),
        Ok(RtdlNodeStateEnum::Canceled) => "Canceled".to_string(),
        Ok(RtdlNodeStateEnum::Timeout) => "Timeout".to_string(),
        Ok(RtdlNodeStateEnum::Paused) => "Paused".to_string(),
        Ok(RtdlNodeStateEnum::Verifying) => "Verifying".to_string(),
        Err(_) => format!("Unknown({state})"),
    }
}

/// Render an RTDL node kind as the tree operator name used in plan logs.
pub(super) fn rtdl_node_kind_name(kind: u32) -> String {
    match kind {
        RTDL_SEQUENCE => "sequence".to_string(),
        RTDL_PARALLEL => "parallel".to_string(),
        RTDL_DO => "do".to_string(),
        _ => format!("unknown({kind})"),
    }
}

/// Shorten free-form payloads so one log event stays readable on one line.
pub(super) fn compact_preview(value: &str, max_chars: usize) -> String {
    let flattened = value.replace('\n', "\\n");
    let mut preview: String = flattened.chars().take(max_chars).collect();
    if flattened.chars().count() > max_chars {
        preview.push_str("...");
    }
    preview
}

/// Bound a tool result without turning structured JSON into an invalid prefix.
/// Scene list contracts keep the identifiers needed for a targeted follow-up
/// while explicitly reporting whether any records were omitted.
pub(super) fn compact_tool_result(contract_id: &str, value: &str, max_chars: usize) -> String {
    let original_chars = value.chars().count();
    if original_chars <= max_chars {
        return value.to_string();
    }

    let projection = match contract_id {
        "robonix/system/scene/list_objects" => Some(("objects", &["id", "label"][..])),
        "robonix/system/scene/list_regions" => Some((
            "regions",
            &["id", "kind", "name", "stale", "stale_reason"][..],
        )),
        _ => None,
    };
    if let (Some((array_key, fields)), Ok(parsed)) =
        (projection, serde_json::from_str::<serde_json::Value>(value))
        && let Some(items) = parsed.get(array_key).and_then(|entry| entry.as_array())
    {
        let mut projected: Vec<serde_json::Value> = items
            .iter()
            .map(|item| {
                let mut record = serde_json::Map::new();
                if let Some(source) = item.as_object() {
                    for field in fields {
                        if let Some(field_value) = source.get(*field) {
                            record.insert((*field).to_string(), field_value.clone());
                        }
                    }
                }
                serde_json::Value::Object(record)
            })
            .collect();
        loop {
            let returned = projected.len();
            let mut root = serde_json::Map::new();
            root.insert(
                array_key.to_string(),
                serde_json::Value::Array(projected.clone()),
            );
            for key in ["map_id", "stamp_unix"] {
                if let Some(field_value) = parsed.get(key) {
                    root.insert(key.to_string(), field_value.clone());
                }
            }
            root.insert(
                "_robonix_truncation".to_string(),
                serde_json::json!({
                    "truncated": true,
                    "complete_record_index": returned == items.len(),
                    "original_chars": original_chars,
                    "total_records": items.len(),
                    "returned_records": returned,
                    "omitted_fields": true,
                    "instruction": "Use a narrower capability for full record details; never infer absence when complete_record_index is false."
                }),
            );
            let encoded = serde_json::Value::Object(root).to_string();
            if encoded.chars().count() <= max_chars {
                return encoded;
            }
            if projected.is_empty() {
                break;
            }
            projected.pop();
        }
    }

    let mut preview_chars = max_chars / 3;
    loop {
        let encoded = serde_json::json!({
            "_robonix_truncation": {
                "truncated": true,
                "original_chars": original_chars,
                "complete": false,
                "instruction": "The preview is incomplete; do not infer that an omitted value is absent."
            },
            "preview": compact_preview(value, preview_chars),
        })
        .to_string();
        if encoded.chars().count() <= max_chars || preview_chars == 0 {
            return encoded;
        }
        preview_chars /= 2;
    }
}

/// Recover the LLM-facing capability name from an expanded capability call.
pub(super) fn call_display_name(call: &CapabilityCall) -> String {
    format!("{}.{}", call.provider_id, llm_name(&call.contract_id))
}

/// Append one node and its descendants to the human-readable plan summary.
pub(super) fn append_plan_node_summary(
    plan: &Plan,
    node_index: usize,
    depth: usize,
    out: &mut Vec<String>,
) {
    let Some(node) = plan.nodes.get(node_index) else {
        out.push(format!("{}[{node_index}] missing-node", "  ".repeat(depth)));
        return;
    };
    let indent = "  ".repeat(depth);
    let mut line = format!(
        "{indent}[{node_index}] {} op_id={} desc='{}'",
        rtdl_node_kind_name(node.node_kind),
        node.op_id,
        compact_preview(&node.description, 160),
    );
    if let Some(call) = node.call.as_ref() {
        line.push_str(&format!(
            " cap={} args={}",
            call_display_name(call),
            compact_preview(&call.args_json, 240)
        ));
    }
    out.push(line);
    for child in &node.children {
        append_plan_node_summary(plan, *child as usize, depth + 1, out);
    }
}

/// Format a plan as an indented tree instead of exposing arena child arrays.
pub(super) fn format_plan_summary(plan: &Plan) -> Vec<String> {
    let mut lines = Vec::new();
    append_plan_node_summary(plan, plan.root_index as usize, 0, &mut lines);
    lines
}

/// Emit the compact plan-start log block for one expanded RTDL plan.
pub(super) fn log_plan_start(plan: &Plan, description: &str, round: u32, calls: usize) {
    info!(
        "[pilot/rtdl] -- plan start plan_id={} round={} calls={} --",
        plan.plan_id, round, calls
    );
    info!(
        "[pilot/rtdl] rtdl_plan_description='{}'",
        compact_preview(description, 240)
    );
    info!("[pilot/rtdl] rtdl_plan:");
    for line in format_plan_summary(plan) {
        info!("[pilot/rtdl] {line}");
    }
}

/// Build compact extra detail for non-success terminal node states.
pub(super) fn terminal_node_detail(ns: &RtdlNodeState) -> String {
    if !is_terminal_executor_state(ns.state) || ns.state == RtdlNodeStateEnum::Succeeded as u32 {
        return String::new();
    }
    let Some(result) = ns.leaf_result.as_ref() else {
        return String::new();
    };
    if !result.error.trim().is_empty() {
        return format!(" error='{}'", compact_preview(&result.error, 180));
    }
    if !result.output.trim().is_empty() {
        return format!(" output='{}'", compact_preview(&result.output, 180));
    }
    String::new()
}

/// Emit one readable node-state event without numeric state or kind codes.
pub(super) fn log_node_state(plan_id: &str, ns: &RtdlNodeState) {
    let mut line = format!(
        "[pilot/forest] plan_id={} node={} op_id={} state={} desc='{}'",
        plan_id,
        ns.node_index,
        ns.op_id,
        rtdl_state_name(ns.state),
        compact_preview(&ns.description, 160),
    );
    if !ns.operator_detail.trim().is_empty() {
        line.push_str(&format!(
            " detail='{}'",
            compact_preview(&ns.operator_detail, 180)
        ));
    }
    line.push_str(&terminal_node_detail(ns));
    debug!("{line}");
}

/// Pick the plan-level completion state shown in the forest completion log.
pub(super) fn plan_completion_state(results: &[RtdlNodeState], any_failed: bool) -> String {
    if !any_failed {
        return "Succeeded".to_string();
    }
    for preferred in [
        RtdlNodeStateEnum::Failed as u32,
        RtdlNodeStateEnum::Timeout as u32,
        RtdlNodeStateEnum::Canceled as u32,
    ] {
        if results.iter().any(|ns| ns.state == preferred) {
            return rtdl_state_name(preferred);
        }
    }
    results
        .iter()
        .find(|ns| ns.state != RtdlNodeStateEnum::Succeeded as u32)
        .map(|ns| rtdl_state_name(ns.state))
        .unwrap_or_else(|| "Failed".to_string())
}

/// Emit the readable plan completion line, including non-success terminal nodes.
pub(super) fn log_plan_complete(plan_id: &str, results: &[RtdlNodeState], any_failed: bool) {
    let state = plan_completion_state(results, any_failed);
    let mut line = format!(
        "[pilot/forest] plan_id={} complete state={} terminal_nodes={}",
        plan_id,
        state,
        results.len()
    );
    let non_success: Vec<String> = results
        .iter()
        .filter(|ns| ns.state != RtdlNodeStateEnum::Succeeded as u32)
        .map(|ns| format!("node={} state={}", ns.node_index, rtdl_state_name(ns.state)))
        .collect();
    if !non_success.is_empty() {
        line.push_str(" non_success=[");
        line.push_str(&non_success.join(", "));
        line.push(']');
    }
    line.push_str("; replanning");
    info!("{line}");
}

pub(super) fn rtdl_result_to_messages(r: &CapabilityCallResult) -> history::ToolResultHistory {
    let mapped = if r.success {
        history::tool_result_to_messages(&r.call_id, &r.output)
    } else {
        history::ToolResultHistory {
            tool_messages: vec![Message::user(&r.output)],
            followup_messages: vec![],
        }
    };

    let tool_messages = mapped
        .tool_messages
        .into_iter()
        .map(|msg| {
            let output = msg.content.unwrap_or_default();
            let feedback = serde_json::json!({
                "leaf_result": {
                    "call_id": r.call_id,
                    "contract_id": r.contract_id,
                    "success": r.success,
                    "output": output,
                    "error": r.error,
                }
            });
            Message::user(&format!(
                "Executor feedback for the current RTDL leaf (not a new user request): {}",
                feedback
            ))
        })
        .collect();

    history::ToolResultHistory {
        tool_messages,
        followup_messages: mapped.followup_messages,
    }
}

// ── System prompt + SOUL ──────────────────────────────────────────────────────
// Optional `SOUL.md` (agent personality) is read from `$ROBONIX_PILOT_SOUL`,
// then `~/.robonix/SOUL.md`. There is no skill index — skill providers surface as
// regular tools through `executor.list_tools`, with descriptions sourced from
// each provider's CAPABILITY.md.
