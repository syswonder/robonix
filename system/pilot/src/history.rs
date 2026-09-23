// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Two concerns live here:
//   1. Mapping an executor tool result (JSON string) back into one or more
//      `Message`s the LLM can ingest. OpenAI-compatible endpoints reject
//      images on `tool` role, so when a tool returns an image we keep the
//      tool result textual and append a synthetic `user` vision message.
//   2. Pre-flight cleanup of `Vec<Message>` before we hand it to the LLM:
//      retain authoritative task and executor records when bounding the
//      working window, and drop tool messages whose preceding assistant
//      tool_call was already evicted (which would otherwise be rejected).

use crate::vlm::Message;
use std::collections::HashSet;

/// Output of `tool_result_to_messages`: messages that go in `tool` role,
/// plus optional follow-up `user` messages (e.g. for an image attachment).
pub struct ToolResultHistory {
    pub tool_messages: Vec<Message>,
    pub followup_messages: Vec<Message>,
}

fn is_image_value(v: &serde_json::Value) -> bool {
    v.get("image_base64")
        .and_then(|x| x.as_str())
        .is_some_and(|data| !data.is_empty())
        || (v.get("width").is_some()
            && v.get("height").is_some()
            && v.get("encoding")
                .and_then(|encoding| encoding.as_str())
                .is_some_and(|encoding| encoding != "error")
            && v.get("data")
                .and_then(|data| data.as_str())
                .is_some_and(|data| !data.is_empty()))
}

/// Return true only for payloads that [`tool_result_to_messages`] will map to
/// an actual image follow-up. Callers use this to avoid retaining malformed
/// image-shaped JSON without normal result-size bounds.
pub fn is_image_output(output: &str) -> bool {
    serde_json::from_str::<serde_json::Value>(output).is_ok_and(|value| is_image_value(&value))
}

/// Build history messages from one executor tool result.
///
/// Normal path: a single `Message` with `role: "tool"` and `tool_call_id = call_id`,
/// carrying `output` (or a short placeholder) in `content`.
///
/// Image path: OpenAI-compatible APIs do not accept image payloads on `tool` messages.
/// We still emit a `tool` line with a text placeholder, then add a synthetic `user`
/// message with `image_base64` so `build_openai_messages` can attach a vision part.
pub fn tool_result_to_messages(call_id: &str, output: &str) -> ToolResultHistory {
    // One `tool` message in the typical case; image-shaped results add follow-up `user`
    // vision messages (see doc above).
    let Ok(v) = serde_json::from_str::<serde_json::Value>(output) else {
        return ToolResultHistory {
            tool_messages: vec![Message::tool_result(call_id, output)],
            followup_messages: vec![],
        };
    };

    if is_image_value(&v)
        && let Some(b64) = v.get("image_base64").and_then(|x| x.as_str())
    {
        let fmt = v.get("format").and_then(|x| x.as_str()).unwrap_or("jpeg");
        return ToolResultHistory {
            tool_messages: vec![Message::tool_result(
                call_id,
                &format!("[{fmt} image attached]"),
            )],
            followup_messages: vec![Message::user_with_image(
                "Executor feedback: the previous capability call returned this image. Use it as observation data for the current task; this is not a new user request.",
                b64.to_string(),
            )],
        };
    }

    // sensor_msgs/msg/Image — matches camera_snapshot / camera_depth_snapshot.
    // Skip encoding="error" (placeholder) and any payload missing real data.
    let img_encoding = v.get("encoding").and_then(|e| e.as_str());
    if is_image_value(&v) && img_encoding.is_some() {
        let enc = img_encoding.unwrap_or("jpeg");
        let b64 = v.get("data").and_then(|d| d.as_str()).unwrap_or("");
        return ToolResultHistory {
            tool_messages: vec![Message::tool_result(
                call_id,
                &format!("[sensor_msgs/Image encoding={enc}]"),
            )],
            followup_messages: vec![Message::user_with_image(
                "Executor feedback: the previous capability call returned this image. Use it as observation data for the current task; this is not a new user request.",
                b64.to_string(),
            )],
        };
    }

    ToolResultHistory {
        tool_messages: vec![Message::tool_result(call_id, output)],
        followup_messages: vec![],
    }
}

/// Whether a user-side record is an authoritative fact that must survive a
/// bounded working window. Pilot writes these labels; keeping them verbatim is
/// safer than hoping a later free-form summary reproduces a task instruction or
/// executor outcome exactly.
fn is_authoritative_record(message: &Message) -> bool {
    if message.role != "user" {
        return false;
    }
    let Some(content) = message.content.as_deref() else {
        return false;
    };
    [
        "User task (authoritative):",
        "User steer (authoritative):",
        "Pilot task-state update (authoritative state",
        "Pilot harness dispatch record",
        "Pilot plan-control result:",
        "Pilot plan-control failure:",
        "Executor feedback scope:",
        "Executor feedback for the current RTDL leaf",
    ]
    .iter()
    .any(|prefix| content.starts_with(prefix))
}

/// Clone authoritative facts for a rolling compaction. This deliberately
/// retains the source records alongside the generated prose summary: a summary
/// is useful navigation, but is not evidence that a user instruction or an
/// executor result has been preserved correctly.
pub fn authoritative_records(history: &[Message]) -> Vec<Message> {
    history
        .iter()
        .filter(|message| is_authoritative_record(message))
        .cloned()
        .collect()
}

/// Bound disposable conversational material without silently evicting user
/// tasks, lifecycle records, dispatches, or executor results. If those
/// authoritative records alone exceed `max`, keep them all; a later explicit
/// compaction can replace them only with a verified durable summary.
pub fn trim(history: &mut Vec<Message>, max: usize) {
    let mut remove = history.len().saturating_sub(max);
    if remove == 0 {
        return;
    }

    let mut drop = vec![false; history.len()];
    for (index, message) in history.iter().enumerate() {
        if remove == 0 {
            break;
        }
        if !is_authoritative_record(message) {
            drop[index] = true;
            remove -= 1;
        }
    }
    let mut index = 0;
    history.retain(|_| {
        let keep = !drop[index];
        index += 1;
        keep
    });
}

/// Filter `history` to a form OpenAI-compatible endpoints accept:
/// every `tool` message must be preceded by an `assistant` whose
/// `tool_calls` lists its `tool_call_id`. Orphans (e.g. left over from
/// a trim that dropped the assistant) are quietly removed.
pub fn sanitize_for_vlm(history: &[Message]) -> Vec<Message> {
    let mut out: Vec<Message> = Vec::with_capacity(history.len());
    let mut open_tool_call_ids: HashSet<String> = Default::default();

    for msg in history {
        match msg.role.as_str() {
            "assistant" => {
                open_tool_call_ids.clear();
                if let Some(calls) = &msg.tool_calls {
                    for tc in calls {
                        open_tool_call_ids.insert(tc.id.clone());
                    }
                }
                out.push(msg.clone());
            }
            "tool" => {
                let Some(call_id) = msg.tool_call_id.as_ref() else {
                    continue;
                };
                if open_tool_call_ids.remove(call_id) {
                    out.push(msg.clone());
                }
            }
            _ => {
                open_tool_call_ids.clear();
                out.push(msg.clone());
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::{authoritative_records, trim};
    use crate::vlm::Message;

    #[test]
    fn trim_keeps_tasks_and_executor_outcomes_verbatim() {
        let mut history = vec![
            Message::user("User task (authoritative): inspect the loading dock"),
            Message::assistant("I will inspect it."),
            Message::user(
                "Executor feedback for the current RTDL leaf (not a new user request): succeeded",
            ),
            Message::assistant("The inspection is complete."),
        ];
        trim(&mut history, 2);
        assert_eq!(history.len(), 2);
        assert_eq!(
            history[0].content.as_deref(),
            Some("User task (authoritative): inspect the loading dock")
        );
        assert_eq!(
            history[1].content.as_deref(),
            Some("Executor feedback for the current RTDL leaf (not a new user request): succeeded")
        );
    }

    #[test]
    fn trim_allows_authoritative_records_to_exceed_the_nominal_cap() {
        let mut history = vec![
            Message::user("User task (authoritative): inspect the loading dock"),
            Message::user("User steer (authoritative): stop at the doorway"),
            Message::user("Executor feedback scope: plan_id=1"),
        ];
        trim(&mut history, 1);
        assert_eq!(history.len(), 3);
    }

    #[test]
    fn authoritative_records_excludes_disposable_narration() {
        let history = vec![
            Message::user("User task (authoritative): inspect the loading dock"),
            Message::assistant("I will inspect it."),
            Message::user("Executor feedback scope: plan_id=1"),
        ];
        let retained = authoritative_records(&history);
        assert_eq!(retained.len(), 2);
        assert_eq!(retained[0].content, history[0].content);
        assert_eq!(retained[1].content, history[2].content);
    }
}
