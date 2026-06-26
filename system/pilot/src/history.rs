// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Two concerns live here:
//   1. Mapping an executor tool result (JSON string) back into one or more
//      `Message`s the LLM can ingest. OpenAI-compatible endpoints reject
//      images on `tool` role, so when a tool returns an image we keep the
//      tool result textual and append a synthetic `user` vision message.
//   2. Pre-flight cleanup of `Vec<Message>` before we hand it to the LLM:
//      trim to MAX_HISTORY and drop tool messages whose preceding assistant
//      tool_call was already evicted (which would otherwise be rejected).

use crate::vlm::Message;
use std::collections::HashSet;

/// Output of `tool_result_to_messages`: messages that go in `tool` role,
/// plus optional follow-up `user` messages (e.g. for an image attachment).
pub struct ToolResultHistory {
    pub tool_messages: Vec<Message>,
    pub followup_messages: Vec<Message>,
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

    if let Some(b64) = v.get("image_base64").and_then(|x| x.as_str()) {
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
    if v.get("width").is_some()
        && v.get("height").is_some()
        && img_encoding.is_some()
        && img_encoding != Some("error")
        && v.get("data")
            .and_then(|d| d.as_str())
            .is_some_and(|s| !s.is_empty())
    {
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

/// Drop the oldest messages so `history.len() <= max`. No-op if already short.
pub fn trim(history: &mut Vec<Message>, max: usize) {
    if history.len() > max {
        let remove = history.len() - max;
        history.drain(0..remove);
    }
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
