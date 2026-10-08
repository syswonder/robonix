// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Two concerns live here:
//   1. Mapping an executor tool result (JSON string) back into one or more
//      `Message`s the LLM can ingest. OpenAI-compatible endpoints reject
//      images on `tool` role, so when a tool returns an image we keep the
//      tool result textual and append a synthetic `user` vision message.
//   2. Bounding and cleanup of `Vec<Message>`: plan a compaction that
//      keeps recent messages verbatim and the rest for one bounded summary,
//      and drop tool messages whose preceding assistant tool_call was
//      evicted (which would otherwise be rejected).

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

/// Label of the one rolling summary that stands for everything compacted away.
pub const SUMMARY_LABEL: &str = "[summary of earlier conversation — treat as established context]";

/// Four-byte token estimate of one message's text.
pub fn tokens(message: &Message) -> usize {
    message.content.as_deref().map_or(0, str::len).div_ceil(4)
}

/// Cut a message's text to about `cap` tokens, saying how much was dropped.
pub fn truncated(message: &Message, cap: usize) -> Message {
    let mut out = message.clone();
    if let Some(text) = message.content.as_deref()
        && text.len() > cap * 4
    {
        let mut end = cap * 4;
        while !text.is_char_boundary(end) {
            end -= 1;
        }
        out.content = Some(format!(
            "{}\n[... {} more bytes dropped when history was compacted]",
            &text[..end],
            text.len() - end
        ));
    }
    out
}

/// How one compaction splits history, decided without calling the model.
pub struct CompactionPlan {
    /// Everything older than the tail, oldest first, including any previous
    /// summary. It is folded into the new summary and then dropped.
    pub evicted: Vec<Message>,
    /// The latest user task and the steers that followed it, when they fell
    /// out of the tail: the model must always see what it is working on now.
    pub pinned: Vec<Message>,
    /// The most recent messages, kept verbatim except that no single message
    /// exceeds `message_cap` tokens.
    pub tail: Vec<Message>,
}

/// Keep the newest messages within `tail_cap` tokens, at least one of them,
/// and the current task's own words within `pin_cap`; everything else is
/// evicted. The caller sizes the caps so the result always fits with room to
/// spare, which is what lets compaction run forever without looping.
pub fn plan_compaction(
    history: &[Message],
    tail_cap: usize,
    message_cap: usize,
    pin_cap: usize,
) -> CompactionPlan {
    let mut split = history.len();
    let mut used = 0usize;
    while split > 0 {
        let cost = tokens(&history[split - 1]).min(message_cap);
        if split < history.len() && used + cost > tail_cap {
            break;
        }
        used += cost;
        split -= 1;
    }
    let evicted = history[..split].to_vec();
    let tail = history[split..]
        .iter()
        .map(|message| truncated(message, message_cap))
        .collect();

    let starts = |message: &Message, label: &str| {
        message.role == "user"
            && message
                .content
                .as_deref()
                .is_some_and(|text| text.starts_with(label))
    };
    let mut pinned = Vec::new();
    if let Some(task) = evicted
        .iter()
        .rposition(|m| starts(m, "User task (authoritative):"))
    {
        let mut own_words: Vec<&Message> = std::iter::once(&evicted[task])
            .chain(
                evicted[task + 1..]
                    .iter()
                    .filter(|m| starts(m, "User steer (authoritative):")),
            )
            .collect();
        // The task itself first, then the latest steers that still fit.
        let mut room = pin_cap;
        let task_message = truncated(own_words.remove(0), pin_cap);
        room = room.saturating_sub(tokens(&task_message));
        let mut steers = Vec::new();
        for steer in own_words.into_iter().rev() {
            let steer = truncated(steer, message_cap);
            if tokens(&steer) > room {
                break;
            }
            room -= tokens(&steer);
            steers.push(steer);
        }
        pinned.push(task_message);
        pinned.extend(steers.into_iter().rev());
    }
    CompactionPlan {
        evicted,
        pinned,
        tail,
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

#[cfg(test)]
mod tests {
    use super::{plan_compaction, tokens};
    use crate::vlm::Message;

    #[test]
    fn compaction_keeps_a_bounded_tail_and_pins_the_current_task() {
        let mut history = vec![Message::user(
            "User task (authoritative): inspect the loading dock",
        )];
        for i in 0..40 {
            history.push(Message::assistant(&format!(
                "narration {i} {}",
                "x".repeat(200)
            )));
        }
        history.push(Message::user(
            "User steer (authoritative): stop at the doorway",
        ));
        history.push(Message::user(&"huge executor result ".repeat(2_000)));
        let plan = plan_compaction(&history, 400, 300, 100);
        assert!(plan.tail.iter().map(tokens).sum::<usize>() <= 400 + 20);
        assert!(plan.tail.iter().all(|m| tokens(m) <= 300 + 20));
        assert_eq!(plan.evicted.len() + plan.tail.len(), history.len());
        assert!(
            plan.pinned[0]
                .content
                .as_deref()
                .unwrap()
                .contains("loading dock")
        );
    }
}
