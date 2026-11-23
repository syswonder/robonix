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

/// Media type of a base64 image, read from its first bytes rather than from a
/// label the capability may get wrong; `None` for raw pixels or other data.
pub fn image_media_type(image_base64: &str) -> Option<&'static str> {
    use base64::Engine;
    let head = base64::engine::general_purpose::STANDARD
        .decode(image_base64.get(..16)?)
        .ok()?;
    match head.as_slice() {
        [0xFF, 0xD8, 0xFF, ..] => Some("image/jpeg"),
        [0x89, b'P', b'N', b'G', ..] => Some("image/png"),
        [b'G', b'I', b'F', b'8', ..] => Some("image/gif"),
        [b'R', b'I', b'F', b'F', _, _, _, _, b'W', b'E', b'B', b'P'] => Some("image/webp"),
        _ => None,
    }
}

/// Re-encode raw `sensor_msgs/Image` pixels as a base64 image the model can
/// read: colour as JPEG (quality 90, small like camera snapshots), grayscale
/// and depth as lossless PNG. Depth (16UC1 in mm, 32FC1 in m) is scaled over
/// its valid range, nearer is brighter. `None` for compressed data, unknown
/// encodings or a short buffer.
fn raw_image_for_model(v: &serde_json::Value, encoding: &str) -> Option<String> {
    use base64::Engine;
    let codec = base64::engine::general_purpose::STANDARD;
    let width = u32::try_from(v.get("width")?.as_u64()?).ok()?;
    let height = u32::try_from(v.get("height")?.as_u64()?).ok()?;
    let bytes_per_pixel = match encoding {
        "mono8" | "8UC1" => 1,
        "16UC1" | "mono16" => 2,
        "rgb8" | "bgr8" => 3,
        "rgba8" | "bgra8" | "32FC1" => 4,
        _ => return None,
    };
    let data = codec.decode(v.get("data")?.as_str()?).ok()?;
    let row = width as usize * bytes_per_pixel;
    let step = v
        .get("step")
        .and_then(|s| s.as_u64())
        .map_or(row, |s| s as usize);
    if step < row || data.len() < step * height as usize {
        return None;
    }
    let big_endian = v.get("is_bigendian").and_then(|b| b.as_u64()) == Some(1);
    let pixels: Vec<u8> = (0..height as usize)
        .flat_map(|r| &data[r * step..r * step + row])
        .copied()
        .collect();
    let colour = match encoding {
        "rgb8" => Some(jpeg_encoder::ColorType::Rgb),
        "bgr8" => Some(jpeg_encoder::ColorType::Bgr),
        "rgba8" => Some(jpeg_encoder::ColorType::Rgba),
        "bgra8" => Some(jpeg_encoder::ColorType::Bgra),
        _ => None,
    };
    if let Some(colour) = colour {
        let mut out = Vec::new();
        jpeg_encoder::Encoder::new(&mut out, 90)
            .encode(
                &pixels,
                u16::try_from(width).ok()?,
                u16::try_from(height).ok()?,
                colour,
            )
            .ok()?;
        return Some(codec.encode(out));
    }
    let gray: Vec<u8> = match encoding {
        "mono8" | "8UC1" => pixels,
        _ => {
            let depth: Vec<f32> = pixels
                .chunks_exact(bytes_per_pixel)
                .map(|b| match (bytes_per_pixel, big_endian) {
                    (2, false) => f32::from(u16::from_le_bytes([b[0], b[1]])),
                    (2, true) => f32::from(u16::from_be_bytes([b[0], b[1]])),
                    (_, false) => f32::from_le_bytes([b[0], b[1], b[2], b[3]]),
                    (_, true) => f32::from_be_bytes([b[0], b[1], b[2], b[3]]),
                })
                .collect();
            let valid = |d: &f32| d.is_finite() && *d > 0.0;
            let (near, far) = depth
                .iter()
                .filter(|d| valid(d))
                .fold((f32::MAX, f32::MIN), |(lo, hi), d| (lo.min(*d), hi.max(*d)));
            let span = (far - near).max(f32::EPSILON);
            depth
                .iter()
                .map(|d| {
                    if valid(d) {
                        (255.0 - 223.0 * (d - near) / span) as u8
                    } else {
                        0
                    }
                })
                .collect()
        }
    };
    let mut out = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut out, width, height);
        encoder.set_color(png::ColorType::Grayscale);
        encoder.set_depth(png::BitDepth::Eight);
        encoder.write_header().ok()?.write_image_data(&gray).ok()?;
    }
    Some(codec.encode(out))
}

/// Attach an image the model can read; anything else stays a text note.
fn image_result(call_id: &str, label: &str, image_base64: &str) -> ToolResultHistory {
    let Some(media_type) = image_media_type(image_base64) else {
        return ToolResultHistory {
            tool_messages: vec![Message::tool_result(
                call_id,
                &format!("[{label}: not JPEG, PNG, GIF or WebP, so it was not sent to the model]"),
            )],
            followup_messages: vec![],
        };
    };
    ToolResultHistory {
        tool_messages: vec![Message::tool_result(
            call_id,
            &format!("[{label} attached as {media_type}]"),
        )],
        followup_messages: vec![Message::user_with_image(
            "Executor feedback: the previous capability call returned this image. Use it as observation data for the current task; this is not a new user request.",
            image_base64.to_string(),
        )],
    }
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
        return image_result(call_id, "image", b64);
    }

    // sensor_msgs/msg/Image — matches camera_snapshot / camera_depth_snapshot.
    // Skip encoding="error" (placeholder) and any payload missing real data.
    let img_encoding = v.get("encoding").and_then(|e| e.as_str());
    if is_image_value(&v) && img_encoding.is_some() {
        let enc = img_encoding.unwrap_or("jpeg");
        let label = format!("sensor_msgs/Image encoding={enc}");
        if let Some(image) = raw_image_for_model(&v, enc) {
            return image_result(call_id, &format!("{label}, re-encoded"), &image);
        }
        let b64 = v.get("data").and_then(|d| d.as_str()).unwrap_or("");
        return image_result(call_id, &label, b64);
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
    /// The user's messages about tasks still open, when they fell out of the
    /// tail: the model must always see the words of what it is working on.
    pub pinned: Vec<Message>,
    /// The most recent messages, kept verbatim except that no single message
    /// exceeds `message_cap` tokens.
    pub tail: Vec<Message>,
}

/// Keep the newest messages within `tail_cap` tokens, at least one of them,
/// and the user messages labelled `User message <id> ` for each of
/// `pinned_ids` (the open tasks' messages) within `pin_cap`, newest first when
/// they do not all fit; everything else is evicted. The caller sizes the caps so the result always fits with room to
/// spare, which is what lets compaction run forever without looping.
pub fn plan_compaction(
    history: &[Message],
    tail_cap: usize,
    message_cap: usize,
    pin_cap: usize,
    pinned_ids: &[String],
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

    let is_pinned = |message: &Message| {
        message.role == "user"
            && message.content.as_deref().is_some_and(|text| {
                pinned_ids
                    .iter()
                    .any(|id| text.starts_with(&format!("User message {id} ")))
            })
    };
    let mut pinned = Vec::new();
    let mut room = pin_cap;
    for message in evicted.iter().rev().filter(|message| is_pinned(message)) {
        let message = truncated(message, message_cap.min(room));
        if tokens(&message) > room {
            break;
        }
        room -= tokens(&message);
        pinned.push(message);
    }
    pinned.reverse();
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
    use super::{plan_compaction, tokens, tool_result_to_messages};
    use crate::vlm::Message;

    /// The media type comes from the bytes; raw pixels never reach the model.
    #[test]
    fn images_are_typed_by_content() {
        let png = "iVBORw0KGgoAAAANSUhEUgAAAAEAAAAB";
        let labelled = format!(r#"{{"image_base64":"{png}","format":"jpeg"}}"#);
        let result = tool_result_to_messages("c1", &labelled);
        assert_eq!(
            result.tool_messages[0].content.as_deref(),
            Some("[image attached as image/png]")
        );
        assert_eq!(result.followup_messages.len(), 1);

        // Raw colour pixels become JPEG; grayscale and depth become PNG.
        for (raw, media_type) in [
            (
                r#"{"width":2,"height":1,"step":6,"encoding":"bgr8","data":"AAD/AP8A"}"#,
                "image/jpeg",
            ),
            (
                r#"{"width":1,"height":1,"encoding":"mono8","data":"gA=="}"#,
                "image/png",
            ),
            (
                r#"{"width":2,"height":1,"encoding":"16UC1","data":"6APQBw=="}"#,
                "image/png",
            ),
            (
                r#"{"width":2,"height":1,"encoding":"32FC1","data":"AADAfwAAgD8="}"#,
                "image/png",
            ),
        ] {
            let result = tool_result_to_messages("c2", raw);
            let image = result.followup_messages[0].image_base64.as_deref().unwrap();
            assert_eq!(super::image_media_type(image), Some(media_type), "{raw}");
            assert!(
                result.tool_messages[0]
                    .content
                    .as_deref()
                    .unwrap()
                    .contains("re-encoded")
            );
        }

        let unknown =
            r#"{"width":2,"height":1,"encoding":"yuv422","data":"AAAAAAAAAAAAAAAAAAAAAA=="}"#;
        let result = tool_result_to_messages("c3", unknown);
        assert!(result.followup_messages.is_empty());
        assert!(
            result.tool_messages[0]
                .content
                .as_deref()
                .unwrap()
                .contains("encoding=yuv422: not JPEG")
        );
    }

    #[test]
    fn compaction_keeps_a_bounded_tail_and_pins_the_open_tasks_messages() {
        let mut history = vec![
            Message::user("User message m1 (filed under T1): inspect the loading dock"),
            Message::user("User message m2 (filed under T2): what time is it"),
        ];
        for i in 0..40 {
            history.push(Message::assistant(&format!(
                "narration {i} {}",
                "x".repeat(200)
            )));
        }
        history.push(Message::user(
            "User message m3 (filed under T1): stop at the doorway",
        ));
        history.push(Message::user(&"huge executor result ".repeat(2_000)));
        let open = ["m1".to_string(), "m3".to_string()];
        let plan = plan_compaction(&history, 400, 300, 100, &open);
        assert!(plan.tail.iter().map(tokens).sum::<usize>() <= 400 + 20);
        assert!(plan.tail.iter().all(|m| tokens(m) <= 300 + 20));
        assert_eq!(plan.evicted.len() + plan.tail.len(), history.len());
        let pinned: Vec<&str> = plan
            .pinned
            .iter()
            .filter_map(|m| m.content.as_deref())
            .collect();
        assert_eq!(pinned.len(), 1, "{pinned:?}");
        assert!(pinned[0].contains("loading dock"));
        // T2's message is not an open task's, so it is summarised, not pinned.
        assert!(plan.evicted.iter().any(|m| {
            m.content
                .as_deref()
                .is_some_and(|t| t.contains("what time"))
        }));
    }
}
