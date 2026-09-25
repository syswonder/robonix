// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Session transcripts: an append-only JSON-lines record of a session's
// conversation. History compaction drops old messages from what the model
// sees; the transcript keeps every one of them, and holds enough to rebuild the
// session's history and task state after Pilot restarts.

use crate::planner::TaskState;
use crate::vlm::Message;
use robonix_scribe::{ts_fmt, warn};
use serde_json::{Value, json};
use std::fs::OpenOptions;
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

/// One session's transcript file, and how much of the session has already been
/// written to it.
///
/// The file is `<session dir>/<session_id>/transcript.jsonl`, one JSON record
/// per line, each with `ts` and `kind`. The session dir is
/// `$ROBONIX_SESSION_DIR` (`rbnx boot` sets `<deploy>/rbnx-boot/sessions`), or
/// `./sessions` when unset. Records:
///
/// - `session`: first line of the file, with `session_id` and `pilot_version`.
/// - `message`: one history message (`role`, `content`, ...). An image is
///   recorded by size only, as `image_bytes`.
/// - `task_state`: the session's task state after it changed (`task`, or null).
/// - `compaction`: history was compacted; `history` is the full history that
///   replaced it, and the counts say what was evicted and pinned.
///
/// Replaying the file rebuilds the session: start from the last `compaction`'s
/// `history`, or from empty, and append each later `message`.
pub struct Transcript {
    path: PathBuf,
    session_id: String,
    /// History messages before this index are already in the file.
    written: usize,
    /// The task state last queued for the file.
    task: Option<TaskState>,
    /// Records not yet on disk because a write failed; retried first.
    pending: Vec<Value>,
}

/// A session rebuilt from its transcript.
pub struct Restored {
    pub history: Vec<Message>,
    pub task: Option<TaskState>,
}

impl Transcript {
    /// The transcript of a session whose current `history` and `task` are
    /// already on disk: a new, empty session, or one just restored from it.
    /// Keep one per session for as long as the session lives, so records a
    /// failed write left behind are retried by later turns.
    pub fn new(session_id: &str, history: &[Message], task: Option<&TaskState>) -> Self {
        Self {
            path: path_for(session_id),
            session_id: session_id.to_string(),
            written: history.len(),
            task: task.cloned(),
            pending: Vec::new(),
        }
    }

    /// Append every history message not yet in the file, then the task state
    /// if it changed.
    pub fn record(&mut self, history: &[Message], task: Option<&TaskState>) {
        // History only shrinks through compaction, which resets `written`;
        // clamp anyway so a shorter history never skips new messages.
        let start = self.written.min(history.len());
        self.pending
            .extend(history[start..].iter().map(message_record));
        self.written = history.len();
        if self.task.as_ref() != task {
            self.pending
                .push(json!({ "ts": now(), "kind": "task_state", "task": task }));
            self.task = task.cloned();
        }
        self.flush();
    }

    /// Note a compaction. Call [`Transcript::record`] first so the evicted
    /// messages are already on disk; `history` is the compacted history.
    pub fn record_compaction(
        &mut self,
        history: &[Message],
        evicted: usize,
        pinned: usize,
        summarized: bool,
    ) {
        let record = json!({
            "ts": now(),
            "kind": "compaction",
            "evicted_messages": evicted,
            "pinned_messages": pinned,
            "summarized": summarized,
            "history": history.iter().map(message_fields).collect::<Vec<_>>(),
        });
        self.pending.push(record);
        self.written = history.len();
        self.flush();
    }

    /// Rebuild a session from its transcript, or `None` when it has none.
    ///
    /// A line that does not parse (such as a last line cut short by a crash)
    /// is skipped with a warning. Images are not restored.
    pub fn restore(session_id: &str) -> Option<Restored> {
        replay(&path_for(session_id))
    }

    /// Write the queued records, starting the file with its `session` line
    /// when it is empty.
    ///
    /// All records go out in one append. If it fails, the file is cut back to
    /// its previous length so no half-written line or duplicate is left, the
    /// failure is logged, and the records stay queued for the next call; a
    /// turn never stops for it. A file that does not end in a newline (a crash
    /// mid-line) gets one first, so the fragment stays a line of its own that
    /// replay skips.
    fn flush(&mut self) {
        if self.pending.is_empty() {
            return;
        }
        let result = self.try_append();
        match result {
            Ok(()) => self.pending.clear(),
            Err(error) => warn!(
                "[pilot/transcript] cannot write {} ({} records queued): {error}",
                self.path.display(),
                self.pending.len()
            ),
        }
    }

    fn try_append(&self) -> std::io::Result<()> {
        if let Some(dir) = self.path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let mut file = OpenOptions::new()
            .create(true)
            .read(true)
            .append(true)
            .open(&self.path)?;
        let length = file.metadata()?.len();
        let mut text = String::new();
        if length == 0 {
            let header = json!({
                "ts": now(),
                "kind": "session",
                "session_id": self.session_id,
                "pilot_version": env!("CARGO_PKG_VERSION"),
            });
            text.push_str(&header.to_string());
            text.push('\n');
        } else {
            let mut last = [0u8; 1];
            file.seek(SeekFrom::Start(length - 1))?;
            file.read_exact(&mut last)?;
            if last[0] != b'\n' {
                text.push('\n');
            }
        }
        for record in &self.pending {
            text.push_str(&record.to_string());
            text.push('\n');
        }
        file.write_all(text.as_bytes()).inspect_err(|_| {
            let _ = file.set_len(length);
        })
    }
}

/// Rebuild a session by replaying the transcript at `path`.
fn replay(path: &std::path::Path) -> Option<Restored> {
    let text = std::fs::read_to_string(path).ok()?;
    let mut restored = Restored {
        history: Vec::new(),
        task: None,
    };
    let mut skipped = 0usize;
    for line in text.lines().filter(|line| !line.trim().is_empty()) {
        let Ok(record) = serde_json::from_str::<Value>(line) else {
            skipped += 1;
            continue;
        };
        let applied = match record["kind"].as_str() {
            Some("session") => true,
            Some("message") => parse_message(&record)
                .map(|message| restored.history.push(message))
                .is_some(),
            Some("task_state") => serde_json::from_value(record["task"].clone())
                .map(|task| restored.task = task)
                .is_ok(),
            Some("compaction") => record["history"]
                .as_array()
                .and_then(|messages| messages.iter().map(parse_message).collect())
                .map(|history| restored.history = history)
                .is_some(),
            _ => false,
        };
        if !applied {
            skipped += 1;
        }
    }
    if skipped > 0 {
        warn!(
            "[pilot/transcript] skipped {skipped} unreadable records in {}",
            path.display()
        );
    }
    Some(restored)
}

/// `<session dir>/<session_id>/transcript.jsonl`. Bytes of the session id
/// other than ASCII letters, digits, `-`, and `_` are percent-encoded, so
/// distinct session ids always get distinct directories.
fn path_for(session_id: &str) -> PathBuf {
    let mut dir_name = String::new();
    for byte in session_id.bytes() {
        if byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_') {
            dir_name.push(byte as char);
        } else {
            dir_name.push_str(&format!("%{byte:02X}"));
        }
    }
    let root = std::env::var_os("ROBONIX_SESSION_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("sessions"));
    root.join(dir_name).join("transcript.jsonl")
}

/// A message's fields as stored in the transcript: the image, if any, becomes
/// its size in `image_bytes`.
fn message_fields(message: &Message) -> Value {
    let mut stored = message.clone();
    let image = stored.image_base64.take();
    let mut fields = serde_json::to_value(&stored).unwrap_or_else(|_| json!({}));
    if let (Some(image), Some(fields)) = (image, fields.as_object_mut()) {
        fields.insert("image_bytes".to_string(), json!(image.len()));
    }
    fields
}

fn message_record(message: &Message) -> Value {
    let mut record = json!({ "ts": now(), "kind": "message" });
    if let (Some(record), Value::Object(fields)) = (record.as_object_mut(), message_fields(message))
    {
        record.extend(fields);
    }
    record
}

/// A stored message back as a [`Message`]; bookkeeping fields are ignored.
fn parse_message(record: &Value) -> Option<Message> {
    serde_json::from_value(record.clone()).ok()
}

fn now() -> String {
    let ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    ts_fmt(ns)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn transcript_in(dir: &std::path::Path) -> Transcript {
        Transcript {
            path: dir.join("s.jsonl"),
            session_id: "s".to_string(),
            written: 0,
            task: None,
            pending: Vec::new(),
        }
    }

    fn contents(history: &[Message]) -> Vec<String> {
        history
            .iter()
            .map(|m| m.content.clone().unwrap_or_default())
            .collect()
    }

    #[test]
    fn replaying_the_file_rebuilds_history_and_task_across_a_compaction() {
        let dir = tempfile::tempdir().unwrap();
        let mut transcript = transcript_in(dir.path());
        let mut task = None;
        crate::planner::start_or_resume_task(&mut task, "fetch the cup");

        let mut history = vec![Message::user("task"), Message::assistant("step 1")];
        transcript.record(&history, task.as_ref());
        history.push(Message::user_with_image("look", "aGVsbG8=".to_string()));
        transcript.record(&history, task.as_ref());
        transcript.record(&history, task.as_ref());

        // Compaction replaces all but the last message with a summary.
        history = vec![Message::user("summary"), Message::user("look")];
        transcript.record_compaction(&history, 2, 0, true);
        history.push(Message::assistant("step 2"));
        transcript.record(&history, task.as_ref());

        let text = std::fs::read_to_string(&transcript.path).unwrap();
        let kinds: Vec<String> = text
            .lines()
            .map(|l| serde_json::from_str::<Value>(l).unwrap()["kind"].to_string())
            .collect();
        assert_eq!(
            kinds,
            [
                "\"session\"",
                "\"message\"",
                "\"message\"",
                "\"task_state\"",
                "\"message\"",
                "\"compaction\"",
                "\"message\""
            ]
        );
        // Every message ever sent is kept, the image only by size.
        assert!(text.contains("\"step 1\"") && text.contains("\"image_bytes\":8"));
        assert!(!text.contains("aGVsbG8="));

        std::fs::write(dir.path().join("s.jsonl"), text + "{\"kind\":\"mess").unwrap();
        let restored = replay(&transcript.path).unwrap();
        assert_eq!(contents(&restored.history), ["summary", "look", "step 2"]);
        assert_eq!(restored.task, task);
    }

    #[test]
    fn distinct_session_ids_get_distinct_files() {
        let names: Vec<PathBuf> = ["任务A", "任务B", "a/b", "a_b", "a%2Fb"]
            .iter()
            .map(|id| path_for(id))
            .collect();
        let unique: std::collections::HashSet<_> = names.iter().collect();
        assert_eq!(unique.len(), names.len());
        assert!(names.iter().all(|p| p.ends_with("transcript.jsonl")));
    }

    #[test]
    fn a_failed_write_is_retried_once_and_a_cut_line_stays_separate() {
        let dir = tempfile::tempdir().unwrap();
        // A file where the directory should be makes every write fail.
        let blocked = dir.path().join("blocked");
        std::fs::write(&blocked, "").unwrap();
        let mut transcript = Transcript {
            path: blocked.join("s.jsonl"),
            session_id: "s".to_string(),
            written: 0,
            task: None,
            pending: Vec::new(),
        };
        let mut history = vec![Message::user("one")];
        transcript.record(&history, None);
        history = vec![Message::user("summary")];
        transcript.record_compaction(&history, 1, 0, false);
        assert_eq!(transcript.pending.len(), 2);

        // The disk recovers; an earlier crash left half a line behind.
        std::fs::remove_file(&blocked).unwrap();
        std::fs::create_dir(&blocked).unwrap();
        std::fs::write(&transcript.path, "{\"kind\":\"session\"}\n{\"kind\":\"mess").unwrap();
        history.push(Message::assistant("two"));
        transcript.record(&history, None);
        assert!(transcript.pending.is_empty());

        let restored = replay(&transcript.path).unwrap();
        assert_eq!(contents(&restored.history), ["summary", "two"]);
        let text = std::fs::read_to_string(&transcript.path).unwrap();
        assert_eq!(text.matches("\"one\"").count(), 1);
    }
}
