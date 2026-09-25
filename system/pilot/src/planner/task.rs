// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// The user's task as Pilot tracks it: task state, steers, and the per-task
// response mode.

use super::*;

/// Harness-owned state for the latest user interaction. Long-running work is
/// represented independently by the RTDL forest; it must not keep older user
/// text welded into the current goal forever.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct TaskState {
    pub(super) goal: String,
    pub(super) success_criterion: String,
    pub(super) status: String,
}

pub(super) const DEFAULT_SUCCESS_CRITERION: &str =
    "The user's request is completed and the result has been verified.";

impl TaskState {
    /// Whether the LLM has declared the overall task complete. This is the
    /// authoritative completion signal — an empty RTDL tree alone does not end
    /// the turn.
    pub(super) fn is_done(&self) -> bool {
        self.status == "done"
    }
}

/// Persist authoritative task updates without changing the system prefix.
pub(super) fn append_task_state_record(history: &mut Vec<Message>, state: &TaskState) {
    let record = serde_json::json!({
        "goal": state.goal,
        "success_criterion": state.success_criterion,
        "status": state.status,
    });
    history.push(Message::user(&format!(
        "Pilot task-state update (authoritative state, not a new user instruction): {record}"
    )));
}

/// `context_json`: `{"session_end": true}` (or `robonix_session_end`) — run memory compaction only, no VLM turn.
pub(super) fn task_is_session_end(task: &Task) -> bool {
    let j = task.context_json.trim();
    if j.is_empty() {
        return false;
    }
    serde_json::from_str::<serde_json::Value>(j)
        .ok()
        .and_then(|v| {
            v.get("session_end")
                .or_else(|| v.get("robonix_session_end"))
                .and_then(|x| x.as_bool())
        })
        .unwrap_or(false)
}

/// `context_json.modality` — set by liaison to "text" / "voice" / "api".
/// `None` when the field is missing or context_json is empty/malformed.
pub(super) fn task_modality(task: &Task) -> Option<String> {
    let j = task.context_json.trim();
    if j.is_empty() {
        return None;
    }
    serde_json::from_str::<serde_json::Value>(j)
        .ok()
        .and_then(|v| {
            v.get("modality")
                .and_then(|x| x.as_str())
                .map(str::to_string)
        })
}

pub(super) fn response_mode(task: &Task) -> &'static str {
    if task_modality(task).as_deref() == Some("voice") {
        "Response mode for this task only: voice. Keep user-facing replies brief, plain, and suitable for TTS (about 30 Chinese characters or 50 English words)."
    } else {
        "Response mode for this task only: text. Earlier voice-only constraints no longer apply."
    }
}

/// Skip vector memory prefetch for trivial chit-chat (saves latency and noise).
pub(super) fn skip_memory_prefetch(user_text: &str) -> bool {
    let t = user_text.trim();
    let lower = t.to_lowercase();
    lower == "hi" || lower == "hello"
}

/// Pull every queued mid-task steer into the LLM history as fresh user input.
///
/// A steer is just a `Task` the user submitted while the turn was already
/// running. Draining is non-blocking; returns whether anything was pulled so
/// the caller knows to re-plan. The model decides for itself whether the steer
/// requires a root plan-control meta op.
pub(super) fn append_steer(
    task: Task,
    history: &mut Vec<Message>,
    current_task: &mut Option<TaskState>,
) -> bool {
    let text = task.text.trim();
    if text.is_empty() {
        return false;
    }
    info!("[pilot/steer] mid-task input: {text}");
    history.push(Message::user(&format!(
        "User steer (authoritative): {text}\n{}",
        response_mode(&task)
    )));
    *current_task = Some(TaskState {
        goal: text.to_string(),
        success_criterion: DEFAULT_SUCCESS_CRITERION.to_string(),
        status: "in_progress".to_string(),
    });
    true
}

pub(super) fn drain_steers(
    steer_rx: &mut mpsc::Receiver<Task>,
    history: &mut Vec<Message>,
    current_task: &mut Option<TaskState>,
) -> bool {
    let mut pulled = false;
    while let Ok(task) = steer_rx.try_recv() {
        pulled |= append_steer(task, history, current_task);
    }
    pulled
}

pub(super) fn start_or_resume_task(current_task: &mut Option<TaskState>, user_text: &str) {
    let text = user_text.trim();
    if text.is_empty() {
        return;
    }
    *current_task = Some(TaskState {
        goal: text.to_string(),
        success_criterion: DEFAULT_SUCCESS_CRITERION.to_string(),
        status: "in_progress".to_string(),
    });
}

/// Apply only progress fields from the model. The user-owned goal is immutable
/// within the standing task; steering is appended by the harness above. The
/// model may refine the default success criterion once, but cannot erase or
/// replace an established criterion. Completion is accepted only at a harness
/// safe point with no new or in-flight execution.
pub(super) fn apply_task_update(
    current_task: &mut Option<TaskState>,
    update: TaskState,
    can_finish: bool,
) -> bool {
    let Some(state) = current_task.as_mut() else {
        return false;
    };
    let before = state.clone();
    if update.goal != state.goal {
        warn!(
            "[pilot/rtdl] ignoring model goal replacement {:?}; harness goal remains {:?}",
            update.goal, state.goal
        );
        // The response was sampled for an older interaction. Applying even
        // its status or success criterion can falsely complete and discard a
        // newer steer, so reject the entire stale update.
        return false;
    }
    if state.success_criterion == DEFAULT_SUCCESS_CRITERION
        && !update.success_criterion.trim().is_empty()
    {
        state.success_criterion = update.success_criterion;
    }
    state.status = if update.status == "done" && can_finish {
        "done".to_string()
    } else {
        "in_progress".to_string()
    };
    *state != before
}

/// Parse a non-null `task_update` object into a [`TaskState`].
///
/// Requires exactly `goal`, `success_criterion`, and `status` (all strings),
/// with `status` constrained to `"in_progress"` or `"done"`.
pub(super) fn parse_task_update(v: &serde_json::Value) -> Result<TaskState> {
    let obj = v
        .as_object()
        .ok_or_else(|| anyhow::anyhow!("`task_update` must be null or an object"))?;
    const KEYS: [&str; 3] = ["goal", "success_criterion", "status"];
    if obj.len() != KEYS.len() || !KEYS.iter().all(|k| obj.contains_key(*k)) {
        anyhow::bail!(
            "`task_update` object must contain exactly `goal`, `success_criterion`, and `status`"
        );
    }
    let get = |key: &str| -> Result<String> {
        obj.get(key)
            .and_then(|x| x.as_str())
            .map(str::to_string)
            .ok_or_else(|| anyhow::anyhow!("`task_update.{key}` must be a string"))
    };
    let status = get("status")?;
    if status != "in_progress" && status != "done" {
        anyhow::bail!("`task_update.status` must be \"in_progress\" or \"done\"");
    }
    Ok(TaskState {
        goal: get("goal")?,
        success_criterion: get("success_criterion")?,
        status,
    })
}
