// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Context budget and history compaction: when history outgrows its room, keep the
// recent messages verbatim and fold the rest into one bounded rolling summary.

use super::*;

/// Internal room for the next response and small tokenizer/framing errors.
/// This is deliberately not operator-configurable: deployments only declare
/// the model's actual context window; Pilot owns its budgeting policy.
pub const CONTEXT_RESERVE_TOKENS: usize = 6_144;

// What a compaction leaves behind, as percentages of the history room. They
// add up to 75%, so every compaction frees at least a quarter of the room and
// the next one cannot follow immediately: compaction always makes progress.
/// Recent messages, kept verbatim.
pub(super) const TAIL_SHARE_PCT: usize = 50;
/// The one rolling summary of everything older; also capped in tokens.
pub(super) const SUMMARY_SHARE_PCT: usize = 15;
pub(super) const SUMMARY_MAX_TOKENS: usize = 6_000;
/// The current task's own words, when they fall out of the tail.
pub(super) const PIN_SHARE_PCT: usize = 10;
/// No single kept message, such as one large executor result, may exceed this.
pub(super) const MESSAGE_SHARE_PCT: usize = 20;
/// Floor for the room when the non-history context alone nearly fills the window.
pub(super) const MIN_HISTORY_ROOM: usize = 2_048;

/// Compaction budget using a configured or provider-reported context window.
#[derive(Clone, Debug)]
pub struct HistoryBudget {
    pub context_window_tokens: Option<usize>,
    pub context_window_source: &'static str,
}

impl HistoryBudget {
    /// Preserve the resolved limit, provenance, and completion reserves.
    pub fn new(context_window_tokens: Option<usize>, context_window_source: &'static str) -> Self {
        Self {
            context_window_tokens,
            context_window_source,
        }
    }

    /// Tokens history may use once this round's non-history context and the
    /// reserve are set aside; `None` when the context window is unknown.
    pub(super) fn room(&self, non_history_tokens: usize) -> Option<usize> {
        self.context_window_tokens.map(|limit| {
            limit
                .saturating_sub(non_history_tokens.saturating_add(CONTEXT_RESERVE_TOKENS))
                .max(MIN_HISTORY_ROOM)
        })
    }

    /// Whether history no longer fits in its room.
    pub(super) fn must_compact(&self, history: &[Message], non_history_tokens: usize) -> bool {
        self.room(non_history_tokens)
            .is_some_and(|room| history.iter().map(history::tokens).sum::<usize>() > room)
    }
}

/// Token caps for what one compaction leaves behind in a room of `room` tokens.
pub(super) struct CompactionCaps {
    pub(super) tail: usize,
    pub(super) summary: usize,
    pub(super) pin: usize,
    pub(super) message: usize,
}

impl CompactionCaps {
    pub(super) fn for_room(room: usize) -> Self {
        Self {
            tail: room * TAIL_SHARE_PCT / 100,
            summary: (room * SUMMARY_SHARE_PCT / 100).min(SUMMARY_MAX_TOKENS),
            pin: room * PIN_SHARE_PCT / 100,
            message: room * MESSAGE_SHARE_PCT / 100,
        }
    }
}

/// Once history outgrows its room, keep the recent messages verbatim and fold
/// everything older into one bounded rolling summary.
///
/// The result always fits in 75% of the room (see the shares above), so the
/// next compaction is at least a quarter of the room away. If the summarizer
/// fails, the older messages are still dropped behind a note saying so; a
/// failed summary never leaves history as large as it was, which is what used
/// to make compaction repeat every round.
pub(super) async fn compact_history(
    history: &mut Vec<Message>,
    vlm: &VlmClient,
    budget: &HistoryBudget,
    non_history_tokens: usize,
    cache_epoch: u64,
) -> bool {
    let Some(room) = budget.room(non_history_tokens) else {
        return false;
    };
    if !budget.must_compact(history, non_history_tokens) {
        return false;
    }
    let before_messages = history.len();
    let before_tokens: usize = history.iter().map(history::tokens).sum();
    let caps = CompactionCaps::for_room(room);
    let summary_cap = caps.summary;
    let plan = history::plan_compaction(history, caps.tail, caps.message, caps.pin);

    let previous = plan
        .evicted
        .first()
        .and_then(|m| m.content.as_deref())
        .filter(|text| text.starts_with(history::SUMMARY_LABEL))
        .map(|text| text[history::SUMMARY_LABEL.len()..].trim().to_string());
    let mut request = vec![Message::system(&format!(
        "You compact a robot agent's working memory. Rewrite the conversation below, including \
         any earlier summary at its start, as one plain-text note of at most about {summary_cap} \
         tokens. Keep, in this order of priority: the current goal and its success criteria; \
         what each finished task achieved or why it failed; RTDL work still running; decisions \
         and facts needed to continue. Drop narration. Do not invent facts."
    ))];
    request.extend(history::sanitize_for_vlm(&plan.evicted));
    request.push(Message::user("Write the note now."));
    let completion = if plan.evicted.is_empty() {
        None
    } else {
        collect_vlm_text(vlm, &request)
            .await
            .filter(|completion| !completion.text.trim().is_empty())
    };
    let summarized = completion.is_some();
    let note = match &completion {
        Some(completion) => completion.text.trim().to_string(),
        None => format!(
            "{}\n[{} earlier messages were dropped without a new summary because the summarizer was unavailable.]",
            previous.as_deref().unwrap_or(""),
            plan.evicted.len()
        ),
    };
    let summary = history::truncated(
        &Message::user(&format!("{}\n{}", history::SUMMARY_LABEL, note.trim())),
        summary_cap,
    );

    let evicted = plan.evicted.len();
    let pinned = plan.pinned.len();
    *history = std::iter::once(summary)
        .chain(plan.pinned)
        .chain(plan.tail)
        .collect();
    let after_tokens: usize = history.iter().map(history::tokens).sum();
    info!(
        "[pilot/compaction] {}",
        serde_json::json!({
            "event": "history_compaction",
            "cache_epoch_before": cache_epoch,
            "cache_epoch_after": cache_epoch.saturating_add(1),
            "context_window_tokens": budget.context_window_tokens,
            "context_window_source": budget.context_window_source,
            "non_history_tokens": non_history_tokens,
            "history_room_tokens": room,
            "summary_cap_tokens": summary_cap,
            "summarized": summarized,
            "evicted_messages": evicted,
            "pinned_messages": pinned,
            "history_messages_before": before_messages,
            "history_messages_after": history.len(),
            "history_tokens_before_estimate": before_tokens,
            "history_tokens_after_estimate": after_tokens,
            "summary_input_tokens": completion.as_ref().and_then(|c| c.usage.as_ref()).map(|u| u.prompt_tokens),
            "summary_output_tokens": completion.as_ref().and_then(|c| c.usage.as_ref()).map(|u| u.completion_tokens),
        })
    );
    true
}

pub(super) struct VlmTextCompletion {
    pub(super) text: String,
    pub(super) usage: Option<crate::vlm::VlmUsage>,
}

/// Run one non-streaming VLM completion and retain provider usage for the
/// compaction ledger. Best-effort: callers keep source history if it fails.
pub(super) async fn collect_vlm_text(
    vlm: &VlmClient,
    messages: &[Message],
) -> Option<VlmTextCompletion> {
    let mut stream = vlm
        .chat_stream(messages, &[], None, ReplyShape::Text)
        .await
        .ok()?;
    let mut text = String::new();
    let mut usage = None;
    while let Some(item) = stream.next().await {
        match item.ok()? {
            VlmStreamItem::TextDelta(d) => text.push_str(&d),
            VlmStreamItem::Usage(value) => usage = Some(value),
            VlmStreamItem::ToolCall(_) | VlmStreamItem::Finish => {}
        }
    }
    Some(VlmTextCompletion { text, usage })
}
