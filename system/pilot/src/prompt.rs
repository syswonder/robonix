// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Prompt assembly and provider-usage accounting for Pilot. This module owns
// message ordering; planner owns what each round observes and dispatches.

use crate::discovery::CapDoc;
use crate::history;
use crate::vlm::{Message, VlmUsage};
use robonix_scribe::{debug, info};
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

pub(crate) type PromptSection<'a> = (&'static str, &'a str);

/// Provider-reported token and cache totals for one Pilot interaction.
#[derive(Default)]
pub(crate) struct UsageTotals {
    requests_with_usage: u64,
    input_tokens: u64,
    output_tokens: u64,
    requests_with_cache_metrics: u64,
    cache_hit_requests: u64,
    cache_metric_input_tokens: u64,
    cached_input_tokens: u64,
    uncached_input_tokens: u64,
}

impl UsageTotals {
    /// Accumulate reported usage; absent cache metrics remain unknown.
    pub(crate) fn record(
        &mut self,
        round: u32,
        cache_epoch: u64,
        usage: &VlmUsage,
    ) -> serde_json::Value {
        self.requests_with_usage = self.requests_with_usage.saturating_add(1);
        self.input_tokens = self.input_tokens.saturating_add(usage.prompt_tokens);
        self.output_tokens = self.output_tokens.saturating_add(usage.completion_tokens);

        let cached = usage
            .cached_tokens
            .map(|tokens| tokens.min(usage.prompt_tokens));
        let uncached = cached.map(|tokens| usage.prompt_tokens.saturating_sub(tokens));
        if let (Some(cached), Some(uncached)) = (cached, uncached) {
            self.requests_with_cache_metrics = self.requests_with_cache_metrics.saturating_add(1);
            self.cache_metric_input_tokens = self
                .cache_metric_input_tokens
                .saturating_add(usage.prompt_tokens);
            self.cached_input_tokens = self.cached_input_tokens.saturating_add(cached);
            self.uncached_input_tokens = self.uncached_input_tokens.saturating_add(uncached);
            if cached > 0 {
                self.cache_hit_requests = self.cache_hit_requests.saturating_add(1);
            }
        }

        let request_cache_hit_ratio = cached.and_then(|tokens| {
            (usage.prompt_tokens > 0).then_some(tokens as f64 / usage.prompt_tokens as f64)
        });
        let cumulative_cache_hit_ratio = (self.cache_metric_input_tokens > 0)
            .then_some(self.cached_input_tokens as f64 / self.cache_metric_input_tokens as f64);

        serde_json::json!({
            "event": "vlm_usage",
            "round": round,
            "cache_epoch": cache_epoch,
            "input_tokens": usage.prompt_tokens,
            "output_tokens": usage.completion_tokens,
            "total_tokens": usage.prompt_tokens.saturating_add(usage.completion_tokens),
            "cached_input_tokens": cached,
            "uncached_input_tokens": uncached,
            "cache_hit": cached.map(|tokens| tokens > 0),
            "cache_hit_ratio": request_cache_hit_ratio,
            "provider_prompt_tokens": usage.prompt_tokens,
            "provider_completion_tokens": usage.completion_tokens,
            "provider_cached_tokens": usage.cached_tokens,
            "cumulative": {
                "requests_with_usage": self.requests_with_usage,
                "input_tokens": self.input_tokens,
                "output_tokens": self.output_tokens,
                "total_tokens": self.input_tokens.saturating_add(self.output_tokens),
                "requests_with_cache_metrics": self.requests_with_cache_metrics,
                "cache_hit_requests": self.cache_hit_requests,
                "cache_metric_input_tokens": self.cache_metric_input_tokens,
                "cached_input_tokens": self.cached_input_tokens,
                "uncached_input_tokens": self.uncached_input_tokens,
                "cache_hit_ratio": cumulative_cache_hit_ratio,
            },
        })
    }
}

/// Assemble the exact provider message sequence for a planning request.
/// Standing sections must precede history. The caller persists each rendered
/// runtime context in history before the corresponding assistant reply, so a
/// later request extends rather than reorders this sequence.
pub(crate) fn assemble_planning_messages(
    round: u32,
    capability_cache_hit: bool,
    sections: &[PromptSection<'_>],
    history_messages: &[Message],
) -> Vec<Message> {
    let system = render_context_sections(sections);
    let mut prefix_hasher = DefaultHasher::new();
    system.hash(&mut prefix_hasher);
    let cacheable_prefix_fingerprint = prefix_hasher.finish();
    let sanitized_history = history::sanitize_for_vlm(history_messages);
    let history_bytes: usize = sanitized_history
        .iter()
        .map(|message| message.content.as_deref().map_or(0, str::len))
        .sum();
    let prompt_bytes = system.len() + history_bytes;
    let mut section_metrics = sections
        .iter()
        .map(|(name, content)| {
            serde_json::json!({
                "name": name,
                "bytes": content.len(),
                "estimated_tokens": content.len().div_ceil(4),
            })
        })
        .collect::<Vec<_>>();
    section_metrics.push(serde_json::json!({
        "name": "history",
        "bytes": history_bytes,
        "estimated_tokens": history_bytes.div_ceil(4),
    }));
    info!(
        "[pilot/prompt] {}",
        serde_json::json!({
            "round": round,
            "prompt_text_bytes": prompt_bytes,
            "estimated_input_tokens": prompt_bytes.div_ceil(4),
            "history_bytes": history_bytes,
            "capability_catalog_render_cache_hit": capability_cache_hit,
            "cacheable_prefix_bytes": system.len(),
            "cacheable_prefix_fingerprint": cacheable_prefix_fingerprint,
            "sections": section_metrics,
        })
    );

    let mut messages = Vec::with_capacity(sanitized_history.len() + 2);
    // Keep the stable developer prefix ahead of user-side runtime history.
    messages.push(Message::developer(&system));
    messages.extend(sanitized_history);
    close_trailing_assistant(&mut messages);
    debug!(
        "[pilot/prompt/messages] {}",
        serde_json::json!({ "round": round, "messages": debug_view(&messages) })
    );
    messages
}

/// The exact request messages for the debug log, with inline images reduced to
/// their size. The `info` record above carries only section sizes and token
/// estimates; this one carries the full text and is written only at debug level.
fn debug_view(messages: &[Message]) -> Vec<serde_json::Value> {
    messages
        .iter()
        .map(|message| {
            let mut value = serde_json::to_value(message).unwrap_or_default();
            if let Some(image) = message.image_base64.as_deref() {
                value["image_base64"] = serde_json::json!(format!("<{} bytes>", image.len()));
            }
            value
        })
        .collect()
}

pub(crate) fn render_context_sections(sections: &[PromptSection<'_>]) -> String {
    sections.iter().map(|(_, content)| *content).collect()
}

pub(crate) fn render_capability_docs(docs: &[CapDoc]) -> String {
    if docs.is_empty() {
        return String::new();
    }
    let mut out = String::from(
        "\n\n## Capability docs\nRead a `[skill]` provider's manual with `read_capability_doc` before its first call; primitive and service manuals are optional. Use only the listed provider id.\n",
    );
    for doc in docs {
        let skill = if doc.kind == "skill" { " [skill]" } else { "" };
        out.push_str(&format!(
            "- `{}`{}: {}\n",
            doc.provider_id, skill, doc.description
        ));
    }
    out
}

pub(crate) fn changed_snapshot(previous: &mut String, current: String) -> String {
    if *previous == current {
        return String::new();
    }
    *previous = current.clone();
    current
}

/// Some providers reject a trailing assistant message as a completion prefill.
/// Close it with an explicit next-action user turn instead.
pub(crate) fn close_trailing_assistant(messages: &mut Vec<Message>) {
    if messages
        .last()
        .is_some_and(|message| message.role == "assistant")
    {
        messages.push(Message::user(
            "Continue from the state above. Take the next action, \
             or give your final answer if the task is complete.",
        ));
    }
}
