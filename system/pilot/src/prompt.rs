// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Prompt assembly and provider-usage accounting for Pilot. This module owns
// message ordering; planner owns what each round observes and dispatches.

use crate::history;
use crate::vlm::{Message, VlmUsage};
use robonix_scribe::info;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

pub(crate) struct PromptSection<'a> {
    pub(crate) name: &'static str,
    pub(crate) content: &'a str,
}

/// Provider-reported token accounting for one Pilot interaction. Pricing is
/// deliberately not embedded here because a proxy may route the same model
/// name to differently priced backends. The raw input/output/cache totals are
/// sufficient for an operator to apply the configured provider's price sheet.
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
    pub(crate) fn record(&mut self, round: u32, usage: &VlmUsage) -> serde_json::Value {
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

fn estimated_text_tokens(bytes: usize) -> usize {
    bytes.div_ceil(4)
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
    context_sections: &[PromptSection<'_>],
) -> Vec<Message> {
    let system_bytes = sections.iter().map(|section| section.content.len()).sum();
    let mut system = String::with_capacity(system_bytes);
    for section in sections {
        system.push_str(section.content);
    }
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
        .chain(context_sections.iter())
        .map(|section| {
            serde_json::json!({
                "name": section.name,
                "bytes": section.content.len(),
                "estimated_tokens": estimated_text_tokens(section.content.len()),
            })
        })
        .collect::<Vec<_>>();
    section_metrics.push(serde_json::json!({
        "name": "history",
        "bytes": history_bytes,
        "estimated_tokens": estimated_text_tokens(history_bytes),
    }));
    info!(
        "[pilot/prompt] {}",
        serde_json::json!({
            "round": round,
            "prompt_text_bytes": prompt_bytes,
            "estimated_input_tokens": estimated_text_tokens(prompt_bytes),
            "history_bytes": history_bytes,
            "capability_catalog_render_cache_hit": capability_cache_hit,
            "cacheable_prefix_bytes": system.len(),
            "cacheable_prefix_fingerprint": cacheable_prefix_fingerprint,
            "sections": section_metrics,
        })
    );

    let mut messages = Vec::with_capacity(sanitized_history.len() + 2);
    messages.push(Message::system(&system));
    messages.extend(sanitized_history);
    close_trailing_assistant(&mut messages);
    messages
}

pub(crate) fn render_context_sections(sections: &[PromptSection<'_>]) -> String {
    let bytes = sections.iter().map(|section| section.content.len()).sum();
    let mut context = String::with_capacity(bytes);
    for section in sections {
        context.push_str(section.content);
    }
    context
}

/// Some providers reject a trailing assistant message as a completion prefill.
/// Close it with an explicit next-action user turn instead.
pub(crate) fn close_trailing_assistant(messages: &mut Vec<Message>) {
    let trailing_assistant = messages
        .last()
        .is_some_and(|message| message.role == "assistant");
    if trailing_assistant {
        messages.push(Message::user(
            "Continue from the state above. Take the next action, \
             or give your final answer if the task is complete.",
        ));
    }
}
