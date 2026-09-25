// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Prompt assembly and provider-usage accounting for Pilot. This module owns
// message ordering; planner owns what each round observes and dispatches.

use crate::atlas_pb;
use crate::discovery::CapDoc;
use crate::history;
use crate::planner::DisplayCapability;
use crate::vlm::{Message, VlmUsage};
use robonix_scribe::{debug, info};
use std::collections::BTreeMap;
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
    catalog_unchanged: bool,
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
            "capability_catalog_unchanged": catalog_unchanged,
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

/// The capability catalog as the model last saw it in this task.
///
/// The catalog is carried in history rather than the system prefix, so a
/// provider registering or leaving never invalidates the cached prefix. The
/// first round of a task, and the first round after history compaction,
/// carry the full catalog; later rounds carry only the entries added,
/// changed, or removed since, and nothing when the catalog is unchanged.
///
/// A round's record counts as shown only once that round is written to
/// history; a round dropped for a steer or a stale plan leaves the view as it
/// was, so the next round repeats what the model has not yet seen.
#[derive(Default)]
pub(crate) struct CatalogView {
    shown: Option<BTreeMap<String, String>>,
    staged: Option<BTreeMap<String, String>>,
}

impl CatalogView {
    /// Render this round's catalog record; empty when nothing changed.
    pub(crate) fn update(&mut self, caps: &[DisplayCapability<'_>]) -> String {
        let current = capability_entries(caps);
        let record = match &self.shown {
            None => render_full_catalog(&current),
            Some(shown) => render_catalog_changes(shown, &current),
        };
        self.staged = Some(current);
        record
    }

    /// The round's record is now in history.
    pub(crate) fn commit(&mut self) {
        if let Some(staged) = self.staged.take() {
            self.shown = Some(staged);
        }
    }

    /// Forget what was shown, so the next round carries the full catalog.
    pub(crate) fn reset(&mut self) {
        self.shown = None;
    }
}

/// Maximum inline description length; full documentation is loaded on demand.
pub(crate) const MAX_INLINE_DESCRIPTION_CHARS: usize = 300;

/// Return a character-bounded opening paragraph and whether text was omitted.
pub(crate) fn summarize_description(description: &str) -> (String, bool) {
    let full = description.trim();
    let first = full.split("\n\n").next().unwrap_or(full).trim();
    let summary: String = first.chars().take(MAX_INLINE_DESCRIPTION_CHARS).collect();
    let truncated = summary.len() < full.len();
    (summary, truncated)
}

/// One catalog entry per callable capability, keyed by its capability name,
/// with the description escaped and the input schema inlined.
pub(crate) fn capability_entries(
    display_caps: &[DisplayCapability<'_>],
) -> BTreeMap<String, String> {
    let mut entries = BTreeMap::new();
    for cap in display_caps {
        let c = cap.cap;
        let Some(atlas_pb::transport_params::Kind::Mcp(mcp)) =
            c.params.as_ref().and_then(|params| params.kind.as_ref())
        else {
            continue;
        };
        let schema: serde_json::Value =
            serde_json::from_str(&mcp.input_schema_json).unwrap_or(serde_json::Value::Null);
        let (summary, truncated) = summarize_description(&c.description);
        let description = serde_json::to_string(&summary).unwrap_or_else(|_| "\"\"".to_string());
        let mut entry = format!(
            "- capability_name: {}\n  description: {}\n  args_schema: {}\n",
            cap.display_name, description, schema
        );
        if truncated {
            entry.push_str(&format!(
                "  more: call `read_capability_doc` with provider_id `{}` for this \
                 capability's full description\n",
                cap.provider_id
            ));
        }
        entries.insert(cap.display_name.clone(), entry);
    }
    entries
}

/// The whole catalog, stated as replacing any catalog shown earlier.
pub(crate) fn render_full_catalog(entries: &BTreeMap<String, String>) -> String {
    let mut out = String::from(
        "\n\n## Available capabilities\nThis is the complete catalog; it replaces any earlier catalog in this conversation.\n\n",
    );
    out.extend(entries.values().map(String::as_str));
    out
}

/// What changed since `shown`; empty when nothing did.
fn render_catalog_changes(
    shown: &BTreeMap<String, String>,
    current: &BTreeMap<String, String>,
) -> String {
    let added: Vec<&str> = current
        .iter()
        .filter(|(name, _)| !shown.contains_key(*name))
        .map(|(_, entry)| entry.as_str())
        .collect();
    let changed: Vec<&str> = current
        .iter()
        .filter(|(name, entry)| shown.get(*name).is_some_and(|old| old != *entry))
        .map(|(_, entry)| entry.as_str())
        .collect();
    let removed: Vec<String> = shown
        .keys()
        .filter(|name| !current.contains_key(*name))
        .map(|name| format!("`{name}`"))
        .collect();
    if added.is_empty() && changed.is_empty() && removed.is_empty() {
        return String::new();
    }
    let mut out = String::from(
        "\n\n## Available capabilities: changes\nApply these to the latest catalog above; entries not listed are unchanged.\n",
    );
    if !added.is_empty() {
        out.push_str("Added:\n");
        out.extend(added);
    }
    if !changed.is_empty() {
        out.push_str("Changed (new entry):\n");
        out.extend(changed);
    }
    if !removed.is_empty() {
        out.push_str(&format!(
            "Removed, no longer callable: {}\n",
            removed.join(", ")
        ));
    }
    out
}
