// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Pilot config: how a launched pilot process figures out where atlas is,
// what address to bind, and how to reach the LLM upstream.
//
// Three sources, from lowest to highest priority:
//   1. compiled defaults (atlas endpoint, listen address, id, …)
//   2. optional YAML at `$ROBONIX_CONFIG_PATH` or `--config <path>`
//      (used by `rbnx boot` to write a slice of `system.pilot` from
//      `robonix_manifest.yaml`)
//   3. CLI flags / per-field env vars
//
// Higher-priority source overrides lower. Manual launch only needs the
// minimum: an atlas endpoint, a VLM upstream URL, and an API key.

use anyhow::{Context, Result, bail};
use clap::Parser;
use serde::Deserialize;
use std::path::{Path, PathBuf};

pub const DEFAULT_PILOT_PROVIDER_ID: &str = "pilot";
pub const PILOT_NAMESPACE: &str = "robonix/system/pilot";
pub const DEFAULT_ATLAS_ENDPOINT: &str = "127.0.0.1:50051";
pub const DEFAULT_LISTEN: &str = "127.0.0.1:50071";
pub const DEFAULT_VLM_FORMAT: &str = "openai";

/// One direct-provider model-family capacity built into Pilot.
///
/// This is a convenience fallback, not evidence about an arbitrary
/// OpenAI-compatible proxy. Callers must inspect the `builtin_registry` log
/// source and override it when their deployment has a smaller model, a server
/// cap, or an alias with different routing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BuiltinModelProfile {
    pub matcher: &'static str,
    pub context_window_tokens: usize,
}

/// Offline fallback copied from the OFOX `/models` catalogue on 2026-09-23.
/// It contains every catalogue entry that declared a non-zero context length
/// (135 at the time of capture), not just a short list of familiar models.
/// Runtime provider metadata remains authoritative because gateways can change.
const FALLBACK_MODEL_CONTEXT_WINDOWS: &[(&str, usize)] = &[
    ("anthropic/claude-fable-5", 1_000_000),
    ("anthropic/claude-fable-5.1", 1_000_000),
    ("anthropic/claude-haiku-4.5", 200_000),
    ("anthropic/claude-opus-4.6", 1_000_000),
    ("anthropic/claude-opus-4.7", 1_000_000),
    ("anthropic/claude-opus-4.8", 1_000_000),
    ("anthropic/claude-opus-5", 1_000_000),
    ("anthropic/claude-opus-5.5", 1_000_000),
    ("anthropic/claude-sonnet-4.6", 1_000_000),
    ("anthropic/claude-sonnet-5", 1_000_000),
    ("deepseek/deepseek-v3.2", 128_000),
    ("deepseek/deepseek-v4-flash-0423", 1_000_000),
    ("deepseek/deepseek-v4-flash-0731", 1_000_000),
    ("deepseek/deepseek-v4-pro-0423", 1_000_000),
    ("deepseek/deepseek-v4-pro-0813", 1_000_000),
    ("deepseek/deepseek-v4.1-flash", 1_000_000),
    ("google/gemini-2.5-flash", 1_048_576),
    ("google/gemini-2.5-flash-image", 32_000),
    ("google/gemini-2.5-flash-lite", 1_048_576),
    ("google/gemini-2.5-pro", 1_048_576),
    ("google/gemini-3-flash-preview", 1_048_576),
    ("google/gemini-3-pro-image", 66_000),
    ("google/gemini-3.1-flash-image", 131_000),
    ("google/gemini-3.1-flash-lite", 1_000_000),
    ("google/gemini-3.1-flash-lite-image", 66_000),
    ("google/gemini-3.1-pro-preview", 1_048_576),
    ("google/gemini-3.5-flash", 1_000_000),
    ("google/gemini-3.5-flash-lite", 1_000_000),
    ("google/gemini-3.6-flash", 1_000_000),
    ("google/gemini-3.7-flash", 1_000_000),
    ("google/gemini-3.8-flash", 1_000_000),
    ("microsoft/mai-image-2.5", 4_100),
    ("microsoft/mai-image-2.5-flash", 4_100),
    ("microsoft/mai-image-2.5-pro", 4_100),
    ("minimax/m2-her", 200_000),
    ("minimax/minimax-m2", 204_800),
    ("minimax/minimax-m2.1", 204_800),
    ("minimax/minimax-m2.1-lightning", 204_800),
    ("minimax/minimax-m2.5", 200_000),
    ("minimax/minimax-m2.5-lightning", 200_000),
    ("minimax/minimax-m2.7", 200_000),
    ("minimax/minimax-m2.7-highspeed", 200_000),
    ("minimax/minimax-m3", 1_131_000),
    ("moonshotai/kimi-k2.6", 262_144),
    ("moonshotai/kimi-k2.7-code", 262_144),
    ("moonshotai/kimi-k2.7-code-highspeed", 262_144),
    ("moonshotai/kimi-k3", 1_048_576),
    ("openai/gpt-4.1", 1_047_576),
    ("openai/gpt-4.1-mini", 1_047_576),
    ("openai/gpt-4o", 128_000),
    ("openai/gpt-4o-mini", 128_000),
    ("openai/gpt-4o-mini-transcribe", 128_000),
    ("openai/gpt-4o-transcribe-diarize", 128_000),
    ("openai/gpt-5", 256_000),
    ("openai/gpt-5-mini", 256_000),
    ("openai/gpt-5-nano", 128_000),
    ("openai/gpt-5.1", 256_000),
    ("openai/gpt-5.1-codex-max", 256_000),
    ("openai/gpt-5.1-codex-mini", 256_000),
    ("openai/gpt-5.2", 512_000),
    ("openai/gpt-5.2-codex", 512_000),
    ("openai/gpt-5.3-codex", 512_000),
    ("openai/gpt-5.4", 1_050_000),
    ("openai/gpt-5.4-mini", 400_000),
    ("openai/gpt-5.4-nano", 400_000),
    ("openai/gpt-5.4-pro", 1_050_000),
    ("openai/gpt-5.5", 1_050_000),
    ("openai/gpt-5.6-luna", 1_050_000),
    ("openai/gpt-5.6-sol", 1_050_000),
    ("openai/gpt-5.6-terra", 1_050_000),
    ("openai/gpt-6-astra", 1_050_000),
    ("openai/gpt-6-luna", 1_050_000),
    ("openai/gpt-6-sol", 1_050_000),
    ("openai/gpt-transcribe", 128_000),
    ("openai/text-embedding-3-large", 8_200),
    ("openai/text-embedding-3-small", 8_200),
    ("qwen/qwen-flash", 1_000_000),
    ("qwen/qwen-image-3.0", 100_000),
    ("qwen/qwen-image-3.0-pro", 100_000),
    ("qwen/qwen-max", 32_000),
    ("qwen/qwen-plus", 1_000_000),
    ("qwen/qwen-turbo", 128_000),
    ("qwen/qwen-vl-max", 128_000),
    ("qwen/qwen3-coder-flash", 1_000_000),
    ("qwen/qwen3-coder-next", 256_000),
    ("qwen/qwen3-coder-plus", 1_000_000),
    ("qwen/qwen3-max", 256_000),
    ("qwen/qwen3.5-122b-a10b", 256_000),
    ("qwen/qwen3.5-27b", 256_000),
    ("qwen/qwen3.5-35b-a3b", 256_000),
    ("qwen/qwen3.5-397b-a17b", 256_000),
    ("qwen/qwen3.5-flash", 1_000_000),
    ("qwen/qwen3.5-plus", 1_000_000),
    ("qwen/qwen3.6-27b", 256_000),
    ("qwen/qwen3.6-flash", 1_000_000),
    ("qwen/qwen3.6-max-preview", 256_000),
    ("qwen/qwen3.6-plus", 1_000_000),
    ("qwen/qwen3.7-max", 1_064_000),
    ("qwen/qwen3.7-plus", 1_064_000),
    ("qwen/qwen3.8-27b", 1_131_072),
    ("qwen/qwen3.8-flash", 1_131_072),
    ("qwen/qwen3.8-max", 1_131_072),
    ("qwen/qwen3.8-max-0902", 1_000_000),
    ("qwen/text-embedding-v4", 8_192),
    ("volcengine/doubao-seed-1-6", 256_000),
    ("volcengine/doubao-seed-1-6-flash", 256_000),
    ("volcengine/doubao-seed-1-6-vision", 256_000),
    ("volcengine/doubao-seed-1-8", 256_000),
    ("volcengine/doubao-seed-2.0-code", 256_000),
    ("volcengine/doubao-seed-2.0-lite", 256_000),
    ("volcengine/doubao-seed-2.0-mini", 256_000),
    ("volcengine/doubao-seed-2.0-pro", 256_000),
    ("volcengine/doubao-seed-2.1-pro", 256_000),
    ("volcengine/doubao-seed-2.1-turbo", 256_000),
    ("volcengine/doubao-seed-character", 256_000),
    ("volcengine/doubao-seed-evolving", 256_000),
    ("volcengine/doubao-seedream-4.5", 100_000),
    ("volcengine/doubao-seedream-5.0-lite", 100_000),
    ("volcengine/doubao-seedream-5.0-pro", 100_000),
    ("x-ai/grok-4.1-fast", 2_000_000),
    ("x-ai/grok-4.20", 2_000_000),
    ("x-ai/grok-4.3", 1_000_000),
    ("x-ai/grok-4.5", 500_000),
    ("x-ai/grok-4.6", 500_000),
    ("x-ai/grok-4.7", 500_000),
    ("z-ai/glm-4.6", 200_000),
    ("z-ai/glm-4.7", 200_000),
    ("z-ai/glm-4.7-flashx", 200_000),
    ("z-ai/glm-5", 200_000),
    ("z-ai/glm-5-turbo", 200_000),
    ("z-ai/glm-5.1", 200_000),
    ("z-ai/glm-5.2", 1_048_576),
    ("z-ai/glm-5.3", 1_048_576),
    ("z-ai/glm-5.3-flash", 1_048_576),
    ("z-ai/glm-5v-turbo", 200_000),
];

fn canonical_model_name(model: &str) -> String {
    model
        .trim()
        .rsplit(['/', ':'])
        .next()
        .unwrap_or_default()
        .to_ascii_lowercase()
        .replace('_', "-")
}

/// Return a context capacity from the complete pinned OFOX catalogue.
/// Exact full IDs win. A bare model name is used only when its canonical
/// terminal name appears once, so a namespace collision never selects a model
/// silently. The live `/models` response is checked before this fallback.
pub fn builtin_model_profile(model: &str) -> Option<BuiltinModelProfile> {
    let exact = FALLBACK_MODEL_CONTEXT_WINDOWS
        .iter()
        .find(|(id, _)| id.eq_ignore_ascii_case(model.trim()));
    let matched = match exact {
        Some(entry) => entry,
        None => {
            let canonical = canonical_model_name(model);
            let mut entries = FALLBACK_MODEL_CONTEXT_WINDOWS
                .iter()
                .filter(|(id, _)| canonical_model_name(id) == canonical);
            let first = entries.next()?;
            entries.next().is_none().then_some(first)?
        }
    };
    Some(BuiltinModelProfile {
        matcher: "ofox_catalogue_2026-09-23",
        context_window_tokens: matched.1,
    })
}

/// Fully-resolved settings the pilot binary runs against.
#[derive(Debug, Clone)]
pub struct PilotConfig {
    pub atlas_endpoint: String,
    pub listen: String,
    pub id: String,
    pub vlm: VlmConfig,
}

#[derive(Debug, Clone)]
pub struct VlmConfig {
    pub upstream: String,
    pub api_key: String,
    pub model: String,
    /// Context limit declared by the deployment. This wins over provider
    /// metadata and the built-in registry because a proxy may expose a logical
    /// model name while routing to a smaller physical context window.
    pub context_window_tokens: Option<usize>,
    /// Capacity reserved for the next planner completion when deciding whether
    /// the conversation can continue without compaction.
    pub reserved_output_tokens: usize,
    /// Extra token headroom for provider framing, tokenizer drift, and
    /// reasoning tokens that are not visible to the chat-completions client.
    pub context_safety_tokens: usize,
    /// Wire dialect. Currently only "openai" is implemented; checked at
    /// `resolve` time, kept on the struct for diagnostics / future routing.
    #[allow(dead_code)]
    pub api_format: String,
}

/// CLI surface; every field is optional so config-file mode stays usable
/// without spelling out flags. clap also reads the listed env vars.
#[derive(Parser, Debug)]
#[command(name = "robonix-pilot", about = "Robonix Pilot — VLM planner")]
pub struct Args {
    /// Atlas control-plane endpoint. Also reads `ROBONIX_ATLAS` (the var rbnx /
    /// the Python API / liaison use) as an alias; see `env_atlas`.
    #[arg(long, env = "ROBONIX_ATLAS_ENDPOINT")]
    pub atlas: Option<String>,

    /// Address pilot's SystemPilot gRPC binds to.
    #[arg(long, env = "ROBONIX_PILOT_LISTEN")]
    pub listen: Option<String>,

    /// Override pilot's id (singleton, rarely needed).
    #[arg(long, env = "ROBONIX_PILOT_PROVIDER_ID")]
    pub id: Option<String>,

    /// LLM API base URL (e.g. <https://api.openai.com/v1>).
    #[arg(long, env = "ROBONIX_VLM_UPSTREAM")]
    pub vlm_upstream: Option<String>,

    /// LLM API key.
    #[arg(long, env = "ROBONIX_VLM_API_KEY")]
    pub vlm_api_key: Option<String>,

    /// LLM model identifier.
    #[arg(long, env = "ROBONIX_VLM_MODEL")]
    pub vlm_model: Option<String>,

    /// Deployment-specific total context window. Required when Pilot cannot
    /// obtain a capacity from provider metadata or the built-in model registry.
    #[arg(long, env = "ROBONIX_VLM_CONTEXT_WINDOW_TOKENS")]
    pub vlm_context_window_tokens: Option<usize>,

    /// Tokens reserved for the next planning response while budgeting history.
    #[arg(long, env = "ROBONIX_VLM_RESERVED_OUTPUT_TOKENS")]
    pub vlm_reserved_output_tokens: Option<usize>,

    /// Conservative headroom for provider framing and hidden reasoning tokens.
    #[arg(long, env = "ROBONIX_VLM_CONTEXT_SAFETY_TOKENS")]
    pub vlm_context_safety_tokens: Option<usize>,

    /// LLM API dialect ("openai" only for now).
    #[arg(long, env = "ROBONIX_VLM_FORMAT")]
    pub vlm_format: Option<String>,

    /// YAML config file (rbnx writes this; CLI/env still override individual fields).
    #[arg(long, env = "ROBONIX_CONFIG_PATH")]
    pub config: Option<PathBuf>,

    /// Log level for this component (`debug`/`info`/`warn`/`error`). Sets the
    /// scribe log-file floor; falls back to `SCRIBE_FILE_LEVEL` / `info`.
    /// Normally arrives inside `--config-json`, not as a standalone flag.
    #[arg(long)]
    pub log: Option<String>,

    /// The component's `system.pilot` manifest block, serialized to JSON by
    /// rbnx and passed as one arg (`--config-json '{…}'`). Parsed by the
    /// binary itself — see `robonix_scribe::init_from_config`, which reads the
    /// `log` key from it so the manifest's per-component level reaches the log.
    #[arg(long)]
    pub config_json: Option<String>,
}

/// Optional YAML schema. Field names match `PilotConfig` (flat) so a
/// hand-written file looks like the manifest's `system.pilot` block.
#[derive(Default, Deserialize)]
struct FileConfig {
    #[serde(default)]
    atlas_endpoint: Option<String>,
    #[serde(default)]
    listen: Option<String>,
    #[serde(default)]
    id: Option<String>,
    #[serde(default)]
    vlm: Option<FileVlmConfig>,
}

#[derive(Default, Deserialize)]
struct FileVlmConfig {
    #[serde(default)]
    upstream: Option<String>,
    #[serde(default)]
    api_key: Option<String>,
    #[serde(default)]
    model: Option<String>,
    #[serde(default)]
    api_format: Option<String>,
    #[serde(default)]
    context_window_tokens: Option<usize>,
    #[serde(default)]
    reserved_output_tokens: Option<usize>,
    #[serde(default)]
    context_safety_tokens: Option<usize>,
}

const DEFAULT_RESERVED_OUTPUT_TOKENS: usize = 4_096;
const DEFAULT_CONTEXT_SAFETY_TOKENS: usize = 2_048;

impl PilotConfig {
    /// Build the resolved config from CLI args (which already pulled env
    /// vars). Reads optional YAML; CLI/env still override file fields.
    pub fn resolve(args: Args) -> Result<Self> {
        let file_cfg: FileConfig = match &args.config {
            Some(path) => load_yaml(path)?,
            None => FileConfig::default(),
        };
        let file_vlm = file_cfg.vlm.unwrap_or_default();

        let atlas_endpoint = args
            .atlas
            .or_else(env_atlas)
            .or(file_cfg.atlas_endpoint)
            .unwrap_or_else(|| DEFAULT_ATLAS_ENDPOINT.to_string());
        let listen = args
            .listen
            .or(file_cfg.listen)
            .unwrap_or_else(|| DEFAULT_LISTEN.to_string());
        let id = args
            .id
            .or(file_cfg.id)
            .unwrap_or_else(|| DEFAULT_PILOT_PROVIDER_ID.to_string());
        let api_format = args
            .vlm_format
            .or(file_vlm.api_format)
            .unwrap_or_else(|| DEFAULT_VLM_FORMAT.to_string());
        if api_format != "openai" {
            bail!("vlm api_format='{api_format}' not supported (only 'openai')");
        }

        let upstream = args
            .vlm_upstream
            .or(file_vlm.upstream)
            .filter(|s| !s.trim().is_empty())
            .ok_or_else(|| {
                missing_field("vlm.upstream", "ROBONIX_VLM_UPSTREAM", "--vlm-upstream")
            })?;
        let api_key = args
            .vlm_api_key
            .or(file_vlm.api_key)
            .filter(|s| !s.trim().is_empty())
            .ok_or_else(|| missing_field("vlm.api_key", "ROBONIX_VLM_API_KEY", "--vlm-api-key"))?;
        let model = args
            .vlm_model
            .or(file_vlm.model)
            .filter(|s| !s.trim().is_empty())
            .ok_or_else(|| missing_field("vlm.model", "ROBONIX_VLM_MODEL", "--vlm-model"))?;
        let context_window_tokens = args
            .vlm_context_window_tokens
            .or(file_vlm.context_window_tokens)
            .filter(|tokens| *tokens > 0);
        let reserved_output_tokens = args
            .vlm_reserved_output_tokens
            .or(file_vlm.reserved_output_tokens)
            .filter(|tokens| *tokens > 0)
            .unwrap_or(DEFAULT_RESERVED_OUTPUT_TOKENS);
        let context_safety_tokens = args
            .vlm_context_safety_tokens
            .or(file_vlm.context_safety_tokens)
            .unwrap_or(DEFAULT_CONTEXT_SAFETY_TOKENS);

        Ok(Self {
            atlas_endpoint,
            listen,
            id,
            vlm: VlmConfig {
                upstream,
                api_key,
                model,
                context_window_tokens,
                reserved_output_tokens,
                context_safety_tokens,
                api_format,
            },
        })
    }
}

/// Read the `ROBONIX_ATLAS` env var as an atlas-endpoint alias.
///
/// rbnx, the Python API, and liaison all configure the atlas endpoint via
/// `ROBONIX_ATLAS`, while executor/pilot historically only honored
/// `ROBONIX_ATLAS_ENDPOINT` (the clap `env`). Accepting `ROBONIX_ATLAS` here as
/// well means a single env var configures every component. Without it, setting
/// only `ROBONIX_ATLAS` left pilot silently falling back to
/// `DEFAULT_ATLAS_ENDPOINT` (127.0.0.1:50051) — it would then dial the wrong
/// atlas and log 127.0.0.1 even after the operator "changed" the endpoint.
/// Empty values are ignored so an exported-but-blank var doesn't shadow later
/// sources.
fn env_atlas() -> Option<String> {
    std::env::var("ROBONIX_ATLAS")
        .ok()
        .filter(|v| !v.is_empty())
}

fn load_yaml(path: &Path) -> Result<FileConfig> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("read pilot config '{}'", path.display()))?;
    serde_yaml::from_str(&raw).with_context(|| format!("parse pilot config '{}'", path.display()))
}

fn missing_field(yaml_path: &str, env_var: &str, flag: &str) -> anyhow::Error {
    anyhow::anyhow!(
        "missing required field '{yaml_path}': set it in --config YAML, env {env_var}, or pass {flag}"
    )
}

#[cfg(test)]
mod tests {
    use super::builtin_model_profile;

    #[test]
    fn builtins_cover_current_ofox_catalogue_and_reject_unknown_suffixes() {
        assert_eq!(
            builtin_model_profile("gpt-5.6-terra").map(|profile| profile.context_window_tokens),
            Some(1_050_000)
        );
        assert_eq!(
            builtin_model_profile("anthropic/claude-opus-5.5")
                .map(|profile| profile.context_window_tokens),
            Some(1_000_000)
        );
        assert!(builtin_model_profile("gpt-5.6-terra-via-small-proxy").is_none());
    }
}
