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
}

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

        Ok(Self {
            atlas_endpoint,
            listen,
            id,
            vlm: VlmConfig {
                upstream,
                api_key,
                model,
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
