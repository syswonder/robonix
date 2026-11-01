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
use robonix_llm_catalog::{
    ModelSpec, Requirements, api_key_for, catalog_from_env, resolve_from_env,
};
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

/// Connection settings used by Pilot's model client.
#[derive(Debug, Clone)]
pub struct VlmConfig {
    pub upstream: String,
    pub api_key: String,
    pub model: String,
    /// Selected catalog id; `None` means an explicit endpoint was configured.
    pub catalog_id: Option<String>,
    /// Context limit declared by the deployment. This wins over provider
    /// metadata and the built-in registry because a proxy may expose a logical
    /// model name while routing to a smaller physical context window.
    pub context_window_tokens: Option<usize>,
    /// Wire dialect. Currently only "openai" is implemented; checked at
    /// `resolve` time, kept on the struct for diagnostics / future routing.
    #[allow(dead_code)]
    pub api_format: String,
}

impl VlmConfig {
    /// The upstream URL without credentials or a query, safe to log.
    pub fn endpoint_for_log(&self) -> String {
        let url = self.upstream.split(['?', '#']).next().unwrap_or("");
        let Some((scheme, rest)) = url.split_once("://") else {
            return url.to_string();
        };
        let (authority, path) = rest.split_once('/').map_or((rest, ""), |(a, p)| (a, p));
        let host = authority
            .rsplit_once('@')
            .map_or(authority, |(_, host)| host);
        if path.is_empty() {
            format!("{scheme}://{host}")
        } else {
            format!("{scheme}://{host}/{path}")
        }
    }
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
    /// obtain a capacity from provider metadata.
    #[arg(long, env = "ROBONIX_VLM_CONTEXT_WINDOW_TOKENS")]
    pub vlm_context_window_tokens: Option<usize>,

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

        let explicit_upstream = args
            .vlm_upstream
            .or(file_vlm.upstream)
            .filter(|value| !value.trim().is_empty());
        let explicit_model = args
            .vlm_model
            .or(file_vlm.model)
            .filter(|value| !value.trim().is_empty());
        let explicit_key = args
            .vlm_api_key
            .or(file_vlm.api_key)
            .filter(|value| !value.trim().is_empty());
        let explicit_context = args
            .vlm_context_window_tokens
            .or(file_vlm.context_window_tokens)
            .filter(|tokens| *tokens > 0);

        // A complete vlm block is explicit deployment config: use it as-is
        // and never let a user-level catalog override it.
        if let Some(upstream) = explicit_upstream {
            let model = explicit_model
                .ok_or_else(|| missing_field("vlm.model", "ROBONIX_VLM_MODEL", "--vlm-model"))?;
            let api_key = explicit_key.ok_or_else(|| {
                missing_field("vlm.api_key", "ROBONIX_VLM_API_KEY", "--vlm-api-key")
            })?;
            return Ok(Self {
                atlas_endpoint,
                listen,
                id,
                vlm: VlmConfig {
                    upstream,
                    api_key,
                    model,
                    catalog_id: None,
                    context_window_tokens: explicit_context,
                    api_format,
                },
            });
        }

        // A model name without an upstream selects a catalog entry. The
        // caller picks the model; the catalog supplies the endpoint and,
        // unless an explicit key is set, the key.
        if let Some(model_id) = explicit_model {
            let catalog = catalog_from_env()?.ok_or_else(|| {
                anyhow::anyhow!(
                    "vlm.model {model_id:?} names a catalog id, but ROBONIX_LLM_CATALOG is unset"
                )
            })?;
            let target = catalog
                .model(&model_id)
                .with_context(|| format!("vlm.model {model_id:?} is not a catalog id"))?;
            return Ok(Self {
                atlas_endpoint,
                listen,
                id,
                vlm: vlm_from_catalog(target, explicit_key, explicit_context, api_format)?,
            });
        }

        // No explicit VLM block: resolve from the catalog by requirements.
        let requirements = env_llm_requirements();
        if let Some(target) = resolve_from_env(&requirements)? {
            return Ok(Self {
                atlas_endpoint,
                listen,
                id,
                vlm: vlm_from_catalog(&target, explicit_key, explicit_context, api_format)?,
            });
        }

        Err(missing_field(
            "vlm.upstream",
            "ROBONIX_VLM_UPSTREAM",
            "--vlm-upstream",
        ))
    }
}

/// Build Pilot's VLM config from a catalog entry. The explicit key, when
/// present, wins over the catalog's `api_key_env`; a keyless entry needs none.
fn vlm_from_catalog(
    target: &ModelSpec,
    explicit_key: Option<String>,
    context_override: Option<usize>,
    api_format: String,
) -> Result<VlmConfig> {
    if target.protocol != "openai-chat" {
        bail!(
            "catalog target {} uses protocol {}; pilot only speaks openai-chat",
            target.id,
            target.protocol
        );
    }
    let api_key = explicit_key.unwrap_or_else(|| api_key_for(target));
    if target.api_key_env.is_some() && api_key.is_empty() {
        bail!(
            "missing {} for catalog target {}",
            target.api_key_env.as_deref().unwrap_or(""),
            target.id
        );
    }
    let context_window_tokens = context_override.or(target.context.map(|tokens| tokens as usize));
    Ok(VlmConfig {
        upstream: target.base_url.clone(),
        api_key,
        model: target.model.clone(),
        catalog_id: Some(target.id.clone()),
        context_window_tokens,
        api_format,
    })
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

/// Features and budgets for the catalog lookup. The defaults describe what
/// Pilot's planning requests need; every value can be overridden per
/// deployment. `json_schema` is intentionally absent: Pilot already downgrades
/// a rejected schema at request time, so requiring the feature here would
/// exclude a capable json_object model such as DeepSeek Flash.
fn env_llm_requirements() -> Requirements {
    Requirements::from_env("vision,tools,stream")
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
    use super::*;
    use std::sync::{Mutex, MutexGuard};

    static ENV_LOCK: Mutex<()> = Mutex::new(());

    struct EnvGuard {
        _lock: MutexGuard<'static, ()>,
        path: Option<PathBuf>,
    }

    impl Drop for EnvGuard {
        fn drop(&mut self) {
            unsafe {
                for name in [
                    "ROBONIX_LLM_CATALOG",
                    "CAT_KEY",
                    "ROBONIX_LLM_FEATURES",
                    "ROBONIX_LLM_INPUT_TOKENS",
                    "ROBONIX_LLM_MAX_OUTPUT_TOKENS",
                    "ROBONIX_LLM_MAX_LATENCY_MS",
                    "ROBONIX_LLM_MAX_COST_USD_PER_CALL",
                    "ROBONIX_LLM_ALLOW_DOWNGRADE",
                ] {
                    std::env::remove_var(name);
                }
            }
            if let Some(path) = &self.path {
                let _ = std::fs::remove_file(path);
            }
        }
    }

    fn catalog_env(name: &str, body: &str) -> EnvGuard {
        let lock = ENV_LOCK.lock().expect("env lock");
        let path = std::env::temp_dir().join(format!(
            "robonix-pilot-config-{}-{name}.json",
            std::process::id()
        ));
        std::fs::write(&path, body).expect("write catalog");
        unsafe {
            std::env::set_var("ROBONIX_LLM_CATALOG", &path);
            std::env::set_var("CAT_KEY", "secret");
            std::env::remove_var("ROBONIX_LLM_FEATURES");
            std::env::remove_var("ROBONIX_LLM_INPUT_TOKENS");
            std::env::remove_var("ROBONIX_LLM_MAX_OUTPUT_TOKENS");
            std::env::remove_var("ROBONIX_LLM_MAX_LATENCY_MS");
            std::env::remove_var("ROBONIX_LLM_MAX_COST_USD_PER_CALL");
            std::env::remove_var("ROBONIX_LLM_ALLOW_DOWNGRADE");
        }
        EnvGuard {
            _lock: lock,
            path: Some(path),
        }
    }

    fn base_args() -> Args {
        Args {
            atlas: Some("127.0.0.1:1".into()),
            listen: Some("127.0.0.1:0".into()),
            id: Some("pilot".into()),
            vlm_upstream: None,
            vlm_api_key: None,
            vlm_model: None,
            vlm_context_window_tokens: None,
            vlm_format: Some("openai".into()),
            config: None,
            log: None,
            config_json: None,
        }
    }

    fn catalog_body(features: &str, key_env: Option<&str>) -> String {
        let key = key_env
            .map(|name| format!(r#""api_key_env": "{name}","#))
            .unwrap_or_default();
        format!(
            r#"{{"models":[{{"id":"catalog/model","provider":"test",
            "base_url":"http://catalog.example/v1","model":"catalog-model",
            {key}"features":{{{features}}},"context":8192}}]}}"#
        )
    }

    #[test]
    fn explicit_vlm_block_does_not_consult_the_catalog() {
        let _env = catalog_env(
            "explicit",
            r#"{"models":[{"id":"text/model","provider":"test",
            "base_url":"http://text.example/v1","model":"text",
            "features":{"tools":true}}]}"#,
        );
        let mut args = base_args();
        args.vlm_upstream = Some("http://explicit.example/v1".into());
        args.vlm_api_key = Some("explicit-key".into());
        args.vlm_model = Some("explicit-model".into());
        args.vlm_context_window_tokens = Some(4096);
        let config = PilotConfig::resolve(args).expect("explicit config");
        assert_eq!(config.vlm.upstream, "http://explicit.example/v1");
        assert_eq!(config.vlm.api_key, "explicit-key");
        assert_eq!(config.vlm.model, "explicit-model");
        assert_eq!(config.vlm.context_window_tokens, Some(4096));
    }

    #[test]
    fn model_id_selects_a_catalog_entry() {
        let _env = catalog_env(
            "model-id",
            &catalog_body(
                r#""vision":true,"tools":true,"stream":true"#,
                Some("CAT_KEY"),
            ),
        );
        let mut args = base_args();
        args.vlm_model = Some("catalog/model".into());
        let config = PilotConfig::resolve(args).expect("catalog id");
        assert_eq!(config.vlm.upstream, "http://catalog.example/v1");
        assert_eq!(config.vlm.api_key, "secret");
        assert_eq!(config.vlm.model, "catalog-model");
        assert_eq!(config.vlm.context_window_tokens, Some(8192));
    }

    #[test]
    fn no_vlm_block_resolves_by_requirements() {
        let _env = catalog_env(
            "default",
            &catalog_body(
                r#""vision":true,"tools":true,"stream":true"#,
                Some("CAT_KEY"),
            ),
        );
        let config = PilotConfig::resolve(base_args()).expect("catalog default");
        assert_eq!(config.vlm.model, "catalog-model");
    }

    #[test]
    fn partial_explicit_block_is_an_error() {
        let _env = catalog_env(
            "partial",
            &catalog_body(
                r#""vision":true,"tools":true,"stream":true"#,
                Some("CAT_KEY"),
            ),
        );
        let mut args = base_args();
        args.vlm_upstream = Some("http://explicit.example/v1".into());
        let error = PilotConfig::resolve(args).expect_err("partial vlm");
        assert!(format!("{error:#}").contains("vlm.model"), "{error:#}");
    }

    #[test]
    fn unknown_catalog_id_is_an_error() {
        let _env = catalog_env(
            "unknown",
            &catalog_body(
                r#""vision":true,"tools":true,"stream":true"#,
                Some("CAT_KEY"),
            ),
        );
        let mut args = base_args();
        args.vlm_model = Some("missing/model".into());
        let error = PilotConfig::resolve(args).expect_err("unknown catalog id");
        assert!(
            format!("{error:#}").contains("not a catalog id"),
            "{error:#}"
        );
    }

    #[test]
    fn keyless_catalog_entry_needs_no_key() {
        let _env = catalog_env(
            "keyless",
            &catalog_body(r#""vision":true,"tools":true,"stream":true"#, None),
        );
        let mut args = base_args();
        args.vlm_model = Some("catalog/model".into());
        let config = PilotConfig::resolve(args).expect("keyless catalog");
        assert_eq!(config.vlm.api_key, "");
    }

    #[test]
    fn explicit_key_wins_over_catalog_key() {
        let _env = catalog_env(
            "explicit-key",
            &catalog_body(
                r#""vision":true,"tools":true,"stream":true"#,
                Some("CAT_KEY"),
            ),
        );
        let mut args = base_args();
        args.vlm_model = Some("catalog/model".into());
        args.vlm_api_key = Some("explicit-key".into());
        let config = PilotConfig::resolve(args).expect("explicit key");
        assert_eq!(config.vlm.api_key, "explicit-key");
    }
}
