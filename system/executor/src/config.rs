// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Executor config — same three-source resolution as pilot:
//   compiled defaults < YAML at $ROBONIX_CONFIG_PATH < CLI flags / env.

use anyhow::{Context, Result, bail};
use clap::Parser;
use serde::Deserialize;
use std::collections::HashSet;
use std::path::{Path, PathBuf};

pub const DEFAULT_EXECUTOR_PROVIDER_ID: &str = "executor";
pub const EXECUTOR_NAMESPACE: &str = "robonix/system/executor";
pub const DEFAULT_ATLAS_ENDPOINT: &str = "127.0.0.1:50051";
pub const DEFAULT_LISTEN: &str = "127.0.0.1:50061";

#[derive(Debug, Clone)]
pub struct ExecutorConfig {
    pub atlas_endpoint: String,
    pub listen: String,
    pub id: String,
    pub verification: Vec<VerificationRule>,
}

/// Route one completed capability call to a verifier provider.
#[derive(Debug, Clone, Deserialize, PartialEq)]
pub struct VerificationRule {
    pub target_contract_id: String,
    #[serde(default)]
    pub target_provider_id: Option<String>,
    pub verifier_provider_id: String,
    #[serde(default = "empty_json_object")]
    pub verifier_args: serde_json::Value,
}

fn empty_json_object() -> serde_json::Value {
    serde_json::json!({})
}

#[derive(Parser, Debug)]
#[command(
    name = "robonix-executor",
    about = "Robonix Executor — tool-call dispatch runtime"
)]
pub struct Args {
    /// Atlas control-plane endpoint. Also reads `ROBONIX_ATLAS` (the var rbnx /
    /// the Python API / liaison use) as an alias; see `env_atlas`.
    #[arg(long, env = "ROBONIX_ATLAS_ENDPOINT")]
    pub atlas: Option<String>,

    /// Address the SystemExecutor gRPC service binds to.
    #[arg(long, env = "ROBONIX_EXECUTOR_LISTEN")]
    pub listen: Option<String>,

    /// Override executor's id (singleton; rarely needed).
    #[arg(long, env = "ROBONIX_EXECUTOR_PROVIDER_ID")]
    pub id: Option<String>,

    /// Optional YAML config file (rbnx writes this; CLI/env still override).
    #[arg(long, env = "ROBONIX_CONFIG_PATH")]
    pub config: Option<PathBuf>,

    /// Log level for this component (`debug`/`info`/`warn`/`error`). Sets the
    /// scribe log-file floor; falls back to `SCRIBE_FILE_LEVEL` / `info`.
    /// Normally arrives inside `--config-json`, not as a standalone flag.
    #[arg(long)]
    pub log: Option<String>,

    /// The component's `system.executor` manifest block, serialized to JSON by
    /// rbnx and passed as one arg (`--config-json '{…}'`). Parsed by the binary
    /// itself — see `robonix_scribe::init_from_config`, which reads the `log`
    /// key from it so the manifest's per-component level reaches the log.
    #[arg(long)]
    pub config_json: Option<String>,
}

#[derive(Default, Deserialize)]
struct FileConfig {
    #[serde(default)]
    atlas_endpoint: Option<String>,
    #[serde(default)]
    listen: Option<String>,
    #[serde(default)]
    id: Option<String>,
    #[serde(default)]
    verification: Option<Vec<VerificationRule>>,
}

#[derive(Default, Deserialize)]
struct ManifestConfig {
    #[serde(default)]
    verification: Option<Vec<VerificationRule>>,
}

impl ExecutorConfig {
    /// Resolve connection settings from the existing CLI/env/YAML sources and
    /// verification rules from the manifest block, falling back to YAML.
    pub fn resolve(args: Args) -> Result<Self> {
        let file_cfg: FileConfig = match &args.config {
            Some(path) => load_yaml(path)?,
            None => FileConfig::default(),
        };
        let manifest_cfg: ManifestConfig = match args.config_json.as_deref() {
            Some(raw) => serde_json::from_str(raw).context("parse Executor --config-json")?,
            None => ManifestConfig::default(),
        };
        let verification = validate_verification_rules(
            manifest_cfg
                .verification
                .or(file_cfg.verification)
                .unwrap_or_default(),
        )?;
        Ok(Self {
            atlas_endpoint: args
                .atlas
                .or_else(env_atlas)
                .or(file_cfg.atlas_endpoint)
                .unwrap_or_else(|| DEFAULT_ATLAS_ENDPOINT.to_string()),
            listen: args
                .listen
                .or(file_cfg.listen)
                .unwrap_or_else(|| DEFAULT_LISTEN.to_string()),
            id: args
                .id
                .or(file_cfg.id)
                .unwrap_or_else(|| DEFAULT_EXECUTOR_PROVIDER_ID.to_string()),
            verification,
        })
    }
}

/// Normalize identifiers and reject ambiguous rules before Executor starts.
fn validate_verification_rules(rules: Vec<VerificationRule>) -> Result<Vec<VerificationRule>> {
    let mut seen = HashSet::new();
    let mut normalized = Vec::with_capacity(rules.len());
    for mut rule in rules {
        rule.target_contract_id = rule.target_contract_id.trim().to_string();
        rule.verifier_provider_id = rule.verifier_provider_id.trim().to_string();
        rule.target_provider_id = rule
            .target_provider_id
            .map(|value| value.trim().to_string())
            .filter(|value| !value.is_empty());
        if rule.target_contract_id.is_empty() {
            bail!("verification target_contract_id must not be empty");
        }
        if rule.verifier_provider_id.is_empty() {
            bail!(
                "verification verifier_provider_id must not be empty for '{}'",
                rule.target_contract_id
            );
        }
        if !rule.verifier_args.is_object() {
            bail!(
                "verification verifier_args must be a JSON object for '{}'",
                rule.target_contract_id
            );
        }
        let key = (
            rule.target_contract_id.clone(),
            rule.target_provider_id.clone(),
        );
        if !seen.insert(key) {
            bail!(
                "duplicate verification rule for contract '{}' and provider '{}'",
                rule.target_contract_id,
                rule.target_provider_id.as_deref().unwrap_or("*")
            );
        }
        normalized.push(rule);
    }
    Ok(normalized)
}

/// Read the `ROBONIX_ATLAS` env var as an atlas-endpoint alias.
///
/// rbnx, the Python API, and liaison all configure the atlas endpoint via
/// `ROBONIX_ATLAS`, while executor/pilot historically only honored
/// `ROBONIX_ATLAS_ENDPOINT` (the clap `env`). Accepting `ROBONIX_ATLAS` here as
/// well means a single env var configures every component. Without it, setting
/// only `ROBONIX_ATLAS` left executor silently falling back to
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
        .with_context(|| format!("read executor config '{}'", path.display()))?;
    serde_yaml::from_str(&raw)
        .with_context(|| format!("parse executor config '{}'", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(config_json: Option<&str>) -> Args {
        Args {
            atlas: None,
            listen: None,
            id: None,
            config: None,
            log: None,
            config_json: config_json.map(str::to_string),
        }
    }

    #[test]
    fn parses_manifest_verification_rules() {
        let cfg = ExecutorConfig::resolve(args(Some(
            r#"{
                "verification": [{
                    "target_contract_id": "robonix/service/navigation/navigate",
                    "verifier_provider_id": "scene_verifier",
                    "verifier_args": {"scene_provider_id": "scene"}
                }]
            }"#,
        )))
        .unwrap();
        assert_eq!(cfg.verification.len(), 1);
        assert_eq!(
            cfg.verification[0].verifier_args["scene_provider_id"],
            "scene"
        );
    }

    #[test]
    fn explicit_empty_manifest_rules_disable_defaults() {
        let cfg = ExecutorConfig::resolve(args(Some(r#"{"verification": []}"#))).unwrap();
        assert!(cfg.verification.is_empty());
    }

    #[test]
    fn rejects_duplicate_rules_at_the_same_specificity() {
        let error = ExecutorConfig::resolve(args(Some(
            r#"{
                "verification": [
                    {"target_contract_id":"cap/a","verifier_provider_id":"v1"},
                    {"target_contract_id":"cap/a","verifier_provider_id":"v2"}
                ]
            }"#,
        )))
        .unwrap_err();
        assert!(error.to_string().contains("duplicate verification rule"));
    }

    #[test]
    fn rejects_non_object_verifier_args() {
        let error = ExecutorConfig::resolve(args(Some(
            r#"{
                "verification": [{
                    "target_contract_id":"cap/a",
                    "verifier_provider_id":"v1",
                    "verifier_args": ["bad"]
                }]
            }"#,
        )))
        .unwrap_err();
        assert!(error.to_string().contains("must be a JSON object"));
    }
}
