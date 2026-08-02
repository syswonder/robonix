// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Executor config — same three-source resolution as pilot:
//   compiled defaults < YAML at $ROBONIX_CONFIG_PATH < CLI flags / env.

use anyhow::{Context, Result};
use clap::Parser;
use robonix_sentinel::{Rule, Sentinel};
use serde::Deserialize;
use std::io::Write;
use std::path::{Path, PathBuf};

pub const DEFAULT_EXECUTOR_PROVIDER_ID: &str = "executor";
pub const EXECUTOR_NAMESPACE: &str = "robonix/system/executor";
pub const DEFAULT_ATLAS_ENDPOINT: &str = "127.0.0.1:50051";
pub const DEFAULT_LISTEN: &str = "127.0.0.1:50061";

#[derive(Debug, Clone)]
pub struct ExecutorConfig {
    pub atlas_endpoint: String,
    pub listen: String,
    pub sentinel_listen: String,
    pub id: String,
    pub sentinel_rules: PathBuf,
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

    /// Address the robot-local Sentinel management gRPC service binds to.
    /// Defaults to the Executor listen address with its port incremented by one.
    #[arg(long, env = "ROBONIX_SENTINEL_LISTEN")]
    pub sentinel_listen: Option<String>,

    /// Override executor's id (singleton; rarely needed).
    #[arg(long, env = "ROBONIX_EXECUTOR_PROVIDER_ID")]
    pub id: Option<String>,

    /// Robot-local Sentinel rules file.
    #[arg(long, env = "ROBONIX_SENTINEL_RULES")]
    pub sentinel_rules: Option<PathBuf>,

    /// Optional versioned seed copied to `sentinel_rules` only on first boot.
    #[arg(long, env = "ROBONIX_SENTINEL_RULES_SEED")]
    pub sentinel_rules_seed: Option<PathBuf>,

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
    sentinel_listen: Option<String>,
    #[serde(default)]
    id: Option<String>,
    #[serde(default)]
    sentinel_rules: Option<PathBuf>,
    #[serde(default)]
    sentinel_rules_seed: Option<PathBuf>,
}

impl ExecutorConfig {
    pub fn resolve(args: Args) -> Result<Self> {
        let file_cfg: FileConfig = match &args.config {
            Some(path) => load_yaml(path)?,
            None => FileConfig::default(),
        };
        let deployment_cfg = args
            .config_json
            .as_deref()
            .map(serde_json::from_str::<FileConfig>)
            .transpose()
            .context("parse Executor deployment config")?
            .unwrap_or_default();
        let data_dir = std::env::var_os("ROBONIX_DATA_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("rbnx-boot/data"));
        let listen = args
            .listen
            .or(deployment_cfg.listen)
            .or(file_cfg.listen)
            .unwrap_or_else(|| DEFAULT_LISTEN.to_string());
        let sentinel_listen = args
            .sentinel_listen
            .or(deployment_cfg.sentinel_listen)
            .or(file_cfg.sentinel_listen)
            .map(Ok)
            .unwrap_or_else(|| adjacent_listen(&listen))?;
        let sentinel_rules = args
            .sentinel_rules
            .or(deployment_cfg.sentinel_rules)
            .or(file_cfg.sentinel_rules)
            .unwrap_or_else(|| data_dir.join("sentinel-rules.json"));
        let sentinel_rules_seed = args
            .sentinel_rules_seed
            .or(deployment_cfg.sentinel_rules_seed)
            .or(file_cfg.sentinel_rules_seed);
        bootstrap_sentinel_rules(&sentinel_rules, sentinel_rules_seed.as_deref())?;
        Ok(Self {
            atlas_endpoint: args
                .atlas
                .or_else(env_atlas)
                .or(deployment_cfg.atlas_endpoint)
                .or(file_cfg.atlas_endpoint)
                .unwrap_or_else(|| DEFAULT_ATLAS_ENDPOINT.to_string()),
            listen,
            sentinel_listen,
            id: args
                .id
                .or(deployment_cfg.id)
                .or(file_cfg.id)
                .unwrap_or_else(|| DEFAULT_EXECUTOR_PROVIDER_ID.to_string()),
            sentinel_rules,
        })
    }
}

fn adjacent_listen(listen: &str) -> Result<String> {
    let mut address: std::net::SocketAddr = listen
        .parse()
        .with_context(|| format!("invalid executor listen address '{listen}'"))?;
    let port = address
        .port()
        .checked_add(1)
        .context("cannot derive Sentinel listen address from port 65535")?;
    address.set_port(port);
    Ok(address.to_string())
}

/// Initialize a robot-local policy from a checked-in seed without ever
/// replacing a policy already managed through Sentinel's admin API.
fn bootstrap_sentinel_rules(path: &Path, seed: Option<&Path>) -> Result<()> {
    if path.exists() {
        return Ok(());
    }
    let Some(seed) = seed else {
        return Ok(());
    };
    let raw = std::fs::read_to_string(seed)
        .with_context(|| format!("read Sentinel rules seed '{}'", seed.display()))?;
    let rules = serde_json::from_str::<Vec<Rule>>(&raw)
        .with_context(|| format!("parse Sentinel rules seed '{}'", seed.display()))?;
    Sentinel::new(rules)
        .with_context(|| format!("validate Sentinel rules seed '{}'", seed.display()))?;

    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("create Sentinel rules directory '{}'", parent.display()))?;
    }
    match std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
    {
        Ok(mut target) => {
            if let Err(error) = target.write_all(raw.as_bytes()) {
                drop(target);
                let _ = std::fs::remove_file(path);
                return Err(error)
                    .with_context(|| format!("bootstrap Sentinel rules '{}'", path.display()));
            }
        }
        Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {}
        Err(error) => {
            return Err(error)
                .with_context(|| format!("bootstrap Sentinel rules '{}'", path.display()));
        }
    }
    Ok(())
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
            atlas: Some("127.0.0.1:50051".to_owned()),
            listen: None,
            sentinel_listen: None,
            id: None,
            sentinel_rules: None,
            sentinel_rules_seed: None,
            config: None,
            log: None,
            config_json: config_json.map(str::to_owned),
        }
    }

    #[test]
    fn deployment_config_resolves_remote_sentinel_and_robot_local_rules() {
        let config = ExecutorConfig::resolve(args(Some(
            r#"{
                "listen":"127.0.0.1:50061",
                "sentinel_listen":"0.0.0.0:50062",
                "sentinel_rules":"/home/robot/.robonix/data/sentinel-rules.json"
            }"#,
        )))
        .expect("Executor deployment config");

        assert_eq!(config.listen, "127.0.0.1:50061");
        assert_eq!(config.sentinel_listen, "0.0.0.0:50062");
        assert_eq!(
            config.sentinel_rules,
            PathBuf::from("/home/robot/.robonix/data/sentinel-rules.json")
        );
    }

    #[test]
    fn sentinel_defaults_to_executor_adjacent_port() {
        let config = ExecutorConfig::resolve(args(None)).expect("default Executor config");
        assert_eq!(config.listen, DEFAULT_LISTEN);
        assert_eq!(config.sentinel_listen, "127.0.0.1:50062");
    }

    #[test]
    fn sentinel_adjacent_port_overflow_is_rejected() {
        let error = ExecutorConfig::resolve(args(Some(r#"{"listen":"127.0.0.1:65535"}"#)))
            .expect_err("adjacent Sentinel port overflow");
        assert!(error.to_string().contains("port 65535"), "{error:#}");
    }

    #[test]
    fn versioned_seed_bootstraps_fresh_rules_without_overwriting_changes() {
        let nonce = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("system clock")
            .as_nanos();
        let temp = std::env::temp_dir().join(format!(
            "executor-sentinel-seed-{}-{nonce}",
            std::process::id()
        ));
        std::fs::create_dir_all(&temp).expect("test directory");
        let seed = temp.join("sentinel-rules.v1.json");
        let target = temp.join("data/sentinel-rules.json");
        let seed_json =
            r#"[{"id":"demo","effect":"deny","conditions":{"contract":"robonix/demo/*"}}]"#;
        std::fs::write(&seed, seed_json).expect("versioned seed");
        let config_json = serde_json::json!({
            "sentinel_rules": target,
            "sentinel_rules_seed": seed,
        })
        .to_string();

        let config = ExecutorConfig::resolve(args(Some(&config_json))).expect("seeded config");
        assert_eq!(config.sentinel_rules, target);
        assert_eq!(
            std::fs::read_to_string(&target).expect("bootstrapped rules"),
            seed_json
        );

        std::fs::write(&target, "[]\n").expect("admin-managed replacement");
        ExecutorConfig::resolve(args(Some(&config_json))).expect("existing rules config");
        assert_eq!(
            std::fs::read_to_string(&target).expect("preserved rules"),
            "[]\n"
        );
        std::fs::remove_dir_all(temp).expect("remove test directory");
    }

    #[test]
    fn checked_in_webots_seed_is_nonempty_and_valid() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../examples/webots/sentinel-rules.v1.json");
        let raw = std::fs::read_to_string(&path).expect("Webots Sentinel seed");
        let rules = serde_json::from_str::<Vec<Rule>>(&raw).expect("parse Webots Sentinel seed");

        assert!(
            !rules.is_empty(),
            "Webots Sentinel seed must not allow-all by omission"
        );
        Sentinel::new(rules).expect("validate Webots Sentinel seed");
    }
}
