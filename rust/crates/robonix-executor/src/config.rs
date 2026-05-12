// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Executor config — same three-source resolution as pilot:
//   compiled defaults < YAML at $ROBONIX_CONFIG_PATH < CLI flags / env.

use anyhow::{Context, Result};
use clap::Parser;
use serde::Deserialize;
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
}

#[derive(Parser, Debug)]
#[command(
    name = "robonix-executor",
    about = "Robonix Executor — tool-call dispatch runtime"
)]
pub struct Args {
    /// Atlas control-plane endpoint.
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

    /// Log filter (env_logger syntax; e.g. `info`, `robonix_executor=debug`).
    /// Default: `robonix_executor=info`. Falls back to `RUST_LOG` if unset.
    #[arg(long)]
    pub log: Option<String>,
}

#[derive(Default, Deserialize)]
struct FileConfig {
    #[serde(default)]
    atlas_endpoint: Option<String>,
    #[serde(default)]
    listen: Option<String>,
    #[serde(default)]
    id: Option<String>,
}

impl ExecutorConfig {
    pub fn resolve(args: Args) -> Result<Self> {
        let file_cfg: FileConfig = match &args.config {
            Some(path) => load_yaml(path)?,
            None => FileConfig::default(),
        };
        Ok(Self {
            atlas_endpoint: args
                .atlas
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
        })
    }
}

fn load_yaml(path: &Path) -> Result<FileConfig> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("read executor config '{}'", path.display()))?;
    serde_yaml::from_str(&raw)
        .with_context(|| format!("parse executor config '{}'", path.display()))
}
