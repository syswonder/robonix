// SPDX-License-Identifier: MulanPSL-2.0

use anyhow::{Context, Result};
use clap::Parser;
use serde::Deserialize;
use std::path::{Path, PathBuf};

pub const DEFAULT_PROVIDER_ID: &str = "health_primitive";
pub const PROVIDER_NAMESPACE: &str = "robonix/primitive/health";
pub const DEFAULT_ATLAS_ENDPOINT: &str = "127.0.0.1:50051";
pub const DEFAULT_LISTEN: &str = "127.0.0.1:50092";
pub const DEFAULT_COLLECT_INTERVAL_MS: u64 = 1000;

#[derive(Debug, Clone)]
pub struct HealthConfig {
    pub atlas_endpoint: String,
    pub listen: String,
    pub id: String,
    pub collect_interval_ms: u64,
}

#[derive(Parser, Debug)]
#[command(
    name = "robonix-health-primitive",
    about = "Robonix Health Primitive — raw sysfs/hwmon sensor data"
)]
pub struct Args {
    #[arg(long, env = "ROBONIX_ATLAS_ENDPOINT")]
    pub atlas: Option<String>,

    #[arg(long, env = "ROBONIX_HEALTH_LISTEN")]
    pub listen: Option<String>,

    #[arg(long, env = "ROBONIX_HEALTH_PROVIDER_ID")]
    pub id: Option<String>,

    #[arg(long, env = "ROBONIX_HEALTH_COLLECT_INTERVAL_MS")]
    pub collect_interval_ms: Option<u64>,

    #[arg(long, env = "ROBONIX_CONFIG_PATH")]
    pub config: Option<PathBuf>,

    #[arg(long)]
    pub log: Option<String>,
}

#[derive(Default, Deserialize)]
struct FileConfig {
    atlas_endpoint: Option<String>,
    listen: Option<String>,
    id: Option<String>,
    collect_interval_ms: Option<u64>,
}

impl HealthConfig {
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
                .unwrap_or_else(|| DEFAULT_PROVIDER_ID.to_string()),
            collect_interval_ms: args
                .collect_interval_ms
                .or(file_cfg.collect_interval_ms)
                .unwrap_or(DEFAULT_COLLECT_INTERVAL_MS),
        })
    }
}

fn load_yaml(path: &Path) -> Result<FileConfig> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("read config '{}'", path.display()))?;
    serde_yaml::from_str(&raw).with_context(|| format!("parse config '{}'", path.display()))
}
