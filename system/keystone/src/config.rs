// SPDX-License-Identifier: MulanPSL-2.0

use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use serde::Deserialize;

pub const DEFAULT_ATLAS_ENDPOINT: &str = "127.0.0.1:50051";
pub const DEFAULT_LISTEN: &str = "0.0.0.0:50095";
pub const DEFAULT_PROVIDER_ID: &str = "keystone";
pub const KEYSTONE_NAMESPACE: &str = "robonix/system/keystone";

#[derive(Debug, Clone)]
pub struct KeystoneConfig {
    pub atlas_endpoint: String,
    pub listen: String,
    pub provider_id: String,
    pub database: PathBuf,
    pub bootstrap_credentials_file: PathBuf,
    pub bootstrap_admin_username: String,
    pub bootstrap_admin_display_name: String,
    pub bootstrap_admin_email: String,
    pub bootstrap_admin_password: Option<String>,
}

#[derive(Parser, Debug)]
#[command(name = "robonix-keystone", about = "Robonix user identity service")]
pub struct Args {
    #[arg(long, env = "ROBONIX_ATLAS_ENDPOINT")]
    pub atlas: Option<String>,

    #[arg(long, env = "ROBONIX_KEYSTONE_LISTEN")]
    pub listen: Option<String>,

    #[arg(long, env = "ROBONIX_KEYSTONE_PROVIDER_ID")]
    pub provider_id: Option<String>,

    #[arg(long, env = "ROBONIX_KEYSTONE_DATABASE")]
    pub database: Option<PathBuf>,

    #[arg(long, env = "ROBONIX_KEYSTONE_BOOTSTRAP_FILE")]
    pub bootstrap_credentials_file: Option<PathBuf>,

    #[arg(long, env = "ROBONIX_KEYSTONE_BOOTSTRAP_ADMIN_USERNAME")]
    pub bootstrap_admin_username: Option<String>,

    #[arg(long, env = "ROBONIX_KEYSTONE_BOOTSTRAP_ADMIN_DISPLAY_NAME")]
    pub bootstrap_admin_display_name: Option<String>,

    #[arg(long, env = "ROBONIX_KEYSTONE_BOOTSTRAP_ADMIN_EMAIL")]
    pub bootstrap_admin_email: Option<String>,

    #[arg(long, env = "ROBONIX_KEYSTONE_BOOTSTRAP_ADMIN_PASSWORD")]
    pub bootstrap_admin_password: Option<String>,

    #[arg(long, env = "ROBONIX_CONFIG_PATH")]
    pub config: Option<PathBuf>,

    #[arg(long)]
    pub config_json: Option<String>,
}

#[derive(Debug, Default, Deserialize)]
struct FileConfig {
    atlas_endpoint: Option<String>,
    listen: Option<String>,
    provider_id: Option<String>,
    database: Option<PathBuf>,
    bootstrap_credentials_file: Option<PathBuf>,
    bootstrap_admin_username: Option<String>,
    bootstrap_admin_display_name: Option<String>,
    bootstrap_admin_email: Option<String>,
}

impl KeystoneConfig {
    /// Resolve command-line, deployment JSON, YAML, environment, and safe defaults.
    pub fn resolve(args: Args) -> Result<Self> {
        let file = if let Some(path) = &args.config {
            let raw = std::fs::read_to_string(path)
                .with_context(|| format!("read Keystone config '{}'", path.display()))?;
            serde_yaml::from_str::<FileConfig>(&raw)
                .with_context(|| format!("parse Keystone config '{}'", path.display()))?
        } else {
            FileConfig::default()
        };
        let deployment = args
            .config_json
            .as_deref()
            .map(serde_json::from_str::<FileConfig>)
            .transpose()
            .context("parse Keystone deployment config")?
            .unwrap_or_default();

        let data_dir = std::env::var_os("ROBONIX_DATA_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("rbnx-boot/data"));
        let database = args
            .database
            .or(deployment.database)
            .or(file.database)
            .unwrap_or_else(|| data_dir.join("keystone.db"));
        let bootstrap_credentials_file = args
            .bootstrap_credentials_file
            .or(deployment.bootstrap_credentials_file)
            .or(file.bootstrap_credentials_file)
            .unwrap_or_else(|| data_dir.join("keystone-bootstrap-admin.txt"));

        Ok(Self {
            atlas_endpoint: args
                .atlas
                .or_else(|| nonempty_env("ROBONIX_ATLAS"))
                .or(deployment.atlas_endpoint)
                .or(file.atlas_endpoint)
                .unwrap_or_else(|| DEFAULT_ATLAS_ENDPOINT.to_string()),
            listen: args
                .listen
                .or(deployment.listen)
                .or(file.listen)
                .unwrap_or_else(|| DEFAULT_LISTEN.to_string()),
            provider_id: args
                .provider_id
                .or(deployment.provider_id)
                .or(file.provider_id)
                .unwrap_or_else(|| DEFAULT_PROVIDER_ID.to_string()),
            database,
            bootstrap_credentials_file,
            bootstrap_admin_username: args
                .bootstrap_admin_username
                .or(deployment.bootstrap_admin_username)
                .or(file.bootstrap_admin_username)
                .unwrap_or_else(|| "admin".to_string()),
            bootstrap_admin_display_name: args
                .bootstrap_admin_display_name
                .or(deployment.bootstrap_admin_display_name)
                .or(file.bootstrap_admin_display_name)
                .unwrap_or_else(|| "Administrator".to_string()),
            bootstrap_admin_email: args
                .bootstrap_admin_email
                .or(deployment.bootstrap_admin_email)
                .or(file.bootstrap_admin_email)
                .unwrap_or_default(),
            bootstrap_admin_password: args.bootstrap_admin_password,
        })
    }
}

fn nonempty_env(name: &str) -> Option<String> {
    std::env::var(name).ok().filter(|value| !value.is_empty())
}
