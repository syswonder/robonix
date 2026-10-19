// SPDX-License-Identifier: MulanPSL-2.0

use std::net::SocketAddr;
use std::path::PathBuf;

use anyhow::{Context, Result, ensure};
use clap::Parser;
use serde::Deserialize;

use crate::{ChronosConfig, ChronosSection, CoreConfig};

#[derive(Parser)]
#[command(
    name = "robonix-chronos",
    about = "Device timestamp observations, clock quality and conversion model streams"
)]
pub(super) struct Args {
    /// Chronos YAML file containing chronos.core; omit for Linux reference only.
    #[arg(long)]
    pub config: Option<PathBuf>,
    /// System manifest block forwarded by rbnx; core overrides the YAML file.
    #[arg(long)]
    pub config_json: Option<String>,
    /// Atlas endpoint; defaults to 127.0.0.1:50051.
    #[arg(long, env = "ROBONIX_ATLAS")]
    pub atlas: Option<String>,
    /// gRPC listen address; defaults to 127.0.0.1:50094.
    #[arg(long)]
    pub listen: Option<SocketAddr>,
    /// Atlas runtime identity; defaults to chronos.
    #[arg(long)]
    pub provider_id: Option<String>,
    /// Log level written to Scribe's file sink.
    #[arg(long)]
    pub log: Option<String>,
}

#[derive(Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct ManifestConfig {
    pub reference_reader_command: Option<Vec<String>>,
    pub collect_observations: Option<bool>,
    pub collect_ros_observations: Option<bool>,
    pub core: Option<CoreConfig>,
    pub listen: Option<SocketAddr>,
    pub atlas_endpoint: Option<String>,
    pub provider_id: Option<String>,
    pub log: Option<String>,
}

pub(super) struct Config {
    pub chronos: ChronosConfig,
    pub listen: SocketAddr,
    pub atlas_endpoint: String,
    pub provider_id: String,
    pub log_json: String,
}

/// Resolve CLI/env over manifest settings, validating before any registration.
pub(super) fn resolve(args: Args) -> Result<Config> {
    let manifest = parse_manifest(args.config_json.as_deref().unwrap_or("{}"))?;
    let mut chronos = match args.config {
        Some(path) => ChronosConfig::from_yaml(
            &std::fs::read_to_string(&path)
                .with_context(|| format!("read Chronos config '{}'", path.display()))?,
        )?,
        None => ChronosConfig::from_yaml(
            "chronos:\n  core:\n    reference:\n      kind: linux_system\n",
        )?,
    };
    if let Some(core) = manifest.core {
        chronos = ChronosConfig {
            chronos: ChronosSection {
                reference_reader_command: chronos.chronos.reference_reader_command,
                core,
                collect_observations: chronos.chronos.collect_observations,
                collect_ros_observations: chronos.chronos.collect_ros_observations,
            },
        };
    }
    chronos.chronos.core.validate()?;
    if let Some(command) = manifest.reference_reader_command {
        chronos.chronos.reference_reader_command = command;
    }
    if let Some(enabled) = manifest.collect_observations {
        chronos.chronos.collect_observations = enabled;
    }
    if let Some(enabled) = manifest.collect_ros_observations {
        chronos.chronos.collect_ros_observations = enabled;
    }
    let provider_id = args
        .provider_id
        .or(manifest.provider_id)
        .unwrap_or_else(|| "chronos".into());
    ensure!(
        !provider_id.trim().is_empty() && provider_id == provider_id.trim(),
        "provider_id must be non-empty and whitespace-normalized"
    );
    let atlas_endpoint = args
        .atlas
        .or(manifest.atlas_endpoint)
        .unwrap_or_else(|| "127.0.0.1:50051".into());
    ensure!(
        !atlas_endpoint.trim().is_empty(),
        "Atlas endpoint must not be empty"
    );
    Ok(Config {
        chronos,
        listen: args
            .listen
            .or(manifest.listen)
            .unwrap_or_else(|| "127.0.0.1:50094".parse().expect("default address")),
        atlas_endpoint,
        provider_id,
        log_json: serde_json::json!({"log": args.log.or(manifest.log)}).to_string(),
    })
}

/// Reject malformed or unknown manifest keys rather than silently using defaults.
pub(super) fn parse_manifest(raw: &str) -> Result<ManifestConfig> {
    if raw.trim() == "null" {
        return Ok(ManifestConfig::default());
    }
    serde_json::from_str(raw).context("parse Chronos config_json")
}
