// SPDX-License-Identifier: MulanPSL-2.0

use crate::runtime_state::RuntimeStateStore;
use anyhow::{Context, Result};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{info, warn};
use serde::Serialize;
use std::path::Path;
use std::process::Stdio;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::{Child, Command};

const JOINT_STATES: &str = "robonix/primitive/arm/joint_states";
const CHASSIS_ODOM: &str = "robonix/primitive/chassis/odom";

#[derive(Debug, Serialize)]
struct Source {
    kind: &'static str,
    provider_id: String,
    topic: String,
    qos: String,
}

pub struct RuntimeMonitor {
    child: Child,
    channel_ids: Vec<String>,
}

pub async fn start(
    atlas: &mut AtlasClient,
    consumer_id: &str,
    state: RuntimeStateStore,
    runtime_dir: &Path,
) -> Result<Option<RuntimeMonitor>> {
    let mut sources = Vec::new();
    let mut channel_ids = Vec::new();
    for (contract_id, kind) in [(JOINT_STATES, "joint_state"), (CHASSIS_ODOM, "odom")] {
        let providers = atlas
            .query_capabilities("", contract_id, atlas_pb::Transport::Ros2)
            .await
            .with_context(|| format!("discover {contract_id}"))?;
        for provider in providers {
            if provider.state != atlas_pb::LifecycleState::StateActive as i32 {
                continue;
            }
            match atlas
                .connect_capability(
                    consumer_id,
                    &provider.id,
                    contract_id,
                    atlas_pb::Transport::Ros2,
                )
                .await
            {
                Ok((channel_id, topic, params)) if !topic.trim().is_empty() => {
                    let qos = match params.kind {
                        Some(atlas_pb::transport_params::Kind::Ros2(value)) => value.qos_profile,
                        _ => String::new(),
                    };
                    sources.push(Source {
                        kind,
                        provider_id: provider.id,
                        topic,
                        qos,
                    });
                    channel_ids.push(channel_id);
                }
                Ok((channel_id, _, _)) => {
                    let _ = atlas.disconnect_capability(&channel_id).await;
                }
                Err(error) => warn!("[soma/state] connect {contract_id} failed: {error:#}"),
            }
        }
    }
    if sources.is_empty() {
        warn!("[soma/state] no ROS 2 arm joint-state or chassis odometry sources discovered");
        return Ok(None);
    }

    std::fs::create_dir_all(runtime_dir)
        .with_context(|| format!("create runtime directory {}", runtime_dir.display()))?;
    let script = runtime_dir.join("soma_runtime_ros.py");
    let config = runtime_dir.join("soma_runtime_sources.json");
    std::fs::write(&script, include_str!("runtime_ros.py"))
        .with_context(|| format!("write {}", script.display()))?;
    std::fs::write(
        &config,
        serde_json::to_vec_pretty(&serde_json::json!({ "sources": sources }))?,
    )
    .with_context(|| format!("write {}", config.display()))?;

    let mut child = Command::new("python3")
        .arg("-u")
        .arg(&script)
        .arg(&config)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .kill_on_drop(true)
        .spawn()
        .context("spawn Soma ROS 2 runtime reader")?;
    let stdout = child
        .stdout
        .take()
        .context("capture runtime reader stdout")?;
    let stderr = child
        .stderr
        .take()
        .context("capture runtime reader stderr")?;
    tokio::spawn({
        let state = state.clone();
        async move {
            let mut lines = BufReader::new(stdout).lines();
            while let Ok(Some(line)) = lines.next_line().await {
                if let Err(error) = state.apply_line(&line).await {
                    warn!("[soma/state] invalid runtime event: {error}: {line}");
                }
            }
        }
    });
    tokio::spawn(async move {
        let mut lines = BufReader::new(stderr).lines();
        while let Ok(Some(line)) = lines.next_line().await {
            warn!("[soma/state/ros] {line}");
        }
    });
    info!(
        "[soma/state] runtime reader started with {} source(s)",
        sources.len()
    );
    Ok(Some(RuntimeMonitor { child, channel_ids }))
}

impl RuntimeMonitor {
    pub async fn shutdown(mut self, atlas: &mut AtlasClient) {
        let _ = self.child.kill().await;
        for channel_id in self.channel_ids {
            let _ = atlas.disconnect_capability(&channel_id).await;
        }
    }
}
