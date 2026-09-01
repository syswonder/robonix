// SPDX-License-Identifier: MulanPSL-2.0

use crate::runtime_state::RuntimeStateStore;
use anyhow::{Context, Result};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{info, warn};
use serde::{Deserialize, Serialize};
use std::path::Path;
use std::process::Stdio;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::{Child, Command};
use tokio::time::{Duration, timeout};

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

#[derive(Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum StartupEvent {
    Ready { subscriptions: usize },
    Warning { message: String },
}

pub async fn start(
    atlas: &mut AtlasClient,
    consumer_id: &str,
    state: RuntimeStateStore,
    runtime_dir: &Path,
    command_template: &[String],
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

    let mut command = match build_command(command_template, &script, &config) {
        Ok(command) => command,
        Err(error) => {
            disconnect_channels(atlas, &channel_ids).await;
            return Err(error);
        }
    };
    let mut child = match command
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .kill_on_drop(true)
        .spawn()
        .context("spawn Soma ROS 2 runtime reader")
    {
        Ok(child) => child,
        Err(error) => {
            disconnect_channels(atlas, &channel_ids).await;
            return Err(error);
        }
    };
    let stdout = match child.stdout.take().context("capture runtime reader stdout") {
        Ok(stdout) => stdout,
        Err(error) => {
            let _ = child.kill().await;
            disconnect_channels(atlas, &channel_ids).await;
            return Err(error);
        }
    };
    let stderr = match child.stderr.take().context("capture runtime reader stderr") {
        Ok(stderr) => stderr,
        Err(error) => {
            let _ = child.kill().await;
            disconnect_channels(atlas, &channel_ids).await;
            return Err(error);
        }
    };
    let mut lines = BufReader::new(stdout).lines();
    let readiness: Result<(String, usize)> = async {
        let startup_line = timeout(Duration::from_secs(15), lines.next_line())
            .await
            .context("Soma runtime reader readiness timed out")??
            .context("Soma runtime reader exited before readiness")?;
        let startup: StartupEvent = serde_json::from_str(&startup_line).with_context(|| {
            format!("invalid Soma runtime reader startup event: {startup_line}")
        })?;
        let subscriptions = match startup {
            StartupEvent::Ready { subscriptions } if subscriptions > 0 => subscriptions,
            StartupEvent::Ready { .. } => {
                anyhow::bail!("Soma runtime reader created no subscriptions")
            }
            StartupEvent::Warning { message } => anyhow::bail!(message),
        };
        Ok((startup_line, subscriptions))
    }
    .await;
    let (startup_line, subscriptions) = match readiness {
        Ok(readiness) => readiness,
        Err(error) => {
            let _ = child.kill().await;
            disconnect_channels(atlas, &channel_ids).await;
            return Err(error);
        }
    };
    state.apply_line(&startup_line).await?;
    tokio::spawn({
        let state = state.clone();
        async move {
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
    info!("[soma/state] runtime reader ready with {subscriptions} subscription(s)");
    Ok(Some(RuntimeMonitor { child, channel_ids }))
}

async fn disconnect_channels(atlas: &mut AtlasClient, channel_ids: &[String]) {
    for channel_id in channel_ids {
        let _ = atlas.disconnect_capability(channel_id).await;
    }
}

fn build_command(template: &[String], script: &Path, config: &Path) -> Result<Command> {
    if template.is_empty() {
        let mut command = Command::new("python3");
        command.arg("-u").arg(script).arg(config);
        return Ok(command);
    }
    let expand = |value: &str| {
        value
            .replace("{script}", &script.to_string_lossy())
            .replace("{config}", &config.to_string_lossy())
    };
    let mut parts = template.iter().map(|value| expand(value));
    let program = parts
        .next()
        .filter(|value| !value.trim().is_empty())
        .context("runtime_reader_command must contain a program")?;
    let mut command = Command::new(program);
    command.args(parts);
    Ok(command)
}

impl RuntimeMonitor {
    pub async fn shutdown(mut self, atlas: &mut AtlasClient) {
        let _ = self.child.kill().await;
        for channel_id in self.channel_ids {
            let _ = atlas.disconnect_capability(&channel_id).await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn expands_runtime_reader_paths_without_a_shell() {
        let command = build_command(
            &[
                "docker".into(),
                "exec".into(),
                "sim".into(),
                "python3".into(),
                "{script}".into(),
                "{config}".into(),
            ],
            Path::new("/tmp/reader.py"),
            Path::new("/tmp/sources.json"),
        )
        .expect("build command");
        let std = command.as_std();
        assert_eq!(std.get_program(), "docker");
        assert_eq!(
            std.get_args().collect::<Vec<_>>(),
            [
                "exec",
                "sim",
                "python3",
                "/tmp/reader.py",
                "/tmp/sources.json"
            ]
        );
    }
}
