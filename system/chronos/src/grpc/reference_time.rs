// SPDX-License-Identifier: MulanPSL-2.0

use std::process::Stdio;
use std::time::Duration;

use anyhow::{Context, Result, ensure};
use robonix_chronos_clock::{VirtualReferenceTime, update_reference_time, wall_timestamp_ns};
use tokio::io::BufReader;
use tokio::process::{Child, Command};
use tokio::task::JoinHandle;

use super::collector::read_ipc_line;

pub(super) struct ReferenceReader {
    child: Child,
    task: JoinHandle<Result<()>>,
    _script: tempfile::NamedTempFile,
}

impl ReferenceReader {
    pub async fn start(template: &[String]) -> Result<Self> {
        let script = tempfile::Builder::new().suffix(".py").tempfile()?;
        std::fs::write(script.path(), include_bytes!("../../reference_ros.py"))?;
        let mut command = if template.is_empty() {
            let mut command = Command::new("python3");
            command.arg("-u").arg(script.path());
            command
        } else {
            let mut parts = template
                .iter()
                .map(|part| part.replace("{script}", &script.path().to_string_lossy()));
            let program = parts
                .next()
                .filter(|program| !program.trim().is_empty())
                .context("reference_reader_command requires a program")?;
            let mut command = Command::new(program);
            command.args(parts);
            command
        };
        let mut child = command
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .kill_on_drop(true)
            .spawn()
            .context("start Chronos /clock reader")?;
        let mut output = BufReader::new(
            child
                .stdout
                .take()
                .context("capture reference reader stdout")?,
        );
        let readiness = async {
            let line = tokio::time::timeout(Duration::from_secs(5), read_ipc_line(&mut output))
                .await
                .context("reference reader startup timed out")??
                .context("reference reader exited before readiness")?;
            let ready: serde_json::Value = serde_json::from_str(&line)?;
            ensure!(
                ready.get("ready").and_then(serde_json::Value::as_bool) == Some(true),
                "invalid reference reader readiness"
            );
            Ok::<_, anyhow::Error>(())
        }
        .await;
        if let Err(error) = readiness {
            stop_reader(&mut child).await?;
            return Err(error);
        }
        // A daemon restart must not reuse the previous process's generation.
        let generation = robonix_chronos_clock::reference_time()?
            .generation
            .checked_add(1)
            .context("reference generation overflow")?
            .max(wall_timestamp_ns()?.max(1));
        update_reference_time(reading(
            0,
            generation,
            false,
            "/clock has not supplied reference time",
        ))?;
        let task = tokio::spawn(async move {
            let result = read_clock(&mut output, generation).await;
            if let Err(error) = invalidate() {
                robonix_scribe::error!("invalidate Chronos reference: {error:#}");
            }
            if let Err(error) = &result {
                robonix_scribe::error!("Chronos /clock reader failed: {error:#}");
            }
            result
        });
        Ok(Self {
            child,
            task,
            _script: script,
        })
    }

    pub fn is_finished(&self) -> bool {
        self.task.is_finished()
    }

    pub async fn stop(mut self) -> Result<()> {
        self.task.abort();
        let _ = (&mut self.task).await;
        invalidate()?;
        stop_reader(&mut self.child).await
    }
}

impl Drop for ReferenceReader {
    fn drop(&mut self) {
        self.task.abort();
    }
}

fn invalidate() -> Result<()> {
    let mut reference = robonix_chronos_clock::reference_time()?;
    reference.available = false;
    reference.reason = "Chronos /clock reader stopped".into();
    update_reference_time(reference)?;
    Ok(())
}

fn reading(
    timestamp_ns: i64,
    generation: u64,
    available: bool,
    reason: &str,
) -> VirtualReferenceTime {
    VirtualReferenceTime {
        timestamp_ns,
        generation,
        generation_changed: false,
        uncertainty_ns: 1,
        available,
        verified: false,
        reason: reason.into(),
    }
}

async fn stop_reader(child: &mut Child) -> Result<()> {
    drop(child.stdin.take());
    match tokio::time::timeout(Duration::from_secs(3), child.wait()).await {
        Ok(result) => {
            result.context("wait for Chronos /clock reader")?;
        }
        Err(_) => {
            child.kill().await.context("stop Chronos /clock reader")?;
        }
    }
    Ok(())
}

async fn read_clock(
    output: &mut (impl tokio::io::AsyncBufRead + Unpin),
    mut generation: u64,
) -> Result<()> {
    let mut previous = None;
    let mut available = false;
    loop {
        let line = tokio::time::timeout(Duration::from_secs(5), read_ipc_line(output))
            .await
            .context("/clock reader health timed out")??
            .context("/clock reader exited")?;
        #[derive(serde::Deserialize)]
        #[serde(deny_unknown_fields)]
        struct Sample {
            timestamp_ns: Option<i64>,
            available: bool,
        }
        let sample: Sample = serde_json::from_str(&line).context("invalid /clock input")?;
        if !sample.available {
            available = false;
            update_reference_time(reading(
                previous.unwrap_or(0),
                generation,
                false,
                if previous.is_some() {
                    "/clock publisher unavailable"
                } else {
                    "/clock has not supplied reference time"
                },
            ))?;
            continue;
        }
        let timestamp_ns = sample
            .timestamp_ns
            .context("available /clock input requires a timestamp")?;
        ensure!(timestamp_ns >= 0, "/clock time must be nonnegative");
        if previous.is_some_and(|previous| timestamp_ns < previous || !available) {
            generation = generation
                .checked_add(1)
                .context("reference generation overflow")?;
        }
        update_reference_time(reading(
            timestamp_ns,
            generation,
            true,
            "selected ROS /clock time",
        ))?;
        previous = Some(timestamp_ns);
        available = true;
    }
}
