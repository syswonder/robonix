// SPDX-License-Identifier: MulanPSL-2.0
//
// body — read robot body (arm/dog joint motor) health data.
//
// v0.1 communicates with a Python subprocess that calls the hardware SDK
// directly. When Soma is ready, this module becomes a thin gRPC client
// consuming Soma's unified health interface — the BodyHealth struct stays
// the same, only the transport changes.

use crate::pb::body::BodyHealth;
use anyhow::Context;
use serde::Deserialize;
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, Command, Stdio};

// ── BodyCollector ──────────────────────────────────────────────────────────

pub struct BodyCollector {
    stdin: ChildStdin,
    reader: BufReader<std::process::ChildStdout>,
    _child: Child,
    body_type: String,
    model: String,
}

impl BodyCollector {
    /// Spawn the Python bridge script. Returns an error if the script
    /// cannot be started.
    pub fn new(
        body_type: &str,
        model: &str,
        script: &str,
        python_bin: &str,
    ) -> anyhow::Result<Self> {
        let mut child = Command::new(python_bin)
            .arg(script)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .with_context(|| format!("spawn body script '{script}' with '{python_bin}'"))?;

        let stdin = child.stdin.take().context("capture body script stdin")?;
        let stdout = child.stdout.take().context("capture body script stdout")?;
        let reader = BufReader::new(stdout);

        Ok(Self {
            stdin,
            reader,
            _child: child,
            body_type: body_type.to_string(),
            model: model.to_string(),
        })
    }

    /// Send a collect command and parse the JSON response into a BodyHealth.
    pub fn collect(&mut self) -> BodyHealth {
        // Issue the command.
        let cmd = serde_json::json!({"cmd": "collect"});
        if let Err(e) = writeln!(self.stdin, "{cmd}") {
            log::warn!("[body] write to script failed: {e:#}");
            return self.fault(&format!("write: {e:#}"));
        }
        if let Err(e) = self.stdin.flush() {
            log::warn!("[body] flush stdin failed: {e:#}");
            return self.fault(&format!("flush: {e:#}"));
        }

        // Read one JSON line back.
        let mut line = String::new();
        if let Err(e) = self.reader.read_line(&mut line) {
            log::warn!("[body] read from script failed: {e:#}");
            return self.fault(&format!("read: {e:#}"));
        }
        if line.trim().is_empty() {
            log::warn!("[body] empty response from script");
            return self.fault("empty response");
        }

        match serde_json::from_str::<BodyJson>(line.trim()) {
            Ok(parsed) => parsed.into_body_health(&self.body_type, &self.model),
            Err(e) => {
                log::warn!("[body] parse JSON failed: {e:#} — raw: {}", line.trim());
                self.fault(&format!("parse: {e:#}"))
            }
        }
    }

    /// Check if the subprocess is still alive.
    pub fn is_alive(&mut self) -> bool {
        match self._child.try_wait() {
            Ok(None) => true,
            Ok(Some(status)) => {
                log::warn!("[body] script exited with {status}");
                false
            }
            Err(e) => {
                log::warn!("[body] try_wait failed: {e:#}");
                false
            }
        }
    }

    /// Attempt to restart the subprocess after a crash.
    pub fn restart(&mut self, script: &str, python_bin: &str) -> anyhow::Result<()> {
        let mut child = Command::new(python_bin)
            .arg(script)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .with_context(|| format!("restart body script '{script}'"))?;

        self.stdin = child.stdin.take().context("capture body script stdin")?;
        let stdout = child.stdout.take().context("capture body script stdout")?;
        self.reader = BufReader::new(stdout);
        self._child = child;
        Ok(())
    }

    fn fault(&self, reason: &str) -> BodyHealth {
        log::warn!("[body] FAULT ({reason})");
        BodyHealth {
            body_type: self.body_type.clone(),
            model: self.model.clone(),
            state: 1, // FAULT
            joints: Vec::new(),
        }
    }
}

// ── JSON wire format ───────────────────────────────────────────────────────

#[derive(Deserialize)]
struct JointJson {
    name: String,
    temperature: f32,
    error_code: u32,
    enabled: bool,
}

#[derive(Deserialize)]
struct BodyJson {
    #[serde(default)]
    state: u32,
    #[serde(default)]
    joints: Vec<JointJson>,
}

impl BodyJson {
    fn into_body_health(self, body_type: &str, model: &str) -> BodyHealth {
        BodyHealth {
            body_type: body_type.to_string(),
            model: model.to_string(),
            state: self.state,
            joints: self
                .joints
                .into_iter()
                .map(|j| crate::pb::body::JointHealth {
                    name: j.name,
                    temperature: j.temperature,
                    error_code: j.error_code,
                    enabled: j.enabled,
                })
                .collect(),
        }
    }
}
