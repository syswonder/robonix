// SPDX-License-Identifier: MulanPSL-2.0
//
// body — read robot body (arm/dog joint motor) health data.
//
// v0.1 communicates with a Python subprocess that calls the hardware SDK
// directly.  BodyCollector learns body_type and model from the script's
// JSON response — no CLI flags needed.
//
// When Soma is ready, BodyCollector becomes a gRPC client consuming Soma's
// unified health interface — the data structures stay the same.

use crate::pb::body::BodyHealth;
use crate::subprocess::SubprocessHandle;
use anyhow::Context;
use serde::Deserialize;

// ── BodyCollector ──────────────────────────────────────────────────────────

pub struct BodyCollector {
    sub: SubprocessHandle,
    /// Cached from the last successful collect; "unknown" until first response.
    body_type: String,
    model: String,
}

impl BodyCollector {
    /// Spawn a Python body-script.  The script reports its own body_type and
    /// model in every JSON response.
    pub fn new(script: &str, python_bin: &str) -> anyhow::Result<Self> {
        let label = format!(
            "body/{}",
            std::path::Path::new(script)
                .file_stem()
                .unwrap_or_default()
                .to_string_lossy()
        );
        let sub = SubprocessHandle::spawn(script, python_bin, &label)
            .with_context(|| format!("spawn body script '{script}'"))?;
        Ok(Self {
            sub,
            body_type: "unknown".to_string(),
            model: "unknown".to_string(),
        })
    }

    /// Send a collect command and return the parsed BodyHealth.
    /// On the first successful response the cached body_type / model are
    /// updated so that subsequent faults carry the right labels.
    pub fn collect(&mut self) -> BodyHealth {
        let line = match self.sub.collect_json() {
            Some(l) => l,
            None => return self.fault("subprocess I/O error"),
        };
        match serde_json::from_str::<BodyJson>(&line) {
            Ok(parsed) => {
                self.body_type = parsed.body_type.clone();
                self.model = parsed.model.clone();
                parsed.into_body_health()
            }
            Err(e) => {
                log::warn!("[body] parse JSON failed: {e:#} — raw: {line}");
                self.fault(&format!("parse: {e:#}"))
            }
        }
    }

    /// Check whether the subprocess is still alive.
    pub fn is_alive(&mut self) -> bool {
        self.sub.is_alive()
    }

    /// Attempt to restart after a crash.
    pub fn restart(&mut self, script: &str, python_bin: &str) -> anyhow::Result<()> {
        self.sub.restart(script, python_bin)
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
    body_type: String,
    #[serde(default)]
    model: String,
    #[serde(default)]
    state: u32,
    #[serde(default)]
    joints: Vec<JointJson>,
}

impl BodyJson {
    fn into_body_health(self) -> BodyHealth {
        BodyHealth {
            body_type: self.body_type,
            model: self.model,
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
