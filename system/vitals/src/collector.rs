// SPDX-License-Identifier: MulanPSL-2.0
//
// collector — single subprocess bridge for board + zero or more body health.
//
// Spawns one Python script that reads all available sensors (sysfs + body SDKs)
// and returns a unified JSON line.  Board data is always present; body data is
// an array — the script returns "bodies": [] when no body hardware is connected.
//
// When Soma is ready, VitalsCollector becomes a gRPC client consuming Soma's
// unified interface — the data structures stay the same.

use crate::normalize::RawReading;
use crate::pb::vitals::{BodyComponent, BodyHealth, PowerState};
use crate::subprocess::SubprocessHandle;
use anyhow::Context;
use serde::Deserialize;

// ── VitalsCollector ─────────────────────────────────────────────────────────

pub struct VitalsCollector {
    sub: SubprocessHandle,
}

impl VitalsCollector {
    /// Spawn the unified Python collector script.
    pub fn new(script: &str, python_bin: &str) -> anyhow::Result<Self> {
        let sub = SubprocessHandle::spawn(script, python_bin, "vitals")
            .with_context(|| format!("spawn vitals script '{script}'"))?;
        Ok(Self { sub })
    }

    /// Send a collect command and return (PowerState, Vec<RawReading>, Vec<BodyHealth>).
    /// On any failure returns empty board data with voltage=-1 and empty bodies.
    pub fn collect(&mut self) -> (PowerState, Vec<RawReading>, Vec<BodyHealth>) {
        let line = match self.sub.collect_json() {
            Some(l) => l,
            None => return Self::empty(),
        };
        match serde_json::from_str::<CollectJson>(&line) {
            Ok(parsed) => parsed.into_parts(),
            Err(e) => {
                log::warn!("[vitals] parse JSON failed: {e:#} — raw: {line}");
                Self::empty()
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

    fn empty() -> (PowerState, Vec<RawReading>, Vec<BodyHealth>) {
        (
            PowerState {
                battery_percent: -1.0,
                voltage: -1.0,
                charging: false,
                remaining_s: -1,
            },
            vec![],
            vec![],
        )
    }
}

// ── JSON wire format (unified) ──────────────────────────────────────────────

#[derive(Deserialize)]
struct ComponentJson {
    name: String,
    #[serde(default)]
    temperature: f32,
    #[serde(default)]
    voltage: f32,
    #[serde(default)]
    current: f32,
    #[serde(default)]
    battery_percent: f32,
}

#[derive(Deserialize)]
struct BodyComponentJson {
    name: String,
    #[serde(default)]
    kind: String,
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
    message: String,
    #[serde(default)]
    components: Vec<BodyComponentJson>,
}

#[derive(Deserialize)]
struct CollectJson {
    #[serde(default)]
    battery_percent: f32,
    #[serde(default)]
    voltage: f32,
    #[serde(default)]
    charging: bool,
    #[serde(default)]
    remaining_s: i64,
    #[serde(default)]
    components: Vec<ComponentJson>,
    #[serde(default)]
    bodies: Vec<BodyJson>,
}

impl CollectJson {
    fn into_parts(self) -> (PowerState, Vec<RawReading>, Vec<BodyHealth>) {
        let power = PowerState {
            battery_percent: self.battery_percent,
            voltage: self.voltage,
            charging: self.charging,
            remaining_s: self.remaining_s,
        };
        let readings: Vec<RawReading> = self
            .components
            .into_iter()
            .map(|c| RawReading {
                name: c.name,
                temp_c: if c.temperature >= 0.0 {
                    Some(c.temperature)
                } else {
                    None
                },
                voltage: if c.voltage >= 0.0 {
                    Some(c.voltage)
                } else {
                    None
                },
                current_a: if c.current >= 0.0 {
                    Some(c.current)
                } else {
                    None
                },
                battery_percent: if c.battery_percent >= 0.0 {
                    Some(c.battery_percent)
                } else {
                    None
                },
            })
            .collect();
        let bodies: Vec<BodyHealth> = self
            .bodies
            .into_iter()
            .map(|b| BodyHealth {
                body_type: b.body_type,
                model: b.model,
                state: b.state,
                message: b.message,
                components: b
                    .components
                    .into_iter()
                    .map(|c| BodyComponent {
                        name: c.name,
                        kind: c.kind,
                        temperature: c.temperature,
                        error_code: c.error_code,
                        enabled: c.enabled,
                    })
                    .collect(),
            })
            .collect();
        (power, readings, bodies)
    }
}
