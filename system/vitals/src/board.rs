// SPDX-License-Identifier: MulanPSL-2.0
//
// board — read board-level health data (CPU/GPU/NVMe temp, voltage).
//
// v0.1 communicates with a Python subprocess (board.py) that reads sysfs
// directly.  When Soma is ready, BoardCollector becomes a gRPC client
// consuming Soma's unified BoardHealth interface — the data structures
// stay the same, only the transport changes.

use crate::normalize::RawReading;
use crate::pb::vitals::PowerState;
use crate::subprocess::SubprocessHandle;
use anyhow::Context;
use serde::Deserialize;

// ── BoardCollector ─────────────────────────────────────────────────────────

pub struct BoardCollector {
    sub: SubprocessHandle,
}

impl BoardCollector {
    /// Spawn the Python sysfs bridge script.
    pub fn new(script: &str, python_bin: &str) -> anyhow::Result<Self> {
        let sub = SubprocessHandle::spawn(script, python_bin, "board")
            .with_context(|| format!("spawn board script '{script}'"))?;
        Ok(Self { sub })
    }

    /// Send a collect command and return (PowerState, Vec<RawReading>).
    /// On any failure returns empty readings with voltage=-1.
    pub fn collect(&mut self) -> (PowerState, Vec<RawReading>) {
        let line = match self.sub.collect_json() {
            Some(l) => l,
            None => return Self::empty(),
        };
        match serde_json::from_str::<BoardJson>(&line) {
            Ok(parsed) => parsed.into_parts(),
            Err(e) => {
                log::warn!("[board] parse JSON failed: {e:#} — raw: {line}");
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

    fn empty() -> (PowerState, Vec<RawReading>) {
        (
            PowerState {
                battery_percent: -1.0,
                voltage: -1.0,
                charging: false,
                remaining_s: -1,
            },
            vec![],
        )
    }
}

// ── JSON wire format ───────────────────────────────────────────────────────

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
struct BoardJson {
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
}

impl BoardJson {
    fn into_parts(self) -> (PowerState, Vec<RawReading>) {
        let power = PowerState {
            battery_percent: self.battery_percent,
            voltage: self.voltage,
            charging: self.charging,
            remaining_s: self.remaining_s,
        };
        let readings = self
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
        (power, readings)
    }
}
