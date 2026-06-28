// SPDX-License-Identifier: MulanPSL-2.0
//
// collect — read raw sensor data from sysfs / hwmon / thermal.
//
// Vitals reads sysfs directly in v0.1. When Soma (the body system component)
// is ready, hardware access will move to Soma and vitals will consume from
// Soma's unified health contract.

use crate::normalize::RawReading;
use crate::pb::vitals::PowerState;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

// ── HwmonMap ────────────────────────────────────────────────────────────────

struct HwmonMap {
    base_dir: PathBuf,
    names: BTreeMap<String, u32>,
}

impl HwmonMap {
    fn scan() -> anyhow::Result<Self> {
        let base_dir = PathBuf::from("/sys/class/hwmon");
        let mut names = BTreeMap::new();
        for entry in std::fs::read_dir(&base_dir)? {
            let entry = entry?;
            let dir_str = entry.file_name().to_string_lossy().to_string();
            if let Some(num_str) = dir_str.strip_prefix("hwmon")
                && let Ok(idx) = num_str.parse::<u32>()
            {
                let name_path = entry.path().join("name");
                if let Ok(name) = std::fs::read_to_string(&name_path) {
                    names.insert(name.trim().to_string(), idx);
                }
            }
        }
        Ok(Self { base_dir, names })
    }

    fn read_sensor(&self, device: &str, file: &str) -> Option<i64> {
        let idx = self.names.get(device)?;
        let path = self.base_dir.join(format!("hwmon{idx}")).join(file);
        std::fs::read_to_string(&path)
            .ok()?
            .trim()
            .parse::<i64>()
            .ok()
    }
}

// ── SysfsCollector ──────────────────────────────────────────────────────────

pub struct SysfsCollector {
    hwmon: HwmonMap,
    thermal_zones: BTreeMap<String, PathBuf>,
}

impl SysfsCollector {
    /// Scan sysfs for available sensors. Returns an error only if hwmon
    /// enumeration fails entirely; missing thermal zones are tolerated.
    pub fn new() -> anyhow::Result<Self> {
        let hwmon = HwmonMap::scan()?;
        let mut thermal_zones = BTreeMap::new();
        let thermal_base = Path::new("/sys/class/thermal");
        if thermal_base.is_dir() {
            for entry in std::fs::read_dir(thermal_base)? {
                let entry = entry?;
                let name_str = entry.file_name().to_string_lossy().to_string();
                if !name_str.starts_with("thermal_zone") {
                    continue;
                }
                let type_path = entry.path().join("type");
                if let Ok(zone_type) = std::fs::read_to_string(&type_path) {
                    thermal_zones.insert(zone_type.trim().to_string(), entry.path().join("temp"));
                }
            }
        }
        Ok(Self {
            hwmon,
            thermal_zones,
        })
    }

    fn read_thermal_c(&self, zone_name: &str) -> Option<f32> {
        let path = self.thermal_zones.get(zone_name)?;
        let raw = std::fs::read_to_string(path).ok()?;
        let millic: f32 = raw.trim().parse().ok()?;
        Some(millic / 1000.0)
    }

    /// Read all sensors and return (power_state, raw_readings).
    pub fn collect(&self) -> (PowerState, Vec<RawReading>) {
        let mut readings: Vec<RawReading> = Vec::new();

        // Thermal zones — strip "-thermal" suffix so names match threshold rules.
        for zone_name in self.thermal_zones.keys() {
            if let Some(temp) = self.read_thermal_c(zone_name) {
                let short = zone_name.strip_suffix("-thermal").unwrap_or(zone_name);
                readings.push(RawReading {
                    name: short.to_string(),
                    temp_c: Some(temp),
                    voltage: None,
                    current_a: None,
                    battery_percent: None,
                });
            }
        }

        // NVMe temperature (milli-°C → °C).
        if let Some(mc) = self.hwmon.read_sensor("nvme", "temp1_input") {
            readings.push(RawReading {
                name: "nvme".into(),
                temp_c: Some(mc as f32 / 1000.0),
                voltage: None,
                current_a: None,
                battery_percent: None,
            });
        }

        // System voltage from INA3221 or INA238 (mV → V).
        let mut voltage: f32 = -1.0;
        if let Some(mv) = self.hwmon.read_sensor("ina3221", "in1_input") {
            voltage = mv as f32 / 1000.0;
        } else if let Some(mv) = self.hwmon.read_sensor("ina238", "in1_input") {
            voltage = mv as f32 / 1000.0;
        }

        let power = PowerState {
            battery_percent: -1.0,
            voltage,
            charging: false,
            remaining_s: -1,
        };

        (power, readings)
    }
}
