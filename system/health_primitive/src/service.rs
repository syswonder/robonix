// SPDX-License-Identifier: MulanPSL-2.0

use crate::pb::contracts::robonix_primitive_health_state_server::RobonixPrimitiveHealthState;
use crate::pb::contracts::robonix_primitive_health_stream_server::RobonixPrimitiveHealthStream;
use crate::pb::health::{
    GetHealthStateRequest, GetHealthStateResponse, HealthState, SensorReading,
    StreamHealthStateRequest,
};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use tokio::sync::RwLock;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

// ── HwmonMap (same pattern as vitals) ─────────────────────────────────────

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

// ── Collector ──────────────────────────────────────────────────────────────

pub struct Collector {
    hwmon: HwmonMap,
    thermal_zones: BTreeMap<String, PathBuf>,
}

impl Collector {
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

    pub fn collect(&self) -> HealthState {
        let mut readings: Vec<SensorReading> = Vec::new();

        // Thermal zones
        for zone_name in self.thermal_zones.keys() {
            if let Some(temp) = self.read_thermal_c(zone_name) {
                // Strip "-thermal" suffix so names match vitals threshold rules.
                let short = zone_name.strip_suffix("-thermal").unwrap_or(zone_name);
                readings.push(SensorReading {
                    name: short.to_string(),
                    temp_c: temp,
                    voltage: -1.0,
                    current_a: -1.0,
                    battery_percent: -1.0,
                });
            }
        }

        // NVMe temp
        if let Some(mc) = self.hwmon.read_sensor("nvme", "temp1_input") {
            readings.push(SensorReading {
                name: "nvme".into(),
                temp_c: mc as f32 / 1000.0,
                voltage: -1.0,
                current_a: -1.0,
                battery_percent: -1.0,
            });
        }

        // System voltage from INA3221 or INA238
        let mut voltage: f32 = -1.0;
        if let Some(mv) = self.hwmon.read_sensor("ina3221", "in1_input") {
            voltage = mv as f32 / 1000.0;
        } else if let Some(mv) = self.hwmon.read_sensor("ina238", "in1_input") {
            voltage = mv as f32 / 1000.0;
        }

        HealthState {
            voltage,
            charging: false,
            remaining_s: -1,
            readings,
        }
    }
}

// ── Service ────────────────────────────────────────────────────────────────

struct HealthStateState {
    latest: HealthState,
    broadcast_tx: tokio::sync::broadcast::Sender<HealthState>,
}

#[derive(Clone)]
pub struct HealthPrimitiveService {
    state: Arc<RwLock<HealthStateState>>,
}

impl HealthPrimitiveService {
    pub fn new() -> Self {
        let (broadcast_tx, _) = tokio::sync::broadcast::channel(64);
        Self {
            state: Arc::new(RwLock::new(HealthStateState {
                latest: HealthState {
                    voltage: -1.0,
                    charging: false,
                    remaining_s: -1,
                    readings: vec![],
                },
                broadcast_tx,
            })),
        }
    }

    pub async fn update_state(&self, s: HealthState) {
        let mut state = self.state.write().await;
        state.latest = s.clone();
        let _ = state.broadcast_tx.send(s);
    }
}

#[tonic::async_trait]
impl RobonixPrimitiveHealthState for HealthPrimitiveService {
    async fn get_health_state(
        &self,
        _request: Request<GetHealthStateRequest>,
    ) -> Result<Response<GetHealthStateResponse>, Status> {
        let state = self.state.read().await;
        Ok(Response::new(GetHealthStateResponse {
            state: Some(state.latest.clone()),
        }))
    }
}

#[tonic::async_trait]
impl RobonixPrimitiveHealthStream for HealthPrimitiveService {
    type StreamHealthStateStream = ReceiverStream<Result<HealthState, Status>>;

    async fn stream_health_state(
        &self,
        _request: Request<StreamHealthStateRequest>,
    ) -> Result<Response<Self::StreamHealthStateStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(64);
        let mut broadcast_rx = {
            let state = self.state.read().await;
            let current = state.latest.clone();
            let _ = tx.send(Ok(current)).await;
            state.broadcast_tx.subscribe()
        };
        tokio::spawn(async move {
            loop {
                match broadcast_rx.recv().await {
                    Ok(s) => {
                        if tx.send(Ok(s)).await.is_err() {
                            break;
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {}
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
                }
            }
        });
        Ok(Response::new(ReceiverStream::new(rx)))
    }
}
