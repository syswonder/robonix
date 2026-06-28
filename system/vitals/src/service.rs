// SPDX-License-Identifier: MulanPSL-2.0
//
// gRPC contract handlers:
//   RobonixServiceVitalsGet.GetVitals(GetVitalsRequest) → GetVitalsResponse  (rpc)
//   RobonixServiceVitalsStream.StreamVitals(StreamVitalsRequest) → stream VitalsSnapshot  (server_stream)
//
// Phase 2: returns a hardcoded/mock snapshot.

use crate::body_threshold;
use crate::pb::contracts::robonix_service_vitals_get_server::RobonixServiceVitalsGet;
use crate::pb::contracts::robonix_service_vitals_stream_server::RobonixServiceVitalsStream;
use crate::pb::vitals::{
    GetVitalsRequest, GetVitalsResponse, PowerState, StreamVitalsRequest, VitalsSnapshot,
};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Instant;
use tokio::sync::RwLock;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

/// Per-component state for change detection.
#[derive(Clone, Debug)]
struct PrevComponentState {
    error_code: u32,
    enabled: bool,
}

/// Shared state: latest snapshot + broadcast channel for StreamVitals subscribers.
/// Tracks per-component health to only push on state transitions (OK↔WARN↔ERROR).
struct VitalsState {
    latest: VitalsSnapshot,
    /// Sender for StreamVitals. Each subscriber gets a new Receiver.
    broadcast_tx: tokio::sync::broadcast::Sender<VitalsSnapshot>,
    /// Previous health per component name, keyed by name.
    prev_health: HashMap<String, u32>,
    /// Previous body-level state, keyed by "{body_type}/{model}".
    prev_body_state: HashMap<String, u32>,
    /// Previous per-component state (error_code, enabled), keyed by "{body_type}/{model}/{component_name}".
    prev_component: HashMap<String, PrevComponentState>,
    /// Previous per-component temperature health, keyed by "{body_type}/{model}/{component_name}".
    prev_component_temp: HashMap<String, crate::body_threshold::JointTempHealth>,
    /// Previous power state for voltage / battery change detection.
    prev_power: Option<PowerState>,
    /// True until the first body reading — suppresses ALERTs on startup.
    first_body: bool,
}

/// VitalsServiceImpl — cheap to clone (Arc).
#[derive(Clone)]
pub struct VitalsServiceImpl {
    state: Arc<RwLock<VitalsState>>,
    #[allow(dead_code)] // Phase 3 will use for uptime in snapshot metadata
    start_time: Instant,
}

impl VitalsServiceImpl {
    pub fn new() -> Self {
        let (broadcast_tx, _) = tokio::sync::broadcast::channel(64);
        let now_ns = monotonic_ns();
        Self {
            state: Arc::new(RwLock::new(VitalsState {
                latest: VitalsSnapshot {
                    ts_ns: now_ns,
                    power: Some(PowerState {
                        battery_percent: -1.0,
                        voltage: -1.0,
                        charging: false,
                        remaining_s: -1,
                    }),
                    components: vec![],
                    bodies: vec![],
                },
                broadcast_tx,
                prev_health: HashMap::new(),
                prev_body_state: HashMap::new(),
                prev_component: HashMap::new(),
                prev_component_temp: HashMap::new(),
                prev_power: None,
                first_body: true,
            })),
            start_time: Instant::now(),
        }
    }

    /// Update the cached snapshot. Broadcasts to StreamVitals subscribers only
    /// when at least one component health transitions (OK→WARN, WARN→ERROR, etc.)
    /// or when the snapshot is the first one collected.
    pub async fn update_snapshot(&self, snapshot: VitalsSnapshot) {
        let mut state = self.state.write().await;

        // Check for health state transitions.
        let mut changed = state.prev_health.is_empty(); // always send first snapshot
        for comp in &snapshot.components {
            let prev_h = state.prev_health.get(&comp.name).copied().unwrap_or(0);
            if comp.health != prev_h {
                log::info!(
                    "[vitals] {} health: {} → {} (value={}, threshold={})",
                    comp.name,
                    health_label(prev_h),
                    health_label(comp.health),
                    comp.value,
                    comp.threshold
                );
                // Log non-OK states more prominently.
                if comp.health == crate::normalize::HEALTH_WARN
                    || comp.health == crate::normalize::HEALTH_ERROR
                {
                    log::warn!("[vitals] ALERT: {} — {}", comp.name, comp.detail);
                }
                changed = true;
            }
        }

        // Always cache the latest.
        state.latest = snapshot.clone();

        // Track current health for next diff.
        state.prev_health.clear();
        for comp in &snapshot.components {
            state.prev_health.insert(comp.name.clone(), comp.health);
        }

        // ── Body health transition detection ──────────────────────────
        // Per-body/per-joint keys mean old entries persist across ticks
        // without needing a clear — stale entries just aren't looked up.

        for body in &snapshot.bodies {
            let model = body.model.as_str();
            let body_key = format!("{}/{}", body.body_type, body.model);

            let msg_display = if body.message.is_empty() {
                String::new()
            } else {
                format!(" ({})", body.message)
            };

            if state.first_body {
                // First body reading — always log baseline at info level.
                log::info!(
                    "[vitals] body: {} ({}/{}){}",
                    body_state_label(body.state),
                    body.body_type,
                    body.model,
                    msg_display
                );
                for comp in &body.components {
                    log::info!(
                        "[vitals] {} ({}) enabled: {}, error_code: 0x{:02X}, temp: {:.0}°C",
                        comp.name,
                        comp.kind,
                        comp.enabled,
                        comp.error_code,
                        comp.temperature
                    );
                }
                changed = true;
            } else {
                // Subsequent readings — log only on transition.

                // Body-level state.
                let prev_body_state = state.prev_body_state.get(&body_key).copied().unwrap_or(0);
                if body.state != prev_body_state || !body.message.is_empty() {
                    if body.state != prev_body_state {
                        log::info!(
                            "[vitals] {} body state: {} → {}{}",
                            body_key,
                            body_state_label(prev_body_state),
                            body_state_label(body.state),
                            msg_display
                        );
                        if body.state != 0 {
                            log::warn!(
                                "[vitals] ALERT: body {} state={}{}",
                                body_key,
                                body_state_label(body.state),
                                msg_display
                            );
                        }
                        changed = true;
                    } else if !body.message.is_empty() {
                        // Message changed while state stayed the same — info only.
                        log::info!(
                            "[vitals] {} message: {}{}",
                            body_key,
                            body.message,
                            if prev_body_state != 0 {
                                format!(" (state={})", body_state_label(body.state))
                            } else {
                                String::new()
                            }
                        );
                    }
                }

                for comp in &body.components {
                    let component_key = format!("{}/{}", body_key, comp.name);
                    let prev = state.prev_component.get(&component_key);
                    let prev_err = prev.map(|p| p.error_code).unwrap_or(0);
                    let prev_en = prev.map(|p| p.enabled).unwrap_or(true);

                    // Error code changes.
                    if comp.error_code != prev_err {
                        log::info!(
                            "[vitals] {} error_code: 0x{:02X} → 0x{:02X}",
                            component_key,
                            prev_err,
                            comp.error_code
                        );
                        if comp.error_code != 0 {
                            log::warn!(
                                "[vitals] ALERT: {} error_code=0x{:02X}, temp={:.0}°C",
                                component_key,
                                comp.error_code,
                                comp.temperature
                            );
                        }
                        changed = true;
                    }

                    // Enable / disable changes.
                    if comp.enabled != prev_en {
                        log::info!(
                            "[vitals] {} enabled: {} → {}",
                            component_key,
                            prev_en,
                            comp.enabled
                        );
                        if !comp.enabled {
                            log::warn!("[vitals] ALERT: {} — disabled", component_key);
                        }
                        changed = true;
                    }

                    // ── Temperature threshold check ──────────────────
                    let (temp_health, temp_detail) =
                        body_threshold::evaluate_temp(comp.temperature, model);
                    let prev_temp = state
                        .prev_component_temp
                        .get(&component_key)
                        .map(|t| t.health)
                        .unwrap_or(body_threshold::HEALTH_OK);
                    if temp_health != prev_temp {
                        log::info!(
                            "[vitals] {} temp health: {} → {} ({:.0}°C)",
                            component_key,
                            health_label(prev_temp),
                            health_label(temp_health),
                            comp.temperature
                        );
                        if temp_health == body_threshold::HEALTH_WARN
                            || temp_health == body_threshold::HEALTH_ERROR
                        {
                            log::warn!("[vitals] ALERT: {} — {}", component_key, temp_detail);
                        }
                        changed = true;
                    }
                }
            }

            // Persist body state for next diff.
            state.prev_body_state.insert(body_key.clone(), body.state);
            for comp in &body.components {
                let component_key = format!("{}/{}", body_key, comp.name);
                let (health, _) = body_threshold::evaluate_temp(comp.temperature, model);
                state.prev_component_temp.insert(
                    component_key.clone(),
                    body_threshold::JointTempHealth { health },
                );
                state.prev_component.insert(
                    component_key,
                    PrevComponentState {
                        error_code: comp.error_code,
                        enabled: comp.enabled,
                    },
                );
            }
        }

        if !snapshot.bodies.is_empty() {
            state.first_body = false;
        }

        // ── Power state change detection ──────────────────────────────
        // Voltage and battery changes don't go through ComponentHealth, so we
        // track them separately.  Without this, power-only changes (e.g. voltage
        // sag) produce no log output and no StreamVitals push.
        let power = snapshot.power.as_ref();
        let prev_power = state.prev_power.as_ref();
        let power_changed = match (power, prev_power) {
            (Some(cur), Some(prev)) => {
                (cur.voltage - prev.voltage).abs() > 0.05
                    || cur.charging != prev.charging
                    || (cur.battery_percent - prev.battery_percent).abs() > 0.5
            }
            (Some(_), None) => true,  // first snapshot, force log
            _ => false,
        };
        if power_changed {
            if let (Some(cur), Some(prev)) = (power, prev_power) {
                if (cur.voltage - prev.voltage).abs() > 0.05 {
                    log::info!(
                        "[vitals] voltage: {:.2}V → {:.2}V",
                        prev.voltage,
                        cur.voltage
                    );
                }
                if cur.charging != prev.charging {
                    log::info!(
                        "[vitals] charging: {} → {}",
                        prev.charging,
                        cur.charging
                    );
                }
                if (cur.battery_percent - prev.battery_percent).abs() > 0.5 {
                    log::info!(
                        "[vitals] battery: {:.0}% → {:.0}%",
                        prev.battery_percent,
                        cur.battery_percent
                    );
                }
            }
            changed = true;
        }
        state.prev_power = power.cloned();

        // Only broadcast on state transitions to avoid flooding subscribers.
        if changed {
            // One-line summary: "cpu:OK(38C) gpu:OK(39C) ..."
            let summary: Vec<String> = snapshot
                .components
                .iter()
                .map(|c| format!("{}:{}({:.0})", c.name, health_label(c.health), c.value))
                .collect();
            let voltage_str = snapshot
                .power
                .as_ref()
                .map(|p| format!("{:.1}V", p.voltage))
                .unwrap_or_else(|| "?V".to_string());
            log::info!("[vitals] {} | {}", voltage_str, summary.join(" "));
            let _ = state.broadcast_tx.send(snapshot);
        }
    }

    /// Return the latest cached snapshot.
    pub async fn latest_snapshot(&self) -> VitalsSnapshot {
        self.state.read().await.latest.clone()
    }

    #[allow(dead_code)] // Phase 3 will expose uptime through snapshot metadata
    pub fn start_instant(&self) -> Instant {
        self.start_time
    }
}

fn health_label(h: u32) -> &'static str {
    match h {
        0 => "OK",
        1 => "WARN",
        2 => "ERROR",
        _ => "UNKNOWN",
    }
}

fn body_state_label(s: u32) -> &'static str {
    match s {
        0 => "NORMAL",
        1 => "FAULT",
        2 => "ESTOP",
        _ => "UNKNOWN",
    }
}

fn monotonic_ns() -> i64 {
    let now = Instant::now();
    now.elapsed().as_nanos() as i64
}

#[tonic::async_trait]
impl RobonixServiceVitalsGet for VitalsServiceImpl {
    async fn get_vitals(
        &self,
        _request: Request<GetVitalsRequest>,
    ) -> Result<Response<GetVitalsResponse>, Status> {
        let snapshot = self.latest_snapshot().await;
        Ok(Response::new(GetVitalsResponse {
            snapshot: Some(snapshot),
        }))
    }
}

#[tonic::async_trait]
impl RobonixServiceVitalsStream for VitalsServiceImpl {
    type StreamVitalsStream = ReceiverStream<Result<VitalsSnapshot, Status>>;

    async fn stream_vitals(
        &self,
        _request: Request<StreamVitalsRequest>,
    ) -> Result<Response<Self::StreamVitalsStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(64);
        let mut broadcast_rx = {
            let state = self.state.read().await;
            // Send current snapshot immediately so the subscriber has initial state.
            let current = state.latest.clone();
            let _ = tx.send(Ok(current)).await;
            state.broadcast_tx.subscribe()
        };

        tokio::spawn(async move {
            loop {
                match broadcast_rx.recv().await {
                    Ok(snapshot) => {
                        if tx.send(Ok(snapshot)).await.is_err() {
                            break; // subscriber disconnected
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                        log::warn!("[vitals] StreamVitals subscriber lagged by {n} messages");
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                        break;
                    }
                }
            }
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}
