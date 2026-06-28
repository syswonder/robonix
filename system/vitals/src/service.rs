// SPDX-License-Identifier: MulanPSL-2.0
//
// gRPC contract handlers:
//   RobonixServiceVitalsGet.GetVitals(GetVitalsRequest) → GetVitalsResponse  (rpc)
//   RobonixServiceVitalsStream.StreamVitals(StreamVitalsRequest) → stream VitalsSnapshot  (server_stream)
//
// Phase 2: returns a hardcoded/mock snapshot.

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

/// Per-joint state for change detection.
#[derive(Clone, Debug)]
struct PrevJointState {
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
    /// Previous body-level state.
    prev_body_state: u32,
    /// Previous per-joint state (error_code, enabled), keyed by name.
    /// Empty until the first body reading arrives.
    prev_joint: HashMap<String, PrevJointState>,
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
                    body: None,
                },
                broadcast_tx,
                prev_health: HashMap::new(),
                prev_body_state: 0,
                prev_joint: HashMap::new(),
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
        if let Some(ref body) = snapshot.body {
            let is_first = state.first_body;

            if body.state != state.prev_body_state {
                log::info!(
                    "[vitals] body state: {} → {}",
                    body_state_label(state.prev_body_state),
                    body_state_label(body.state)
                );
                if !is_first && body.state != 0 {
                    log::warn!(
                        "[vitals] ALERT: body {} ({}) state={}",
                        body.body_type,
                        body.model,
                        body_state_label(body.state)
                    );
                }
                changed = true;
            }
            for joint in &body.joints {
                let prev = state.prev_joint.get(&joint.name);
                let prev_err = prev.map(|p| p.error_code).unwrap_or(0);
                let prev_en = prev.map(|p| p.enabled).unwrap_or(true);
                if joint.error_code != prev_err {
                    log::info!(
                        "[vitals] {} error_code: {} → {}",
                        joint.name,
                        prev_err,
                        joint.error_code
                    );
                    if !is_first && joint.error_code != 0 {
                        log::warn!(
                            "[vitals] ALERT: {} — error_code=0x{:02X}, temp={:.0}°C",
                            joint.name,
                            joint.error_code,
                            joint.temperature
                        );
                    }
                    changed = true;
                }
                if joint.enabled != prev_en {
                    log::info!(
                        "[vitals] {} enabled: {} → {}",
                        joint.name,
                        prev_en,
                        joint.enabled
                    );
                    if !is_first && !joint.enabled {
                        log::warn!("[vitals] ALERT: {} — disabled", joint.name);
                    }
                    changed = true;
                }
            }

            state.first_body = false;

            // Persist body state for next diff.
            state.prev_body_state = body.state;
            state.prev_joint.clear();
            for joint in &body.joints {
                state.prev_joint.insert(
                    joint.name.clone(),
                    PrevJointState {
                        error_code: joint.error_code,
                        enabled: joint.enabled,
                    },
                );
            }
        }

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
