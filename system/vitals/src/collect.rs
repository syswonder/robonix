// SPDX-License-Identifier: MulanPSL-2.0
//
// collect — acquire raw sensor data from a health primitive via gRPC.
//
// Vitals does NOT read sysfs directly. It discovers a `health_primitive`
// through Atlas, then calls `GetHealthState` on the unified contract
// `robonix/primitive/health/state`. The primitive absorbs hardware
// differences; vitals only sees unified fields.

use crate::normalize::RawReading;
use crate::pb::contracts::robonix_primitive_health_state_client::RobonixPrimitiveHealthStateClient;
use crate::pb::health::GetHealthStateRequest;
use crate::pb::vitals::PowerState;
use anyhow::{Context, Result};
use tonic::transport::Channel;

pub struct GrpcCollector {
    client: RobonixPrimitiveHealthStateClient<Channel>,
}

impl GrpcCollector {
    pub fn new(channel: Channel) -> Self {
        Self {
            client: RobonixPrimitiveHealthStateClient::new(channel),
        }
    }

    /// Call the health primitive and convert to vitals' internal types.
    pub async fn collect(&mut self) -> Result<(PowerState, Vec<RawReading>)> {
        let resp = self
            .client
            .get_health_state(GetHealthStateRequest {})
            .await
            .context("call GetHealthState on health primitive")?;

        let state = resp
            .into_inner()
            .state
            .ok_or_else(|| anyhow::anyhow!("GetHealthState returned empty state"))?;

        let power = PowerState {
            battery_percent: -1.0,
            voltage: state.voltage,
            charging: state.charging,
            remaining_s: state.remaining_s,
        };

        let readings: Vec<RawReading> = state
            .readings
            .into_iter()
            .map(|r| RawReading {
                name: r.name,
                temp_c: if r.temp_c >= 0.0 {
                    Some(r.temp_c)
                } else {
                    None
                },
                voltage: if r.voltage >= 0.0 {
                    Some(r.voltage)
                } else {
                    None
                },
                current_a: if r.current_a >= 0.0 {
                    Some(r.current_a)
                } else {
                    None
                },
                battery_percent: if r.battery_percent >= 0.0 {
                    Some(r.battery_percent)
                } else {
                    None
                },
            })
            .collect();

        Ok((power, readings))
    }
}
