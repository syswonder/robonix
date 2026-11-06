// SPDX-License-Identifier: MulanPSL-2.0
//! Map client-owned capability execution to RTDL progress and results.

use crate::pb::pilot::rtdl_node_state::RtdlNodeStateEnum;
use crate::pb::pilot::{CapabilityCall, CapabilityCallResult};
use crate::plan_runtime::{PlanRuntime, RunningAsyncCall};
use crate::rtdl_wire::NodeEventContext;
use robonix_atlas::client::channel::Channel;
use std::time::Duration;

/// The channel owns protocol and execution lifetime; Executor owns RTDL events.
pub async fn run_until_terminal(
    call: &CapabilityCall,
    channel: &Channel,
    node: &NodeEventContext,
    runtime: &PlanRuntime,
) -> (CapabilityCallResult, u32) {
    let execution = async {
        let args = serde_json::from_str(&call.args_json)?;
        if channel.is_async() {
            let accepted = runtime
                .register_or_cancel_async_call(
                    &node.plan_id,
                    RunningAsyncCall {
                        call_id: call.call_id.clone(),
                        channel: channel.clone(),
                    },
                )
                .await;
            if !accepted {
                anyhow::ensure!(
                    channel.is_closed(),
                    "cancellation remains unconfirmed; channel {} retained",
                    channel.channel_id
                );
                return Ok((
                    serde_json::json!({"detail": "plan was cancelled"}),
                    RtdlNodeStateEnum::Canceled as u32,
                ));
            }
        }
        let initial = channel.call(args).await?;
        if channel.is_terminal() {
            channel.close().await?;
            let state = if channel.is_async()
                && initial.get("accepted") == Some(&serde_json::Value::Bool(false))
            {
                RtdlNodeStateEnum::Failed
            } else {
                RtdlNodeStateEnum::Succeeded
            };
            return Ok::<_, anyhow::Error>((initial, state as u32));
        }
        loop {
            if runtime.is_cancelled(&node.plan_id).await {
                channel.close().await?;
                return Ok((
                    serde_json::json!({"detail": "plan was cancelled"}),
                    RtdlNodeStateEnum::Canceled as u32,
                ));
            }
            let status = tokio::time::timeout(Duration::from_secs(30), channel.status()).await??;
            let state = parse_state_name(status["state"].as_str().unwrap_or_default())
                .ok_or_else(|| anyhow::anyhow!("invalid status response: {status}"))?;
            if crate::rtdl_wire::is_terminal_state(state) {
                channel.close().await?;
                return Ok((status, state));
            }
            runtime
                .record_op_state(&node.plan_id, &node.op_id, state)
                .await;
            tokio::time::sleep(Duration::from_secs(2)).await;
        }
    }
    .await;
    runtime
        .unregister_async_call(&node.plan_id, &call.call_id)
        .await;
    match execution {
        Ok((output, state)) => {
            let success = state == RtdlNodeStateEnum::Succeeded as u32;
            (
                CapabilityCallResult {
                    call_id: call.call_id.clone(),
                    provider_id: call.provider_id.clone(),
                    contract_id: call.contract_id.clone(),
                    success,
                    output: if success {
                        output.to_string()
                    } else {
                        String::new()
                    },
                    error: if success {
                        String::new()
                    } else {
                        output["detail"].as_str().unwrap_or_default().to_owned()
                    },
                },
                state,
            )
        }
        Err(error) => {
            let cleanup = if channel.is_terminal() {
                channel.close().await.err()
            } else {
                None
            };
            (
                super::error_result(
                    call,
                    format!(
                        "{error:#}; execution channel {}; cleanup: {cleanup:?}",
                        channel.channel_id
                    ),
                ),
                RtdlNodeStateEnum::Failed as u32,
            )
        }
    }
}

pub fn parse_state_name(s: &str) -> Option<u32> {
    Some(match s.to_ascii_uppercase().as_str() {
        "PENDING" => RtdlNodeStateEnum::Pending,
        "RUNNING" => RtdlNodeStateEnum::Running,
        "SUCCEEDED" => RtdlNodeStateEnum::Succeeded,
        "FAILED" => RtdlNodeStateEnum::Failed,
        "CANCELED" | "CANCELLED" => RtdlNodeStateEnum::Canceled,
        "TIMEOUT" => RtdlNodeStateEnum::Timeout,
        "PAUSED" => RtdlNodeStateEnum::Paused,
        _ => return None,
    } as u32)
}
