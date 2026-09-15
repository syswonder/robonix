// SPDX-License-Identifier: MulanPSL-2.0

use crate::pb::contracts::robonix_system_executor_execute_client::RobonixSystemExecutorExecuteClient;
use crate::pb::executor::rtdl_event::RtdlEventEnum;
use crate::pb::pilot::rtdl_node_state::RtdlNodeStateEnum;
use crate::pb::pilot::{CapabilityCall, Plan, RtdlNode};
use crate::planner::ExecutorConn;
use crate::vlm::Message;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use serde_json::{Value, json};
use std::time::Duration;
use tonic::Request;
use tonic::transport::Channel;
use uuid::Uuid;

const RTDL_DO: u32 = 2;
const SCENE_CONTEXT: &str = "robonix/system/scene/get_robot_context";
const CAMERA_SNAPSHOT: &str = "robonix/primitive/camera/snapshot";

pub async fn collect(
    executor: &ExecutorConn,
    atlas: &mut AtlasClient,
    caps: &[(String, atlas_pb::Capability)],
) -> String {
    let scene_target = caps
        .iter()
        .find(|(_, capability)| capability.contract_id == SCENE_CONTEXT)
        .map(|(provider_id, _)| provider_id.clone());
    let scene = match scene_target {
        Some(provider_id) => query_scene(executor.graph.clone(), provider_id).await,
        None => json!({"available": false, "error": "Scene context contract is not registered"}),
    };
    let providers = match atlas
        .query_capabilities("", "", atlas_pb::Transport::Unspecified)
        .await
    {
        Ok(rows) => rows
            .into_iter()
            .filter(|provider| {
                provider.capabilities.iter().any(|capability| {
                    let id = capability.contract_id.as_str();
                    id.contains("/navigation/")
                        || id.contains("/scene/")
                        || id.contains("/arm/")
                        || id.contains("/chassis/")
                        || id.contains("/skill/")
                })
            })
            .map(|provider| {
                json!({
                    "provider_id": provider.id,
                    "state": lifecycle_name(provider.state),
                    "detail": provider.state_detail,
                })
            })
            .collect::<Vec<_>>(),
        Err(error) => vec![json!({"available": false, "error": error.to_string()})],
    };
    format!(
        "\n\n## Current environment and provider state\n\
         Refreshed immediately before this planning round. Scene owns map pose, \
         room membership, areas, and nearby objects. Atlas state only indicates \
         provider availability; current task progress remains in the in-flight RTDL \
         tree block. Missing or stale state means unknown.\n\n{}\n",
        serde_json::to_string(&json!({
            "scene": scene,
            "provider_availability": providers,
        }))
        .unwrap_or_else(|_| "{}".into())
    )
}

async fn query_scene(
    graph: RobonixSystemExecutorExecuteClient<Channel>,
    provider_id: String,
) -> Value {
    match query_capability(
        graph,
        provider_id,
        SCENE_CONTEXT,
        "scene_snapshot",
        "Read Scene spatial context before planning",
    )
    .await
    {
        Ok(value) => json!({"available": true, "state": value}),
        Err(error) => json!({"available": false, "error": error}),
    }
}

/// Dispatch one capability through Executor and return its output, so a
/// prefetch reaches a provider the same way a planned call would: same plan
/// shape, same lifecycle, same failure reporting.
async fn query_capability(
    mut graph: RobonixSystemExecutorExecuteClient<Channel>,
    provider_id: String,
    contract_id: &str,
    op_id: &str,
    description: &str,
) -> Result<Value, String> {
    let plan_id = format!("state-prefetch-{}", Uuid::new_v4());
    let plan = Plan {
        plan_id: plan_id.clone(),
        session_id: "pilot-state-prefetch".into(),
        round: 0,
        nodes: vec![RtdlNode {
            node_kind: RTDL_DO,
            children: Vec::new(),
            call: Some(CapabilityCall {
                call_id: format!("{plan_id}:0"),
                provider_id,
                contract_id: contract_id.into(),
                args_json: "{}".into(),
            }),
            op_id: op_id.into(),
            description: description.into(),
        }],
        root_index: 0,
    };
    let query = async {
        let mut stream = graph
            .execute(Request::new(plan))
            .await
            .map_err(|error| error.to_string())?
            .into_inner();
        while let Some(event) = stream.message().await.map_err(|error| error.to_string())? {
            if event.event_kind != RtdlEventEnum::NodeState as u32 {
                continue;
            }
            let Some(state) = event.node_state else {
                continue;
            };
            if state.state == RtdlNodeStateEnum::Succeeded as u32 {
                let output = state
                    .leaf_result
                    .map(|result| result.output)
                    .unwrap_or(state.operator_detail);
                return Ok(serde_json::from_str(&output).unwrap_or_else(|_| json!({"raw": output})));
            }
            if matches!(
                RtdlNodeStateEnum::try_from(state.state as i32),
                Ok(RtdlNodeStateEnum::Failed
                    | RtdlNodeStateEnum::Canceled
                    | RtdlNodeStateEnum::Timeout)
            ) {
                return Err(state.operator_detail);
            }
        }
        Err(format!("{contract_id} ended without a terminal result"))
    };
    match tokio::time::timeout(Duration::from_secs(3), query).await {
        Ok(result) => result,
        Err(_) => Err(format!("{contract_id} timed out after 3 seconds")),
    }
}

fn lifecycle_name(state: i32) -> &'static str {
    match atlas_pb::LifecycleState::try_from(state) {
        Ok(atlas_pb::LifecycleState::StateRegistered) => "registered",
        Ok(atlas_pb::LifecycleState::StateInactive) => "inactive",
        Ok(atlas_pb::LifecycleState::StateActive) => "active",
        Ok(atlas_pb::LifecycleState::StateError) => "error",
        Ok(atlas_pb::LifecycleState::StateTerminated) => "terminated",
        _ => "unknown",
    }
}

/// One camera frame for the planning round about to run, or `None`.
///
/// A planner that must elect to look is not equivalent to one handed a current
/// observation. A model that never calls the camera plans without one, and a
/// model that called it some rounds ago carries whatever the history kept —
/// older views of places the robot has already left. This captures a fresh
/// frame per round so the observation in the prompt is the view it has now.
///
/// Off unless `ROBONIX_PILOT_AUTO_CAMERA_OBSERVATION` is set: a body whose
/// planner does not need per-round vision should not pay for an image on every
/// request. `ROBONIX_PILOT_OBSERVATION_CAMERA_PROVIDER` picks the camera when
/// several are registered; without it the first registered camera is used.
/// Every failure path returns `None` — a missing observation degrades the round
/// to what it was before, and must never end the turn.
pub async fn collect_visual_observation(
    executor: &ExecutorConn,
    caps: &[(String, atlas_pb::Capability)],
) -> Option<Message> {
    if !env_flag("ROBONIX_PILOT_AUTO_CAMERA_OBSERVATION") {
        return None;
    }
    let preferred = std::env::var("ROBONIX_PILOT_OBSERVATION_CAMERA_PROVIDER").ok();
    let provider_id = provider_for(caps, CAMERA_SNAPSHOT, preferred.as_deref())?;
    let value = query_capability(
        executor.graph.clone(),
        provider_id,
        CAMERA_SNAPSHOT,
        "camera_observation",
        "Capture the current camera observation before planning",
    )
    .await
    .ok()?;
    camera_observation(&value)
}

/// The provider to read a contract from: the preferred one when it offers the
/// contract, otherwise any registered provider of it. A named preference that
/// does not offer the contract resolves to nothing rather than silently
/// reading a different camera than the caller asked for.
fn provider_for(
    caps: &[(String, atlas_pb::Capability)],
    contract_id: &str,
    preferred_provider: Option<&str>,
) -> Option<String> {
    let matches_contract =
        |capability: &atlas_pb::Capability| capability.contract_id == contract_id;
    if let Some(preferred) = preferred_provider {
        return caps
            .iter()
            .find(|(provider_id, capability)| {
                provider_id == preferred && matches_contract(capability)
            })
            .map(|(provider_id, _)| provider_id.clone());
    }
    caps.iter()
        .find(|(_, capability)| matches_contract(capability))
        .map(|(provider_id, _)| provider_id.clone())
}

/// Turn a camera snapshot payload into a user message carrying the image.
///
/// Accepts both the `image_base64` shape and `sensor_msgs/Image`'s `data`.
/// `encoding = "error"` is the primitive's placeholder for "no frame", and an
/// empty payload is the same thing said differently; neither is an observation.
fn camera_observation(value: &Value) -> Option<Message> {
    let encoding = value
        .get("encoding")
        .and_then(Value::as_str)
        .unwrap_or("jpeg");
    if encoding.eq_ignore_ascii_case("error") {
        return None;
    }
    let image = value
        .get("image_base64")
        .or_else(|| value.get("data"))
        .and_then(Value::as_str)
        .filter(|image| !image.is_empty())?;
    Some(Message::user_with_image(
        "Current camera observation, captured immediately before this planning \
         round. This is the view the robot has now; earlier images in the \
         history are older views from other places.",
        image.to_string(),
    ))
}

fn env_flag(name: &str) -> bool {
    std::env::var(name).ok().is_some_and(|value| {
        matches!(
            value.trim().to_ascii_lowercase().as_str(),
            "1" | "true" | "yes" | "on"
        )
    })
}

#[cfg(test)]
mod tests {
    use super::lifecycle_name;
    use robonix_atlas::pb::LifecycleState;

    #[test]
    fn lifecycle_labels_are_stable() {
        assert_eq!(lifecycle_name(LifecycleState::StateActive as i32), "active");
        assert_eq!(lifecycle_name(LifecycleState::StateError as i32), "error");
    }
}
