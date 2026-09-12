// SPDX-License-Identifier: MulanPSL-2.0
//
// Pilot-side native Soma awareness.
//
// Soma exposes robot body data as gRPC contracts. Pilot fetches the
// default robot's Soma YAML once at startup, then injects it into every
// turn's system prompt so the model knows its body without the user first
// calling a bridge tool.
//
// Deliberately YAML only. The URDF is a full kinematic XML tree whose link
// and joint geometry the planner never reasons over, so injecting it only
// spent context and gave the model a second, lower-level body description to
// contradict soma.yaml with. Soma still serves get_urdf for consumers that
// need the kinematics; it just does not belong in a prompt.

use crate::pb::contracts::{
    robonix_system_soma_get_health_client::RobonixSystemSomaGetHealthClient,
    robonix_system_soma_get_yaml_client::RobonixSystemSomaGetYamlClient,
};
use crate::pb::soma::{GetHealthRequest, GetYamlRequest};
use anyhow::{Context, Result};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_scribe::warn;

const GET_YAML_CONTRACT: &str = "robonix/system/soma/get_yaml";
const GET_HEALTH_CONTRACT: &str = "robonix/system/soma/get_health";

pub async fn fetch_runtime_prompt_block(atlas: &mut AtlasClient, consumer_id: &str) -> String {
    match fetch_health(atlas, consumer_id).await {
        Ok(value) => format!(
            "\n\n## Current embodiment state (from Soma)\n\
             This snapshot was refreshed immediately before planning. Fresh fields are \
             authoritative. Stale or missing fields mean unknown; never reconstruct them \
             from conversation history. `likely_holding` means the calibrated gripper is \
             not fully open; it does not identify the object.\n\n{}\n",
            serde_json::to_string(&value).unwrap_or_else(|_| "{}".into())
        ),
        Err(error) => format!(
            "\n\n## Current embodiment state (from Soma)\n\
             {{\"available\":false,\"error\":{}}}\n",
            serde_json::to_string(&error.to_string()).unwrap_or_else(|_| "\"unknown\"".into())
        ),
    }
}

async fn fetch_health(atlas: &mut AtlasClient, consumer_id: &str) -> Result<serde_json::Value> {
    let (channel_id, _provider_id, channel) =
        atlas_client::connect_to_capability(atlas, consumer_id, GET_HEALTH_CONTRACT)
            .await
            .context("connect to Soma get_health")?;
    let result = async {
        let response = RobonixSystemSomaGetHealthClient::new(channel)
            .get_health(GetHealthRequest {})
            .await
            .context("call Soma get_health")?
            .into_inner();
        let snapshot = response
            .snapshot
            .context("Soma has not published a health snapshot yet")?;
        let components: Vec<_> = snapshot
            .components
            .into_iter()
            .map(|component| {
                drop_absent(serde_json::json!({
                    "id": component.id,
                    "parent_id": component.parent_id,
                    "kind": component.kind,
                    "health": component.health,
                    "operational_state": component.operational_state,
                    "online": component.online,
                    "detail": component.detail,
                }))
            })
            .collect();
        let actuators: Vec<_> = snapshot
            .actuators
            .into_iter()
            .map(|actuator| {
                drop_absent(serde_json::json!({
                    "component_id": actuator.component_id,
                    "joint_name": actuator.joint_name,
                    "position": actuator.position.map(|value| serde_json::json!({
                        "value": value.value, "unit": value.unit, "quality": value.quality,
                    })),
                    "communication_ok": actuator.communication_ok,
                }))
            })
            .collect();
        let metrics: Vec<_> = snapshot
            .metrics
            .into_iter()
            .map(|metric| {
                drop_absent(serde_json::json!({
                    "component_id": metric.component_id,
                    "name": metric.name,
                    "value": metric.value.map(|value| serde_json::json!({
                        "value": value.value, "unit": value.unit, "quality": value.quality,
                    })),
                }))
            })
            .collect();
        Ok::<_, anyhow::Error>(drop_absent(serde_json::json!({
            "available": true,
            "body_id": snapshot.body_id,
            "seq": snapshot.seq,
            "source_ts_ns": snapshot.source_ts_ns,
            "ttl_ms": snapshot.ttl_ms,
            "components": components,
            "actuators": actuators,
            "metrics": metrics,
            "safety": snapshot.safety.map(|safety| drop_absent(serde_json::json!({
                "motion_allowed": safety.motion_allowed,
                "motor_power_allowed": safety.motor_power_allowed,
                "aggregate_state": safety.aggregate_state,
                "detail": safety.detail,
            }))),
        })))
    }
    .await;
    let _ = atlas.disconnect_capability(&channel_id).await;
    result
}

pub async fn fetch_system_prompt_block(
    atlas: &mut AtlasClient,
    consumer_id: &str,
) -> Result<Option<String>> {
    let yaml = match fetch_yaml(atlas, consumer_id).await {
        Ok(text) => text,
        Err(e) => {
            warn!("[pilot/soma] get_yaml unavailable; continuing without Soma context: {e:#}");
            return Ok(None);
        }
    };
    let mut block = String::from(
        "\n\n## Robot Body Context (from Soma)\n\n\
         This is the robot's self-description, loaded automatically at Pilot startup. \
         Treat it as authoritative HARD CONSTRAINTS for the robot's body, sensors, \
         frames, limits, and deployment-specific notes. Do not ask the user to call \
         Soma manually unless this context is absent or stale.\n\n\
         ### Hard planning rules from Soma\n\n\
         - Sensor placement and modality in `soma.yaml` are binding. Do not invent \
         sensors, viewpoints, arms, grippers, or degrees of freedom that are not listed.\n\
         - Before planning an observation, match the user's requested viewpoint \
         (front / rear / left / right / top, etc.) against the listed sensors' \
         `placement`, `human_label`, and `cannot_do` notes.\n\
         - If the requested viewpoint is not directly available from the sensors \
         listed in Soma, say so explicitly. Do NOT call a camera with one placement \
         and describe its image as if it came from a different placement.\n\
         - If a viewpoint can be achieved only by moving the base (for example, \
         rotate 180 degrees, then use the front camera), state that plan clearly \
         and use motion + observation capabilities rather than pretending a missing \
         sensor exists.\n\n\
         ### soma.yaml (compact JSON)\n\n```json\n",
    );
    block.push_str(&compact_yaml(&yaml));
    block.push_str("\n```\n");
    Ok(Some(block))
}

/// Drop fields that carry nothing from a health snapshot before it reaches the
/// prompt.
///
/// The snapshot serializes every field of every component and actuator, so a
/// nominal body spends context on empty strings and nulls on every planning
/// call. The prompt already tells the model that a missing field means unknown,
/// which is exactly what an empty value means, so omitting them removes cost
/// without removing meaning.
fn drop_absent(value: serde_json::Value) -> serde_json::Value {
    match value {
        serde_json::Value::Object(fields) => serde_json::Value::Object(
            fields
                .into_iter()
                .map(|(key, child)| (key, drop_absent(child)))
                .filter(|(_, child)| !json_is_absent(child))
                .collect(),
        ),
        serde_json::Value::Array(items) => serde_json::Value::Array(
            items
                .into_iter()
                .map(drop_absent)
                .filter(|item| !json_is_absent(item))
                .collect(),
        ),
        other => other,
    }
}

/// Whether a snapshot field carries nothing: null, or an empty string, array,
/// or object.
fn json_is_absent(value: &serde_json::Value) -> bool {
    match value {
        serde_json::Value::Null => true,
        serde_json::Value::String(text) => text.is_empty(),
        serde_json::Value::Array(items) => items.is_empty(),
        serde_json::Value::Object(fields) => fields.is_empty(),
        _ => false,
    }
}

/// Body keys a planner cannot act on.
///
/// `urdf` names a kinematic file in the provider's own filesystem namespace:
/// the planner cannot open it, and the geometry it points at is exactly what
/// this module already declines to inject. `footprint` is a collision polygon
/// for navigation to consume, not for a planner choosing which capability to
/// call; `dimensions` survives, which is the part a planner reasons over.
const UNACTIONABLE_BODY_KEYS: [&str; 2] = ["urdf", "footprint"];

/// Serialize Soma YAML without comments or presentation whitespace, projected
/// onto the fields a planner acts on.
fn compact_yaml(raw: &str) -> String {
    match serde_yaml::from_str::<serde_yaml::Value>(raw) {
        Ok(value) => serde_json::to_string(&project_body(value, None))
            .unwrap_or_else(|_| raw.trim().to_string()),
        Err(error) => {
            warn!("[pilot/soma] could not compact soma.yaml; keeping source text: {error}");
            raw.trim().to_string()
        }
    }
}

/// Project a Soma body description onto what a planner acts on.
///
/// Three things go: keys in `UNACTIONABLE_BODY_KEYS`; the description of each
/// exported capability, which the capability catalog in the same prompt already
/// carries verbatim (the component-to-capability mapping stays, since the
/// catalog does not say which part of the body offers a capability); and absent
/// values, which cost context on every planning call and say nothing that the
/// prompt's own "missing means unknown" rule does not already say.
///
/// `parent_key` is the mapping key this value was reached under, which is how
/// an exported capability is told apart from any other mapping with a `path`.
fn project_body(value: serde_yaml::Value, parent_key: Option<&str>) -> serde_yaml::Value {
    use serde_yaml::Value;
    match value {
        Value::Mapping(mapping) => {
            let mut projected = serde_yaml::Mapping::new();
            for (key, child) in mapping {
                let name = key.as_str().unwrap_or_default().to_string();
                if UNACTIONABLE_BODY_KEYS.contains(&name.as_str()) {
                    continue;
                }
                if parent_key == Some("capabilities") && name == "description" {
                    continue;
                }
                let child = project_body(child, Some(&name));
                if is_absent(&child) {
                    continue;
                }
                projected.insert(key, child);
            }
            Value::Mapping(projected)
        }
        Value::Sequence(items) => Value::Sequence(
            items
                .into_iter()
                .map(|item| project_body(item, parent_key))
                .filter(|item| !is_absent(item))
                .collect(),
        ),
        other => other,
    }
}

/// Whether a projected value carries nothing: null, or an empty string,
/// sequence, or mapping.
fn is_absent(value: &serde_yaml::Value) -> bool {
    use serde_yaml::Value;
    match value {
        Value::Null => true,
        Value::String(text) => text.is_empty(),
        Value::Sequence(items) => items.is_empty(),
        Value::Mapping(mapping) => mapping.is_empty(),
        _ => false,
    }
}

async fn fetch_yaml(atlas: &mut AtlasClient, consumer_id: &str) -> Result<String> {
    let (channel_id, _provider_id, channel) =
        atlas_client::connect_to_capability(atlas, consumer_id, GET_YAML_CONTRACT)
            .await
            .context("connect to Soma get_yaml")?;
    let result = async {
        let mut client = RobonixSystemSomaGetYamlClient::new(channel);
        let response = client
            .get_yaml(GetYamlRequest {
                robot_id: String::new(),
            })
            .await
            .context("call Soma get_yaml")?
            .into_inner();
        Ok::<_, anyhow::Error>(response.yaml_text)
    }
    .await;
    let _ = atlas.disconnect_capability(&channel_id).await;
    result
}

#[cfg(test)]
mod tests {
    use super::{compact_yaml, drop_absent};

    #[test]
    fn representative_soma_context_is_smaller_without_dropping_body_facts() {
        let yaml = include_str!("../../../examples/webots/soma.yaml");
        let compact = compact_yaml(yaml);
        let before = yaml.len();
        let after = compact.len();
        eprintln!(
            "representative Soma prompt bytes: before={before} after={after} reduction={:.1}%",
            100.0 * (before - after) as f64 / before as f64
        );
        assert!(after < before);

        // The facts a planner reasons over survive: where a sensor sits, what
        // the body can and cannot do, and which part offers which capability.
        assert!(compact.contains("front"));
        assert!(compact.contains("can_do"));
        assert!(compact.contains("robonix/service/navigation/navigate"));
        assert!(compact.contains("chassis"));

        // A kinematic file path the planner cannot open, a collision polygon
        // navigation consumes, and capability descriptions the capability
        // catalog already carries verbatim do not.
        assert!(!compact.contains("tiago_webots.urdf"));
        assert!(!compact.contains("footprint"));
        assert!(!compact.contains("Navigate to a 2D goal in the mapped scene."));
    }

    #[test]
    fn a_nominal_health_snapshot_does_not_spend_context_on_empty_fields() {
        let snapshot = serde_json::json!({
            "available": true,
            "body_id": "robot",
            "components": [{"id": "base", "parent_id": "", "online": true, "detail": ""}],
            "metrics": [],
            "safety": serde_json::Value::Null,
        });
        let projected = drop_absent(snapshot);
        let text = serde_json::to_string(&projected).unwrap();
        assert!(text.contains("\"id\":\"base\""));
        assert!(text.contains("\"online\":true"));
        assert!(!text.contains("parent_id"));
        assert!(!text.contains("detail"));
        assert!(!text.contains("metrics"));
        assert!(!text.contains("safety"));
    }
}
