// SPDX-License-Identifier: MulanPSL-2.0

//! Robot-local Sentinel policy storage, admin management, and dispatch checks.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use chrono::{Datelike, Local, Timelike};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_sentinel::{EvaluationContext, Rule, Sentinel};
use tokio::sync::RwLock;
use tonic::{Request, Response, Status};

use crate::pb::contracts::robonix_system_keystone_get_profile_client::RobonixSystemKeystoneGetProfileClient;
use crate::pb::contracts::robonix_system_scene_get_robot_context_client::RobonixSystemSceneGetRobotContextClient;
use crate::pb::contracts::robonix_system_sentinel_list_rules_server::RobonixSystemSentinelListRules;
use crate::pb::contracts::robonix_system_sentinel_replace_rules_server::RobonixSystemSentinelReplaceRules;
use crate::pb::contracts::robonix_system_soma_get_health_client::RobonixSystemSomaGetHealthClient;
use crate::pb::contracts::robonix_system_vitals_get_client::RobonixSystemVitalsGetClient;
use crate::pb::keystone::GetProfileRequest;
use crate::pb::semantic_map::GetRobotContextRequest;
use crate::pb::sentinel::{
    ListRulesRequest, ListRulesResponse, ReplaceRulesRequest, ReplaceRulesResponse,
};
use crate::pb::soma::{GetHealthRequest, Scalar, SomaHealthSnapshot};
use crate::pb::vitals::{GetVitalsRequest, VitalsSnapshot};
use crate::service::ExecutorServiceImpl;

const KEYSTONE_GET_PROFILE: &str = "robonix/system/keystone/get_profile";
const SOMA_GET_HEALTH: &str = "robonix/system/soma/get_health";
const VITALS_GET: &str = "robonix/system/vitals/get";
const SCENE_GET_ROBOT_CONTEXT: &str = "robonix/system/scene/get_robot_context";
const ATLAS_CONTROL_TIMEOUT: Duration = Duration::from_millis(250);
const SNAPSHOT_TIMEOUT: Duration = Duration::from_millis(750);
const ADMIN_RPC_TIMEOUT: Duration = Duration::from_secs(2);
pub const SENTINEL_PROVIDER_ID: &str = "sentinel";
pub const SENTINEL_NAMESPACE: &str = "robonix/system/sentinel";

/// Canonical identity resolved directly from Keystone by Executor.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SentinelIdentity {
    pub known: bool,
    pub user_id: String,
    pub roles: Vec<String>,
}

/// Shared policy state. Replacements are validated and persisted before publish.
#[derive(Clone)]
pub struct SentinelRuntime {
    path: Arc<PathBuf>,
    policy: Arc<RwLock<Sentinel>>,
}

impl SentinelRuntime {
    /// Load the robot-local rule file, or start with an empty allow-by-default policy.
    pub fn load(path: PathBuf) -> Result<Self> {
        let rules = if path.exists() {
            let raw = std::fs::read_to_string(&path)
                .with_context(|| format!("read Sentinel rules '{}'", path.display()))?;
            serde_json::from_str::<Vec<Rule>>(&raw)
                .with_context(|| format!("parse Sentinel rules '{}'", path.display()))?
        } else {
            Vec::new()
        };
        let policy = Sentinel::new(rules).context("validate Sentinel rules")?;
        Ok(Self {
            path: Arc::new(path),
            policy: Arc::new(RwLock::new(policy)),
        })
    }

    /// Serialize the active rule set for the admin UI.
    pub async fn rules_json(&self) -> Result<String> {
        serde_json::to_string_pretty(self.policy.read().await.list_rules())
            .context("serialize Sentinel rules")
    }

    /// Validate and persist a complete replacement before making it active.
    pub async fn replace_rules(&self, raw: &str) -> Result<String> {
        let rules = serde_json::from_str::<Vec<Rule>>(raw).context("parse rules_json")?;
        let replacement = Sentinel::new(rules).context("validate rules_json")?;
        let normalized = serde_json::to_string_pretty(replacement.list_rules())
            .context("serialize rules_json")?;
        persist_atomically(&self.path, &normalized).await?;
        *self.policy.write().await = replacement;
        Ok(normalized)
    }

    /// Evaluate one call using canonical Keystone identity and fresh system snapshots.
    pub async fn check(
        &self,
        atlas: &AtlasClient,
        consumer_id: &str,
        contract_id: &str,
        args_json: &str,
        identity: &SentinelIdentity,
    ) -> robonix_sentinel::Decision {
        let args = match parse_capability_args(args_json) {
            Ok(args) => args,
            Err(decision) => return decision,
        };
        let policy = self.policy.read().await.clone();
        let state =
            collect_trusted_state(atlas, consumer_id, &policy.required_context_roots()).await;
        let now = Local::now();
        let context = EvaluationContext {
            identity_known: identity.known,
            user_id: (!identity.user_id.is_empty()).then(|| identity.user_id.clone()),
            roles: identity.roles.clone(),
            state,
            weekday: now.weekday().number_from_monday() as u8,
            minute_of_day: (now.hour() * 60 + now.minute()) as u16,
        };
        policy.check(contract_id, &args, &context)
    }
}

fn parse_capability_args(args_json: &str) -> Result<serde_json::Value, robonix_sentinel::Decision> {
    serde_json::from_str(args_json).map_err(|error| robonix_sentinel::Decision {
        allow: false,
        indeterminate: true,
        rule_id: "invalid_args_json".to_owned(),
        reason: format!("capability args_json is not valid JSON: {error}"),
    })
}

/// Write to a sibling temporary file and rename it over the active policy.
async fn persist_atomically(path: &Path, contents: &str) -> Result<()> {
    if let Some(parent) = path.parent() {
        tokio::fs::create_dir_all(parent)
            .await
            .with_context(|| format!("create Sentinel data directory '{}'", parent.display()))?;
    }
    let temporary = path.with_extension("json.tmp");
    tokio::fs::write(&temporary, contents)
        .await
        .with_context(|| format!("write temporary Sentinel rules '{}'", temporary.display()))?;
    tokio::fs::rename(&temporary, path)
        .await
        .with_context(|| format!("publish Sentinel rules '{}'", path.display()))?;
    Ok(())
}

async fn collect_trusted_state(
    atlas: &AtlasClient,
    consumer_id: &str,
    roots: &BTreeSet<String>,
) -> serde_json::Value {
    let (soma, vitals, scene) = tokio::join!(
        collect_optional_root(roots, "soma", fetch_soma(atlas.clone(), consumer_id)),
        collect_optional_root(roots, "vitals", fetch_vitals(atlas.clone(), consumer_id)),
        collect_optional_root(roots, "scene", fetch_scene(atlas.clone(), consumer_id)),
    );
    let mut state = serde_json::Map::new();
    for (name, value) in [("soma", soma), ("vitals", vitals), ("scene", scene)] {
        if let Some(value) = value {
            state.insert(name.to_owned(), value);
        }
    }
    serde_json::Value::Object(state)
}

async fn collect_optional_root<F>(
    roots: &BTreeSet<String>,
    name: &str,
    fetch: F,
) -> Option<serde_json::Value>
where
    F: std::future::Future<Output = Result<serde_json::Value>>,
{
    if !roots.contains(name) {
        return None;
    }
    Some(match fetch.await {
        Ok(value) => value,
        Err(error) => serde_json::json!({
            "available": false,
            "error": format!("{error:#}"),
        }),
    })
}

async fn fetch_soma(mut atlas: AtlasClient, consumer_id: &str) -> Result<serde_json::Value> {
    let (channel_id, _, channel) = tokio::time::timeout(
        ATLAS_CONTROL_TIMEOUT,
        atlas_client::connect_to_capability(&mut atlas, consumer_id, SOMA_GET_HEALTH),
    )
    .await
    .context("connect to Soma get_health timed out")?
    .context("connect to Soma get_health")?;
    let result = match tokio::time::timeout(SNAPSHOT_TIMEOUT, async {
        let response = RobonixSystemSomaGetHealthClient::new(channel)
            .get_health(GetHealthRequest {})
            .await
            .context("call Soma get_health")?
            .into_inner();
        let snapshot = response
            .snapshot
            .context("Soma has not published a health snapshot")?;
        Ok(soma_json(snapshot))
    })
    .await
    {
        Ok(result) => result,
        Err(_) => Err(anyhow::anyhow!(
            "Soma snapshot timed out after {} ms",
            SNAPSHOT_TIMEOUT.as_millis()
        )),
    };
    disconnect_bounded(&mut atlas, &channel_id).await;
    result
}

fn soma_json(snapshot: SomaHealthSnapshot) -> serde_json::Value {
    let fresh = snapshot_is_fresh(snapshot.soma_ts_ns, snapshot.ttl_ms);
    let metadata = serde_json::json!({
        "available": true,
        "fresh": fresh,
        "source_ts_ns": snapshot.source_ts_ns,
        "soma_ts_ns": snapshot.soma_ts_ns,
        "ttl_ms": snapshot.ttl_ms,
    });
    if !fresh {
        return metadata;
    }

    let components = snapshot
        .components
        .into_iter()
        .map(|component| {
            let id = component.id.clone();
            (
                id,
                serde_json::json!({
                    "parent_id": component.parent_id,
                    "kind": component.kind,
                    "name": component.name,
                    "frame_id": component.frame_id,
                    "model": component.model,
                    "health": component.health,
                    "operational_state": component.operational_state,
                    "present": component.present,
                    "online": component.online,
                    "detail": component.detail,
                }),
            )
        })
        .collect::<BTreeMap<_, _>>();
    let actuators = snapshot
        .actuators
        .into_iter()
        .map(|actuator| {
            let id = actuator.component_id.clone();
            (
                id,
                serde_json::json!({
                    "joint_name": actuator.joint_name,
                    "position": actuator.position.map(scalar_json),
                    "velocity": actuator.velocity.map(scalar_json),
                    "effort": actuator.effort.map(scalar_json),
                    "current": actuator.current.map(scalar_json),
                    "voltage": actuator.voltage.map(scalar_json),
                    "motor_temp": actuator.motor_temp.map(scalar_json),
                    "driver_temp": actuator.driver_temp.map(scalar_json),
                    "torque_enabled": actuator.torque_enabled,
                    "brake_engaged": actuator.brake_engaged,
                    "communication_ok": actuator.communication_ok,
                    "vendor_mode": actuator.vendor_mode,
                    "vendor_error_code": actuator.vendor_error_code,
                    "status_flags": actuator.status_flags,
                }),
            )
        })
        .collect::<BTreeMap<_, _>>();
    let mut metrics = BTreeMap::<String, BTreeMap<String, serde_json::Value>>::new();
    for metric in snapshot.metrics {
        let value = metric.value.map(scalar_json);
        metrics.entry(metric.component_id).or_default().insert(
            metric.name,
            serde_json::json!({"value": value, "source_key": metric.source_key}),
        );
    }
    let safety = snapshot.safety.map(|safety| {
        serde_json::json!({
            "motion_allowed": safety.motion_allowed,
            "motor_power_allowed": safety.motor_power_allowed,
            "aggregate_state": safety.aggregate_state,
            "detail": safety.detail,
        })
    });
    let mut output = metadata.as_object().cloned().unwrap_or_default();
    output.insert(
        "data".to_owned(),
        serde_json::json!({
            "schema_version": snapshot.schema_version,
            "body_id": snapshot.body_id,
            "seq": snapshot.seq,
            "components": components,
            "actuators": actuators,
            "metrics": metrics,
            "safety": safety,
        }),
    );
    serde_json::Value::Object(output)
}

fn scalar_json(value: Scalar) -> serde_json::Value {
    serde_json::json!({
        "value": value.value,
        "unit": value.unit,
        "quality": value.quality,
    })
}

fn snapshot_is_fresh(timestamp_ns: i64, ttl_ms: u32) -> bool {
    if timestamp_ns <= 0 || ttl_ms == 0 {
        return false;
    }
    let Ok(now) = SystemTime::now().duration_since(UNIX_EPOCH) else {
        return false;
    };
    let now_ns = i128::try_from(now.as_nanos()).unwrap_or(i128::MAX);
    let timestamp_ns = i128::from(timestamp_ns);
    let age_ns = now_ns.saturating_sub(timestamp_ns);
    age_ns >= 0 && age_ns <= i128::from(ttl_ms) * 1_000_000
}

async fn fetch_vitals(mut atlas: AtlasClient, consumer_id: &str) -> Result<serde_json::Value> {
    let (channel_id, _, channel) = tokio::time::timeout(
        ATLAS_CONTROL_TIMEOUT,
        atlas_client::connect_to_capability(&mut atlas, consumer_id, VITALS_GET),
    )
    .await
    .context("connect to Vitals get timed out")?
    .context("connect to Vitals get")?;
    let result = match tokio::time::timeout(SNAPSHOT_TIMEOUT, async {
        let response = RobonixSystemVitalsGetClient::new(channel)
            .get_vitals(GetVitalsRequest {})
            .await
            .context("call Vitals get")?
            .into_inner();
        let snapshot = response.snapshot.context("Vitals returned no snapshot")?;
        Ok(vitals_json(snapshot))
    })
    .await
    {
        Ok(result) => result,
        Err(_) => Err(anyhow::anyhow!(
            "Vitals snapshot timed out after {} ms",
            SNAPSHOT_TIMEOUT.as_millis()
        )),
    };
    disconnect_bounded(&mut atlas, &channel_id).await;
    result
}

fn vitals_json(snapshot: VitalsSnapshot) -> serde_json::Value {
    let power = snapshot.power.map(|power| {
        serde_json::json!({
            "battery_percent": power.battery_percent,
            "voltage": power.voltage,
            "charging": power.charging,
            "remaining_s": power.remaining_s,
        })
    });
    let components = snapshot
        .components
        .into_iter()
        .map(|component| {
            let name = component.name.clone();
            (
                name,
                serde_json::json!({
                    "health": component.health,
                    "detail": component.detail,
                    "value": component.value,
                    "threshold": component.threshold,
                }),
            )
        })
        .collect::<BTreeMap<_, _>>();
    let bodies = snapshot
        .bodies
        .into_iter()
        .map(|body| {
            let key = body.body_type.clone();
            let body_components = body
                .components
                .into_iter()
                .map(|component| {
                    let component_key = if component.id.is_empty() {
                        component.name.clone()
                    } else {
                        component.id.clone()
                    };
                    (
                        component_key,
                        serde_json::json!({
                            "name": component.name,
                            "kind": component.kind,
                            "temperature": component.temperature,
                            "error_code": component.error_code,
                            "enabled": component.enabled,
                            "parent_id": component.parent_id,
                            "model": component.model,
                        }),
                    )
                })
                .collect::<BTreeMap<_, _>>();
            (
                key,
                serde_json::json!({
                    "model": body.model,
                    "state": body.state,
                    "message": body.message,
                    "components": body_components,
                }),
            )
        })
        .collect::<BTreeMap<_, _>>();
    serde_json::json!({
        "available": true,
        "data": {
            "ts_ns": snapshot.ts_ns,
            "power": power,
            "components": components,
            "bodies": bodies,
        }
    })
}

async fn fetch_scene(mut atlas: AtlasClient, consumer_id: &str) -> Result<serde_json::Value> {
    let (channel_id, _, channel) = tokio::time::timeout(
        ATLAS_CONTROL_TIMEOUT,
        atlas_client::connect_to_capability(&mut atlas, consumer_id, SCENE_GET_ROBOT_CONTEXT),
    )
    .await
    .context("connect to Scene get_robot_context timed out")?
    .context("connect to Scene get_robot_context")?;
    let result = match tokio::time::timeout(SNAPSHOT_TIMEOUT, async {
        let response = RobonixSystemSceneGetRobotContextClient::new(channel)
            .get_robot_context(GetRobotContextRequest {})
            .await
            .context("call Scene get_robot_context")?
            .into_inner();
        let fresh = response.pose_known && !response.stale;
        let mut output = serde_json::json!({
            "available": true,
            "fresh": fresh,
            "reason": response.reason,
            "observed_at_unix": response.observed_at_unix,
            "snapshot_at_unix": response.snapshot_at_unix,
        });
        if fresh {
            output.as_object_mut().unwrap().insert("data".to_owned(), serde_json::json!({
                "map_id": response.map_id,
                "pose": {"x": response.x, "y": response.y, "z": response.z, "yaw": response.yaw},
                "room_id": response.room_id,
                "room_name": response.room_name,
                "containing_area_ids": response.containing_area_ids,
                "containing_area_names": response.containing_area_names,
            }));
        }
        Ok(output)
    })
    .await
    {
        Ok(result) => result,
        Err(_) => Err(anyhow::anyhow!(
            "Scene snapshot timed out after {} ms",
            SNAPSHOT_TIMEOUT.as_millis()
        )),
    };
    disconnect_bounded(&mut atlas, &channel_id).await;
    result
}

async fn disconnect_bounded(atlas: &mut AtlasClient, channel_id: &str) {
    let _ = tokio::time::timeout(
        ATLAS_CONTROL_TIMEOUT,
        atlas.disconnect_capability(channel_id),
    )
    .await;
}

/// Resolve one opaque Keystone session into canonical identity. Caller-provided
/// user IDs or roles are never inputs to this operation.
pub(crate) async fn resolve_session_identity(
    atlas: &AtlasClient,
    provider_id: &str,
    token: &str,
) -> Result<SentinelIdentity, Status> {
    if token.is_empty() {
        return Err(Status::unauthenticated("login session is required"));
    }
    let mut atlas = atlas.clone();
    let (channel_id, _, channel) = tokio::time::timeout(
        ATLAS_CONTROL_TIMEOUT,
        atlas_client::connect_to_capability(&mut atlas, provider_id, KEYSTONE_GET_PROFILE),
    )
    .await
    .map_err(|_| Status::deadline_exceeded("resolve Keystone timed out"))?
    .map_err(|error| Status::unavailable(format!("resolve Keystone: {error:#}")))?;
    let response = match tokio::time::timeout(
        ADMIN_RPC_TIMEOUT,
        RobonixSystemKeystoneGetProfileClient::new(channel).get_profile(GetProfileRequest {
            session_token: token.to_owned(),
        }),
    )
    .await
    {
        Ok(result) => result.map(|response| response.into_inner()),
        Err(_) => Err(Status::deadline_exceeded(
            "Keystone profile lookup timed out",
        )),
    };
    disconnect_bounded(&mut atlas, &channel_id).await;
    let response = response?;
    let user = response
        .user
        .ok_or_else(|| Status::internal("Keystone returned an empty profile"))?;
    Ok(SentinelIdentity {
        known: true,
        user_id: user.user_id,
        roles: user.roles,
    })
}

/// Resolve the Keystone session on every admin request; UI role checks are not trusted.
async fn require_admin(atlas: &AtlasClient, provider_id: &str, token: &str) -> Result<(), Status> {
    let identity = resolve_session_identity(atlas, provider_id, token).await?;
    if !identity.roles.iter().any(|role| role == "admin") {
        return Err(Status::permission_denied("administrator role is required"));
    }
    Ok(())
}

#[tonic::async_trait]
impl RobonixSystemSentinelListRules for ExecutorServiceImpl {
    async fn list_rules(
        &self,
        request: Request<ListRulesRequest>,
    ) -> Result<Response<ListRulesResponse>, Status> {
        let input = request.into_inner();
        require_admin(&self.atlas, SENTINEL_PROVIDER_ID, &input.session_token).await?;
        let rules_json = self
            .sentinel
            .rules_json()
            .await
            .map_err(|error| Status::internal(format!("read Sentinel rules: {error:#}")))?;
        Ok(Response::new(ListRulesResponse { rules_json }))
    }
}

#[tonic::async_trait]
impl RobonixSystemSentinelReplaceRules for ExecutorServiceImpl {
    async fn replace_rules(
        &self,
        request: Request<ReplaceRulesRequest>,
    ) -> Result<Response<ReplaceRulesResponse>, Status> {
        let input = request.into_inner();
        require_admin(&self.atlas, SENTINEL_PROVIDER_ID, &input.session_token).await?;
        let rules_json = self
            .sentinel
            .replace_rules(&input.rules_json)
            .await
            .map_err(|error| {
                Status::invalid_argument(format!("invalid Sentinel rules: {error:#}"))
            })?;
        Ok(Response::new(ReplaceRulesResponse { rules_json }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn malformed_capability_args_fail_closed_instead_of_becoming_empty_object() {
        let decision = parse_capability_args("{not-json").unwrap_err();
        assert!(!decision.allow);
        assert!(decision.indeterminate);
        assert_eq!(decision.rule_id, "invalid_args_json");
        assert!(decision.reason.contains("not valid JSON"));
    }

    #[test]
    fn fresh_soma_snapshot_preserves_boolean_false_as_typed_state() {
        let now_ns = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as i64;
        let value = soma_json(SomaHealthSnapshot {
            source_ts_ns: now_ns,
            soma_ts_ns: now_ns,
            ttl_ms: 2_000,
            safety: Some(crate::pb::soma::SafetyState {
                motion_allowed: false,
                ..Default::default()
            }),
            ..Default::default()
        });

        assert_eq!(
            value.pointer("/fresh").and_then(serde_json::Value::as_bool),
            Some(true)
        );
        assert_eq!(
            value
                .pointer("/data/safety/motion_allowed")
                .and_then(serde_json::Value::as_bool),
            Some(false)
        );
    }

    #[test]
    fn stale_soma_snapshot_does_not_publish_policy_data() {
        let value = soma_json(SomaHealthSnapshot {
            source_ts_ns: 1,
            soma_ts_ns: 1,
            ttl_ms: 1,
            ..Default::default()
        });

        assert_eq!(
            value.pointer("/fresh").and_then(serde_json::Value::as_bool),
            Some(false)
        );
        assert!(value.pointer("/data").is_none());
    }
}
