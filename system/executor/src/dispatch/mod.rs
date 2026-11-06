// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
//! Route capabilities to in-process builtins or client-owned MCP execution.

pub mod async_poll;
pub mod builtin;
pub mod grpc;

use std::collections::HashSet;
use std::sync::Mutex;
use std::time::Duration;

use anyhow::{Context, Result};
use tonic::Request;
use tonic::transport::Endpoint;

use crate::pb::lifecycle::{DriverRequest, DriverResponse};
use crate::pb::pilot::{CapabilityCall, CapabilityCallResult};
use crate::plan_runtime::PlanRuntime;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::{debug, info};

const CMD_ACTIVATE: u32 = 1;
const DRIVER_ACTIVATE_TIMEOUT: Duration = Duration::from_secs(60);
const DEPLOY_CONSUMER_ID: &str = "com.robonix.executor.skill_activate";

static ACTIVATED: Mutex<Option<HashSet<String>>> = Mutex::new(None);

fn mark_activated(provider_id: &str) -> bool {
    let mut g = ACTIVATED.lock().expect("ACTIVATED poisoned");
    let set = g.get_or_insert_with(HashSet::new);
    set.insert(provider_id.to_string())
}

fn already_activated(provider_id: &str) -> bool {
    let g = ACTIVATED.lock().expect("ACTIVATED poisoned");
    g.as_ref().is_some_and(|s| s.contains(provider_id))
}

fn is_skill_namespace(ns: &str) -> bool {
    let mut parts = ns.split('/').filter(|p| !p.is_empty());
    let first = parts.next();
    let second = parts.next();
    matches!(first, Some("robonix")) && matches!(second, Some("skill"))
        || matches!(first, Some("skill"))
}

/// Dispatch a single CapabilityCall and return its result.
///
/// `self_provider_id` is the executor's own provider_id (used to short-circuit
/// builtins that target this process). `atlas` is used to ConnectCapability
/// for any external provider call; the channel is released as soon as the call
/// finishes.
pub async fn dispatch(
    call: &CapabilityCall,
    self_provider_id: &str,
    atlas: &mut AtlasClient,
    runtime: &PlanRuntime,
    plan_id: &str,
) -> CapabilityCallResult {
    dispatch_with_timeout(call, self_provider_id, atlas, runtime, plan_id, None).await
}

/// Bound verifier waiting without releasing an execution that may still be active.
pub async fn dispatch_with_timeout(
    call: &CapabilityCall,
    self_provider_id: &str,
    atlas: &mut AtlasClient,
    runtime: &PlanRuntime,
    plan_id: &str,
    timeout: Option<Duration>,
) -> CapabilityCallResult {
    if call.provider_id == self_provider_id {
        return builtin::execute(call, runtime, atlas, plan_id).await;
    }

    let channel = match connect_channel(call, self_provider_id, atlas).await {
        Ok(channel) => channel,
        Err(error) => return error_result(call, format!("{error:#}")),
    };
    let execute = async {
        let output = channel.call(serde_json::from_str(&call.args_json)?).await?;
        channel.close().await?;
        Ok::<_, anyhow::Error>(output.to_string())
    };
    let outcome = match timeout {
        Some(duration) => tokio::time::timeout(duration, execute)
            .await
            .map_err(anyhow::Error::from)
            .and_then(|r| r),
        None => execute.await,
    };
    match outcome {
        Ok(output) => CapabilityCallResult {
            call_id: call.call_id.clone(),
            provider_id: call.provider_id.clone(),
            contract_id: call.contract_id.clone(),
            success: true,
            output,
            error: String::new(),
        },
        Err(error) => {
            let cleanup = if channel.is_terminal() {
                channel.close().await.err()
            } else {
                None
            };
            error_result(
                call,
                format!(
                    "{error:#}; execution channel {}; cleanup: {cleanup:?}",
                    channel.channel_id
                ),
            )
        }
    }
}

pub async fn connect_channel(
    call: &CapabilityCall,
    consumer_id: &str,
    atlas: &mut AtlasClient,
) -> Result<robonix_atlas::client::channel::Channel> {
    anyhow::ensure!(
        serde_json::from_str::<serde_json::Value>(&call.args_json)?.is_object(),
        "MCP request must be a JSON object"
    );
    ensure_skill_active(atlas, &call.provider_id)
        .await
        .context("Driver(CMD_ACTIVATE) failed")?;
    atlas
        .connect_channel(
            consumer_id,
            &call.provider_id,
            &call.contract_id,
            atlas_pb::Transport::Mcp,
        )
        .await
}

/// If `provider_id` is a skill that hasn't been activated in this process
/// yet, resolve its `*/driver` capability and send Driver(CMD_ACTIVATE).
/// No-op for primitives, services, system providers, skills already in
/// ACTIVE (per atlas), and skills already activated in this executor
/// process (sticky cache).
async fn ensure_skill_active(atlas: &mut AtlasClient, provider_id: &str) -> Result<()> {
    if already_activated(provider_id) {
        debug!("[skill-activate] {provider_id}: already activated, skipping CMD_ACTIVATE");
        return Ok(());
    }
    let providers = atlas
        .query_capabilities(provider_id, "", atlas_pb::Transport::Unspecified)
        .await
        .with_context(|| format!("query_capabilities({provider_id})"))?;
    let Some(provider) = providers.into_iter().next() else {
        info!(
            "[skill-activate] {provider_id}: not in atlas, letting connect_capability surface the error"
        );
        return Ok(());
    };
    if !is_skill_namespace(&provider.namespace) {
        debug!(
            "[skill-activate] {provider_id} (ns={}): not a skill, no CMD_ACTIVATE",
            provider.namespace
        );
        return Ok(());
    }
    if provider.state == atlas_pb::LifecycleState::StateActive as i32 {
        info!("[skill-activate] {provider_id}: already ACTIVE per atlas, marking sticky");
        mark_activated(provider_id);
        return Ok(());
    }
    if provider.state != atlas_pb::LifecycleState::StateInactive as i32 {
        let state = lifecycle_state_label(provider.state);
        anyhow::bail!(
            "skill {} is {}; automatic activation is only valid from INACTIVE. The executor \
             will not repeat CMD_ACTIVATE after an activation error; recover or restart the \
             provider lifecycle first",
            provider_id,
            state
        );
    }
    info!(
        "[skill-activate] {provider_id} (ns={}, state={}): sending Driver(CMD_ACTIVATE)",
        provider.namespace, provider.state
    );
    let driver_contract = provider
        .capabilities
        .iter()
        .find(|c| c.contract_id.ends_with("/driver"))
        .map(|c| c.contract_id.clone())
        .ok_or_else(|| anyhow::anyhow!("skill {provider_id} has no */driver capability"))?;
    let svc_name = contract_id_to_service_name(&driver_contract);
    let (channel_id, endpoint, _) = atlas
        .connect_capability(
            DEPLOY_CONSUMER_ID,
            provider_id,
            &driver_contract,
            atlas_pb::Transport::Grpc,
        )
        .await
        .with_context(|| format!("ConnectCapability({driver_contract})"))?;
    let normalized = if endpoint.starts_with("http") {
        endpoint
    } else {
        format!("http://{endpoint}")
    };
    let result = async {
        let channel = Endpoint::new(normalized.clone())
            .with_context(|| format!("invalid driver endpoint '{normalized}'"))?
            .connect()
            .await
            .with_context(|| format!("dial driver at '{normalized}'"))?;
        let path: tonic::codegen::http::uri::PathAndQuery =
            format!("/robonix.contracts.{svc_name}/Driver")
                .parse()
                .with_context(|| format!("build gRPC path for '{driver_contract}'"))?;
        let mut grpc = tonic::client::Grpc::new(channel);
        grpc.ready().await.with_context(|| "gRPC ready")?;
        let codec: tonic_prost::ProstCodec<DriverRequest, DriverResponse> = Default::default();
        let resp = tokio::time::timeout(
            DRIVER_ACTIVATE_TIMEOUT,
            grpc.unary(
                Request::new(DriverRequest {
                    command: CMD_ACTIVATE,
                    config_json: String::new(),
                }),
                path,
                codec,
            ),
        )
        .await
        .map_err(|_| {
            anyhow::anyhow!("Driver(CMD_ACTIVATE) timed out after {DRIVER_ACTIVATE_TIMEOUT:?}")
        })?
        .with_context(|| "Driver(CMD_ACTIVATE) RPC failed")?;
        Ok::<_, anyhow::Error>(resp.into_inner())
    }
    .await;
    let _ = atlas.disconnect_capability(&channel_id).await;
    let r = result?;
    if !r.ok {
        anyhow::bail!(
            "Driver(CMD_ACTIVATE) returned ok=false (state={}, error={})",
            r.state,
            r.error
        );
    }
    mark_activated(provider_id);
    Ok(())
}

fn lifecycle_state_label(state: i32) -> &'static str {
    if state == atlas_pb::LifecycleState::StateRegistered as i32 {
        "REGISTERED"
    } else if state == atlas_pb::LifecycleState::StateInactive as i32 {
        "INACTIVE"
    } else if state == atlas_pb::LifecycleState::StateActive as i32 {
        "ACTIVE"
    } else if state == atlas_pb::LifecycleState::StateError as i32 {
        "ERROR"
    } else if state == atlas_pb::LifecycleState::StateTerminated as i32 {
        "TERMINATED"
    } else {
        "UNSPECIFIED"
    }
}

/// Mirrors robonix_codegen::contract_gen::contract_id_to_service_name.
/// `robonix/skill/explore/driver` → `RobonixSkillExploreDriver`. Uniform
/// PascalCase per `/`-segment, no prefix stripping.
fn contract_id_to_service_name(id: &str) -> String {
    id.split('/')
        .filter(|x| !x.is_empty())
        .map(|seg| {
            seg.split('_')
                .filter(|p| !p.is_empty())
                .map(|p| {
                    let mut c = p.chars();
                    match c.next() {
                        Some(f) => f
                            .to_uppercase()
                            .chain(c.flat_map(char::to_lowercase))
                            .collect::<String>(),
                        None => String::new(),
                    }
                })
                .collect::<String>()
        })
        .collect()
}

pub(crate) fn error_result(call: &CapabilityCall, msg: String) -> CapabilityCallResult {
    CapabilityCallResult {
        call_id: call.call_id.clone(),
        provider_id: call.provider_id.clone(),
        contract_id: call.contract_id.clone(),
        success: false,
        output: String::new(),
        error: msg,
    }
}

#[cfg(test)]
mod tests {
    use super::lifecycle_state_label;
    use robonix_atlas::pb::LifecycleState;

    #[test]
    fn lifecycle_state_labels_are_actionable() {
        assert_eq!(
            lifecycle_state_label(LifecycleState::StateInactive as i32),
            "INACTIVE"
        );
        assert_eq!(
            lifecycle_state_label(LifecycleState::StateError as i32),
            "ERROR"
        );
        assert_eq!(
            lifecycle_state_label(LifecycleState::StateTerminated as i32),
            "TERMINATED"
        );
    }
}
