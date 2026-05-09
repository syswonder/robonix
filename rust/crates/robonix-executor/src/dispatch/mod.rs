// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// dispatch/mod.rs — route a CapabilityCall to its provider.
//
// Two paths:
//   1. cap_id == executor's own cap_id → run an in-process builtin
//      (file ops / shell). The contract_id leaf names the operation.
//   2. else → ConnectCapability(cap_id, contract_id, MCP) on atlas →
//      MCP call to the returned endpoint → DisconnectCapability.
//
// Skills sit at INACTIVE after `rbnx boot`. Right before dispatching
// to one, the executor sends Driver(CMD_ACTIVATE) on its `*/driver`
// interface to flip it to ACTIVE — that's where the skill actually
// allocates its hot resources (frontier loop / nav subscribers / VLA
// worker / …). We track which skill cap_ids we've already activated in
// this executor process; subsequent calls skip the CMD_ACTIVATE RPC to
// keep latency flat (sticky policy). A future eviction algorithm
// (EXECUTOR_EVICTION_POLICY=deactivate) will drive CMD_DEACTIVATE.
//
// The grpc dispatch helper exists for future non-MCP contracts but is not
// on the LLM-callable path today.

pub mod builtin;
pub mod grpc;
pub mod mcp;

use std::collections::HashSet;
use std::sync::Mutex;
use std::time::Duration;

use anyhow::{Context, Result};
use tonic::Request;
use tonic::transport::Endpoint;

use crate::pb::lifecycle::{DriverRequest, DriverResponse};
use crate::pb::pilot::{CapabilityCall, CapabilityCallResult};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;

const CMD_ACTIVATE: u32 = 1;
const DRIVER_ACTIVATE_TIMEOUT: Duration = Duration::from_secs(60);
const DEPLOY_CONSUMER_ID: &str = "com.robonix.executor.skill_activate";

static ACTIVATED: Mutex<Option<HashSet<String>>> = Mutex::new(None);

fn mark_activated(cap_id: &str) -> bool {
    let mut g = ACTIVATED.lock().expect("ACTIVATED poisoned");
    let set = g.get_or_insert_with(HashSet::new);
    set.insert(cap_id.to_string())
}

fn already_activated(cap_id: &str) -> bool {
    let g = ACTIVATED.lock().expect("ACTIVATED poisoned");
    g.as_ref().is_some_and(|s| s.contains(cap_id))
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
/// `self_cap_id` is the executor's own capability_id (used to short-circuit
/// builtins that target this process). `atlas` is used to ConnectCapability
/// for any external cap call; the channel is released as soon as the call
/// finishes.
pub async fn dispatch(
    call: &CapabilityCall,
    self_cap_id: &str,
    atlas: &mut AtlasClient,
) -> CapabilityCallResult {
    if call.cap_id == self_cap_id {
        return builtin::execute(call).await;
    }

    if let Err(e) = ensure_skill_runnable(atlas, &call.cap_id).await {
        return error_result(call, format!("Driver(CMD_ACTIVATE) failed: {e:#}"));
    }

    let (channel_id, endpoint, _params) = match atlas
        .connect_capability(
            self_cap_id,
            &call.cap_id,
            &call.contract_id,
            atlas_pb::Transport::Mcp,
        )
        .await
    {
        Ok(triple) => triple,
        Err(e) => {
            return error_result(call, format!("ConnectCapability failed: {e:#}"));
        }
    };

    let result = mcp::execute(call, &endpoint).await;

    let _ = atlas.disconnect_capability(&channel_id).await;
    result
}

/// If `cap_id` is a skill that hasn't been activated in this process
/// yet, resolve its `*/driver` interface and send Driver(CMD_ACTIVATE).
/// No-op for primitives, services, system caps, and skills already in
/// ACTIVE.
async fn ensure_skill_runnable(atlas: &mut AtlasClient, cap_id: &str) -> Result<()> {
    if already_activated(cap_id) {
        log::debug!("[skill-activate] {cap_id}: already activated, skipping CMD_ACTIVATE");
        return Ok(());
    }
    let recs = atlas
        .query_capabilities(cap_id, "", atlas_pb::Transport::Unspecified)
        .await
        .with_context(|| format!("query_capabilities({cap_id})"))?;
    let Some(rec) = recs.into_iter().next() else {
        log::info!(
            "[skill-activate] {cap_id}: not in atlas, letting connect_capability surface the error"
        );
        return Ok(());
    };
    if !is_skill_namespace(&rec.namespace) {
        log::debug!(
            "[skill-activate] {cap_id} (ns={}): not a skill, no CMD_ACTIVATE",
            rec.namespace
        );
        return Ok(());
    }
    if rec.state == atlas_pb::CapabilityState::StateActive as i32 {
        log::info!("[skill-activate] {cap_id}: already ACTIVE per atlas, marking sticky");
        mark_activated(cap_id);
        return Ok(());
    }
    log::info!(
        "[skill-activate] {cap_id} (ns={}, state={}): sending Driver(CMD_ACTIVATE)",
        rec.namespace,
        rec.state
    );
    let driver_contract = rec
        .interfaces
        .iter()
        .find(|i| i.contract_id.ends_with("/driver"))
        .map(|i| i.contract_id.clone())
        .ok_or_else(|| anyhow::anyhow!("skill {cap_id} has no */driver interface"))?;
    let svc_name = contract_id_to_service_name(&driver_contract);
    let (channel_id, endpoint, _) = atlas
        .connect_capability(
            DEPLOY_CONSUMER_ID,
            cap_id,
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
    mark_activated(cap_id);
    Ok(())
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
        cap_id: call.cap_id.clone(),
        contract_id: call.contract_id.clone(),
        success: false,
        output: String::new(),
        error: msg,
    }
}
