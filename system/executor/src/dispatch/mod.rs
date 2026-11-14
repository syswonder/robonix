// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
//! Route capabilities to in-process builtins or client-owned MCP execution.

pub mod async_poll;
pub mod builtin;
pub mod grpc;

use std::collections::HashMap;
use std::sync::{Arc, LazyLock, Mutex};
use std::time::Duration;

use crate::pb::lifecycle::{DriverRequest, LifecycleDriver};
use crate::pb::pilot::{CapabilityCall, CapabilityCallResult};
use crate::plan_runtime::PlanRuntime;
use anyhow::{Context, Result};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::client::channel::{Channel, ChannelContract, Mcp};
use robonix_atlas::pb as atlas_pb;

const CMD_ACTIVATE: u32 = 1;
const DRIVER_ACTIVATE_TIMEOUT: Duration = Duration::from_secs(60);
const DEPLOY_CONSUMER_ID: &str = "com.robonix.executor.skill_activate";

type ActivationSlot = Arc<tokio::sync::Mutex<Option<(String, Channel<LifecycleDriver>)>>>;
static ACTIVATIONS: LazyLock<Mutex<HashMap<String, ActivationSlot>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

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
        .connect_channel::<Mcp>(
            consumer_id,
            &call.provider_id,
            &call.contract_id,
            atlas_pb::Transport::Mcp,
        )
        .await
}

/// Serialize skill activation and retain failed or uncertain Driver calls.
async fn ensure_skill_active(atlas: &mut AtlasClient, provider_id: &str) -> Result<()> {
    let providers = atlas
        .query_capabilities(provider_id, "", atlas_pb::Transport::Unspecified)
        .await?;
    let Some(provider) = providers.first() else {
        return Ok(());
    };
    if provider.kind != atlas_pb::Kind::Skill as i32 {
        return Ok(());
    }
    let slot = {
        let mut slots = ACTIVATIONS.lock().unwrap();
        // Keep slots used by callers, including those waiting for the activation lock.
        slots.retain(|_, slot| {
            Arc::strong_count(slot) > 1
                || slot.try_lock().map_or(true, |previous| previous.is_some())
        });
        if provider.state == atlas_pb::LifecycleState::StateActive as i32
            && !slots.contains_key(provider_id)
        {
            return Ok(());
        }
        anyhow::ensure!(
            slots.contains_key(provider_id) || slots.len() < 1024,
            "skill activation capacity exhausted"
        );
        slots.entry(provider_id.to_owned()).or_default().clone()
    };
    let mut previous = slot.lock().await;
    // The preceding activation may have completed while this caller was waiting.
    let providers = atlas
        .query_capabilities(provider_id, "", atlas_pb::Transport::Unspecified)
        .await?;
    let provider = providers
        .first()
        .context("skill disappeared before activation")?;
    if provider.state == atlas_pb::LifecycleState::StateActive as i32 {
        if let Some((_, channel)) = previous.as_ref()
            && channel.is_terminal()
        {
            channel.close().await?;
        }
        *previous = None;
        return Ok(());
    }
    anyhow::ensure!(
        provider.state == atlas_pb::LifecycleState::StateInactive as i32,
        "skill {provider_id} is {}; automatic activation requires INACTIVE",
        atlas_pb::LifecycleState::try_from(provider.state)
            .unwrap_or(atlas_pb::LifecycleState::StateUnspecified)
            .as_str_name()
            .trim_start_matches("STATE_")
    );
    if let Some((registration, channel)) = previous.as_ref()
        && registration == &provider.registration_id
    {
        anyhow::bail!(
            "previous skill activation did not confirm ACTIVE; channel {}; recover the lifecycle before retrying",
            channel.channel_id
        );
    }
    let mut drivers = provider
        .capabilities
        .iter()
        .filter(|c| c.transport == atlas_pb::Transport::Grpc as i32);
    let legacy_driver = format!("{}/driver", provider.namespace);
    let driver = drivers
        .clone()
        .find(|c| c.contract_id == LifecycleDriver::ID)
        .or_else(|| drivers.find(|c| c.contract_id == legacy_driver))
        .context("skill must declare a gRPC Driver")?;
    let channel = atlas
        .connect_channel::<LifecycleDriver>(
            DEPLOY_CONSUMER_ID,
            provider_id,
            &driver.contract_id,
            atlas_pb::Transport::Grpc,
        )
        .await?;
    *previous = Some((provider.registration_id.clone(), channel.clone()));
    let response = tokio::time::timeout(
        DRIVER_ACTIVATE_TIMEOUT,
        channel.call(DriverRequest {
            command: CMD_ACTIVATE,
            config_json: String::new(),
        }),
    )
    .await
    .with_context(|| {
        format!(
            "Driver activation timed out; channel {} retained",
            channel.channel_id
        )
    })?;
    let cleanup = if channel.is_terminal() {
        channel.close().await
    } else {
        Ok(())
    };
    let response = response?;
    cleanup?;
    anyhow::ensure!(
        response.ok && response.state.eq_ignore_ascii_case("active"),
        "Driver(CMD_ACTIVATE) failed: {response:?}"
    );
    Ok(())
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
    #[tokio::test]
    #[ignore = "requires test_channel.py's live Atlas and gRPC fixtures"]
    async fn skill_activation() -> anyhow::Result<()> {
        use super::ensure_skill_active;
        use robonix_atlas::client::{AtlasClient, grpc_params};
        use robonix_atlas::pb::{LifecycleState, Transport};
        use std::time::Duration;
        let mut atlas = AtlasClient::connect(std::env::var("ROBONIX_TEST_ATLAS")?).await?;
        let endpoint = std::env::var("ROBONIX_TEST_GRPC")?;
        let control = std::env::var("ROBONIX_TEST_CONTROL")?;
        let provider = "executor-skill-test";
        async fn register(
            atlas: &mut AtlasClient,
            provider: &str,
            endpoint: &str,
            legacy: bool,
        ) -> anyhow::Result<()> {
            atlas
                .register_skill(provider, "robonix/skill/channel_test", "")
                .await?;
            let (contract, service) = if legacy {
                (
                    "robonix/skill/channel_test/driver",
                    "robonix.contracts.RobonixSkillChannelTestDriver",
                )
            } else {
                (
                    "robonix/lifecycle/driver",
                    "robonix.contracts.RobonixLifecycleDriver",
                )
            };
            atlas
                .declare_capability(
                    provider,
                    contract,
                    Transport::Grpc,
                    endpoint,
                    grpc_params(
                        "robonix_contracts.proto",
                        service,
                        format!("/{service}/Driver"),
                    ),
                )
                .await?;
            atlas
                .set_lifecycle_state(provider, LifecycleState::StateInactive, "")
                .await?;
            Ok(())
        }
        let configure = |provider: &str, delay: f64, reject: bool| -> anyhow::Result<()> {
            std::fs::write(
                &control,
                serde_json::json!({"delay": delay, "driver_reject": reject,
                "driver_provider": provider})
                .to_string(),
            )?;
            Ok(())
        };
        let activations = || -> usize {
            std::fs::read_to_string(std::path::Path::new(&control).with_extension("grpc-trace"))
                .unwrap_or_default()
                .lines()
                .filter(|line| line.contains("\"Driver\""))
                .count()
        };
        register(&mut atlas, provider, &endpoint, false).await?;
        configure("", 0., false)?;
        ensure_skill_active(&mut atlas, provider).await?;
        assert!(ensure_skill_active(&mut atlas, provider).await.is_err());
        atlas
            .set_lifecycle_state(provider, LifecycleState::StateActive, "")
            .await?;
        ensure_skill_active(&mut atlas, provider).await?;
        atlas.unregister(provider).await?;
        register(&mut atlas, provider, &endpoint, false).await?;
        configure(provider, 0.1, false)?;
        let before = activations();
        let mut other = atlas.clone();
        let (a, b) = tokio::join!(
            ensure_skill_active(&mut atlas, provider),
            ensure_skill_active(&mut other, provider)
        );
        a?;
        b?;
        assert_eq!(activations(), before + 1);
        ensure_skill_active(&mut atlas, provider).await?;
        assert_eq!(activations(), before + 1);
        atlas.unregister(provider).await?;
        register(&mut atlas, provider, &endpoint, false).await?;
        ensure_skill_active(&mut atlas, provider).await?;
        assert_eq!(activations(), before + 2);

        atlas.unregister(provider).await?;
        register(&mut atlas, provider, &endpoint, false).await?;
        configure(provider, 0.2, false)?;
        assert!(
            tokio::time::timeout(
                Duration::from_millis(20),
                ensure_skill_active(&mut atlas, provider)
            )
            .await
            .is_err()
        );
        assert!(ensure_skill_active(&mut atlas, provider).await.is_err());
        tokio::time::sleep(Duration::from_millis(300)).await;
        ensure_skill_active(&mut atlas, provider).await?;
        assert_eq!(activations(), before + 3);

        atlas.unregister(provider).await?;
        register(&mut atlas, provider, &endpoint, false).await?;
        configure(provider, 0., true)?;
        assert!(ensure_skill_active(&mut atlas, provider).await.is_err());
        assert!(ensure_skill_active(&mut atlas, provider).await.is_err());
        assert_eq!(activations(), before + 4);
        atlas
            .set_lifecycle_state(provider, LifecycleState::StateInactive, "")
            .await?;
        configure(provider, 0., false)?;
        assert!(ensure_skill_active(&mut atlas, provider).await.is_err());
        assert_eq!(activations(), before + 4);
        atlas.unregister(provider).await?;
        register(&mut atlas, provider, &endpoint, true).await?;
        configure(provider, 0., false)?;
        ensure_skill_active(&mut atlas, provider).await?;
        assert_eq!(activations(), before + 5);
        atlas.unregister(provider).await?;
        register(&mut atlas, provider, &endpoint, true).await?;
        configure(provider, 0.2, false)?;
        assert!(
            tokio::time::timeout(
                Duration::from_millis(20),
                ensure_skill_active(&mut atlas, provider)
            )
            .await
            .is_err()
        );
        assert!(ensure_skill_active(&mut atlas, provider).await.is_err());
        tokio::time::sleep(Duration::from_millis(300)).await;
        ensure_skill_active(&mut atlas, provider).await?;
        assert_eq!(activations(), before + 6);
        atlas.unregister(provider).await?;
        register(&mut atlas, provider, &endpoint, true).await?;
        configure(provider, 0., true)?;
        assert!(ensure_skill_active(&mut atlas, provider).await.is_err());
        assert!(ensure_skill_active(&mut atlas, provider).await.is_err());
        assert_eq!(activations(), before + 7);
        atlas.unregister(provider).await?;
        for index in 0..1025 {
            let provider = format!("executor-skill-churn-{index}");
            register(&mut atlas, &provider, &endpoint, false).await?;
            configure(&provider, 0., false)?;
            ensure_skill_active(&mut atlas, &provider).await?;
            ensure_skill_active(&mut atlas, &provider).await?;
            atlas.unregister(&provider).await?;
        }
        assert_eq!(activations(), before + 7 + 1025);
        std::fs::write(&control, "{}")?;
        Ok(())
    }
}
