// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//! Execute one RTDL do node, including async polling and result verification.
use crate::active_plans::PlanRuntime;
use crate::dispatch::{async_poll, async_registry};
use crate::pb::executor::RtdlEvent;
use crate::pb::pilot::rtdl_node_state::RtdlNodeStateEnum;
use crate::pb::pilot::{CapabilityCall, CapabilityCallResult};
use crate::rtdl_events::{self, NodeEventContext};
use crate::sentinel_runtime::{SENTINEL_PROVIDER_ID, SentinelIdentity, SentinelRuntime};
use crate::verification::{self, VerificationPolicy};
use robonix_atlas::client::AtlasClient;
use robonix_scribe::{info, warn};
use std::sync::Arc;
use tokio::sync::mpsc::Sender;
use tokio::task::JoinHandle;
use tonic::Status;

/// Immediate call outcome and its optional background result verification.
pub(crate) struct NodeExecution {
    /// Whether the immediate outcome failed, before any background verification finishes.
    pub(crate) failed: bool,
    /// Pending background verification task, returning true if verification fails.
    pub(crate) verifier: Option<JoinHandle<bool>>,
}

impl NodeExecution {
    /// Build an outcome with no background verification still pending.
    fn finished(failed: bool) -> Self {
        Self {
            failed,
            verifier: None,
        }
    }
}

/// Wait for one verifier, treating a panicked or aborted task as failed verification.
pub(crate) async fn wait_for_verifier(verifier: JoinHandle<bool>) -> bool {
    match verifier.await {
        Ok(failed) => failed,
        Err(error) => {
            warn!("[executor/verification] overlap verifier failed to join: {error}");
            true
        }
    }
}

/// Owned inputs for one background verifier.
struct OverlapVerifier {
    /// Completed capability call whose result is being checked.
    call: CapabilityCall,
    /// Node identity and task description used for verifier input and final events.
    node: NodeEventContext,
    /// Sends the verified final node state to the plan's RPC event stream.
    tx: Sender<Result<RtdlEvent, Status>>,
    /// Atlas connection used to resolve and dispatch the verifier capability.
    atlas: AtlasClient,
    /// Executor's own provider ID used as the verifier call's consumer identity.
    provider_id: String,
    /// Shared plan state used to track verifier calls and record the final node state.
    runtime: PlanRuntime,
    /// Rules selecting the verifier and its configured arguments for this call.
    policy: Arc<VerificationPolicy>,
    /// Successful capability result retained as the basis for the verified result.
    original: CapabilityCallResult,
}

/// Map a completed leaf call to its RTDL state. Cancellation wins over the
/// provider result: a command terminated by cancel_plan normally returns
/// success=false, but that is an expected CANCELED outcome, not a FAILED tool.
fn leaf_terminal_state(success: bool, cancelled: bool) -> u32 {
    if cancelled {
        RtdlNodeStateEnum::Canceled as u32
    } else if success {
        RtdlNodeStateEnum::Succeeded as u32
    } else {
        RtdlNodeStateEnum::Failed as u32
    }
}

/// Dispatch one RTDL `do` node and stream node_state events.
#[allow(clippy::too_many_arguments)]
pub(crate) async fn execute_call(
    call: &CapabilityCall,
    node: NodeEventContext,
    tx: Sender<Result<RtdlEvent, Status>>,
    mut atlas: AtlasClient,
    provider_id: String,
    runtime: PlanRuntime,
    sentinel: SentinelRuntime,
    identity: SentinelIdentity,
    verification: Arc<VerificationPolicy>,
) -> NodeExecution {
    let decision = sentinel
        .check(
            &atlas,
            SENTINEL_PROVIDER_ID,
            &call.contract_id,
            &call.args_json,
            &identity,
        )
        .await;
    if !decision.allow {
        let result = CapabilityCallResult {
            call_id: call.call_id.clone(),
            provider_id: call.provider_id.clone(),
            contract_id: call.contract_id.clone(),
            success: false,
            output: String::new(),
            error: format!(
                "Sentinel denied capability call by rule '{}': {}",
                decision.rule_id, decision.reason
            ),
        };
        warn!(
            "[sentinel] denied call_id={} contract='{}' rule='{}': {}",
            call.call_id, call.contract_id, decision.rule_id, decision.reason
        );
        runtime
            .record_op_state(&node.plan_id, &node.op_id, RtdlNodeStateEnum::Failed as u32)
            .await;
        let _ = tx
            .send(Ok(rtdl_events::node_state_from_result(
                &node,
                result,
                RtdlNodeStateEnum::Failed as u32,
            )))
            .await;
        return NodeExecution::finished(true);
    }

    // Log the args too (bounded) so the log shows what each call requested —
    // useful for debugging capability calls. Truncated to keep large payloads
    // (images, file content) from bloating the log.
    let args_preview: String = call.args_json.chars().take(256).collect();
    let args_ellipsis = if call.args_json.len() > 256 {
        "…"
    } else {
        ""
    };
    info!(
        "[executor] dispatching call_id={} provider='{}' contract='{}' args={}{}",
        call.call_id, call.provider_id, call.contract_id, args_preview, args_ellipsis,
    );

    // Mark the op running so get_plan_status shows the in-flight node. Live
    // async states may replace it, then this function records one final state.
    runtime
        .record_op_state(
            &node.plan_id,
            &node.op_id,
            RtdlNodeStateEnum::Running as u32,
        )
        .await;

    let async_group = if call.provider_id == provider_id {
        Ok(None)
    } else {
        async_registry::resolve_async_group(&mut atlas, &call.provider_id, &call.contract_id).await
    };

    let (mut result, mut state) = match async_group {
        Err(error) => {
            let r = CapabilityCallResult {
                call_id: call.call_id.clone(),
                provider_id: call.provider_id.clone(),
                contract_id: call.contract_id.clone(),
                success: false,
                output: String::new(),
                error,
            };
            (r, RtdlNodeStateEnum::Failed as u32)
        }
        Ok(Some(group)) => {
            async_poll::run_until_terminal(call, &group, &provider_id, &mut atlas, &node, &runtime)
                .await
        }
        Ok(None) => {
            let r =
                crate::dispatch::dispatch(call, &provider_id, &mut atlas, &runtime, &node.plan_id)
                    .await;
            let cancelled = runtime.is_cancelled(&node.plan_id).await;
            let state = leaf_terminal_state(r.success, cancelled);
            (r, state)
        }
    };

    let eligible_for_verification = result.success
        && state == RtdlNodeStateEnum::Succeeded as u32
        && !runtime.is_cancelled(&node.plan_id).await;
    let overlap = eligible_for_verification
        && verification.overlap()
        && verification.rule_for(call).is_some();

    if overlap {
        runtime
            .record_op_state(
                &node.plan_id,
                &node.op_id,
                RtdlNodeStateEnum::Verifying as u32,
            )
            .await;
        let _ = tx
            .send(Ok(rtdl_events::node_state_from_result(
                &node,
                result.clone(),
                RtdlNodeStateEnum::Verifying as u32,
            )))
            .await;
        log_success(call, &result);

        let verifier = tokio::spawn(
            OverlapVerifier {
                call: call.clone(),
                node,
                tx,
                atlas,
                provider_id,
                runtime,
                policy: verification,
                original: result,
            }
            .run(),
        );
        return NodeExecution {
            failed: false,
            verifier: Some(verifier),
        };
    }

    if eligible_for_verification {
        result = verification::verify_result(
            verification.as_ref(),
            call,
            &node,
            result,
            &provider_id,
            &mut atlas,
            &runtime,
        )
        .await;
        if !result.success {
            state = RtdlNodeStateEnum::Failed as u32;
        }
    }

    if runtime.is_cancelled(&node.plan_id).await {
        state = RtdlNodeStateEnum::Canceled as u32;
    }
    runtime
        .record_op_state(&node.plan_id, &node.op_id, state)
        .await;
    let _ = tx
        .send(Ok(rtdl_events::node_state_from_result(
            &node,
            result.clone(),
            state,
        )))
        .await;
    let failed = !result.success;

    if result.success {
        log_success(call, &result);
    } else {
        warn!("[executor] '{}' failed: {}", call.contract_id, result.error);
    }

    NodeExecution::finished(failed)
}

impl OverlapVerifier {
    /// Run the verifier and publish the leaf's final state without side effects.
    async fn run(mut self) -> bool {
        let verified = verification::verify_result(
            self.policy.as_ref(),
            &self.call,
            &self.node,
            self.original,
            &self.provider_id,
            &mut self.atlas,
            &self.runtime,
        )
        .await;
        // Entering VERIFYING means capability execution already completed.
        // A later plan cancellation therefore cannot replace this leaf's
        // verifier-owned SUCCEEDED/FAILED final state.
        let state = leaf_terminal_state(verified.success, false);
        self.runtime
            .record_op_state(&self.node.plan_id, &self.node.op_id, state)
            .await;
        let _ = self
            .tx
            .send(Ok(rtdl_events::node_state_from_result(
                &self.node,
                verified.clone(),
                state,
            )))
            .await;
        if !verified.success {
            warn!(
                "[executor] '{}' verification finalized call_id={} as failed: {}",
                self.call.contract_id, self.call.call_id, verified.error
            );
        }
        !verified.success
    }
}

/// Log one successful capability output with a bounded payload preview.
fn log_success(call: &CapabilityCall, result: &CapabilityCallResult) {
    let preview: String = result.output.chars().take(512).collect();
    let ellipsis = if result.output.len() > 512 { "..." } else { "" };
    info!(
        "[executor] '{}' ok: {}{}",
        call.contract_id, preview, ellipsis
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::VerificationRule;
    use crate::sentinel_runtime::{SentinelIdentity, SentinelRuntime};
    use robonix_atlas::service::{AtlasRegistry, serve_atlas};
    use std::net::SocketAddr;
    use std::time::Duration;
    const RTDL_DO: u32 = 2;

    /// Construct a target call with stable provider and contract identity.
    fn call(id: &str) -> CapabilityCall {
        CapabilityCall {
            call_id: id.to_string(),
            provider_id: "provider".to_string(),
            contract_id: "robonix/test/cap".to_string(),
            args_json: "{}".to_string(),
        }
    }

    fn reserve_address() -> SocketAddr {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("reserve port");
        listener.local_addr().expect("reserved address")
    }

    fn sentinel_runtime() -> SentinelRuntime {
        SentinelRuntime::load(std::env::temp_dir().join(format!(
            "executor-capability-sentinel-{}.json",
            uuid::Uuid::new_v4()
        )))
        .expect("empty Sentinel policy")
    }

    /// Preserve verifier outcomes and fail closed if its background task cannot finish.
    #[tokio::test]
    async fn overlap_verifier_reports_outcome_and_join_failure() {
        assert!(!wait_for_verifier(tokio::spawn(async { false })).await);
        assert!(wait_for_verifier(tokio::spawn(async { true })).await);
        assert!(wait_for_verifier(tokio::spawn(async { panic!("test verifier panic") })).await);
        let verifier = tokio::spawn(std::future::pending::<bool>());
        verifier.abort();
        assert!(wait_for_verifier(verifier).await);
    }

    /// Global overlap=false retains one synchronous verified terminal event.
    #[tokio::test]
    async fn synchronous_verification_never_emits_verifying() {
        let atlas_addr = reserve_address();
        let atlas_server =
            tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), atlas_addr));
        let atlas = AtlasClient::connect_with_retry(
            format!("http://{atlas_addr}"),
            50,
            Duration::from_millis(10),
        )
        .await
        .expect("connect test Atlas");
        let runtime = PlanRuntime::default();
        runtime.register_plan("p").await.unwrap();
        let contract_id = "robonix/system/executor/builtin/get_all_plans";
        let call = CapabilityCall {
            call_id: "p:1".into(),
            provider_id: "executor".into(),
            contract_id: contract_id.into(),
            args_json: "{}".into(),
        };
        let (tx, mut rx) = tokio::sync::mpsc::channel(2);
        let execution = execute_call(
            &call,
            NodeEventContext {
                plan_id: "p".into(),
                node_index: 1,
                node_kind: RTDL_DO,
                op_id: "op_1".into(),
                description: "verify synchronously".into(),
            },
            tx,
            atlas,
            "executor".into(),
            runtime,
            sentinel_runtime(),
            SentinelIdentity::default(),
            Arc::new(VerificationPolicy::new(
                false,
                vec![VerificationRule {
                    target_contract_id: contract_id.into(),
                    target_provider_id: None,
                    verifier_provider_id: "missing_verifier".into(),
                    verifier_args: serde_json::json!({}),
                }],
            )),
        )
        .await;

        assert!(execution.failed);
        assert!(execution.verifier.is_none());
        let terminal = rx.recv().await.unwrap().unwrap().node_state.unwrap();
        assert_eq!(terminal.state, RtdlNodeStateEnum::Failed as u32);
        assert!(rx.try_recv().is_err());
        atlas_server.abort();
    }

    /// A background verifier emits only its own node's final failure.
    #[tokio::test]
    async fn overlap_verifier_failure_finalizes_only_its_node() {
        let atlas_addr = reserve_address();
        let atlas_server =
            tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), atlas_addr));
        let atlas = AtlasClient::connect_with_retry(
            format!("http://{atlas_addr}"),
            50,
            Duration::from_millis(10),
        )
        .await
        .expect("connect test Atlas");
        let runtime = PlanRuntime::default();
        runtime.register_plan("p").await.unwrap();
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);
        let target_call = call("p:1");
        let verifier = OverlapVerifier {
            call: target_call.clone(),
            node: NodeEventContext {
                plan_id: "p".into(),
                node_index: 1,
                node_kind: RTDL_DO,
                op_id: "op_1".into(),
                description: "verify this node only".into(),
            },
            tx,
            atlas,
            provider_id: "executor".into(),
            runtime,
            policy: Arc::new(VerificationPolicy::new(
                true,
                vec![VerificationRule {
                    target_contract_id: target_call.contract_id.clone(),
                    target_provider_id: None,
                    verifier_provider_id: "missing_verifier".into(),
                    verifier_args: serde_json::json!({}),
                }],
            )),
            original: CapabilityCallResult {
                call_id: target_call.call_id,
                provider_id: target_call.provider_id,
                contract_id: target_call.contract_id,
                success: true,
                output: "done".into(),
                error: String::new(),
            },
        };

        assert!(verifier.run().await);
        let event = rx.recv().await.unwrap().unwrap();
        let final_state = event.node_state.unwrap();
        assert_eq!(final_state.node_index, 1);
        assert_eq!(final_state.state, RtdlNodeStateEnum::Failed as u32);
        assert!(!final_state.leaf_result.unwrap().success);
        assert!(rx.try_recv().is_err());
        atlas_server.abort();
    }

    /// Cancellation does not overwrite a leaf that already entered VERIFYING.
    #[tokio::test]
    async fn canceled_plan_keeps_verifying_leaf_success() {
        let atlas_addr = reserve_address();
        let atlas_server =
            tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), atlas_addr));
        let mut atlas = AtlasClient::connect_with_retry(
            format!("http://{atlas_addr}"),
            50,
            Duration::from_millis(10),
        )
        .await
        .expect("connect test Atlas");
        let runtime = PlanRuntime::default();
        runtime.register_plan("p").await.unwrap();
        runtime
            .cancel_plan_control("p", 0, "executor", &mut atlas)
            .await;
        assert!(runtime.is_cancelled("p").await);
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);
        let target_call = call("p:1");
        let verifier = OverlapVerifier {
            call: target_call.clone(),
            node: NodeEventContext {
                plan_id: "p".into(),
                node_index: 1,
                node_kind: RTDL_DO,
                op_id: "op_1".into(),
                description: "already completed capability".into(),
            },
            tx,
            atlas,
            provider_id: "executor".into(),
            runtime,
            policy: Arc::new(VerificationPolicy::new(true, Vec::new())),
            original: CapabilityCallResult {
                call_id: target_call.call_id,
                provider_id: target_call.provider_id,
                contract_id: target_call.contract_id,
                success: true,
                output: "done".into(),
                error: String::new(),
            },
        };

        assert!(!verifier.run().await);
        let final_state = rx.recv().await.unwrap().unwrap().node_state.unwrap();
        assert_eq!(final_state.state, RtdlNodeStateEnum::Succeeded as u32);
        assert!(final_state.leaf_result.unwrap().success);
        atlas_server.abort();
    }

    /// A canceled command reports cancellation even when its process exits unsuccessfully.
    #[test]
    fn canceled_leaf_is_not_reported_as_failed_provider_work() {
        assert_eq!(
            leaf_terminal_state(false, true),
            RtdlNodeStateEnum::Canceled as u32
        );
        assert_eq!(
            leaf_terminal_state(false, false),
            RtdlNodeStateEnum::Failed as u32
        );
        assert_eq!(
            leaf_terminal_state(true, false),
            RtdlNodeStateEnum::Succeeded as u32
        );
    }
}
