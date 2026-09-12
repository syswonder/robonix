// SPDX-License-Identifier: MulanPSL-2.0
//
// Executor-owned result verification. A configured rule turns one successful
// capability result into a synchronous call to a provider implementing the
// shared `robonix/service/verifier/verify` contract.

use std::time::Duration;

use serde::Deserialize;

use crate::config::VerificationRule;
use crate::dispatch;
use crate::pb::pilot::{CapabilityCall, CapabilityCallResult};
use crate::plan_runtime::PlanRuntime;
use crate::rtdl_wire::NodeEventContext;
use robonix_atlas::client::AtlasClient;
use robonix_scribe::{info, warn};

pub const VERIFY_CONTRACT_ID: &str = "robonix/service/verifier/verify";
const VERIFICATION_TIMEOUT: Duration = Duration::from_secs(60);

/// Immutable rule set shared by all concurrently executing plans.
#[derive(Clone, Debug, Default)]
pub struct VerificationPolicy {
    rules: Vec<VerificationRule>,
}

impl VerificationPolicy {
    pub fn new(rules: Vec<VerificationRule>) -> Self {
        Self { rules }
    }

    pub fn len(&self) -> usize {
        self.rules.len()
    }

    /// Exact provider+contract rules override a contract-only fallback.
    pub fn rule_for(&self, call: &CapabilityCall) -> Option<&VerificationRule> {
        self.rules
            .iter()
            .find(|rule| {
                rule.target_contract_id == call.contract_id
                    && rule.target_provider_id.as_deref() == Some(call.provider_id.as_str())
            })
            .or_else(|| {
                self.rules.iter().find(|rule| {
                    rule.target_contract_id == call.contract_id && rule.target_provider_id.is_none()
                })
            })
    }
}

#[derive(Debug, Deserialize)]
struct VerifyResponse {
    passed: bool,
    detail: String,
}

/// Verify a successful result when a rule matches. The verifier call itself
/// goes straight through dispatch and is never recursively verified.
pub async fn verify_result(
    policy: &VerificationPolicy,
    call: &CapabilityCall,
    node: &NodeEventContext,
    original: CapabilityCallResult,
    self_provider_id: &str,
    atlas: &mut AtlasClient,
    runtime: &PlanRuntime,
) -> CapabilityCallResult {
    let Some(rule) = policy.rule_for(call) else {
        return original;
    };

    let verifier_call = build_verifier_call(rule, call, &node.description, &original);
    info!(
        "[executor/verification] call_id={} target='{}' verifier='{}'",
        call.call_id, call.contract_id, rule.verifier_provider_id
    );
    let response = dispatch::dispatch_with_timeout(
        &verifier_call,
        self_provider_id,
        atlas,
        runtime,
        &node.plan_id,
        Some(VERIFICATION_TIMEOUT),
    )
    .await;
    if !response.success {
        return unavailable(
            original,
            format!(
                "verifier '{}' failed: {}",
                rule.verifier_provider_id, response.error
            ),
        );
    }

    let parsed: VerifyResponse = match serde_json::from_str(&response.output) {
        Ok(parsed) => parsed,
        Err(error) => {
            return unavailable(
                original,
                format!(
                    "verifier '{}' returned invalid response: {error}",
                    rule.verifier_provider_id
                ),
            );
        }
    };
    apply_verdict(original, parsed)
}

/// Build the common two-field Verify request. The nested payload stays opaque
/// to Executor beyond carrying the original call context and configured args.
fn build_verifier_call(
    rule: &VerificationRule,
    call: &CapabilityCall,
    description: &str,
    result: &CapabilityCallResult,
) -> CapabilityCall {
    let payload = serde_json::json!({
        "target_provider_id": call.provider_id,
        "target_contract_id": call.contract_id,
        "target_description": description,
        "target_args": json_or_string(&call.args_json),
        "target_output": json_or_string(&result.output),
        "verifier_args": rule.verifier_args,
    });
    CapabilityCall {
        call_id: format!("{}:verify", call.call_id),
        provider_id: rule.verifier_provider_id.clone(),
        contract_id: VERIFY_CONTRACT_ID.to_string(),
        args_json: serde_json::json!({
            "call_id": call.call_id,
            "args_json": payload.to_string(),
        })
        .to_string(),
    }
}

fn json_or_string(raw: &str) -> serde_json::Value {
    serde_json::from_str(raw).unwrap_or_else(|_| serde_json::Value::String(raw.to_string()))
}

fn apply_verdict(
    mut original: CapabilityCallResult,
    verdict: VerifyResponse,
) -> CapabilityCallResult {
    if verdict.passed {
        info!(
            "[executor/verification] call_id={} passed: {}",
            original.call_id, verdict.detail
        );
        return original;
    }
    let detail = if verdict.detail.trim().is_empty() {
        "verifier rejected the capability result".to_string()
    } else {
        verdict.detail
    };
    warn!(
        "[executor/verification] call_id={} rejected: {}",
        original.call_id, detail
    );
    original.success = false;
    original.error = format!("result verification failed: {detail}");
    original
}

fn unavailable(mut original: CapabilityCallResult, reason: String) -> CapabilityCallResult {
    warn!(
        "[executor/verification] call_id={} unavailable: {}",
        original.call_id, reason
    );
    original.success = false;
    original.error = format!("result verification unavailable: {reason}");
    original
}

#[cfg(test)]
mod tests {
    use super::*;
    use robonix_atlas::service::{AtlasRegistry, serve_atlas};
    use std::net::SocketAddr;
    use std::sync::Arc;

    fn rule(provider: Option<&str>, verifier: &str) -> VerificationRule {
        VerificationRule {
            target_contract_id: "cap/target".to_string(),
            target_provider_id: provider.map(str::to_string),
            verifier_provider_id: verifier.to_string(),
            verifier_args: serde_json::json!({"camera_provider_id":"front_camera"}),
        }
    }

    fn call() -> CapabilityCall {
        CapabilityCall {
            call_id: "p:0".to_string(),
            provider_id: "arm".to_string(),
            contract_id: "cap/target".to_string(),
            args_json: r#"{"object":"bottle"}"#.to_string(),
        }
    }

    fn result() -> CapabilityCallResult {
        CapabilityCallResult {
            call_id: "p:0".to_string(),
            provider_id: "arm".to_string(),
            contract_id: "cap/target".to_string(),
            success: true,
            output: r#"{"accepted":true}"#.to_string(),
            error: String::new(),
        }
    }

    fn node() -> NodeEventContext {
        NodeEventContext {
            plan_id: "p".to_string(),
            node_index: 0,
            node_kind: 0,
            op_id: "op".to_string(),
            description: "place bottle".to_string(),
        }
    }

    fn reserve_address() -> SocketAddr {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("reserve port");
        listener.local_addr().expect("reserved address")
    }

    #[test]
    fn exact_provider_rule_wins_over_contract_fallback() {
        let policy =
            VerificationPolicy::new(vec![rule(None, "fallback"), rule(Some("arm"), "exact")]);
        assert_eq!(
            policy.rule_for(&call()).unwrap().verifier_provider_id,
            "exact"
        );
    }

    #[test]
    fn verifier_request_contains_runtime_context_and_opaque_args() {
        let verify = build_verifier_call(
            &rule(None, "vlm_verifier"),
            &call(),
            "place bottle",
            &result(),
        );
        let outer: serde_json::Value = serde_json::from_str(&verify.args_json).unwrap();
        let payload: serde_json::Value =
            serde_json::from_str(outer["args_json"].as_str().unwrap()).unwrap();
        assert_eq!(verify.contract_id, VERIFY_CONTRACT_ID);
        assert_eq!(payload["target_args"]["object"], "bottle");
        assert_eq!(payload["target_output"]["accepted"], true);
        assert_eq!(payload["target_description"], "place bottle");
        assert_eq!(
            payload["verifier_args"]["camera_provider_id"],
            "front_camera"
        );
    }

    #[test]
    fn rejected_verdict_preserves_output_and_fails_original_call() {
        let output = result().output;
        let verified = apply_verdict(
            result(),
            VerifyResponse {
                passed: false,
                detail: "bottle is outside the box".to_string(),
            },
        );
        assert!(!verified.success);
        assert_eq!(verified.output, output);
        assert!(verified.error.contains("result verification failed"));
    }

    #[test]
    fn passed_verdict_keeps_original_result_unchanged() {
        let original = result();
        let verified = apply_verdict(
            original.clone(),
            VerifyResponse {
                passed: true,
                detail: "visible".to_string(),
            },
        );
        assert_eq!(verified, original);
    }

    #[test]
    fn unavailable_verifier_preserves_output_but_fails_closed() {
        let output = result().output;
        let verified = unavailable(result(), "not registered".to_string());
        assert!(!verified.success);
        assert_eq!(verified.output, output);
        assert!(verified.error.contains("result verification unavailable"));
    }

    #[tokio::test]
    async fn missing_verifier_provider_fails_closed_over_atlas() {
        let atlas_addr = reserve_address();
        let registry = Arc::new(AtlasRegistry::default());
        let atlas_server = tokio::spawn(serve_atlas(registry, atlas_addr));
        let mut atlas = AtlasClient::connect_with_retry(
            format!("http://{atlas_addr}"),
            50,
            Duration::from_millis(10),
        )
        .await
        .expect("connect test Atlas");
        let policy = VerificationPolicy::new(vec![rule(None, "missing_verifier")]);
        let verified = verify_result(
            &policy,
            &call(),
            &node(),
            result(),
            "executor",
            &mut atlas,
            &PlanRuntime::default(),
        )
        .await;
        assert!(!verified.success);
        assert!(verified.error.contains("result verification unavailable"));
        assert!(verified.error.contains("ConnectCapability failed"));
        atlas_server.abort();
    }
}
