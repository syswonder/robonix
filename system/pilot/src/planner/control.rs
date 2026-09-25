// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Plan control: stop and cancel operations the model requests out of band,
// which never run as RTDL capability calls.

use super::*;

#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) enum MetaPlanOp {
    Cancel {
        plan_id: String,
        wait_ms: u64,
    },
    CancelAll {
        wait_ms: u64,
    },
    StopAt {
        plan_id: String,
        op_id: String,
        when: String,
    },
}

impl MetaPlanOp {
    pub(super) fn cancellation_targets(&self, forest: &HashMap<String, TreeMeta>) -> Vec<String> {
        match self {
            Self::Cancel { plan_id, .. } => vec![plan_id.clone()],
            Self::CancelAll { .. } => forest
                .iter()
                .filter(|(_, meta)| !meta.control_only)
                .map(|(plan_id, _)| plan_id.clone())
                .collect::<Vec<_>>(),
            Self::StopAt { plan_id, .. } => vec![plan_id.clone()],
        }
    }
}

pub(super) fn parse_meta_plan_op(rtdl: &serde_json::Value) -> Result<Option<MetaPlanOp>> {
    let Some(obj) = rtdl.as_object() else {
        return Ok(None);
    };
    let Some(op) = obj.get("op").and_then(|value| value.as_str()) else {
        return Ok(None);
    };
    // Normalize quoted and numeric identifiers to strings for Executor.
    let string = |key: &str| -> Result<String> {
        let value = obj
            .get(key)
            .and_then(|value| match value {
                serde_json::Value::String(text) => Some(text.clone()),
                serde_json::Value::Number(number) => Some(number.to_string()),
                _ => None,
            })
            .ok_or_else(|| anyhow::anyhow!("meta op `{op}` requires string or integer `{key}`"))?;
        if value.trim().is_empty() {
            anyhow::bail!("meta op `{op}` requires non-empty `{key}`");
        }
        Ok(value)
    };
    let wait_ms = || {
        obj.get("wait_ms")
            .and_then(|value| value.as_u64())
            .unwrap_or(5_000)
            .min(30_000)
    };
    let parsed = match op {
        "cancel_plan" => MetaPlanOp::Cancel {
            plan_id: string("plan_id")?,
            wait_ms: wait_ms(),
        },
        "cancel_all" => MetaPlanOp::CancelAll { wait_ms: wait_ms() },
        "stop_plan_at" => {
            let when = obj
                .get("when")
                .and_then(|value| value.as_str())
                .unwrap_or("on_complete")
                .to_string();
            if !matches!(when.as_str(), "on_enter" | "on_complete") {
                anyhow::bail!("meta op `stop_plan_at` requires when=on_enter or on_complete");
            }
            MetaPlanOp::StopAt {
                plan_id: string("plan_id")?,
                op_id: string("target_op_id")?,
                when,
            }
        }
        _ => return Ok(None),
    };
    Ok(Some(parsed))
}

pub(super) async fn execute_meta_plan_op(
    executor: &mut ExecutorConn,
    op: &MetaPlanOp,
) -> Result<String> {
    let request = match op {
        MetaPlanOp::Cancel { plan_id, wait_ms } => ControlPlanRequest {
            action: "cancel".to_string(),
            plan_id: plan_id.clone(),
            op_id: String::new(),
            when: String::new(),
            wait_ms: *wait_ms,
        },
        MetaPlanOp::CancelAll { wait_ms } => ControlPlanRequest {
            action: "cancel_all".to_string(),
            plan_id: String::new(),
            op_id: String::new(),
            when: String::new(),
            wait_ms: *wait_ms,
        },
        MetaPlanOp::StopAt {
            plan_id,
            op_id,
            when,
        } => ControlPlanRequest {
            action: "stop_at".to_string(),
            plan_id: plan_id.clone(),
            op_id: op_id.clone(),
            when: when.clone(),
            wait_ms: 0,
        },
    };
    let timeout = Duration::from_millis(request.wait_ms.saturating_add(2_000).max(2_000));
    let response = tokio::time::timeout(
        timeout,
        executor.control.control_plan(Request::new(request)),
    )
    .await
    .context("Executor plan-control RPC timed out")??
    .into_inner();
    if !response.success {
        anyhow::bail!(response.error);
    }
    Ok(response.message)
}

/// Render the in-flight forest as a system-prompt block so the LLM can see what
/// is still running and reference a `plan_id` to cancel it. Empty when no tree
/// is running. Trees are ordered by numeric plan id for stable output.
/// True when every `do` node is a plan-control builtin and there is at least
/// one. Plan-control trees are not themselves cancellable task work; advertising
/// them makes the model inspect or cancel its own control actions.
pub(super) fn is_control_only(plan: &Plan) -> bool {
    let mut has_do = false;
    for n in &plan.nodes {
        if n.node_kind != RTDL_DO {
            continue;
        }
        has_do = true;
        let leaf = n
            .call
            .as_ref()
            .map(|c| c.contract_id.rsplit('/').next().unwrap_or(""))
            .unwrap_or("");
        if !matches!(
            leaf,
            "cancel_plan"
                | "cancel_all_plans"
                | "get_all_plans"
                | "get_plan_status"
                | "stop_plan_at"
        ) {
            return false;
        }
    }
    has_do
}

pub(super) fn mixes_control_inspection_with_action(plan: &Plan) -> bool {
    let leaves: Vec<&str> = plan
        .nodes
        .iter()
        .filter_map(|node| node.call.as_ref())
        .filter_map(|call| call.contract_id.rsplit('/').next())
        .collect();
    let has_inspection = leaves
        .iter()
        .any(|leaf| matches!(*leaf, "get_plan_status" | "get_all_plans"));
    has_inspection
        && leaves
            .iter()
            .any(|leaf| !matches!(*leaf, "get_plan_status" | "get_all_plans"))
}

pub(super) fn plan_call_signatures(plan: &Plan) -> HashSet<String> {
    plan.nodes
        .iter()
        .filter_map(|node| node.call.as_ref())
        .map(|call| {
            let args = serde_json::from_str::<serde_json::Value>(&call.args_json)
                .ok()
                .and_then(|value| serde_json::to_string(&value).ok())
                .unwrap_or_else(|| call.args_json.clone());
            format!("{}|{}|{args}", call.provider_id, call.contract_id)
        })
        .collect()
}

pub(super) fn duplicate_in_flight_signature(
    signatures: &HashSet<String>,
    forest: &HashMap<String, TreeMeta>,
) -> Option<String> {
    signatures.iter().find_map(|signature| {
        forest
            .values()
            .any(|meta| meta.call_signatures.contains(signature))
            .then(|| signature.clone())
    })
}

pub(super) fn plan_cancel_targets(plan: &Plan) -> Vec<String> {
    plan.nodes
        .iter()
        .filter_map(|node| node.call.as_ref())
        .filter(|call| call.contract_id.rsplit('/').next() == Some("cancel_plan"))
        .filter_map(|call| serde_json::from_str::<serde_json::Value>(&call.args_json).ok())
        .filter_map(|args| {
            args.get("plan_id")
                .and_then(|value| value.as_str())
                .map(str::to_string)
        })
        .collect()
}

pub(super) fn invalid_cancel_target(
    targets: &[String],
    forest: &HashMap<String, TreeMeta>,
    cancel_requested: &HashSet<String>,
) -> Option<String> {
    targets.iter().find_map(|target| {
        let invalid = cancel_requested.contains(target)
            || forest.get(target).is_none_or(|meta| meta.control_only);
        invalid.then(|| target.clone())
    })
}
