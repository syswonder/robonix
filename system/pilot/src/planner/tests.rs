use super::{
    CapabilityTargetMap, CatalogView, DEFAULT_SUCCESS_CRITERION, HistoryBudget, MetaPlanOp,
    RTDL_DO, RTDL_PARALLEL, RTDL_PROTOCOL_REMINDER, RTDL_SEQUENCE, TaskState, TreeMeta, TreeStep,
    UsageTotals, append_request_context, append_steer, append_task_state_record, apply_task_update,
    assemble_planning_messages, build_capability_target_map, build_display_capabilities,
    build_executor_active_block, build_forest_block, compact_tool_result,
    configured_vlm_idle_timeout, duplicate_in_flight_signature, expand_rtdl_to_plan,
    extract_json_object, feed_results_into_history, format_plan_summary, invalid_cancel_target,
    is_control_only, is_legacy_plan_control_contract, is_terminal_executor_state,
    mixes_control_inspection_with_action, parse_meta_plan_op, parse_rtdl_assistant_response,
    parse_task_update, plan_call_signatures, record_dispatched_plan, rtdl_node_kind_name,
    rtdl_recovery_final_text, rtdl_state_name, should_replan_after_plan_done, skip_memory_prefetch,
    start_or_resume_task, task_is_session_end, upsert_terminal_result,
};
use crate::pb::pilot::rtdl_node_state::RtdlNodeStateEnum;
use crate::pb::pilot::{CapabilityCall, CapabilityCallResult, Plan, RtdlNode, RtdlNodeState, Task};
use crate::prompt::{MAX_INLINE_DESCRIPTION_CHARS, summarize_description};
use crate::vlm::{Message, VlmUsage};
use robonix_atlas::pb as atlas_pb;
use serde_json::json;
use std::collections::{HashMap, HashSet};
use std::time::Duration;

#[test]
fn compaction_always_frees_a_quarter_of_the_room() {
    // History keeps growing across many tasks; each compaction must bring it
    // under 75% of the room so the next one is not due at once.
    let budget = HistoryBudget::new(Some(32_768), "deployment_config");
    let non_history = 8_000;
    let room = budget.room(non_history).unwrap();
    let caps = super::compaction::CompactionCaps::for_room(room);
    let mut history: Vec<Message> = Vec::new();
    let mut compactions = 0;
    for step in 0..2_000 {
        history.push(Message::user(&format!(
            "User task (authoritative): task {step} {}",
            "t".repeat(40)
        )));
        history.push(Message::user(&format!(
            "Executor feedback scope: {}",
            "r".repeat(1_500)
        )));
        if !budget.must_compact(&history, non_history) {
            continue;
        }
        let plan = crate::history::plan_compaction(&history, caps.tail, caps.message, caps.pin);
        let summary = crate::history::truncated(
            &Message::user(&format!(
                "{} {}",
                crate::history::SUMMARY_LABEL,
                "s".repeat(100_000)
            )),
            caps.summary,
        );
        history = std::iter::once(summary)
            .chain(plan.pinned)
            .chain(plan.tail)
            .collect();
        let used: usize = history.iter().map(crate::history::tokens).sum();
        assert!(
            used <= room * 3 / 4 + 64,
            "compaction {compactions} left {used} of {room}"
        );
        assert!(!budget.must_compact(&history, non_history));
        compactions += 1;
    }
    assert!(compactions >= 50);
}

#[test]
fn history_budget_waits_for_the_declared_context_limit() {
    let budget = HistoryBudget::new(Some(32_768), "deployment_config");
    let history = vec![Message::user(&"x".repeat(24_000))];
    assert!(!budget.must_compact(&history, 2_000));
    assert!(budget.must_compact(&history, 21_000));

    let unknown = HistoryBudget::new(None, "unavailable");
    assert!(!unknown.must_compact(&history, 100_000));
}

#[test]
fn vlm_idle_timeout_is_bounded_and_has_a_responsive_default() {
    assert_eq!(configured_vlm_idle_timeout(None), Duration::from_secs(30));
    assert_eq!(
        configured_vlm_idle_timeout(Some("1")),
        Duration::from_secs(5)
    );
    assert_eq!(
        configured_vlm_idle_timeout(Some("600")),
        Duration::from_secs(300)
    );
    assert_eq!(
        configured_vlm_idle_timeout(Some("bad")),
        Duration::from_secs(30)
    );
}

#[test]
fn usage_log_carries_per_request_and_cumulative_cache_metrics() {
    let mut totals = UsageTotals::default();
    let first = totals.record(
        0,
        0,
        &VlmUsage {
            prompt_tokens: 1_200,
            completion_tokens: 80,
            cached_tokens: Some(900),
        },
    );
    assert_eq!(first["input_tokens"], 1_200);
    assert_eq!(first["output_tokens"], 80);
    assert_eq!(first["cached_input_tokens"], 900);
    assert_eq!(first["uncached_input_tokens"], 300);
    assert_eq!(first["cache_hit"], true);
    assert_eq!(first["cache_epoch"], 0);
    assert_eq!(first["cumulative"]["cache_hit_requests"], 1);

    let second = totals.record(
        1,
        1,
        &VlmUsage {
            prompt_tokens: 800,
            completion_tokens: 20,
            cached_tokens: Some(0),
        },
    );
    assert_eq!(second["cumulative"]["requests_with_usage"], 2);
    assert_eq!(second["cumulative"]["input_tokens"], 2_000);
    assert_eq!(second["cumulative"]["output_tokens"], 100);
    assert_eq!(second["cumulative"]["cached_input_tokens"], 900);
    assert_eq!(second["cumulative"]["uncached_input_tokens"], 1_100);
    assert_eq!(second["cumulative"]["cache_hit_requests"], 1);
}

/// Repeated final events retain only the latest node result defensively.
#[test]
fn repeated_terminal_state_replaces_the_prior_result() {
    let mut results = vec![RtdlNodeState {
        node_index: 4,
        state: RtdlNodeStateEnum::Succeeded as u32,
        ..Default::default()
    }];

    upsert_terminal_result(
        &mut results,
        RtdlNodeState {
            node_index: 4,
            state: RtdlNodeStateEnum::Failed as u32,
            ..Default::default()
        },
    );

    assert_eq!(results.len(), 1);
    assert_eq!(results[0].state, RtdlNodeStateEnum::Failed as u32);
}

fn test_capability(provider: &str, leaf: &str) -> (String, atlas_pb::Capability) {
    (
        provider.to_string(),
        atlas_pb::Capability {
            provider_id: provider.to_string(),
            contract_id: format!("robonix/service/test/{leaf}"),
            transport: atlas_pb::Transport::Mcp as i32,
            params: Some(atlas_pb::TransportParams {
                kind: Some(atlas_pb::transport_params::Kind::Mcp(atlas_pb::McpParams {
                    input_schema_json: format!(
                        r#"{{"type":"object","properties":{{"{leaf}":{{"type":"string"}}}}}}"#
                    ),
                })),
            }),
            description: format!("Run {leaf}"),
            ..Default::default()
        },
    )
}

#[test]
fn later_round_protocol_is_compact_but_keeps_admission_rules() {
    assert!(RTDL_PROTOCOL_REMINDER.len() < 2_000);
    for required in [
        "capability_name",
        "sequence",
        "parallel",
        "cancel_plan",
        "plan_id/call_id",
        "task_update.goal",
        "Scene regions",
        "Never call a skill's cancel capability",
    ] {
        assert!(RTDL_PROTOCOL_REMINDER.contains(required));
    }
}

#[test]
fn catalog_is_full_once_then_changes_only_and_three_step_tree_stays_one_plan() {
    let capabilities = vec![
        test_capability("demo", "observe"),
        test_capability("demo", "remember"),
        test_capability("demo", "report"),
    ];
    let display = build_display_capabilities(&capabilities, &HashSet::new());
    let mut view = CatalogView::default();
    let first = view.update(&display);
    assert!(first.contains("complete catalog") && first.contains("demo.test_report"));
    // A dropped round never reached history, so the full catalog repeats.
    assert!(view.update(&display).contains("complete catalog"));
    view.commit();
    assert_eq!(view.update(&display), "");

    let fewer = build_display_capabilities(&capabilities[..2], &HashSet::new());
    let change = view.update(&fewer);
    assert!(change.contains("Removed, no longer callable: `demo.test_report`"));
    assert!(!change.contains("demo.test_observe"));
    view.commit();
    assert!(
        view.update(&display)
            .contains("Added:\n- capability_name: demo.test_report")
    );
    view.reset();
    assert!(view.update(&display).contains("complete catalog"));

    let targets = build_capability_target_map(&display);
    let rtdl = json!({
        "op": "sequence",
        "op_id": 0,
        "description": "observe, remember, then report",
        "children": [
            {"op":"do","op_id":0,"description":"observe","cap":"demo.test_observe","args":{"observe":"room"}},
            {"op":"do","op_id":0,"description":"remember","cap":"demo.test_remember","args":{"remember":"room"}},
            {"op":"do","op_id":0,"description":"report","cap":"demo.test_report","args":{"report":"room"}}
        ]
    });
    let plan =
        expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 1, "multi-step").unwrap();
    assert_eq!(super::plan_call_count(&plan), 3);
    assert_eq!(plan.round, 1);
}

#[test]
fn large_region_results_remain_valid_json_and_keep_stable_ids() {
    let regions: Vec<_> = (0..24)
        .map(|index| {
            json!({
                "id": format!("scene.room.anno.{index}"),
                "kind": "room",
                "name": format!("room {index}"),
                "points_xy": vec![index as f64; 300],
                "stale": false,
                "stale_reason": "",
            })
        })
        .collect();
    let original = json!({
        "regions": regions,
        "map_id": "3f_demo",
        "stamp_unix": 123.0,
    })
    .to_string();
    assert!(original.chars().count() > 4096);

    let compact = compact_tool_result("robonix/system/scene/list_regions", &original, 4096);
    let parsed: serde_json::Value = serde_json::from_str(&compact).unwrap();
    let compact_regions = parsed["regions"].as_array().unwrap();
    assert_eq!(compact_regions.len(), 24);
    assert_eq!(compact_regions[23]["id"], "scene.room.anno.23");
    assert_eq!(parsed["_robonix_truncation"]["complete_record_index"], true);
    assert!(compact.chars().count() <= 4096);
}

#[test]
fn arbitrary_large_text_is_marked_incomplete_in_valid_json() {
    let compact = compact_tool_result("example/large", &"x".repeat(9000), 4096);
    let parsed: serde_json::Value = serde_json::from_str(&compact).unwrap();
    assert_eq!(parsed["_robonix_truncation"]["truncated"], true);
    assert_eq!(parsed["_robonix_truncation"]["complete"], false);
    assert!(compact.chars().count() <= 4096);
}

#[test]
fn malformed_image_shape_is_bounded_as_text() {
    let original = json!({
        "width": 640,
        "height": 480,
        "encoding": "error",
        "data": "x".repeat(9000),
    })
    .to_string();
    assert!(!crate::history::is_image_output(&original));
    let compact = compact_tool_result("camera/snapshot", &original, 4096);
    assert!(compact.chars().count() <= 4096);
}

#[test]
fn a_one_line_capability_description_reaches_the_catalog_whole() {
    let description = "Take one RGB snapshot from the head camera.";
    let (summary, truncated) = summarize_description(description);
    assert_eq!(summary, description);
    assert!(!truncated);
}

#[test]
fn a_capability_manual_is_summarized_to_its_opening_paragraph() {
    // A provider that writes its request/response manual into `description`
    // would otherwise ship that manual on every planning call, for every
    // caller, whether or not anyone uses the capability.
    let description = format!(
        "Search memory using a 3-stage pipeline.\n\n\
         Request JSON schema:\n{}\n\nResponse JSON:\n{}",
        "x".repeat(1200),
        "y".repeat(1200)
    );
    let (summary, truncated) = summarize_description(&description);
    assert_eq!(summary, "Search memory using a 3-stage pipeline.");
    assert!(truncated);
}

#[test]
fn an_overlong_opening_paragraph_is_cut_on_a_character_boundary() {
    // The fixture must be multi-byte to prove the cut counts characters
    // rather than bytes: a provider writing its description in a non-Latin
    // script is the case that would panic if it did not.
    let description = "。".repeat(MAX_INLINE_DESCRIPTION_CHARS + 50); // i18n-ok
    let (summary, truncated) = summarize_description(&description);
    assert_eq!(summary.chars().count(), MAX_INLINE_DESCRIPTION_CHARS);
    assert!(truncated);
}

#[test]
fn oversized_projected_scalar_and_escaped_preview_stay_bounded() {
    let original = json!({
        "regions": [{
            "id": format!("scene.room.{}", "\\\"\n".repeat(3000)),
            "kind": "room",
            "name": "large",
        }],
        "map_id": "demo",
    })
    .to_string();
    let compact = compact_tool_result("robonix/system/scene/list_regions", &original, 4096);
    serde_json::from_str::<serde_json::Value>(&compact).unwrap();
    assert!(compact.chars().count() <= 4096);
}

#[test]
fn executor_snapshot_is_authoritative_across_turns() {
    let block = build_executor_active_block(Some(
        r#"{"count":99,"plans":[{"plan_id":"8","description":"greet","ops":[]}]}"#,
    ));
    assert!(block.contains("\"count\":1"));
    assert!(block.contains("\"plan_id\":\"8\""));
    assert!(block.contains("long-running skills started by earlier interactions"));
}

#[test]
fn unavailable_executor_snapshot_forbids_guessing_zero() {
    let block = build_executor_active_block(None);
    assert!(block.contains("status: unavailable"));
    assert!(block.contains("Never guess a task count"));
}

#[test]
fn only_requested_cancellation_replans_for_final_confirmation() {
    assert!(should_replan_after_plan_done(false, false, true, true));
    assert!(should_replan_after_plan_done(true, true, true, true));
    assert!(!should_replan_after_plan_done(true, true, false, true));
    assert!(!should_replan_after_plan_done(true, false, true, true));
    assert!(!should_replan_after_plan_done(true, true, true, false));
}

#[test]
fn a_meta_op_id_may_arrive_unquoted() {
    // JSON spells an identifier either 1 or "1", and Pilot puts it back on
    // the wire as a string either way. Rejecting the unquoted form cost
    // four of thirty-five planning rounds in one deepseek-v3.2 episode.
    let quoted = parse_meta_plan_op(&json!({
        "op": "stop_plan_at", "plan_id": "1", "target_op_id": "1",
    }))
    .unwrap();
    let unquoted = parse_meta_plan_op(&json!({
        "op": "stop_plan_at", "plan_id": 1, "target_op_id": 1,
    }))
    .unwrap();
    assert_eq!(quoted, unquoted);
}

#[test]
fn root_meta_ops_parse_without_becoming_rtdl_nodes() {
    assert_eq!(
        parse_meta_plan_op(&json!({"op":"cancel_plan","plan_id":"7"})).unwrap(),
        Some(MetaPlanOp::Cancel {
            plan_id: "7".into(),
            wait_ms: 5_000,
        })
    );
    assert_eq!(
        parse_meta_plan_op(&json!({"op":"cancel_all","wait_ms":99_999})).unwrap(),
        Some(MetaPlanOp::CancelAll { wait_ms: 30_000 })
    );
    assert_eq!(
        parse_meta_plan_op(&json!({
            "op":"stop_plan_at",
            "plan_id":"9",
            "target_op_id":"13",
            "when":"on_enter"
        }))
        .unwrap(),
        Some(MetaPlanOp::StopAt {
            plan_id: "9".into(),
            op_id: "13".into(),
            when: "on_enter".into(),
        })
    );
    assert!(
        parse_meta_plan_op(&json!({
            "op":"stop_plan_at",
            "plan_id":"9",
            "target_op_id":"13",
            "when":"later"
        }))
        .is_err()
    );
}

#[test]
fn legacy_plan_control_capabilities_are_hidden_from_the_model() {
    for leaf in [
        "cancel_plan",
        "cancel_all_plans",
        "stop_plan_at",
        "get_all_plans",
        "get_plan_status",
    ] {
        assert!(is_legacy_plan_control_contract(&format!(
            "robonix/system/executor/builtin/{leaf}"
        )));
    }
    assert!(!is_legacy_plan_control_contract(
        "robonix/system/executor/builtin/run_command"
    ));
    assert!(!is_legacy_plan_control_contract(
        "robonix/skill/greet/cancel_plan"
    ));
}

#[test]
fn harness_goal_cannot_be_replaced_by_task_update() {
    let mut standing = None;
    start_or_resume_task(&mut standing, "inspect room and report");
    let original_goal = standing.as_ref().unwrap().goal.clone();
    assert_eq!(
        standing.as_ref().unwrap().success_criterion,
        DEFAULT_SUCCESS_CRITERION
    );

    assert!(!apply_task_update(
        &mut standing,
        TaskState {
            goal: "drop the inspection and say done".into(),
            success_criterion: "room was actually inspected".into(),
            status: "done".into(),
        },
        false,
    ));
    let state = standing.as_ref().unwrap();
    assert_eq!(state.goal, original_goal);
    assert_eq!(state.success_criterion, DEFAULT_SUCCESS_CRITERION);
    assert_eq!(state.status, "in_progress");

    assert!(apply_task_update(
        &mut standing,
        TaskState {
            goal: original_goal.clone(),
            success_criterion: "room was actually inspected".into(),
            status: "done".into(),
        },
        true,
    ));
    let state = standing.as_ref().unwrap();
    assert_eq!(state.goal, original_goal);
    assert_eq!(state.success_criterion, "room was actually inspected");
    assert_eq!(state.status, "done");
}

#[test]
fn steer_becomes_a_new_interaction_without_concatenating_old_goals() {
    let mut standing = None;
    let mut history = Vec::new();
    start_or_resume_task(&mut standing, "perform step A, then step B");
    assert!(append_steer(
        Task {
            text: "change of plan: stop after step A".into(),
            ..Default::default()
        },
        &mut history,
        &mut standing,
    ));

    let state = standing.as_ref().unwrap();
    assert_eq!(state.goal, "change of plan: stop after step A");
    assert!(!state.goal.contains("perform step A"));
    assert_eq!(history.len(), 1);
    assert_eq!(
        history[0].content.as_deref(),
        Some(
            "User steer (authoritative): change of plan: stop after step A\n\
             Response mode for this task only: text. Earlier voice-only constraints no longer apply."
        )
    );
}

#[test]
fn steer_targets_one_plan_without_discarding_independent_work() {
    let mut standing = None;
    let mut history = Vec::new();
    start_or_resume_task(&mut standing, "go to the meeting room");

    let mut forest = HashMap::new();
    for (plan_id, description, capability) in [
        ("11", "navigate to the meeting room", "navigation_navigate"),
        ("5", "watch for passersby", "greet_greet"),
    ] {
        forest.insert(
            plan_id.to_string(),
            TreeMeta {
                description: description.into(),
                control_only: false,
                call_signatures: HashSet::new(),
                steps: vec![TreeStep {
                    op_id: format!("op-{plan_id}"),
                    description: description.into(),
                    capability: capability.into(),
                }],
            },
        );
    }

    assert!(append_steer(
        Task {
            text: "cancel the meeting-room trip and return to room 315".into(),
            ..Default::default()
        },
        &mut history,
        &mut standing,
    ));
    assert_eq!(
        standing.as_ref().unwrap().goal,
        "cancel the meeting-room trip and return to room 315"
    );

    let prompt = build_forest_block(&forest, &HashSet::new());
    assert!(prompt.contains("plan_id=11"));
    assert!(prompt.contains("plan_id=5"));
    assert!(prompt.contains("navigate to the meeting room"));
    assert!(prompt.contains("watch for passersby"));
    assert_eq!(
        invalid_cancel_target(&["11".into()], &forest, &HashSet::new()),
        None
    );
    assert_eq!(
        invalid_cancel_target(&["5".into()], &forest, &HashSet::new()),
        None
    );

    let requested = HashSet::from(["11".to_string()]);
    assert_eq!(
        invalid_cancel_target(&["11".into()], &forest, &requested),
        Some("11".to_string())
    );
    assert_eq!(
        invalid_cancel_target(&["5".into()], &forest, &requested),
        None
    );
}

#[test]
fn forest_prompt_distinguishes_immediate_cancel_from_boundary_stop() {
    let mut forest = HashMap::new();
    forest.insert(
        "4".to_string(),
        TreeMeta {
            description: "ordered multi-step task".into(),
            control_only: false,
            call_signatures: HashSet::new(),
            steps: vec![
                TreeStep {
                    op_id: "op-restaurant".into(),
                    description: "move to restaurant".into(),
                    capability: "navigate".into(),
                },
                TreeStep {
                    op_id: "op-meeting".into(),
                    description: "move to meeting room".into(),
                    capability: "navigate".into(),
                },
            ],
        },
    );
    let prompt = build_forest_block(&forest, &HashSet::new());
    assert!(prompt.contains("op_id=op-restaurant"));
    assert!(prompt.contains("move to meeting room"));
    assert!(prompt.contains("target is the currently running step"));
    assert!(prompt.contains("do not query status"));
    assert!(prompt.contains("on_complete"));
    assert!(prompt.contains("on_enter"));
}

#[test]
fn executor_feedback_is_scoped_to_its_independent_tree() {
    let mut history = Vec::new();
    feed_results_into_history(
        &mut history,
        "9",
        "start greet watch",
        &[CapabilityCallResult {
            call_id: "9:0".into(),
            contract_id: "robonix/skill/greet/greet".into(),
            success: false,
            error: "activation failed".into(),
            ..Default::default()
        }],
    );
    let scope = history[0].content.as_deref().unwrap_or_default();
    assert!(scope.contains("plan_id=9"));
    assert!(scope.contains("start greet watch"));
    assert!(scope.contains("does not cancel or invalidate other in-flight trees"));
}

#[test]
fn completed_plan_context_preserves_original_call_before_replanning() {
    let original_target = 1.4430711285352669;
    let plan = Plan {
        plan_id: "5".into(),
        nodes: vec![RtdlNode {
            node_kind: RTDL_DO,
            op_id: "6".into(),
            description: "navigate to the original one-metre target".into(),
            call: Some(CapabilityCall {
                call_id: "5:0".into(),
                provider_id: "nav2".into(),
                contract_id: "robonix/service/navigation/navigate".into(),
                args_json: json!({
                    "goal": {
                        "header": {"frame_id": "map"},
                        "pose": {"position": {"x": original_target, "y": -0.0019468723}}
                    }
                })
                .to_string(),
            }),
            ..Default::default()
        }],
        ..Default::default()
    };
    let mut history = Vec::new();
    record_dispatched_plan(&mut history, &plan, "move forward one metre");
    feed_results_into_history(
        &mut history,
        "5",
        "move forward one metre",
        &[CapabilityCallResult {
            call_id: "5:0".into(),
            contract_id: "robonix/service/navigation/navigate".into(),
            success: true,
            output: r#"{"state":"SUCCEEDED","detail":"last_pose=(1.160,-0.050)"}"#.into(),
            ..Default::default()
        }],
    );

    let visible = crate::history::sanitize_for_vlm(&history);
    let context = visible
        .iter()
        .filter_map(|message| message.content.as_deref())
        .collect::<Vec<_>>()
        .join("\n");
    assert!(context.contains("Pilot harness dispatch record"));
    assert!(context.contains("\"plan_id\":\"5\""));
    assert!(context.contains("\"call_id\":\"5:0\""));
    assert!(context.contains(&original_target.to_string()));
    assert!(context.contains("SUCCEEDED"));
}

#[test]
fn duplicate_in_flight_calls_are_detected_by_canonical_signature() {
    let plan = Plan {
        plan_id: "2".into(),
        nodes: vec![RtdlNode {
            node_kind: RTDL_DO,
            call: Some(CapabilityCall {
                provider_id: "executor".into(),
                contract_id: "test/run".into(),
                args_json: r#"{"b":2,"a":1}"#.into(),
                ..Default::default()
            }),
            ..Default::default()
        }],
        ..Default::default()
    };
    let signatures = plan_call_signatures(&plan);
    let mut forest = HashMap::new();
    forest.insert(
        "1".to_string(),
        TreeMeta {
            description: "same call".into(),
            control_only: false,
            call_signatures: signatures.clone(),
            steps: Vec::new(),
        },
    );
    assert!(duplicate_in_flight_signature(&signatures, &forest).is_some());
}

#[test]
fn inspection_result_must_arrive_before_new_action_is_admitted() {
    let plan = Plan {
        nodes: vec![
            RtdlNode {
                node_kind: RTDL_DO,
                call: Some(CapabilityCall {
                    contract_id: "robonix/system/executor/builtin/get_plan_status".into(),
                    ..Default::default()
                }),
                ..Default::default()
            },
            RtdlNode {
                node_kind: RTDL_DO,
                call: Some(CapabilityCall {
                    contract_id: "robonix/system/executor/builtin/run_command".into(),
                    ..Default::default()
                }),
                ..Default::default()
            },
        ],
        ..Default::default()
    };
    assert!(mixes_control_inspection_with_action(&plan));

    let inspection_only = Plan {
        nodes: vec![plan.nodes[0].clone()],
        ..Default::default()
    };
    assert!(!mixes_control_inspection_with_action(&inspection_only));
}

#[test]
fn cancel_target_must_be_live_and_not_already_requested() {
    let mut forest = HashMap::new();
    forest.insert(
        "7".to_string(),
        TreeMeta {
            description: "drive".into(),
            control_only: false,
            call_signatures: HashSet::new(),
            steps: Vec::new(),
        },
    );
    let targets = vec!["7".to_string()];
    assert!(invalid_cancel_target(&targets, &forest, &HashSet::new()).is_none());
    assert_eq!(
        invalid_cancel_target(&targets, &forest, &HashSet::from(["7".to_string()])),
        Some("7".to_string())
    );
    assert_eq!(
        invalid_cancel_target(&["8".to_string()], &forest, &HashSet::new()),
        Some("8".to_string())
    );
}

#[test]
fn rtdl_recovery_final_text_hides_internal_error() {
    let text = rtdl_recovery_final_text();
    assert!(text.contains("valid robot plan"));
    assert!(!text.contains("expand RTDL"));
    assert!(!text.contains("capability call"));
    assert!(!text.contains("assistant content preview"));
}

fn task(ctx: &str) -> Task {
    Task {
        task_id: "t".into(),
        session_id: "s".into(),
        source: 0,
        text: String::new(),
        audio_data: Vec::new(),
        context_json: ctx.into(),
        timestamp_ms: 0,
    }
}

#[test]
fn session_end_explicit() {
    assert!(task_is_session_end(&task(r#"{"session_end":true}"#)));
}

#[test]
fn session_end_legacy_alias() {
    assert!(task_is_session_end(&task(
        r#"{"robonix_session_end":true}"#
    )));
}

#[test]
fn session_end_false_or_absent() {
    assert!(!task_is_session_end(&task("")));
    assert!(!task_is_session_end(&task(r#"{"foo":1}"#)));
    assert!(!task_is_session_end(&task(r#"{"session_end":false}"#)));
}

#[test]
fn skip_prefetch_chitchat() {
    assert!(skip_memory_prefetch("hi"));
    assert!(skip_memory_prefetch("Hello"));
}

#[test]
fn no_skip_prefetch_real_query() {
    assert!(!skip_memory_prefetch("open the door"));
    assert!(!skip_memory_prefetch("find me a red cup"));
}

fn single_do_plan(contract_leaf: &str) -> Plan {
    Plan {
        plan_id: "p".into(),
        session_id: "s".into(),
        round: 0,
        root_index: 0,
        nodes: vec![RtdlNode {
            node_kind: RTDL_DO,
            children: vec![],
            call: Some(CapabilityCall {
                call_id: "p:0".into(),
                provider_id: "executor".into(),
                contract_id: format!("robonix/system/executor/builtin/{contract_leaf}"),
                args_json: "{}".into(),
            }),
            op_id: "op_1".into(),
            description: "control action".into(),
        }],
    }
}

#[test]
fn plan_control_builtins_are_control_only() {
    for leaf in [
        "cancel_plan",
        "cancel_all_plans",
        "get_all_plans",
        "get_plan_status",
        "stop_plan_at",
    ] {
        assert!(is_control_only(&single_do_plan(leaf)), "{leaf}");
    }
    assert!(!is_control_only(&single_do_plan("list_dir")));
}

#[test]
fn rtdl_state_names_are_human_readable() {
    assert_eq!(rtdl_state_name(0), "Pending");
    assert_eq!(rtdl_state_name(2), "Succeeded");
    assert_eq!(rtdl_state_name(3), "Failed");
    assert_eq!(rtdl_state_name(4), "Canceled");
    assert_eq!(rtdl_state_name(5), "Timeout");
    assert_eq!(rtdl_state_name(7), "Verifying");
    assert_eq!(rtdl_state_name(999), "Unknown(999)");
    assert!(!is_terminal_executor_state(
        RtdlNodeStateEnum::Verifying as u32
    ));
}

#[test]
fn rtdl_node_kind_names_are_human_readable() {
    assert_eq!(rtdl_node_kind_name(RTDL_SEQUENCE), "sequence");
    assert_eq!(rtdl_node_kind_name(RTDL_PARALLEL), "parallel");
    assert_eq!(rtdl_node_kind_name(RTDL_DO), "do");
    assert_eq!(rtdl_node_kind_name(99), "unknown(99)");
}

#[test]
fn rtdl_response_requires_exact_top_level_keys() {
    // Old two-key envelope is now rejected.
    let err =
        parse_rtdl_assistant_response(r#"{"content":"x","rtdl":{"op":"sequence","children":[]}}"#)
            .unwrap_err();
    assert!(
        err.to_string()
            .contains("exactly `content`, `rtdl_description`, `rtdl`, and `task_update`")
    );
}

#[test]
fn rtdl_response_parses_full_envelope() {
    let env = parse_rtdl_assistant_response(
        r#"{
            "content":"on it",
            "rtdl_description":"fetch water",
            "rtdl":{"op":"sequence","children":[]},
            "task_update":{"goal":"bring water","success_criterion":"cup by user","status":"in_progress"}
        }"#,
    )
    .unwrap();
    assert_eq!(env.content, "on it");
    assert_eq!(env.rtdl_description, "fetch water");
    assert!(env.rtdl.is_object());
    assert_eq!(
        env.task_update,
        Some(TaskState {
            goal: "bring water".into(),
            success_criterion: "cup by user".into(),
            status: "in_progress".into(),
        })
    );
}

#[test]
fn rtdl_response_tolerates_prose_preamble() {
    // Observed real failure: the model narrates a line, then emits the JSON
    // on the next line. The leading prose must be stripped, not rejected.
    let env = parse_rtdl_assistant_response(
        "Let me take a photo to check the scene, then turn left.\n{\"content\":\"on it\",\"rtdl_description\":\"turn\",\"rtdl\":{\"op\":\"sequence\",\"children\":[]},\"task_update\":null}",
    )
    .unwrap();
    assert_eq!(env.content, "on it");
    assert!(env.task_update.is_none());
}

#[test]
fn extract_json_object_skips_prose_and_braces_in_strings() {
    // Leading prose dropped; a `}` inside a string value does not end it.
    let got = extract_json_object("hi: {\"a\":\"x}y\",\"b\":1} trailing");
    assert_eq!(got, Some("{\"a\":\"x}y\",\"b\":1}"));
    // No object at all → None, so the caller still hits the real parse error.
    assert_eq!(extract_json_object("no json here"), None);
}

#[test]
fn rtdl_response_task_update_null_is_none() {
    let env = parse_rtdl_assistant_response(
        r#"{"content":"x","rtdl_description":"","rtdl":{"op":"sequence","children":[]},"task_update":null}"#,
    )
    .unwrap();
    assert!(env.task_update.is_none());
}

#[test]
fn task_update_rejects_unknown_status() {
    let err = parse_task_update(&json!({
        "goal":"g","success_criterion":"c","status":"paused"
    }))
    .unwrap_err();
    assert!(err.to_string().contains("status"));
}

#[test]
fn task_update_rejects_missing_field() {
    let err = parse_task_update(&json!({ "goal":"g","status":"done" })).unwrap_err();
    assert!(err.to_string().contains("exactly"));
}

#[test]
fn rtdl_expands_sequence_to_plan_calls() {
    let mut targets = CapabilityTargetMap::new();
    targets.insert(
        "camera_snapshot".to_string(),
        (
            "cap-camera".to_string(),
            "robonix/primitive/camera/snapshot".to_string(),
        ),
    );
    targets.insert(
        "chassis_move".to_string(),
        (
            "cap-chassis".to_string(),
            "robonix/primitive/chassis/move".to_string(),
        ),
    );

    let rtdl = json!({
        "op": "sequence",
        "children": [
            { "op": "do", "cap": "camera_snapshot", "args": {} },
            { "op": "do", "cap": "chassis_move", "args": { "linear": 0.1 } }
        ]
    });
    let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 7, "").unwrap();

    assert_eq!(plan.plan_id, "p");
    assert_eq!(plan.session_id, "s");
    assert_eq!(plan.round, 7);
    assert_eq!(plan.nodes.len(), 3);
    assert_eq!(plan.root_index, 0);
    assert_eq!(plan.nodes[0].node_kind, RTDL_SEQUENCE);
    assert_eq!(plan.nodes[0].children, vec![1, 2]);
    let first = plan.nodes[1].call.as_ref().unwrap();
    let second = plan.nodes[2].call.as_ref().unwrap();
    assert_eq!(plan.nodes[1].node_kind, RTDL_DO);
    assert_eq!(first.call_id, "p:0");
    assert_eq!(first.provider_id, "cap-camera");
    assert_eq!(first.contract_id, "robonix/primitive/camera/snapshot");
    assert_eq!(first.args_json, "{}");
    assert_eq!(second.call_id, "p:1");
    assert_eq!(second.args_json, r#"{"linear":0.1}"#);
}

#[test]
fn rtdl_expands_parallel_root() {
    let mut targets = CapabilityTargetMap::new();
    targets.insert(
        "camera_snapshot".to_string(),
        (
            "cap-camera".to_string(),
            "robonix/primitive/camera/snapshot".to_string(),
        ),
    );
    targets.insert(
        "read_temp".to_string(),
        (
            "cap-temp".to_string(),
            "robonix/primitive/sensor/temp".to_string(),
        ),
    );

    let rtdl = json!({
        "op": "parallel",
        "children": [
            { "op": "do", "cap": "camera_snapshot", "args": {} },
            { "op": "do", "cap": "read_temp", "args": { "unit": "c" } }
        ]
    });
    let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 1, "").unwrap();

    assert_eq!(plan.root_index, 0);
    assert_eq!(plan.nodes[0].node_kind, RTDL_PARALLEL);
    assert_eq!(plan.nodes[0].children, vec![1, 2]);
    assert_eq!(plan.nodes[1].call.as_ref().unwrap().call_id, "p:0");
    assert_eq!(plan.nodes[2].call.as_ref().unwrap().call_id, "p:1");
}

#[test]
fn format_plan_summary_uses_tree_shape_and_compact_cap_names() {
    let mut targets = CapabilityTargetMap::new();
    targets.insert(
        "nav2.navigation_status".to_string(),
        (
            "nav2".to_string(),
            "robonix/service/navigation/status".to_string(),
        ),
    );
    let rtdl = json!({
        "op": "sequence",
        "description": "poll navigation status",
        "children": [
            {
                "op": "do",
                "description": "check current navigation goal",
                "cap": "nav2.navigation_status",
                "args": { "goal_id": "" }
            }
        ]
    });
    let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 1, "").unwrap();
    let summary = format_plan_summary(&plan).join("\n");

    assert!(summary.contains("[0] sequence"));
    assert!(summary.contains("[1] do"));
    assert!(summary.contains("cap=nav2.navigation_status"));
    assert!(summary.contains(r#"args={"goal_id":""}"#));
    assert!(summary.contains("  [1] do"));
    assert!(!summary.contains("kind="));
    assert!(!summary.contains("state="));
    assert!(!summary.contains("children"));
    assert!(!summary.contains("robonix/service/navigation/status"));
    assert!(!summary.contains("call_id"));
}

#[test]
fn rtdl_nested_call_ids_follow_json_traversal_order() {
    let mut targets = CapabilityTargetMap::new();
    for name in ["a", "b", "c"] {
        targets.insert(
            name.to_string(),
            (format!("provider-{name}"), format!("robonix/test/{name}")),
        );
    }

    let rtdl = json!({
        "op": "sequence",
        "children": [
            { "op": "do", "cap": "a", "args": {} },
            {
                "op": "parallel",
                "children": [
                    { "op": "do", "cap": "b", "args": {} },
                    { "op": "do", "cap": "c", "args": {} }
                ]
            }
        ]
    });
    let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 1, "").unwrap();
    let calls: Vec<_> = plan
        .nodes
        .iter()
        .filter_map(|node| node.call.as_ref())
        .map(|call| call.call_id.as_str())
        .collect();
    assert_eq!(calls, vec!["p:0", "p:1", "p:2"]);
}

#[test]
fn rtdl_empty_sequence_generates_root_node() {
    let targets = CapabilityTargetMap::new();
    let rtdl = json!({ "op": "sequence", "children": [] });
    let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0, "").unwrap();

    assert_eq!(plan.root_index, 0);
    assert_eq!(plan.nodes.len(), 1);
    assert_eq!(plan.nodes[0].node_kind, RTDL_SEQUENCE);
    assert!(plan.nodes[0].children.is_empty());
}

#[test]
fn rtdl_rejects_out_field() {
    let mut targets = CapabilityTargetMap::new();
    targets.insert(
        "camera_snapshot".to_string(),
        (
            "cap-camera".to_string(),
            "robonix/primitive/camera/snapshot".to_string(),
        ),
    );
    let rtdl = json!({
        "op": "do",
        "cap": "camera_snapshot",
        "args": {},
        "out": { "image": "img" }
    });
    let err = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0, "").unwrap_err();
    assert!(err.to_string().contains("unexpected field `out`"));
}

#[test]
fn rtdl_uses_model_node_description_over_synthesized() {
    let mut targets = CapabilityTargetMap::new();
    targets.insert(
        "camera_snapshot".to_string(),
        (
            "cap-camera".to_string(),
            "robonix/primitive/camera/snapshot".to_string(),
        ),
    );
    // Every node carries op_id (always 0 — pilot reassigns) plus a
    // model-authored node-level description.
    let rtdl = json!({
        "op": "sequence",
        "op_id": 0,
        "description": "inspect the doorway",
        "children": [
            {
                "op": "do",
                "op_id": 0,
                "description": "take a camera snapshot of the door",
                "cap": "camera_snapshot",
                "args": {}
            }
        ]
    });
    let plan = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0, "").unwrap();
    assert_eq!(plan.nodes[0].description, "inspect the doorway");
    assert_eq!(
        plan.nodes[1].description,
        "take a camera snapshot of the door"
    );
    // The model's op_id=0 is ignored; pilot assigns non-empty unique ids.
    assert!(!plan.nodes[0].op_id.is_empty());
    assert_ne!(plan.nodes[0].op_id, plan.nodes[1].op_id);
}

#[test]
fn rtdl_rejects_parallel_non_array_children() {
    let targets = CapabilityTargetMap::new();
    let rtdl = json!({ "op": "parallel", "children": {} });
    let err = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0, "").unwrap_err();
    assert!(err.to_string().contains("children must be an array"));
}

#[test]
fn rtdl_rejects_do_non_object_args() {
    let mut targets = CapabilityTargetMap::new();
    targets.insert(
        "camera_snapshot".to_string(),
        (
            "cap-camera".to_string(),
            "robonix/primitive/camera/snapshot".to_string(),
        ),
    );
    let rtdl = json!({ "op": "do", "cap": "camera_snapshot", "args": [] });
    let err = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0, "").unwrap_err();
    assert!(err.to_string().contains("args must be an object"));
}

#[test]
fn rtdl_rejects_unknown_op() {
    let targets = CapabilityTargetMap::new();
    let rtdl = json!({ "op": "race", "children": [] });
    let err = expand_rtdl_to_plan(&rtdl, &targets, "p".into(), "s".into(), 0, "").unwrap_err();
    assert!(err.to_string().contains("unknown operator"));
}

/// Request history preserves corrections, authoritative state, and prefix bytes.
#[test]
fn planning_requests_preserve_context_and_extend_the_provider_prefix() {
    let stable = [("system", "standing system and RTDL contract")];
    let live = [("executor_state", "runtime round 0"), ("voice", "")];
    let mut history = vec![Message::user("User task (authoritative): inspect room")];
    append_request_context(
        &mut history,
        &crate::prompt::render_context_sections(&live),
        Some("Pilot validation feedback: emit valid RTDL"),
    );
    let first = assemble_planning_messages(0, true, &stable, &history);
    assert_eq!(
        first
            .iter()
            .map(|m| (m.role.as_str(), m.content.as_deref().unwrap()))
            .collect::<Vec<_>>(),
        vec![
            ("system", "standing system and RTDL contract"),
            ("user", "User task (authoritative): inspect room"),
            ("user", "runtime round 0"),
            ("user", "Pilot validation feedback: emit valid RTDL"),
        ],
    );

    history.push(Message::assistant("I will inspect the room."));
    append_task_state_record(
        &mut history,
        &TaskState {
            goal: "inspect room".to_string(),
            success_criterion: "inspection result returned".to_string(),
            status: "in_progress".to_string(),
        },
    );
    let record = history.last().unwrap();
    assert_eq!(record.role, "user");
    let record = record.content.as_deref().unwrap();
    let (_, json) = record.split_once(": ").unwrap();
    assert_eq!(
        serde_json::from_str::<serde_json::Value>(json).unwrap(),
        json!({
            "goal": "inspect room",
            "success_criterion": "inspection result returned",
            "status": "in_progress",
        })
    );
    append_request_context(&mut history, "runtime round 1", None);
    let second = assemble_planning_messages(1, true, &stable, &history);
    assert_eq!(
        serde_json::to_value(&first).unwrap(),
        serde_json::to_value(&second[..first.len()]).unwrap(),
    );
}

/// Empty context adds no turn; a trailing assistant requires a user continuation.
#[test]
fn planning_requests_close_only_a_trailing_assistant() {
    for message in [Message::user("do the task"), Message::assistant("thinking")] {
        let mut history = vec![message];
        append_request_context(&mut history, "", None);
        assert_eq!(history.len(), 1);
        let messages = assemble_planning_messages(1, true, &[], &history);
        let needs_continue = history[0].role == "assistant";
        assert_eq!(messages.len(), if needs_continue { 3 } else { 2 });
        assert_eq!(messages.last().unwrap().role, "user");
        if needs_continue {
            assert!(
                messages
                    .last()
                    .unwrap()
                    .content
                    .as_deref()
                    .unwrap()
                    .starts_with("Continue from the state above")
            );
        } else {
            assert_eq!(messages[1].content, history[0].content);
        }
    }
}
