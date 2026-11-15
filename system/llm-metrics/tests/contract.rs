// SPDX-License-Identifier: MulanPSL-2.0
//! Runs the Python package's usage cases and checks the written record.

use std::path::Path;

use robonix_llm_metrics::{Call, Status, Usage};
use serde_json::Value;

/// The cases live in the Python package so both writers read one file; the
/// crate is therefore testable inside the monorepo only.
#[test]
fn usage_matches_the_shared_cases() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../pylib/robonix-api/robonix_api/llm/contract/usage_cases.json");
    let fixture: Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    for case in fixture["cases"].as_array().unwrap() {
        let usage = serde_json::to_value(Usage::from_response(&case["body"])).unwrap();
        assert_eq!(usage, case["expected"], "{}", case["name"]);
    }
}

#[test]
fn a_call_writes_one_record_with_the_last_reported_usage() {
    let directory = tempfile::tempdir().unwrap();
    let path = Some(directory.path());
    {
        let mut call = Call::start(
            path,
            "pilot",
            "plan",
            "https://k@api.example.com/v1",
            "m",
            true,
        );
        call.retry();
        call.observe(&serde_json::json!({"model": "m-1", "choices": []}));
        call.observe(&serde_json::json!({"usage": {"prompt_tokens": 9, "completion_tokens": 3}}));
        call.finish(Status::Ok, Some(200));
    }
    drop(Call::start(path, "pilot", "summary", "", "m", true));
    drop(Call::start(None, "pilot", "unrecorded", "", "m", true));
    let text = std::fs::read_to_string(
        directory
            .path()
            .join(format!("pilot-{}.jsonl", std::process::id())),
    )
    .unwrap();
    let records: Vec<Value> = text
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();
    assert_eq!(records.len(), 2);
    let first = &records[0];
    assert_eq!(first["host"], "api.example.com");
    assert_eq!(
        (first["status"].as_str(), first["attempts"].as_u64()),
        (Some("ok"), Some(2))
    );
    assert_eq!(
        (
            first["input_tokens"].as_u64(),
            first["output_tokens"].as_u64()
        ),
        (Some(9), Some(3))
    );
    assert_eq!(first["provider_model"], "m-1");
    assert_eq!(records[1]["status"], "cancelled");
}
