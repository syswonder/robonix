// SPDX-License-Identifier: MulanPSL-2.0
//! Runs the same resolve cases as the Python package.

use std::path::Path;

use robonix_llm_catalog::{Catalog, Requirements};
use serde::Deserialize;

/// Shared contract file: one catalog and the cases that exercise it.
#[derive(Debug, Deserialize)]
struct Fixture {
    /// Catalog used by every case in the file.
    catalog: Catalog,
    /// Ordered cases with their expected results.
    cases: Vec<Case>,
}

/// One named resolve case from the shared contract file.
#[derive(Debug, Deserialize)]
struct Case {
    /// Human-readable name used in assertion messages.
    name: String,
    /// Capability and budget inputs for this case.
    requirements: CaseRequirements,
    /// Expected model ids in preference order.
    expected_ids: Vec<String>,
    /// Whether the first result must carry the downgrade flag.
    #[serde(default)]
    expected_downgraded: bool,
}

/// Request requirements as they appear in the JSON contract file.
#[derive(Debug, Deserialize)]
struct CaseRequirements {
    /// Capabilities the model must support.
    #[serde(default)]
    capabilities: Vec<String>,
    /// Estimated input tokens.
    #[serde(default)]
    input_tokens: u64,
    /// Maximum tokens the model may generate.
    #[serde(default)]
    max_output_tokens: u64,
    /// Optional p95 latency budget.
    #[serde(default)]
    max_latency_ms: Option<f64>,
    /// Optional per-call cost budget.
    #[serde(default)]
    max_cost_usd_per_call: Option<f64>,
    /// Whether schema downgrade is allowed.
    #[serde(default = "default_true")]
    allow_downgrade: bool,
}

fn default_true() -> bool {
    true
}

fn contract_path() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../pylib/robonix-api/robonix_api/llm/contract/resolve_cases.json")
}

#[test]
fn shared_resolve_cases() {
    let text = std::fs::read_to_string(contract_path()).expect("read resolve_cases.json");
    let fixture: Fixture = serde_json::from_str(&text).expect("parse resolve_cases.json");
    for case in &fixture.cases {
        let requirements = Requirements {
            capabilities: case.requirements.capabilities.clone(),
            input_tokens: case.requirements.input_tokens,
            max_output_tokens: case.requirements.max_output_tokens,
            max_latency_ms: case.requirements.max_latency_ms,
            max_cost_usd_per_call: case.requirements.max_cost_usd_per_call,
            allow_downgrade: case.requirements.allow_downgrade,
        };
        let targets = fixture.catalog.resolve(&requirements);
        let ids: Vec<String> = targets.iter().map(|model| model.id.clone()).collect();
        assert_eq!(ids, case.expected_ids, "case {:?}", case.name);
        if let Some(first) = targets.first() {
            assert_eq!(
                first.downgraded, case.expected_downgraded,
                "case {:?} downgraded flag",
                case.name
            );
        }
    }
}

#[test]
fn loads_the_example_yaml_catalog() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../pylib/robonix-api/robonix_api/llm/catalog.example.yaml");
    let catalog = Catalog::load(&path).expect("load catalog.example.yaml");
    assert_eq!(catalog.models.len(), 3);
    let deepseek = catalog
        .models
        .iter()
        .find(|model| model.id == "deepseek/flash")
        .expect("deepseek entry");
    assert_eq!(deepseek.base_url, "https://api.deepseek.com");
    assert!(deepseek.supports("vision"));
    assert!(!deepseek.supports("json_schema"));
}

#[test]
fn looks_up_a_model_by_id() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../pylib/robonix-api/robonix_api/llm/catalog.example.yaml");
    let catalog = Catalog::load(&path).expect("load catalog.example.yaml");
    let deepseek = catalog.model("deepseek/flash").expect("deepseek entry");
    assert_eq!(deepseek.provider, "deepseek");
    assert!(catalog.model("absent").is_err());
}
