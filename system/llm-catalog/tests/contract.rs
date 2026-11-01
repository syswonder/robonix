// SPDX-License-Identifier: MulanPSL-2.0
//! Runs the same resolve cases as the Python package.

use std::path::Path;

use robonix_llm_catalog::{ModelCatalog, Requirements};
use serde::Deserialize;

/// Shared contract file: one catalog and the cases that exercise it.
#[derive(Debug, Deserialize)]
struct Fixture {
    /// ModelCatalog used by every case in the file.
    catalog: ModelCatalog,
    /// Ordered cases with their expected results.
    cases: Vec<Case>,
}

/// One named resolve case from the shared contract file.
#[derive(Debug, Deserialize)]
struct Case {
    /// Human-readable name used in assertion messages.
    name: String,
    /// Required features for this case.
    requirements: CaseRequirements,
    /// Expected model ids in preference order.
    expected_ids: Vec<String>,
}

/// Request requirements as they appear in the JSON contract file.
#[derive(Debug, Deserialize)]
struct CaseRequirements {
    /// Features the model must support.
    #[serde(default)]
    features: Vec<String>,
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
            features: case.requirements.features.clone(),
        };
        let targets = fixture.catalog.resolve(&requirements);
        let ids: Vec<String> = targets.iter().map(|model| model.id.clone()).collect();
        assert_eq!(ids, case.expected_ids, "case {:?}", case.name);
    }
}

#[test]
fn loads_the_example_yaml_catalog() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../pylib/robonix-api/robonix_api/llm/catalog.example.yaml");
    let catalog = ModelCatalog::load(&path).expect("load catalog.example.yaml");
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
    let catalog = ModelCatalog::load(&path).expect("load catalog.example.yaml");
    let deepseek = catalog.model("deepseek/flash").expect("deepseek entry");
    assert_eq!(deepseek.provider, "deepseek");
    assert!(catalog.model("absent").is_err());
}

#[test]
fn rejects_invalid_api_key_env_names() {
    let path = std::env::temp_dir()
        .join(format!("catalog-key-names-{}.json", std::process::id()));
    for name in [
        "",
        " KEY ",
        "1KEY",
        "KEY,OTHER",
        "KEY=value",
        "KEY[0]",
        "BASH_VERSINFO[$(printf${IFS}UNSAFE>&2)0]",
    ] {
        let data = serde_json::json!({"models": [{
            "id": "local/test", "provider": "vllm",
            "base_url": "http://127.0.0.1:8000/v1", "model": "local",
            "api_key_env": name,
        }]});
        std::fs::write(&path, data.to_string()).expect("write catalog");
        let result = ModelCatalog::load(&path);
        std::fs::remove_file(&path).expect("remove catalog");
        assert!(result.is_err(), "accepted key variable {name:?}");
    }
}

#[test]
fn shared_validation_cases() {
    let cases_path = contract_path().with_file_name("validation_cases.json");
    let text = std::fs::read_to_string(cases_path).expect("read validation cases");
    let cases: serde_json::Value = serde_json::from_str(&text).unwrap();
    let path = std::env::temp_dir()
        .join(format!("catalog-validation-{}.yaml", std::process::id()));
    let write_model = |model| {
        std::fs::write(&path, serde_json::json!({"models": [model]}).to_string()).unwrap();
    };
    write_model(cases["model"].clone());
    let catalog = ModelCatalog::load(&path).expect("normalized catalog");
    let model = serde_json::to_value(&catalog.models[0]).unwrap();
    for (key, value) in cases["normalized"].as_object().unwrap() {
        assert_eq!(&model[key], value, "normalized {key}");
    }
    for overrides in cases["invalid"].as_array().unwrap() {
        let mut model = cases["model"].clone();
        model
            .as_object_mut()
            .unwrap()
            .extend(overrides.as_object().unwrap().clone());
        write_model(model);
        assert!(ModelCatalog::load(&path).is_err(), "accepted {overrides}");
    }
    for field in [
        "latency_p95_ms",
        "price_input_per_mtok",
        "price_output_per_mtok",
    ] {
        for value in cases["non_finite"].as_array().unwrap() {
            let mut model = cases["model"].clone();
            model[field] = "NON_FINITE".into();
            let text = serde_json::json!({"models": [model]})
                .to_string()
                .replace("\"NON_FINITE\"", value.as_str().unwrap());
            std::fs::write(&path, text).unwrap();
            assert!(ModelCatalog::load(&path).is_err(), "accepted {field}={value}");
        }
    }
    std::fs::remove_file(path).unwrap();
}
