// SPDX-License-Identifier: MulanPSL-2.0
//! Environment-driven resolution: keyless local targets and missing keys.

use std::path::PathBuf;
use std::sync::Mutex;

use robonix_llm_catalog::{Requirements, api_key_for, resolve_from_env};

/// Env mutation is process-global; serialize the tests in this file.
static ENV_LOCK: Mutex<()> = Mutex::new(());

fn catalog_path(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!(
        "robonix-llm-catalog-{}-{name}.json",
        std::process::id()
    ))
}

fn with_catalog<T>(name: &str, body: &str, f: impl FnOnce() -> T) -> T {
    let _guard = ENV_LOCK.lock().expect("env lock");
    let path = catalog_path(name);
    std::fs::write(&path, body).expect("write catalog");
    unsafe {
        std::env::set_var("ROBONIX_LLM_CATALOG", &path);
        std::env::remove_var("FAST_KEY");
        std::env::remove_var("SLOW_KEY");
    }
    let result = f();
    unsafe {
        std::env::remove_var("ROBONIX_LLM_CATALOG");
    }
    let _ = std::fs::remove_file(&path);
    result
}

#[test]
fn keyless_target_wins_on_latency() {
    let body = r#"{
      "models": [
        {"id": "local/keyless", "provider": "vllm", "base_url": "http://127.0.0.1:8000/v1",
         "model": "local", "latency_p95_ms": 100, "features": {"vision": true}},
        {"id": "cloud/keyed", "provider": "test", "base_url": "https://api.example/v1",
         "model": "cloud", "api_key_env": "SLOW_KEY", "latency_p95_ms": 2000,
         "features": {"vision": true}}
      ]
    }"#;
    with_catalog("keyless-wins", body, || {
        unsafe { std::env::set_var("SLOW_KEY", "secret") };
        let target = resolve_from_env(&Requirements {
            features: vec!["vision".into()],
        })
        .expect("resolve")
        .expect("catalog enabled");
        assert_eq!(target.id, "local/keyless");
        assert_eq!(api_key_for(&target), "");
    });
}

#[test]
fn missing_keys_are_an_error() {
    let body = r#"{
      "models": [
        {"id": "fast/model", "provider": "test", "base_url": "https://api.example/v1",
         "model": "fast", "api_key_env": "FAST_KEY", "latency_p95_ms": 100,
         "features": {"vision": true}},
        {"id": "slow/model", "provider": "test", "base_url": "https://api.example/v1",
         "model": "slow", "api_key_env": "SLOW_KEY", "latency_p95_ms": 200,
         "features": {"vision": true}}
      ]
    }"#;
    with_catalog("missing-keys", body, || {
        let error = resolve_from_env(&Requirements {
            features: vec!["vision".into()],
        })
        .expect_err("missing keys must fail");
        let message = format!("{error:#}");
        assert!(message.contains("FAST_KEY"), "{message}");
        assert!(message.contains("SLOW_KEY"), "{message}");
    });
}

#[test]
fn empty_api_key_env_is_rejected() {
    let body = r#"{
      "models": [
        {"id": "bad", "provider": "test", "base_url": "https://api.example/v1",
         "model": "bad", "api_key_env": "", "features": {"vision": true}}
      ]
    }"#;
    with_catalog("empty-key-env", body, || {
        let error = resolve_from_env(&Requirements::default()).expect_err("empty key env");
        assert!(format!("{error:#}").contains("api_key_env"), "{error:#}");
    });
}

#[test]
fn missing_schema_key_does_not_relax_features() {
    let body = r#"{"models":[
        {"id":"schema","provider":"test","base_url":"http://cloud.example/v1",
         "model":"schema","api_key_env":"FAST_KEY","features":{"json_schema":true}},
        {"id":"json","provider":"test","base_url":"http://127.0.0.1:8000/v1",
         "model":"json","features":{"json_object":true}}
    ]}"#;
    with_catalog("strict-schema", body, || {
        let requirements = Requirements {
            features: vec!["json_schema".into()],
        };
        let error = resolve_from_env(&requirements).expect_err("missing schema key");
        assert!(format!("{error:#}").contains("FAST_KEY"), "{error:#}");
        unsafe { std::env::set_var("FAST_KEY", "dummy") };
        let target = resolve_from_env(&requirements).unwrap().unwrap();
        assert_eq!(target.id, "schema");
    });
}
