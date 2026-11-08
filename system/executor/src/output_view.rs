// SPDX-License-Identifier: MulanPSL-2.0
//! Model-visible copies of outputs; verification keeps the original result.
use crate::pb::pilot::CapabilityCallResult;
use std::path::PathBuf;

/// Persist long text before returning a reference the Executor shell can inspect.
pub async fn prepare(result: &CapabilityCallResult) -> CapabilityCallResult {
    let limit = crate::output_file::inline_budget();
    let root = crate::output_file::root();
    let budget = crate::output_file::storage_budget();
    prepare_in(result, root, limit, budget).await
}

async fn prepare_in(
    result: &CapabilityCallResult,
    root: PathBuf,
    limit: usize,
    budget: u64,
) -> CapabilityCallResult {
    let mut view = result.clone();
    if result.output.len() <= limit || is_image(&result.output) {
        return view;
    }
    let text = result.output.clone();
    let bytes = text.len();
    let saved =
        tokio::task::spawn_blocking(move || crate::output_file::save(&root, &text, budget)).await;
    let preview: String = result.output.chars().take(512).collect();
    view.output = match saved {
        Ok(Ok(path)) => serde_json::json!({
            "output_file": path,
            "output_bytes": bytes,
            "call_id": result.call_id,
            "provider_id": result.provider_id,
            "contract_id": result.contract_id,
            "complete_file": true,
            "preview": preview,
            "instruction": "The preview is incomplete. Use run_command with rg -n (or POSIX grep -n -F) to search this saved result, then sed -n to read relevant lines. Do not repeat the original action. This is the original observation, not live state."
        }).to_string(),
        error => serde_json::json!({
            "output_bytes": bytes,
            "complete_file": false,
            "preview": preview,
            "storage_error": format!("{error:?}"),
            "instruction": "Complete output could not be retained. The original action's success is unchanged. Report missing information; do not automatically repeat the action."
        }).to_string(),
    };
    view
}

fn is_image(output: &str) -> bool {
    let Ok(value) = serde_json::from_str::<serde_json::Value>(output) else {
        return false;
    };
    value
        .get("image_base64")
        .and_then(|v| v.as_str())
        .is_some_and(|v| !v.is_empty())
        || (value.get("width").is_some()
            && value.get("height").is_some()
            && value
                .get("encoding")
                .and_then(|v| v.as_str())
                .is_some_and(|v| v != "error")
            && value
                .get("data")
                .and_then(|v| v.as_str())
                .is_some_and(|v| !v.is_empty()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn long_result_is_searchable_without_changing_action_or_original() {
        let root = std::env::temp_dir().join(format!("robonix-view-{}", uuid::Uuid::new_v4()));
        let original = CapabilityCallResult {
            call_id: "one-physical-action".into(),
            success: true,
            output: format!(
                "{}\nTarget identity provenance: stable-id-972",
                "manual section\n".repeat(3000)
            ),
            ..Default::default()
        };
        let view = prepare_in(&original, root.clone(), 8192, 1024 * 1024).await;
        let metadata: serde_json::Value = serde_json::from_str(&view.output).unwrap();
        assert!(view.success);
        assert_eq!(view.call_id, original.call_id);
        assert!(view.output.len() < 4096);
        let stored = std::fs::read_to_string(metadata["output_file"].as_str().unwrap()).unwrap();
        assert_eq!(stored, original.output);
        assert!(stored.ends_with("stable-id-972"));
        let failed_save = prepare_in(&original, root.clone(), 8192, 1).await;
        assert!(failed_save.success);
        assert!(failed_save.output.contains("storage_error"));
        std::fs::remove_dir_all(root).unwrap();
    }

    #[tokio::test]
    async fn short_results_and_images_keep_their_existing_representation() {
        let root = PathBuf::from("/unused");
        for text in ["small result".to_owned(), serde_json::json!({"width": 10, "height": 10, "encoding": "jpeg", "data": "a".repeat(2000)}).to_string()] {
            let result = CapabilityCallResult { success: true, output: text.clone(), ..Default::default() };
            assert_eq!(prepare_in(&result, root.clone(), 100, 1).await.output, text);
        }
        assert!(!is_image(
            r#"{"width":10,"height":10,"encoding":"error","data":"bad"}"#
        ));
    }
}
