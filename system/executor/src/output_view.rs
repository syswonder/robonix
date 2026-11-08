// SPDX-License-Identifier: MulanPSL-2.0
//! Model-visible copy of a capability result.
//!
//! Text longer than the inline limit is saved whole under the Executor
//! workspace and replaced by a reference that the model searches with the
//! `run_command` builtin. Verification always uses the original result.
use crate::pb::pilot::CapabilityCallResult;
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

/// Characters of a saved result shown to the model next to its path.
const PREVIEW_CHARS: usize = 512;

/// Serializes saves so that eviction and writing see a consistent directory.
static SAVE: Mutex<()> = Mutex::new(());
static SERIAL: AtomicU64 = AtomicU64::new(0);

fn env_or<T: std::str::FromStr>(name: &str, default: T) -> T {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(default)
}

/// Return `result` with long text replaced by a reference to its saved copy.
pub async fn prepare(result: &CapabilityCallResult) -> CapabilityCallResult {
    let dir = crate::dispatch::builtin::workspace_root().join(".robonix/tool-output");
    let inline = env_or("ROBONIX_OUTPUT_INLINE_BYTES", 16 * 1024);
    let budget = env_or("ROBONIX_OUTPUT_STORAGE_BYTES", 256 * 1024 * 1024);
    prepare_in(result, dir, inline, budget).await
}

async fn prepare_in(
    result: &CapabilityCallResult,
    dir: PathBuf,
    inline: usize,
    budget: u64,
) -> CapabilityCallResult {
    let mut view = result.clone();
    if result.output.len() <= inline || is_image(&result.output) {
        return view;
    }
    let text = result.output.clone();
    let saved = tokio::task::spawn_blocking(move || save(&dir, &text, budget))
        .await
        .unwrap_or_else(|error| Err(io::Error::other(error)));
    let observed_unix = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let preview: String = result.output.chars().take(PREVIEW_CHARS).collect();
    let mut reference = serde_json::json!({
        "call_id": result.call_id,
        "contract_id": result.contract_id,
        "observed_unix": observed_unix,
        "output_bytes": result.output.len(),
        "output_lines": result.output.lines().count(),
        "preview": preview,
    });
    match saved {
        Ok(path) => {
            reference["output_file"] = path.display().to_string().into();
            reference["note"] = "Preview only. Search output_file with run_command (rg -n, \
                or grep -n -F), then read the matching lines with sed -n. The file is this \
                call's result at observed_unix, not current state; do not repeat the call."
                .into();
        }
        Err(error) => {
            reference["storage_error"] = error.to_string().into();
            reference["note"] = "The full result could not be saved; only the preview exists. \
                The call's outcome is unchanged; do not repeat it to recover the text."
                .into();
        }
    }
    view.output = reference.to_string();
    view
}

/// Write `text` to a new file, first deleting the oldest saved outputs as
/// needed to keep the directory within `budget` bytes.
fn save(dir: &Path, text: &str, budget: u64) -> io::Result<PathBuf> {
    let size = text.len() as u64;
    if size > budget {
        return Err(io::Error::other(format!(
            "{size} bytes exceed the {budget}-byte output storage budget"
        )));
    }
    let _guard = SAVE.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
    let mut builder = fs::DirBuilder::new();
    builder.recursive(true);
    #[cfg(unix)]
    std::os::unix::fs::DirBuilderExt::mode(&mut builder, 0o700);
    builder.create(dir)?;
    evict_oldest(dir, budget - size)?;
    let stamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let serial = SERIAL.fetch_add(1, Ordering::Relaxed);
    let path = dir.join(format!("{stamp}-{serial}.txt"));
    let mut options = fs::OpenOptions::new();
    options.write(true).create_new(true);
    #[cfg(unix)]
    std::os::unix::fs::OpenOptionsExt::mode(&mut options, 0o600);
    let written = options.open(&path)?.write_all(text.as_bytes());
    if let Err(error) = written {
        let _ = fs::remove_file(&path);
        return Err(error);
    }
    Ok(path)
}

/// Delete files, oldest first, until at most `allowed` bytes remain in `dir`.
fn evict_oldest(dir: &Path, allowed: u64) -> io::Result<()> {
    let mut files = Vec::new();
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let metadata = entry.metadata()?;
        if metadata.is_file() {
            files.push((metadata.modified()?, entry.path(), metadata.len()));
        }
    }
    let mut used: u64 = files.iter().map(|file| file.2).sum();
    files.sort();
    for (_, path, len) in files {
        if used <= allowed {
            break;
        }
        match fs::remove_file(&path) {
            Ok(()) => used -= len,
            Err(error) if error.kind() == io::ErrorKind::NotFound => used -= len,
            Err(error) => return Err(error),
        }
    }
    Ok(())
}

/// Images keep their existing multimodal path in Pilot.
fn is_image(output: &str) -> bool {
    let Ok(value) = serde_json::from_str::<serde_json::Value>(output) else {
        return false;
    };
    let text = |key: &str| value.get(key).and_then(|v| v.as_str());
    text("image_base64").is_some_and(|data| !data.is_empty())
        || (value.get("width").is_some()
            && value.get("height").is_some()
            && text("encoding").is_some_and(|encoding| encoding != "error")
            && text("data").is_some_and(|data| !data.is_empty()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Temporary directory removed when the test ends, including on panic.
    struct TempDir(PathBuf);
    impl TempDir {
        fn new() -> Self {
            let path =
                std::env::temp_dir().join(format!("robonix-output-{}", uuid::Uuid::new_v4()));
            fs::create_dir(&path).unwrap();
            Self(path)
        }
    }
    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    fn result(output: String) -> CapabilityCallResult {
        CapabilityCallResult {
            call_id: "p:0".into(),
            contract_id: "robonix/system/executor/builtin/read_capability_doc".into(),
            success: true,
            output,
            ..Default::default()
        }
    }

    #[tokio::test]
    async fn long_result_is_saved_whole_and_original_is_unchanged() {
        let dir = TempDir::new();
        let text = format!("{}目标：stable-id-972\n", "manual line\n".repeat(3000)); // i18n-ok: UTF-8 tail fixture
        let original = result(text.clone());
        let view = prepare_in(&original, dir.0.clone(), 8192, 1 << 20).await;
        assert_eq!(original.output, text);
        assert!(view.success);
        assert!(view.output.len() < 2048);
        let reference: serde_json::Value = serde_json::from_str(&view.output).unwrap();
        assert_eq!(reference["call_id"], "p:0");
        assert_eq!(reference["output_lines"], 3001);
        assert!(reference["observed_unix"].as_u64().unwrap() > 0);
        let saved = fs::read(reference["output_file"].as_str().unwrap()).unwrap();
        assert_eq!(saved, text.as_bytes());
    }

    #[tokio::test]
    async fn short_results_and_images_are_not_saved() {
        let dir = TempDir::new();
        let image = serde_json::json!({"width": 2, "height": 2, "encoding": "jpeg", "data": "a".repeat(4000)});
        for text in ["small".to_owned(), image.to_string()] {
            let view = prepare_in(&result(text.clone()), dir.0.clone(), 100, 1 << 20).await;
            assert_eq!(view.output, text);
        }
        assert_eq!(fs::read_dir(&dir.0).unwrap().count(), 0);
        assert!(!is_image(
            r#"{"width":1,"height":1,"encoding":"error","data":"x"}"#
        ));
    }

    #[tokio::test]
    async fn failed_save_reports_preview_without_changing_success() {
        let dir = TempDir::new();
        let view = prepare_in(&result("x".repeat(5000)), dir.0.clone(), 100, 1000).await;
        assert!(view.success);
        let reference: serde_json::Value = serde_json::from_str(&view.output).unwrap();
        assert!(reference.get("output_file").is_none());
        assert!(
            reference["storage_error"]
                .as_str()
                .unwrap()
                .contains("budget")
        );
        assert_eq!(reference["preview"].as_str().unwrap().len(), PREVIEW_CHARS);
    }

    #[test]
    fn full_budget_evicts_oldest_files_only() {
        let dir = TempDir::new();
        let first = save(&dir.0, "aaaaaa", 10).unwrap();
        std::thread::sleep(std::time::Duration::from_millis(10));
        let second = save(&dir.0, "bbbb", 10).unwrap();
        let third = save(&dir.0, "cccccc", 10).unwrap();
        assert!(!first.exists());
        assert_eq!(fs::read_to_string(second).unwrap(), "bbbb");
        assert_eq!(fs::read_to_string(third).unwrap(), "cccccc");
        assert!(save(&dir.0, "too large for budget", 10).is_err());
        assert_eq!(fs::read_dir(&dir.0).unwrap().count(), 2);
    }
}
