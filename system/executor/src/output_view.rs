// SPDX-License-Identifier: MulanPSL-2.0
//! Saved copies of long capability output.
//!
//! A result longer than the inline limit is kept whole in a file under the
//! system temporary directory, and the model receives the file's path, size,
//! line count and a preview, which it searches with the `run_command` builtin.
//! Nothing here limits or deletes saved files: they live as long as the
//! temporary directory keeps them. Verification always uses the original result.
use crate::pb::pilot::CapabilityCallResult;
use std::fs;
use std::io::{self, BufRead, Write};
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

/// Characters of a saved file shown to the model next to its path.
const PREVIEW_CHARS: usize = 512;

/// How the model should use a saved file instead of repeating the call.
pub(crate) const SEARCH_NOTE: &str = "Preview only. Search the saved file with run_command \
    (rg -n, or grep -n -F), then read the matching lines with sed -n. It holds this call's \
    output at observed_unix, not current state; do not repeat the call to recover it.";

/// What the model should do when output could not be saved completely.
pub(crate) const STORAGE_NOTE: &str = "The full output could not be saved. Tell the user the \
    system temporary directory needs attention (see storage_error) and say what is missing. \
    The call's outcome is unchanged; do not repeat it to recover the text.";

static SERIAL: AtomicU64 = AtomicU64::new(0);

/// Results longer than this many UTF-8 bytes are saved instead of inlined.
pub(crate) fn inline_bytes() -> usize {
    std::env::var("ROBONIX_OUTPUT_INLINE_BYTES")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(32 * 1024)
}

/// This process's private directory under the system temporary directory.
pub(crate) fn output_dir() -> io::Result<PathBuf> {
    static DIR: Mutex<Option<PathBuf>> = Mutex::new(None);
    let mut dir = DIR.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
    if let Some(path) = dir.as_ref() {
        return Ok(path.clone());
    }
    let path = std::env::temp_dir().join(format!(
        "robonix-output-{}-{}",
        std::process::id(),
        now_unix()
    ));
    let mut builder = fs::DirBuilder::new();
    #[cfg(unix)]
    std::os::unix::fs::DirBuilderExt::mode(&mut builder, 0o700);
    builder.create(&path)?;
    *dir = Some(path.clone());
    Ok(path)
}

pub(crate) fn now_unix() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs()
}

/// Create a new private file for one output.
pub(crate) fn create(dir: &Path) -> io::Result<(PathBuf, fs::File)> {
    let serial = SERIAL.fetch_add(1, Ordering::Relaxed);
    let path = dir.join(format!("output-{serial}.txt"));
    let mut options = fs::OpenOptions::new();
    options.write(true).create_new(true);
    #[cfg(unix)]
    std::os::unix::fs::OpenOptionsExt::mode(&mut options, 0o600);
    let file = options.open(&path)?;
    Ok((path, file))
}

/// Path, size, line count and preview of a saved file.
pub(crate) fn describe(path: &Path) -> io::Result<serde_json::Value> {
    let mut reader = io::BufReader::new(fs::File::open(path)?);
    let (mut bytes, mut lines, mut last, mut head) = (0u64, 0u64, b'\n', Vec::new());
    loop {
        let chunk = reader.fill_buf()?;
        let Some(&end) = chunk.last() else { break };
        let room = (PREVIEW_CHARS * 4).saturating_sub(head.len());
        head.extend_from_slice(&chunk[..chunk.len().min(room)]);
        lines += chunk.iter().filter(|byte| **byte == b'\n').count() as u64;
        bytes += chunk.len() as u64;
        last = end;
        let read = chunk.len();
        reader.consume(read);
    }
    if last != b'\n' {
        lines += 1;
    }
    let preview: String = String::from_utf8_lossy(&head)
        .chars()
        .take(PREVIEW_CHARS)
        .collect();
    Ok(serde_json::json!({
        "file": path,
        "bytes": bytes,
        "lines": lines,
        "preview": preview,
    }))
}

/// Return `result` with long text replaced by a reference to its saved copy.
pub async fn prepare(result: &CapabilityCallResult) -> CapabilityCallResult {
    prepare_in(result, inline_bytes(), output_dir).await
}

async fn prepare_in(
    result: &CapabilityCallResult,
    inline: usize,
    dir: impl FnOnce() -> io::Result<PathBuf> + Send + 'static,
) -> CapabilityCallResult {
    let mut view = result.clone();
    if result.output.len() <= inline || is_image(&result.output) {
        return view;
    }
    let text = result.output.clone();
    let saved = tokio::task::spawn_blocking(move || save(&dir()?, &text))
        .await
        .unwrap_or_else(|error| Err(io::Error::other(error)));
    let mut reference = serde_json::json!({
        "call_id": result.call_id,
        "contract_id": result.contract_id,
        "observed_unix": now_unix(),
    });
    match saved {
        Ok(output) => {
            reference["output"] = output;
            reference["note"] = SEARCH_NOTE.into();
        }
        Err(error) => {
            robonix_scribe::warn!(
                "[executor] could not save output of call_id={} to the temporary directory: {error}",
                result.call_id
            );
            let preview: String = result.output.chars().take(PREVIEW_CHARS).collect();
            reference["output"] = serde_json::json!({
                "bytes": result.output.len(),
                "preview": preview,
            });
            reference["storage_error"] = error.to_string().into();
            reference["note"] = STORAGE_NOTE.into();
        }
    }
    view.output = reference.to_string();
    view
}

fn save(dir: &Path, text: &str) -> io::Result<serde_json::Value> {
    let (path, mut file) = create(dir)?;
    if let Err(error) = file.write_all(text.as_bytes()) {
        let _ = fs::remove_file(&path);
        return Err(error);
    }
    describe(&path)
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
pub(crate) mod tests {
    use super::*;

    /// Temporary directory removed when the test ends, including on panic.
    pub(crate) struct TempDir(pub PathBuf);
    impl TempDir {
        pub(crate) fn new() -> Self {
            let path =
                std::env::temp_dir().join(format!("robonix-output-{}", uuid::Uuid::new_v4()));
            fs::create_dir(&path).unwrap();
            Self(path)
        }
        fn source(&self) -> impl FnOnce() -> io::Result<PathBuf> + Send + 'static {
            let path = self.0.clone();
            move || Ok(path)
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
        let view = prepare_in(&original, 8192, dir.source()).await;
        assert_eq!(original.output, text);
        assert!(view.success);
        assert!(view.output.len() < 2048);
        let reference: serde_json::Value = serde_json::from_str(&view.output).unwrap();
        assert_eq!(reference["call_id"], "p:0");
        assert_eq!(reference["output"]["lines"], 3001);
        assert_eq!(reference["output"]["bytes"], text.len());
        assert!(reference["observed_unix"].as_u64().unwrap() > 0);
        let saved = fs::read(reference["output"]["file"].as_str().unwrap()).unwrap();
        assert_eq!(saved, text.as_bytes());
    }

    #[tokio::test]
    async fn short_results_and_images_are_not_saved() {
        let dir = TempDir::new();
        let image = serde_json::json!({"width": 2, "height": 2, "encoding": "jpeg", "data": "a".repeat(4000)});
        for text in ["small".to_owned(), image.to_string()] {
            let view = prepare_in(&result(text.clone()), 100, dir.source()).await;
            assert_eq!(view.output, text);
        }
        assert_eq!(fs::read_dir(&dir.0).unwrap().count(), 0);
        assert!(!is_image(
            r#"{"width":1,"height":1,"encoding":"error","data":"x"}"#
        ));
    }

    #[tokio::test]
    async fn failed_save_reports_preview_without_changing_success() {
        let view = prepare_in(&result("x".repeat(5000)), 100, || {
            Err(io::Error::from_raw_os_error(28))
        })
        .await;
        assert!(view.success);
        let reference: serde_json::Value = serde_json::from_str(&view.output).unwrap();
        assert!(reference["output"].get("file").is_none());
        assert!(!reference["storage_error"].as_str().unwrap().is_empty());
        assert_eq!(reference["note"], STORAGE_NOTE);
        assert_eq!(
            reference["output"]["preview"].as_str().unwrap().len(),
            PREVIEW_CHARS
        );
    }

    #[test]
    fn saved_outputs_are_never_deleted_or_overwritten() {
        let dir = TempDir::new();
        let path = |saved: serde_json::Value| PathBuf::from(saved["file"].as_str().unwrap());
        let first = path(save(&dir.0, "first").unwrap());
        let second = path(save(&dir.0, "second").unwrap());
        assert_ne!(first, second);
        assert_eq!(fs::read_to_string(first).unwrap(), "first");
        assert_eq!(fs::read_to_string(second).unwrap(), "second");
    }
}
