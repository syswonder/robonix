// SPDX-License-Identifier: MulanPSL-2.0
//! Saved copies of long capability output.
//!
//! A result longer than the inline limit is kept whole in a file under the
//! Executor workspace, and the model receives the file's path, size, line
//! count and a preview, which it searches with the `run_command` builtin.
//! Verification always uses the original result.
use crate::pb::pilot::CapabilityCallResult;
use std::fs;
use std::io::{self, BufRead, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

/// Characters of a saved file shown to the model next to its path.
const PREVIEW_CHARS: usize = 512;

/// How the model should use a saved file instead of repeating the call.
pub(crate) const SEARCH_NOTE: &str = "Preview only. Search the saved file with run_command \
    (rg -n, or grep -n -F), then read the matching lines with sed -n. It holds this call's \
    output at observed_unix, not current state; do not repeat the call to recover it.";

static SERIAL: AtomicU64 = AtomicU64::new(0);

/// Where output is saved, when it is saved, and how much may be kept.
pub(crate) struct Limits {
    pub dir: PathBuf,
    /// Results longer than this many UTF-8 bytes are saved instead of inlined.
    pub inline: usize,
    /// Bytes the directory may hold; the oldest files are deleted to make room.
    pub budget: u64,
}

impl Limits {
    pub fn from_env() -> Self {
        fn env_or<T: std::str::FromStr>(name: &str, default: T) -> T {
            std::env::var(name)
                .ok()
                .and_then(|value| value.parse().ok())
                .unwrap_or(default)
        }
        Self {
            dir: crate::dispatch::builtin::workspace_root().join(".robonix/tool-output"),
            inline: env_or("ROBONIX_OUTPUT_INLINE_BYTES", 16 * 1024),
            budget: env_or("ROBONIX_OUTPUT_STORAGE_BYTES", 256 * 1024 * 1024),
        }
    }
}

pub(crate) fn now_unix() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs()
}

/// Create a new private file for one output.
pub(crate) fn create(dir: &Path) -> io::Result<(PathBuf, fs::File)> {
    let mut builder = fs::DirBuilder::new();
    builder.recursive(true);
    #[cfg(unix)]
    std::os::unix::fs::DirBuilderExt::mode(&mut builder, 0o700);
    builder.create(dir)?;
    let stamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let serial = SERIAL.fetch_add(1, Ordering::Relaxed);
    let path = dir.join(format!("{stamp}-{}-{serial}.txt", std::process::id()));
    let mut options = fs::OpenOptions::new();
    options.write(true).create_new(true);
    #[cfg(unix)]
    std::os::unix::fs::OpenOptionsExt::mode(&mut options, 0o600);
    let file = options.open(&path)?;
    Ok((path, file))
}

/// Delete files other than `keep`, oldest first, until `dir` holds at most `budget` bytes.
pub(crate) fn evict_oldest(dir: &Path, budget: u64, keep: &[&Path]) -> io::Result<()> {
    let mut files = Vec::new();
    let mut used = 0;
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let metadata = entry.metadata()?;
        if metadata.is_file() {
            used += metadata.len();
            if !keep.contains(&entry.path().as_path()) {
                files.push((metadata.modified()?, entry.path(), metadata.len()));
            }
        }
    }
    files.sort();
    for (_, path, len) in files {
        if used <= budget {
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
    prepare_in(result, Limits::from_env()).await
}

async fn prepare_in(result: &CapabilityCallResult, limits: Limits) -> CapabilityCallResult {
    let mut view = result.clone();
    if result.output.len() <= limits.inline || is_image(&result.output) {
        return view;
    }
    let text = result.output.clone();
    let saved = tokio::task::spawn_blocking(move || save(&limits, &text))
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
            let preview: String = result.output.chars().take(PREVIEW_CHARS).collect();
            reference["output"] = serde_json::json!({
                "bytes": result.output.len(),
                "preview": preview,
            });
            reference["storage_error"] = error.to_string().into();
            reference["note"] = "The full result could not be saved; only the preview exists. \
                The call's outcome is unchanged; do not repeat it to recover the text."
                .into();
        }
    }
    view.output = reference.to_string();
    view
}

fn save(limits: &Limits, text: &str) -> io::Result<serde_json::Value> {
    if text.len() as u64 > limits.budget {
        return Err(io::Error::other(format!(
            "{} bytes exceed the {}-byte output storage budget",
            text.len(),
            limits.budget
        )));
    }
    let (path, mut file) = create(&limits.dir)?;
    if let Err(error) = file.write_all(text.as_bytes()) {
        let _ = fs::remove_file(&path);
        return Err(error);
    }
    evict_oldest(&limits.dir, limits.budget, &[&path])?;
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
        pub(crate) fn limits(&self, inline: usize, budget: u64) -> Limits {
            Limits {
                dir: self.0.join("out"),
                inline,
                budget,
            }
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
        let view = prepare_in(&original, dir.limits(8192, 1 << 20)).await;
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
            let view = prepare_in(&result(text.clone()), dir.limits(100, 1 << 20)).await;
            assert_eq!(view.output, text);
        }
        assert!(!dir.0.join("out").exists());
        assert!(!is_image(
            r#"{"width":1,"height":1,"encoding":"error","data":"x"}"#
        ));
    }

    #[tokio::test]
    async fn failed_save_reports_preview_without_changing_success() {
        let dir = TempDir::new();
        let view = prepare_in(&result("x".repeat(5000)), dir.limits(100, 1000)).await;
        assert!(view.success);
        let reference: serde_json::Value = serde_json::from_str(&view.output).unwrap();
        assert!(reference["output"].get("file").is_none());
        assert!(
            reference["storage_error"]
                .as_str()
                .unwrap()
                .contains("budget")
        );
        assert_eq!(
            reference["output"]["preview"].as_str().unwrap().len(),
            PREVIEW_CHARS
        );
    }

    #[test]
    fn full_budget_evicts_oldest_files_only() {
        let dir = TempDir::new();
        let limits = dir.limits(0, 10);
        let path = |saved: serde_json::Value| PathBuf::from(saved["file"].as_str().unwrap());
        let first = path(save(&limits, "aaaaaa").unwrap());
        std::thread::sleep(std::time::Duration::from_millis(10));
        let second = path(save(&limits, "bbbb").unwrap());
        let third = path(save(&limits, "cccccc").unwrap());
        assert!(!first.exists());
        assert_eq!(fs::read_to_string(second).unwrap(), "bbbb");
        assert_eq!(fs::read_to_string(third).unwrap(), "cccccc");
        assert!(save(&limits, "too large for budget").is_err());
        assert_eq!(fs::read_dir(&limits.dir).unwrap().count(), 2);
    }
}
