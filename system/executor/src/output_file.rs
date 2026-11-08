// SPDX-License-Identifier: MulanPSL-2.0
//! Retain complete capability output for inspection with ordinary shell tools.

use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

static WRITES: Mutex<()> = Mutex::new(());
static NEXT_FILE: AtomicU64 = AtomicU64::new(0);

/// Shared directory used by Executor and its shell commands.
pub fn root() -> PathBuf {
    std::env::var_os("ROBONIX_WORKSPACE")
        .map(PathBuf::from)
        .unwrap_or_else(|| std::env::current_dir().unwrap_or_else(|_| PathBuf::from(".")))
        .join(".robonix/tool-output")
}

/// Maximum retained bytes, independent of the model's inline budget.
pub fn storage_budget() -> u64 {
    std::env::var("ROBONIX_OUTPUT_STORAGE_BYTES")
        .ok()
        .and_then(|v| v.parse().ok())
        .filter(|v| *v > 0)
        .unwrap_or(256 * 1024 * 1024)
}

/// Model-visible text threshold selected from the measured retrieval comparison.
pub fn inline_budget() -> usize {
    std::env::var("ROBONIX_OUTPUT_INLINE_BYTES")
        .ok()
        .and_then(|v| v.parse().ok())
        .filter(|v| *v > 0)
        .unwrap_or(32 * 1024)
}

fn create(root: &Path) -> io::Result<(PathBuf, fs::File)> {
    let mut builder = fs::DirBuilder::new();
    builder.recursive(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::DirBuilderExt;
        builder.mode(0o700);
    }
    builder.create(root)?;
    if fs::symlink_metadata(root)?.file_type().is_symlink() {
        return Err(io::Error::other("output directory must not be a symlink"));
    }
    let root = root.canonicalize()?;
    let stamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(io::Error::other)?
        .as_nanos();
    let serial = NEXT_FILE.fetch_add(1, Ordering::Relaxed);
    let path = root.join(format!(
        "output-{}-{stamp}-{serial}.txt",
        std::process::id()
    ));
    let mut options = OpenOptions::new();
    options.write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }
    let file = options.open(&path)?;
    Ok((path, file))
}

fn append(path: &Path, file: &mut fs::File, bytes: &[u8], budget: u64) -> io::Result<()> {
    let _guard = WRITES
        .lock()
        .map_err(|_| io::Error::other("output writer lock poisoned"))?;
    let mut used = 0u64;
    for entry in fs::read_dir(
        path.parent()
            .ok_or_else(|| io::Error::other("missing output directory"))?,
    )? {
        let metadata = entry?.metadata()?;
        if metadata.is_file() {
            used = used.saturating_add(metadata.len());
        }
    }
    if used.saturating_add(bytes.len() as u64) > budget {
        return Err(io::Error::other(format!(
            "output storage budget exhausted; incomplete output retained at {}",
            path.display()
        )));
    }
    file.write_all(bytes)
}

/// Save a complete UTF-8 result without replacing another call's file.
pub fn save(root: &Path, content: &str, directory_budget: u64) -> io::Result<PathBuf> {
    let (path, mut file) = create(root)?;
    if let Err(error) =
        append(&path, &mut file, content.as_bytes(), directory_budget).and_then(|_| file.sync_all())
    {
        drop(file);
        let _ = fs::remove_file(&path);
        return Err(error);
    }
    Ok(path)
}

/// Spool a process stream with bounded memory; retain raw bytes even for binary output.
pub async fn collect(
    mut reader: impl tokio::io::AsyncRead + Unpin,
    root: PathBuf,
    inline: usize,
    budget: u64,
) -> io::Result<String> {
    use tokio::io::AsyncReadExt;
    let mut initial = Vec::new();
    (&mut reader)
        .take(inline.saturating_add(1) as u64)
        .read_to_end(&mut initial)
        .await?;
    if initial.len() <= inline
        && let Ok(text) = std::str::from_utf8(&initial)
    {
        // Retrieval must remain usable even when retained-output storage is full.
        return Ok(text.to_owned());
    }
    let mut total = initial.len();
    let preview = initial[..initial.len().min(inline)].to_vec();
    let (path, mut file) = tokio::task::spawn_blocking(move || {
        let (path, mut file) = create(&root)?;
        if let Err(error) = append(&path, &mut file, &initial, budget) {
            drop(file);
            let _ = fs::remove_file(&path);
            return Err(io::Error::other(format!(
                "initial output write failed; newly created file was removed: {error}"
            )));
        }
        Ok::<_, io::Error>((path, file))
    })
    .await
    .map_err(io::Error::other)??;
    let mut buffer = vec![0u8; 64 * 1024];
    loop {
        let count = reader.read(&mut buffer).await?;
        if count == 0 {
            break;
        }
        total = total.saturating_add(count);
        let chunk = buffer[..count].to_vec();
        let destination = path.clone();
        file = tokio::task::spawn_blocking(move || {
            append(&destination, &mut file, &chunk, budget)?;
            Ok::<_, io::Error>(file)
        })
        .await
        .map_err(io::Error::other)??;
    }
    tokio::task::spawn_blocking(move || file.sync_all())
        .await
        .map_err(io::Error::other)??;
    let preview = String::from_utf8_lossy(&preview)
        .chars()
        .take(512)
        .collect::<String>();
    Ok(format!(
        "Full output saved to: {}\nBytes: {total}; complete file; preview only:\n{preview}\nSearch this file with rg or grep; do not repeat the original command.",
        path.display()
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::process::Command;

    /// Private test directory removed after every test, including unwinding.
    struct Directory(PathBuf);
    impl Directory {
        fn new() -> Self {
            let serial = NEXT_FILE.fetch_add(1, Ordering::Relaxed);
            let stamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos();
            let path = std::env::temp_dir().join(format!(
                "robonix-output-test-{}-{stamp}-{serial}",
                std::process::id()
            ));
            fs::create_dir(&path).unwrap();
            Self(path)
        }
    }
    impl Drop for Directory {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    #[tokio::test]
    async fn rejected_stream_does_not_leave_empty_files() {
        let root = Directory::new();
        fs::write(root.0.join("existing.txt"), b"full").unwrap();
        for _ in 0..3 {
            assert!(
                collect(&b"long result"[..], root.0.clone(), 4, 4)
                    .await
                    .is_err()
            );
        }
        assert_eq!(fs::read_dir(&root.0).unwrap().count(), 1);
    }

    #[tokio::test]
    async fn short_retrieval_works_with_exhausted_storage() {
        let root = Directory::new();
        fs::write(root.0.join("existing.txt"), b"full").unwrap();
        let result = collect(&b"target at line 9000\n"[..], root.0.clone(), 100, 4)
            .await
            .unwrap();
        assert_eq!(result, "target at line 9000\n");
        assert_eq!(fs::read_dir(&root.0).unwrap().count(), 1);
    }

    #[tokio::test]
    async fn stream_larger_than_old_limit_is_retained_with_tail() {
        use tokio::io::AsyncReadExt;
        let root = Directory::new();
        let tail = b"\nTAIL-IDENTITY-984\n";
        let stream = tokio::io::repeat(b'x')
            .take(17 * 1024 * 1024)
            .chain(&tail[..]);
        let result = collect(stream, root.0.clone(), 1024, 32 * 1024 * 1024)
            .await
            .unwrap();
        let path = result
            .lines()
            .next()
            .unwrap()
            .strip_prefix("Full output saved to: ")
            .unwrap();
        let bytes = fs::read(path).unwrap();
        assert_eq!(bytes.len(), 17 * 1024 * 1024 + tail.len());
        assert!(bytes[..17 * 1024 * 1024].iter().all(|b| *b == b'x'));
        assert!(bytes.ends_with(tail));
        assert!(result.len() < 2048);
    }

    #[tokio::test]
    async fn binary_stream_is_saved_without_utf8_replacement() {
        let root = Directory::new();
        let bytes = [0xff, 0x00, 0x80, 0x41];
        let result = collect(&bytes[..], root.0.clone(), 100, 1024)
            .await
            .unwrap();
        let path = result
            .lines()
            .next()
            .unwrap()
            .strip_prefix("Full output saved to: ")
            .unwrap();
        assert_eq!(fs::read(path).unwrap(), bytes);
    }

    #[tokio::test]
    async fn quota_failure_keeps_partial_stream_and_reports_incompleteness() {
        use tokio::io::AsyncReadExt;
        let root = Directory::new();
        let stream = tokio::io::repeat(b'x').take(128 * 1024);
        let error = collect(stream, root.0.clone(), 100, 70_000)
            .await
            .unwrap_err();
        assert!(error.to_string().contains("incomplete output retained"));
        let files = fs::read_dir(&root.0)
            .unwrap()
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        assert_eq!(files.len(), 1);
        assert_eq!(files[0].metadata().unwrap().len(), 65536 + 101);
    }

    #[test]
    fn real_rg_finds_tail_and_sed_reads_original_context() {
        let root = Directory::new();
        let content = format!(
            "{}目标位置：柜子七号\nverification: blue-cup-987\n", // i18n-ok: UTF-8 retrieval fixture
            "unrelated observation\n".repeat(100_000)
        );
        let path = save(&root.0, &content, 4 * 1024 * 1024).unwrap();
        let found = Command::new("rg")
            .args(["-n", "-F", "--", "blue-cup-987"])
            .arg(&path)
            .output()
            .unwrap();
        assert!(found.status.success());
        assert_eq!(
            String::from_utf8(found.stdout).unwrap(),
            "100002:verification: blue-cup-987\n"
        );
        let read = Command::new("sed")
            .args(["-n", "100001,100002p"])
            .arg(&path)
            .output()
            .unwrap();
        assert!(read.status.success());
        assert_eq!(
            String::from_utf8(read.stdout).unwrap(),
            "目标位置：柜子七号\nverification: blue-cup-987\n" // i18n-ok: exact UTF-8 retrieval
        );
        assert_eq!(fs::read_to_string(path).unwrap(), content);
    }

    #[test]
    fn posix_fallback_works_without_rg_and_distinguishes_errors() {
        let root = Directory::new();
        let tools = root.0.join("bin");
        fs::create_dir(&tools).unwrap();
        #[cfg(unix)]
        for name in ["grep", "sed", "head", "tail"] {
            let resolved = Command::new("sh")
                .args(["-c", &format!("command -v {name}")])
                .output()
                .unwrap();
            assert!(resolved.status.success());
            let executable = String::from_utf8(resolved.stdout).unwrap();
            std::os::unix::fs::symlink(executable.trim(), tools.join(name)).unwrap();
        }
        let saved = save(&root.0, "alpha\nneedle value\nomega\n", 1024).unwrap();
        let output = Command::new("/bin/sh").env("PATH", &tools)
            .args(["-c", "if command -v rg >/dev/null 2>&1; then exit 99; else grep -n -F -e needle \"$1\"; fi", "search"])
            .arg(&saved).output().unwrap();
        assert_eq!(output.status.code(), Some(0));
        assert_eq!(
            String::from_utf8(output.stdout).unwrap(),
            "2:needle value\n"
        );
        let absent = Command::new(tools.join("grep"))
            .args(["-n", "-F", "-e", "absent"])
            .arg(&saved)
            .output()
            .unwrap();
        assert_eq!(absent.status.code(), Some(1));
        let failed = Command::new(tools.join("grep"))
            .args(["-n", "-F", "-e", "needle"])
            .arg(root.0.join("missing"))
            .output()
            .unwrap();
        assert!(failed.status.code().unwrap() > 1);
        assert!(!failed.stderr.is_empty());
    }

    #[test]
    fn quota_rejection_does_not_damage_prior_results() {
        let root = Directory::new();
        let first = save(&root.0, "original", 10).unwrap();
        assert!(save(&root.0, "second", 10).is_err());
        assert_eq!(fs::read_to_string(first).unwrap(), "original");
        assert_eq!(fs::read_dir(&root.0).unwrap().count(), 1);
    }

    #[test]
    fn distinct_calls_never_overwrite_and_files_survive_reopening() {
        let root = Directory::new();
        let first = save(&root.0, "first", 100).unwrap();
        let second = save(&root.0, "second", 100).unwrap();
        assert_ne!(first, second);
        assert_eq!(fs::read_to_string(first).unwrap(), "first");
        assert_eq!(fs::read_to_string(second).unwrap(), "second");
    }
}
