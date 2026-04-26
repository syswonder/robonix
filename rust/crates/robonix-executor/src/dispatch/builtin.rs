// SPDX-License-Identifier: MulanPSL-2.0
// dispatch/builtin.rs — built-in tool implementations

use crate::pb::pilot::CapabilityCallResult;
use serde::Deserialize;
use std::path::{Path, PathBuf};

/// Workspace root for file operations. All file paths are resolved relative to
/// this directory, and path traversal beyond it is rejected.
fn workspace_root() -> PathBuf {
    std::env::var("ROBONIX_WORKSPACE")
        .map(PathBuf::from)
        .unwrap_or_else(|_| std::env::current_dir().unwrap_or_else(|_| PathBuf::from(".")))
}

/// Resolve a user-supplied path and ensure it stays within the workspace root.
/// Returns the canonical path on success, or an error if the path escapes the
/// allowed directory (path traversal).
fn safe_resolve(user_path: &str) -> anyhow::Result<PathBuf> {
    let root = workspace_root()
        .canonicalize()
        .unwrap_or_else(|_| workspace_root());
    let candidate = if Path::new(user_path).is_absolute() {
        PathBuf::from(user_path)
    } else {
        root.join(user_path)
    };
    // Canonicalize to resolve ".." components. If the file doesn't exist yet
    // (write_file), canonicalize the parent directory instead.
    let resolved = if candidate.exists() {
        candidate.canonicalize()?
    } else {
        let parent = candidate
            .parent()
            .ok_or_else(|| anyhow::anyhow!("invalid path: no parent directory"))?;
        let parent_resolved = parent.canonicalize().map_err(|_| {
            anyhow::anyhow!("parent directory does not exist: {}", parent.display())
        })?;
        parent_resolved.join(
            candidate
                .file_name()
                .ok_or_else(|| anyhow::anyhow!("invalid path: no file name"))?,
        )
    };
    if !resolved.starts_with(&root) {
        anyhow::bail!(
            "path traversal denied: {} resolves outside workspace {}",
            user_path,
            root.display()
        );
    }
    Ok(resolved)
}

pub async fn execute(call_id: &str, name: &str, args_json: &str) -> CapabilityCallResult {
    let output = run(name, args_json).await;
    match output {
        Ok(out) => CapabilityCallResult {
            call_id: call_id.to_string(),
            capability_name: name.to_string(),
            success: true,
            output: out,
            error: String::new(),
        },
        Err(e) => CapabilityCallResult {
            call_id: call_id.to_string(),
            capability_name: name.to_string(),
            success: false,
            output: String::new(),
            error: e.to_string(),
        },
    }
}

async fn run(name: &str, args_json: &str) -> anyhow::Result<String> {
    match name {
        "read_file" => read_file(args_json),
        "write_file" => write_file(args_json),
        "patch_file" => patch_file(args_json),
        "list_dir" => list_dir(args_json),
        "run_command" => run_command(args_json).await,
        other => anyhow::bail!("unknown builtin: {}", other),
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

fn truncate(s: &str, max: usize) -> String {
    if s.len() <= max {
        s.to_string()
    } else {
        // Find a valid UTF-8 char boundary at or before `max` to avoid panic.
        let mut end = max;
        while end > 0 && !s.is_char_boundary(end) {
            end -= 1;
        }
        format!("{}…(truncated, {} bytes total)", &s[..end], s.len())
    }
}

#[derive(Deserialize)]
struct ReadArgs {
    path: String,
}
#[derive(Deserialize)]
struct WriteArgs {
    path: String,
    content: String,
}
#[derive(Deserialize)]
struct PatchArgs {
    path: String,
    old: String,
    new: String,
}
#[derive(Deserialize)]
struct ListArgs {
    path: Option<String>,
}
#[derive(Deserialize)]
struct CmdArgs {
    command: String,
}

fn read_file(args: &str) -> anyhow::Result<String> {
    let a: ReadArgs = serde_json::from_str(args)?;
    let path = safe_resolve(&a.path)?;
    Ok(truncate(&std::fs::read_to_string(path)?, 8000))
}

fn write_file(args: &str) -> anyhow::Result<String> {
    let a: WriteArgs = serde_json::from_str(args)?;
    let path = safe_resolve(&a.path)?;
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).ok();
    }
    std::fs::write(&path, &a.content)?;
    Ok(format!(
        "wrote {} bytes to {}",
        a.content.len(),
        path.display()
    ))
}

fn patch_file(args: &str) -> anyhow::Result<String> {
    let a: PatchArgs = serde_json::from_str(args)?;
    let path = safe_resolve(&a.path)?;
    let content = std::fs::read_to_string(&path)?;
    if !content.contains(&a.old) {
        anyhow::bail!("old string not found in file");
    }
    std::fs::write(&path, content.replacen(&a.old, &a.new, 1))?;
    Ok(format!("patched {}", path.display()))
}

fn list_dir(args: &str) -> anyhow::Result<String> {
    let a: ListArgs = serde_json::from_str(args)?;
    let dir = a.path.as_deref().unwrap_or(".");
    let resolved = safe_resolve(dir)?;
    let mut items: Vec<String> = std::fs::read_dir(resolved)?
        .flatten()
        .map(|e| {
            let ft = e
                .file_type()
                .map(|t| if t.is_dir() { "dir" } else { "file" })
                .unwrap_or("?");
            format!("{} {}", ft, e.file_name().to_string_lossy())
        })
        .collect();
    items.sort();
    Ok(items.join("\n"))
}

/// Maximum command length to prevent abuse.
const MAX_COMMAND_LEN: usize = 8192;
/// Command execution timeout.
const COMMAND_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(120);

async fn run_command(args: &str) -> anyhow::Result<String> {
    let a: CmdArgs = serde_json::from_str(args)?;
    if a.command.len() > MAX_COMMAND_LEN {
        anyhow::bail!(
            "command too long ({} bytes, max {})",
            a.command.len(),
            MAX_COMMAND_LEN
        );
    }
    let child = tokio::process::Command::new("bash")
        .arg("-c")
        .arg(&a.command)
        .output();
    let out = tokio::time::timeout(COMMAND_TIMEOUT, child)
        .await
        .map_err(|_| anyhow::anyhow!("command timed out after {}s", COMMAND_TIMEOUT.as_secs()))?
        .map_err(|e| anyhow::anyhow!("failed to execute command: {e}"))?;
    let mut result = String::new();
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    if !stdout.is_empty() {
        result.push_str(&truncate(&stdout, 4000));
    }
    if !stderr.is_empty() {
        if !result.is_empty() {
            result.push('\n');
        }
        result.push_str("stderr: ");
        result.push_str(&truncate(&stderr, 2000));
    }
    if result.is_empty() {
        result = format!("exit code: {}", out.status.code().unwrap_or(-1));
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Path traversal tests ─────────────────────────────────────────────

    #[test]
    fn path_traversal_dotdot_is_rejected() {
        // Set workspace to a temp dir so we have a known root
        let tmp = std::env::temp_dir().join("rbnx_test_ws");
        std::fs::create_dir_all(&tmp).unwrap();
        unsafe { std::env::set_var("ROBONIX_WORKSPACE", tmp.to_str().unwrap()) };

        // ../../../etc/passwd must be rejected
        let result = safe_resolve("../../../etc/passwd");
        assert!(
            result.is_err(),
            "path traversal with ../../../etc/passwd should fail"
        );
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("path traversal denied") || err_msg.contains("does not exist"),
            "error should mention traversal or nonexistent path, got: {err_msg}"
        );
    }

    #[test]
    fn path_traversal_absolute_outside_workspace_is_rejected() {
        let tmp = std::env::temp_dir().join("rbnx_test_ws2");
        std::fs::create_dir_all(&tmp).unwrap();
        unsafe { std::env::set_var("ROBONIX_WORKSPACE", tmp.to_str().unwrap()) };

        // /etc/hostname is a real file outside workspace
        let result = safe_resolve("/etc/hostname");
        assert!(
            result.is_err(),
            "absolute path /etc/hostname outside workspace should fail"
        );
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("path traversal denied"),
            "error should mention traversal, got: {err_msg}"
        );
    }

    #[test]
    fn path_resolve_within_workspace_is_allowed() {
        let tmp = std::env::temp_dir().join("rbnx_test_ws3");
        std::fs::create_dir_all(&tmp).unwrap();
        // Create a test file inside workspace
        let test_file = tmp.join("allowed.txt");
        std::fs::write(&test_file, "hello").unwrap();
        unsafe { std::env::set_var("ROBONIX_WORKSPACE", tmp.to_str().unwrap()) };

        let result = safe_resolve("allowed.txt");
        assert!(
            result.is_ok(),
            "path within workspace should be allowed: {:?}",
            result.err()
        );

        // Cleanup
        let _ = std::fs::remove_file(&test_file);
        let _ = std::fs::remove_dir(&tmp);
    }

    // ── execute() integration tests for path traversal ───────────────────

    #[tokio::test]
    async fn read_file_rejects_path_traversal() {
        let tmp = std::env::temp_dir().join("rbnx_test_read");
        std::fs::create_dir_all(&tmp).unwrap();
        unsafe { std::env::set_var("ROBONIX_WORKSPACE", tmp.to_str().unwrap()) };

        let result = execute("test-1", "read_file", r#"{"path": "../../../etc/passwd"}"#).await;
        assert!(!result.success, "read_file should fail for path traversal");
        assert!(
            result.error.contains("traversal") || result.error.contains("does not exist"),
            "error should explain traversal, got: {}",
            result.error
        );
    }

    #[tokio::test]
    async fn write_file_rejects_path_traversal() {
        let tmp = std::env::temp_dir().join("rbnx_test_write");
        std::fs::create_dir_all(&tmp).unwrap();
        unsafe { std::env::set_var("ROBONIX_WORKSPACE", tmp.to_str().unwrap()) };

        let result = execute(
            "test-2",
            "write_file",
            r#"{"path": "/tmp/outside_workspace_evil.txt", "content": "pwned"}"#,
        )
        .await;
        assert!(
            !result.success,
            "write_file should fail for path outside workspace"
        );

        // File should NOT have been created
        assert!(
            !std::path::Path::new("/tmp/outside_workspace_evil.txt").exists(),
            "file outside workspace must not be created"
        );
    }

    // ── Command length limit test ────────────────────────────────────────

    #[tokio::test]
    async fn run_command_rejects_oversized_command() {
        let long_cmd = "a".repeat(MAX_COMMAND_LEN + 1);
        let args = format!(r#"{{"command": "{}"}}"#, long_cmd);
        let result = execute("test-3", "run_command", &args).await;
        assert!(!result.success, "oversized command should be rejected");
        assert!(
            result.error.contains("command too long"),
            "error should mention 'command too long', got: {}",
            result.error
        );
    }

    #[tokio::test]
    async fn run_command_accepts_normal_command() {
        let result = execute("test-4", "run_command", r#"{"command": "echo hello"}"#).await;
        assert!(
            result.success,
            "normal command should succeed: {}",
            result.error
        );
        assert!(
            result.output.contains("hello"),
            "output should contain 'hello', got: {}",
            result.output
        );
    }

    // ── UTF-8 truncation safety test ─────────────────────────────────────

    #[test]
    fn truncate_ascii_works() {
        let s = "hello world";
        assert_eq!(truncate(s, 100), "hello world");
        let t = truncate(s, 5);
        assert!(
            t.starts_with("hello"),
            "should start with 'hello', got: {t}"
        );
        assert!(
            t.contains("truncated"),
            "should contain 'truncated', got: {t}"
        );
    }

    #[test]
    fn truncate_multibyte_utf8_does_not_panic() {
        // Each Chinese char is 3 bytes in UTF-8
        let s = "你好世界测试数据"; // 8 chars × 3 bytes = 24 bytes
        // Truncate at byte 7 — in the middle of the 3rd char '世'
        let result = truncate(s, 7);
        // Should NOT panic, and should truncate at a valid boundary
        assert!(
            result.contains("truncated"),
            "should be truncated, got: {result}"
        );
        // The truncated prefix should be valid UTF-8 (it is, since we're returning a String)
        assert!(
            result.starts_with("你好"),
            "should keep first 2 chars, got: {result}"
        );
    }

    #[test]
    fn truncate_emoji_boundary() {
        // 🚀 is 4 bytes in UTF-8
        let s = "A🚀B🚀C"; // 1 + 4 + 1 + 4 + 1 = 11 bytes
        // Truncate at byte 3, which is in the middle of 🚀
        let result = truncate(s, 3);
        assert!(result.contains("truncated"), "should be truncated");
        // Should only keep "A" since 🚀 starts at byte 1 and ends at byte 5
        assert!(
            result.starts_with("A"),
            "should start with 'A', got: {result}"
        );
    }

    // ── Unknown builtin test ─────────────────────────────────────────────

    #[tokio::test]
    async fn unknown_builtin_returns_error() {
        let result = execute("test-5", "evil_tool", "{}").await;
        assert!(!result.success);
        assert!(result.error.contains("unknown builtin"));
    }
}
