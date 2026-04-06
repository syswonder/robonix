// SPDX-License-Identifier: MulanPSL-2.0
// dispatch/builtin.rs — built-in tool implementations

use crate::pilot::TaskCallResult;
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

pub async fn execute(call_id: &str, name: &str, args_json: &str) -> TaskCallResult {
    let output = run(name, args_json).await;
    match output {
        Ok(out) => TaskCallResult {
            call_id: call_id.to_string(),
            tool_name: name.to_string(),
            success: true,
            output: out,
            error: String::new(),
        },
        Err(e) => TaskCallResult {
            call_id: call_id.to_string(),
            tool_name: name.to_string(),
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
        let end = s.floor_char_boundary(max);
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
    Ok(format!("wrote {} bytes to {}", a.content.len(), path.display()))
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
