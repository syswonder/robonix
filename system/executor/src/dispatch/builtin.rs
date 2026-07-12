// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// dispatch/builtin.rs — built-in tool implementations

use crate::pb::pilot::CapabilityCallResult;
use crate::plan_runtime::PlanRuntime;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use serde::Deserialize;
use std::path::{Path, PathBuf};

/// Workspace root for file operations. All file paths are resolved relative to
/// this directory, and path traversal beyond it is rejected.
/// > `wheatfox's note: the definition of this "workspace" is where all the built-in tools
/// > like read_file, write_file, exec command will be ran, as the linux CWD`
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

use crate::pb::pilot::CapabilityCall;

/// One in-process builtin (no network, runs in executor's own process).
/// `call.contract_id`'s last segment names the operation —
/// e.g. `robonix/system/executor/builtin/read_file` → `read_file`.
pub async fn execute(
    call: &CapabilityCall,
    runtime: &PlanRuntime,
    self_provider_id: &str,
    atlas: &mut AtlasClient,
) -> CapabilityCallResult {
    let op = call
        .contract_id
        .rsplit_once('/')
        .map(|(_, leaf)| leaf)
        .unwrap_or(call.contract_id.as_str());
    if op == "cancel_plan" {
        return runtime
            .cancel_plan_builtin(call, self_provider_id, atlas)
            .await;
    }
    if op == "stop_plan_at" {
        return runtime.stop_plan_at_builtin(call).await;
    }
    if op == "stop_after_current" {
        return runtime.stop_after_current_builtin(call).await;
    }
    if op == "get_plan_status" {
        return runtime.get_plan_status_builtin(call).await;
    }
    if op == "get_all_plans" {
        return runtime.get_all_plans_builtin(call).await;
    }
    if op == "read_capability_doc" {
        return read_capability_doc(call, atlas).await;
    }

    let result = run(op, &call.args_json).await;
    let mut out = CapabilityCallResult {
        call_id: call.call_id.clone(),
        provider_id: call.provider_id.clone(),
        contract_id: call.contract_id.clone(),
        ..Default::default()
    };
    match result {
        Ok(s) => {
            out.success = true;
            out.output = s;
        }
        Err(e) => {
            out.success = false;
            out.error = e.to_string();
        }
    }
    out
}

async fn run(op: &str, args_json: &str) -> anyhow::Result<String> {
    match op {
        "read_file" => read_file(args_json),
        "write_file" => write_file(args_json),
        "patch_file" => patch_file(args_json),
        "list_dir" => list_dir(args_json),
        "run_command" => run_command(args_json).await,
        other => anyhow::bail!("unknown builtin: {}", other),
    }
}

/// Static metadata for the builtin ops. Used by main.rs to declare them
/// against atlas at startup so pilot can discover them like any other provider.
pub struct BuiltinSpec {
    pub op: &'static str,
    pub description: &'static str,
    pub input_schema_json: &'static str,
}

pub const BUILTINS: &[BuiltinSpec] = &[
    BuiltinSpec {
        op: "read_file",
        description: "Read a file and return its contents",
        input_schema_json: r#"{"type":"object","properties":{"path":{"type":"string","description":"Absolute or relative file path"}},"required":["path"]}"#,
    },
    BuiltinSpec {
        op: "write_file",
        description: "Write content to a file (creates or overwrites)",
        input_schema_json: r#"{"type":"object","properties":{"path":{"type":"string"},"content":{"type":"string"}},"required":["path","content"]}"#,
    },
    BuiltinSpec {
        op: "patch_file",
        description: "Replace the first occurrence of a string in a file",
        input_schema_json: r#"{"type":"object","properties":{"path":{"type":"string"},"old":{"type":"string"},"new":{"type":"string"}},"required":["path","old","new"]}"#,
    },
    BuiltinSpec {
        op: "list_dir",
        description: "List files and directories at a path",
        input_schema_json: r#"{"type":"object","properties":{"path":{"type":"string","description":"Directory path (default: current dir)"}}}"#,
    },
    BuiltinSpec {
        op: "run_command",
        description: "Run a shell command and return stdout/stderr",
        input_schema_json: r#"{"type":"object","properties":{"command":{"type":"string"}},"required":["command"]}"#,
    },
    BuiltinSpec {
        op: "cancel_plan",
        description: "Cancellation for an in-flight RTDL plan by plan_id",
        input_schema_json: r#"{"type":"object","properties":{"plan_id":{"type":"string","description":"RTDL Plan.plan_id to cancel"},"wait_ms":{"type":"integer","minimum":0,"description":"Optional milliseconds to wait for the target plan to stop; default 5000"}},"required":["plan_id"]}"#,
    },
    BuiltinSpec {
        op: "get_all_plans",
        description: "List every in-flight RTDL plan with its plan_id, a short description of the task, op_count, cancelled flag, and number of armed stop points. Call this to discover which plans are currently running, then inspect one with get_plan_status before stopping it. Takes no arguments.",
        input_schema_json: r#"{"type":"object","properties":{}}"#,
    },
    BuiltinSpec {
        op: "get_plan_status",
        description: "Inspect an in-flight RTDL plan: returns its ops, each with op_id, kind, description, current state (pending/running/succeeded/failed/canceled/timeout/paused) and any armed stop_point. Call this to find the op_id and live progress of a running plan before issuing stop_plan_at or cancel_plan. Errors if the plan is not active (stale/wrong id or already finished) — use get_all_plans to list running plans.",
        input_schema_json: r#"{"type":"object","properties":{"plan_id":{"type":"string","description":"RTDL Plan.plan_id to inspect"}},"required":["plan_id"]}"#,
    },
    BuiltinSpec {
        op: "stop_plan_at",
        description: "Set a stop point on an in-flight RTDL plan: when execution reaches the op with the given op_id, cancel the whole plan. Use 'on_complete' (default) to stop right after that op finishes, or 'on_enter' to stop the moment it is reached, before it runs. op_ids are the per-node identifiers shown in RTDL node_state events.",
        input_schema_json: r#"{"type":"object","properties":{"plan_id":{"type":"string","description":"RTDL Plan.plan_id to set the stop point on"},"op_id":{"type":"string","description":"Node op_id at which to stop (cancel) the plan"},"when":{"type":"string","enum":["on_enter","on_complete"],"description":"on_enter = before the op runs; on_complete = after it finishes. Default on_complete."}},"required":["plan_id","op_id"]}"#,
    },
    BuiltinSpec {
        op: "stop_after_current",
        description: "Atomically stop one in-flight sequential RTDL plan after its currently running step completes. Use this directly when the user says 'finish the current step, then stop' or 'do not start the next step'. It takes only plan_id and avoids a get_plan_status round trip. If execution is between steps it stops before the next pending step. It rejects ambiguous parallel plans with multiple running leaves; only then inspect and use stop_plan_at.",
        input_schema_json: r#"{"type":"object","properties":{"plan_id":{"type":"string","description":"RTDL Plan.plan_id whose current sequential step may finish, after which the plan stops"}},"required":["plan_id"]}"#,
    },
    BuiltinSpec {
        op: "read_capability_doc",
        description: "Read a capability provider's full CAPABILITY.md manual by provider_id. Call this to learn how to use a provider before invoking it. Only providers listed as having a doc carry one; never guess or read a file path for docs.",
        input_schema_json: r#"{"type":"object","properties":{"provider_id":{"type":"string","description":"The provider_id whose CAPABILITY.md to read"}},"required":["provider_id"]}"#,
    },
];

#[derive(Deserialize)]
struct DocArgs {
    provider_id: String,
}

/// Apply the `read_capability_doc` builtin: fetch a provider's registered
/// CAPABILITY.md *content* from atlas and return it as markdown text.
///
/// Unlike `read_file`, this never touches the filesystem — atlas serves the
/// content the provider sent at registration, so it works regardless of the
/// provider's (possibly containerised) mount layout. Errors when the provider
/// is unknown or registered no doc; output is truncated to keep prompt size
/// bounded.
async fn read_capability_doc(
    call: &CapabilityCall,
    atlas: &mut AtlasClient,
) -> CapabilityCallResult {
    let mut out = CapabilityCallResult {
        call_id: call.call_id.clone(),
        provider_id: call.provider_id.clone(),
        contract_id: call.contract_id.clone(),
        ..Default::default()
    };
    let provider_id = match serde_json::from_str::<DocArgs>(&call.args_json) {
        Ok(a) => a.provider_id.trim().to_string(),
        Err(e) => {
            out.error = format!("invalid read_capability_doc args: {e}");
            return out;
        }
    };
    if provider_id.is_empty() {
        out.error = "read_capability_doc: provider_id is required".to_string();
        return out;
    }
    match atlas
        .query_capabilities(&provider_id, "", atlas_pb::Transport::Unspecified)
        .await
    {
        Ok(providers) => match providers.iter().find(|p| p.id == provider_id) {
            Some(p) if !p.capability_md.is_empty() => {
                out.success = true;
                out.output = truncate(&p.capability_md, 12000);
            }
            Some(_) => {
                out.error = format!("provider '{provider_id}' registered no CAPABILITY.md");
            }
            None => {
                out.error = format!("no provider '{provider_id}' registered in atlas");
            }
        },
        Err(e) => {
            out.error = format!("atlas query for '{provider_id}' failed: {e}");
        }
    }
    out
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
    // TODO: use standard tools like sed, awk, etc.
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
        // Each euro sign is 3 bytes in UTF-8
        let s = "€€€€€€€€"; // 8 chars × 3 bytes = 24 bytes
        // Truncate at byte 7 — in the middle of the 3rd char '€'
        let result = truncate(s, 7);
        // Should NOT panic, and should truncate at a valid boundary
        assert!(
            result.contains("truncated"),
            "should be truncated, got: {result}"
        );
        // The truncated prefix should be valid UTF-8 (it is, since we're returning a String)
        assert!(
            result.starts_with("€€"),
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
}
