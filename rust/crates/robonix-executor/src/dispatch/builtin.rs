// SPDX-License-Identifier: MulanPSL-2.0
// dispatch/builtin.rs — built-in tool implementations

use crate::pilot::TaskCallResult;
use serde::Deserialize;
use std::path::Path;

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
        format!("{}…(truncated, {} bytes total)", &s[..max], s.len())
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
    Ok(truncate(&std::fs::read_to_string(&a.path)?, 8000))
}

fn write_file(args: &str) -> anyhow::Result<String> {
    let a: WriteArgs = serde_json::from_str(args)?;
    if let Some(parent) = Path::new(&a.path).parent() {
        std::fs::create_dir_all(parent).ok();
    }
    std::fs::write(&a.path, &a.content)?;
    Ok(format!("wrote {} bytes to {}", a.content.len(), a.path))
}

fn patch_file(args: &str) -> anyhow::Result<String> {
    let a: PatchArgs = serde_json::from_str(args)?;
    let content = std::fs::read_to_string(&a.path)?;
    if !content.contains(&a.old) {
        anyhow::bail!("old string not found in file");
    }
    std::fs::write(&a.path, content.replacen(&a.old, &a.new, 1))?;
    Ok(format!("patched {}", a.path))
}

fn list_dir(args: &str) -> anyhow::Result<String> {
    let a: ListArgs = serde_json::from_str(args)?;
    let dir = a.path.as_deref().unwrap_or(".");
    let mut items: Vec<String> = std::fs::read_dir(dir)?
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

async fn run_command(args: &str) -> anyhow::Result<String> {
    let a: CmdArgs = serde_json::from_str(args)?;
    let out = tokio::process::Command::new("bash")
        .arg("-c")
        .arg(&a.command)
        .output()
        .await?;
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
