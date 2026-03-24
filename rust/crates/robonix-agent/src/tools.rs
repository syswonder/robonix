use anyhow::Result;
use serde::Deserialize;
use std::path::Path;

use crate::vlm::ToolDef;

#[derive(Deserialize)]
struct ReadFileArgs {
    path: String,
}

#[derive(Deserialize)]
struct WriteFileArgs {
    path: String,
    content: String,
}

#[derive(Deserialize)]
struct PatchFileArgs {
    path: String,
    old: String,
    new: String,
}

#[derive(Deserialize)]
struct ListDirArgs {
    path: Option<String>,
}

#[derive(Deserialize)]
struct RunCommandArgs {
    command: String,
}

pub fn builtin_tool_defs() -> Vec<ToolDef> {
    vec![
        ToolDef::new(
            "read_file",
            "Read a file and return its contents",
            serde_json::json!({
                "type": "object",
                "properties": {
                    "path": {"type": "string", "description": "Absolute or relative file path"}
                },
                "required": ["path"]
            }),
        ),
        ToolDef::new(
            "write_file",
            "Write content to a file (creates or overwrites)",
            serde_json::json!({
                "type": "object",
                "properties": {
                    "path": {"type": "string", "description": "File path to write to"},
                    "content": {"type": "string", "description": "Content to write"}
                },
                "required": ["path", "content"]
            }),
        ),
        ToolDef::new(
            "patch_file",
            "Replace a substring in a file (first occurrence)",
            serde_json::json!({
                "type": "object",
                "properties": {
                    "path": {"type": "string", "description": "File path"},
                    "old": {"type": "string", "description": "Text to find"},
                    "new": {"type": "string", "description": "Replacement text"}
                },
                "required": ["path", "old", "new"]
            }),
        ),
        ToolDef::new(
            "list_dir",
            "List files and directories at a path",
            serde_json::json!({
                "type": "object",
                "properties": {
                    "path": {"type": "string", "description": "Directory path (default: current dir)"}
                }
            }),
        ),
        ToolDef::new(
            "run_command",
            "Run a shell command and return stdout/stderr",
            serde_json::json!({
                "type": "object",
                "properties": {
                    "command": {"type": "string", "description": "Shell command to execute"}
                },
                "required": ["command"]
            }),
        ),
    ]
}

pub const BUILTIN_NAMES: &[&str] = &[
    "read_file", "write_file", "patch_file", "list_dir", "run_command",
];

pub async fn execute_builtin(name: &str, args_json: &str) -> Result<String> {
    match name {
        "read_file" => {
            let a: ReadFileArgs = serde_json::from_str(args_json)?;
            match std::fs::read_to_string(&a.path) {
                Ok(c) => Ok(truncate(&c, 8000)),
                Err(e) => Ok(format!("error: {e}")),
            }
        }
        "write_file" => {
            let a: WriteFileArgs = serde_json::from_str(args_json)?;
            if let Some(parent) = Path::new(&a.path).parent() {
                std::fs::create_dir_all(parent).ok();
            }
            match std::fs::write(&a.path, &a.content) {
                Ok(()) => Ok(format!("wrote {} bytes to {}", a.content.len(), a.path)),
                Err(e) => Ok(format!("error: {e}")),
            }
        }
        "patch_file" => {
            let a: PatchFileArgs = serde_json::from_str(args_json)?;
            match std::fs::read_to_string(&a.path) {
                Ok(content) => {
                    if !content.contains(&a.old) {
                        return Ok("error: old string not found in file".into());
                    }
                    let patched = content.replacen(&a.old, &a.new, 1);
                    std::fs::write(&a.path, &patched)?;
                    Ok(format!("patched {}", a.path))
                }
                Err(e) => Ok(format!("error: {e}")),
            }
        }
        "list_dir" => {
            let a: ListDirArgs = serde_json::from_str(args_json)?;
            let dir = a.path.as_deref().unwrap_or(".");
            match std::fs::read_dir(dir) {
                Ok(entries) => {
                    let mut items: Vec<String> = Vec::new();
                    for e in entries.flatten() {
                        let ft = e.file_type().map(|t| if t.is_dir() { "dir" } else { "file" }).unwrap_or("?");
                        items.push(format!("{} {}", ft, e.file_name().to_string_lossy()));
                    }
                    items.sort();
                    Ok(items.join("\n"))
                }
                Err(e) => Ok(format!("error: {e}")),
            }
        }
        "run_command" => {
            let a: RunCommandArgs = serde_json::from_str(args_json)?;
            let output = tokio::process::Command::new("bash")
                .arg("-c")
                .arg(&a.command)
                .output()
                .await?;
            let stdout = String::from_utf8_lossy(&output.stdout);
            let stderr = String::from_utf8_lossy(&output.stderr);
            let mut result = String::new();
            if !stdout.is_empty() {
                result.push_str(&truncate(&stdout, 4000));
            }
            if !stderr.is_empty() {
                if !result.is_empty() { result.push('\n'); }
                result.push_str("stderr: ");
                result.push_str(&truncate(&stderr, 2000));
            }
            if result.is_empty() {
                result = format!("exit code: {}", output.status.code().unwrap_or(-1));
            }
            Ok(result)
        }
        _ => Ok(format!("unknown builtin tool: {name}")),
    }
}

fn truncate(s: &str, max: usize) -> String {
    if s.len() <= max {
        s.to_string()
    } else {
        format!("{}…(truncated, {} bytes total)", &s[..max], s.len())
    }
}
