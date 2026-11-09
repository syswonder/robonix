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
    safe_resolve_in(&workspace_root(), user_path)
}

/// [`safe_resolve`] against an explicit root. Tests call this directly: the
/// workspace root comes from a process-wide environment variable, and setting
/// it from parallel tests leaks one test's root into another's file ops.
fn safe_resolve_in(workspace: &Path, user_path: &str) -> anyhow::Result<PathBuf> {
    let root = workspace
        .canonicalize()
        .unwrap_or_else(|_| workspace.to_path_buf());
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
    plan_id: &str,
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
    if op == "get_plan_status" {
        return runtime.get_plan_status_builtin(call).await;
    }
    if op == "get_all_plans" {
        return runtime.get_all_plans_builtin(call).await;
    }
    if op == "read_capability_doc" {
        return read_capability_doc(call, atlas).await;
    }

    let result = run(op, &call.args_json, runtime, plan_id).await;
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

async fn run(
    op: &str,
    args_json: &str,
    runtime: &PlanRuntime,
    plan_id: &str,
) -> anyhow::Result<String> {
    match op {
        "read_file" => read_file(args_json),
        "write_file" => write_file(args_json),
        "patch_file" => patch_file(args_json),
        "list_dir" => list_dir(args_json),
        "run_command" => run_command(args_json, runtime, plan_id).await,
        other => anyhow::bail!("unknown builtin: {}", other),
    }
}

/// Static metadata for the builtin ops. Used by main.rs to declare them
/// against atlas at startup so pilot can discover them like any other provider.
pub struct BuiltinSpec {
    /// Operation name used as the builtin contract's final path segment.
    pub op: &'static str,
    /// Human-readable capability description advertised through Atlas.
    pub description: &'static str,
    /// JSON Schema describing the arguments accepted by this builtin.
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
        description: "List every in-flight RTDL plan with its plan_id, a short description of the task, op_count, and cancelled flag. Call this to discover which plans are currently running, then inspect one with get_plan_status before stopping it. Takes no arguments.",
        input_schema_json: r#"{"type":"object","properties":{}}"#,
    },
    BuiltinSpec {
        op: "get_plan_status",
        description: "Inspect an in-flight RTDL plan: returns its ops, each with op_id, kind, description, current state (pending/running/verifying/succeeded/failed/canceled/timeout/paused). Call this to inspect live progress before issuing cancel_plan. Errors if the plan is not active (stale/wrong id or already finished) — use get_all_plans to list running plans.",
        input_schema_json: r#"{"type":"object","properties":{"plan_id":{"type":"string","description":"RTDL Plan.plan_id to inspect"}},"required":["plan_id"]}"#,
    },
    BuiltinSpec {
        op: "read_capability_doc",
        description: "Read a capability provider's full CAPABILITY.md manual by provider_id. Call this to learn how to use a provider before invoking it. Only providers listed as having a doc carry one; never guess or read a file path for docs.",
        input_schema_json: r#"{"type":"object","properties":{"provider_id":{"type":"string","description":"The provider_id whose CAPABILITY.md to read"}},"required":["provider_id"]}"#,
    },
];

/// JSON arguments selecting whose registered capability documentation to read.
#[derive(Deserialize)]
struct DocArgs {
    /// Provider ID used to look up the registered CAPABILITY.md content in Atlas.
    provider_id: String,
}

/// Apply the `read_capability_doc` builtin: fetch a provider's registered
/// CAPABILITY.md *content* from atlas and return it as markdown text.
///
/// Unlike `read_file`, this never touches the filesystem — atlas serves the
/// content the provider sent at registration, so it works regardless of the
/// provider's (possibly containerised) mount layout. Errors when the provider
/// is unknown or registered no doc. Long results are retained as files when
/// the model-visible event is prepared.
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
            Some(p) => {
                let doc = render_provider_doc(p);
                if doc.is_empty() {
                    out.error = format!("provider '{provider_id}' published no documentation");
                } else {
                    out.success = true;
                    out.output = doc;
                }
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

/// Everything atlas knows that helps a caller use `provider`: its CAPABILITY.md
/// when it registered one, followed by the full description of each capability
/// the provider offers.
///
/// The planning catalogue prints only the opening paragraph of a description,
/// so this is where a caller reads the rest. Serving the descriptions as well
/// as the manual matters because a provider registers its descriptions from its
/// contracts, always, while CAPABILITY.md is hand-written and usually absent —
/// answering only from the manual left this builtin with nothing to return for
/// most providers.
fn render_provider_doc(provider: &atlas_pb::CapabilityProvider) -> String {
    let mut doc = String::new();
    let manual = provider.capability_md.trim();
    if !manual.is_empty() {
        doc.push_str(manual);
        doc.push('\n');
    }
    for cap in &provider.capabilities {
        let description = cap.description.trim();
        if description.is_empty() {
            continue;
        }
        doc.push_str(&format!("\n## {}\n\n{}\n", cap.contract_id, description));
    }
    doc
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// JSON arguments for reading a text file within the workspace.
#[derive(Deserialize)]
struct ReadArgs {
    /// File to read, resolved against and confined to the workspace root.
    path: String,
}
/// JSON arguments for creating or overwriting a text file within the workspace.
#[derive(Deserialize)]
struct WriteArgs {
    /// Destination file, resolved against and confined to the workspace root.
    path: String,
    /// Text written as the file's complete contents.
    content: String,
}
/// JSON arguments for replacing the first matching text in a workspace file.
#[derive(Deserialize)]
struct PatchArgs {
    /// File to modify, resolved against and confined to the workspace root.
    path: String,
    /// Literal text whose first occurrence is replaced.
    old: String,
    /// Replacement text inserted in place of the matched occurrence.
    new: String,
}
/// JSON arguments for listing entries in a workspace directory.
#[derive(Deserialize)]
struct ListArgs {
    /// Directory to list relative to the workspace root; omission selects the root.
    path: Option<String>,
}
/// JSON arguments for executing a shell command through the builtin.
#[derive(Deserialize)]
struct CmdArgs {
    /// Shell source passed to `bash -c` for execution.
    command: String,
}

fn read_file(args: &str) -> anyhow::Result<String> {
    let a: ReadArgs = serde_json::from_str(args)?;
    let path = safe_resolve(&a.path)?;
    if std::fs::metadata(&path)?.len() > crate::output_view::inline_bytes() as u64 {
        // Point at the file itself rather than copying it.
        let mut reference = crate::output_view::describe(&path)?;
        reference["note"] = "Too long to inline. Search this file with run_command (rg -n, or \
            grep -n -F), then read the matching lines with sed -n."
            .into();
        return Ok(reference.to_string());
    }
    Ok(std::fs::read_to_string(path)?)
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
async fn run_command(args: &str, runtime: &PlanRuntime, plan_id: &str) -> anyhow::Result<String> {
    let dir = crate::output_view::output_dir().map_err(|error| {
        anyhow::anyhow!("command not started: the temporary directory is unusable: {error}")
    })?;
    run_command_in(
        args,
        runtime,
        plan_id,
        &dir,
        crate::output_view::inline_bytes(),
    )
    .await
}

/// Run a shell command with stdout and stderr copied to files, so Executor
/// holds almost none of its output in memory. Short output is read back
/// inline; long output stays on disk and is described for searching.
async fn run_command_in(
    args: &str,
    runtime: &PlanRuntime,
    plan_id: &str,
    dir: &Path,
    inline: usize,
) -> anyhow::Result<String> {
    let a: CmdArgs = serde_json::from_str(args)?;
    if a.command.len() > MAX_COMMAND_LEN {
        anyhow::bail!(
            "command too long ({} bytes, max {})",
            a.command.len(),
            MAX_COMMAND_LEN
        );
    }
    if runtime.is_cancelled(plan_id).await {
        anyhow::bail!("command canceled before spawn");
    }

    let (stdout_path, stdout_file) = crate::output_view::create(dir)?;
    let (stderr_path, stderr_file) = crate::output_view::create(dir)?;
    let paths = [stdout_path.as_path(), stderr_path.as_path()];
    use std::process::Stdio;
    let mut command = tokio::process::Command::new("bash");
    command
        .arg("-c")
        .arg(&a.command)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .kill_on_drop(true)
        .process_group(0);
    let mut child = match command.spawn() {
        Ok(child) => child,
        Err(error) => {
            paths
                .iter()
                .for_each(|path| drop(std::fs::remove_file(path)));
            anyhow::bail!("failed to execute command: {error}");
        }
    };
    let pgid = child
        .id()
        .ok_or_else(|| anyhow::anyhow!("spawned command has no pid"))?;
    let (stdout, stderr) = (child.stdout.take(), child.stderr.take());
    let output = async move {
        let (stdout, stderr, status) = tokio::join!(
            spool(stdout, stdout_file),
            spool(stderr, stderr_file),
            child.wait()
        );
        std::io::Result::Ok((stdout?.or(stderr?), status?))
    };
    tokio::pin!(output);

    enum StopReason {
        Canceled,
        Timeout,
    }
    let canceled = async {
        loop {
            if runtime.is_cancelled(plan_id).await {
                break;
            }
            tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        }
    };
    tokio::pin!(canceled);
    let deadline = tokio::time::sleep(COMMAND_TIMEOUT);
    tokio::pin!(deadline);

    let stop_reason = tokio::select! {
        result = &mut output => {
            let (failure, status) =
                result.map_err(|e| anyhow::anyhow!("failed to execute command: {e}"))?;
            return Ok(command_result(&paths, status.code(), failure, inline)?);
        }
        _ = &mut canceled => StopReason::Canceled,
        _ = &mut deadline => StopReason::Timeout,
    };

    terminate_command_group(pgid, nix::sys::signal::Signal::SIGTERM);
    let finished = match tokio::time::timeout(std::time::Duration::from_secs(2), &mut output).await
    {
        Ok(result) => result,
        Err(_) => {
            terminate_command_group(pgid, nix::sys::signal::Signal::SIGKILL);
            output.await
        }
    };
    let reason = match stop_reason {
        StopReason::Canceled => "command canceled".to_owned(),
        StopReason::Timeout => format!("command timed out after {}s", COMMAND_TIMEOUT.as_secs()),
    };
    let failure = finished.ok().and_then(|(failure, _)| failure);
    anyhow::bail!(
        "{reason}; output written before it stopped:\n{}",
        command_result(&paths, None, failure, inline)?
    )
}

/// Copy a command stream to its file. A failed write, such as a full temporary
/// directory, is returned and the rest is drained, so storing output never
/// blocks or breaks the command.
async fn spool(
    stream: Option<impl tokio::io::AsyncRead + Unpin>,
    file: std::fs::File,
) -> std::io::Result<Option<std::io::Error>> {
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    let mut file = tokio::fs::File::from_std(file);
    let mut failure = None;
    if let Some(mut stream) = stream {
        let mut buffer = vec![0; 64 * 1024];
        loop {
            let count = stream.read(&mut buffer).await?;
            if count == 0 {
                break;
            }
            if failure.is_none() {
                failure = file.write_all(&buffer[..count]).await.err();
            }
        }
    }
    Ok(failure.or(file.flush().await.err()))
}

/// Inline both streams when together they fit the inline limit (deleting the
/// files); otherwise keep the non-empty files and describe them. A storage
/// failure is always reported, because the files are then incomplete.
fn command_result(
    paths: &[&Path; 2],
    code: Option<i32>,
    failure: Option<std::io::Error>,
    inline: usize,
) -> std::io::Result<String> {
    let sizes = paths.map(|path| std::fs::metadata(path).map_or(0, |metadata| metadata.len()));
    if failure.is_none() && sizes.iter().sum::<u64>() <= inline as u64 {
        let text =
            format_command_output(&std::fs::read(paths[0])?, &std::fs::read(paths[1])?, code);
        paths
            .iter()
            .for_each(|path| drop(std::fs::remove_file(path)));
        return Ok(text);
    }
    let mut reference = serde_json::json!({
        "observed_unix": crate::output_view::now_unix(),
        "exit_code": code,
        "note": crate::output_view::SEARCH_NOTE,
    });
    if let Some(error) = failure {
        robonix_scribe::warn!("[executor] command output is incomplete: {error}");
        reference["storage_error"] = error.to_string().into();
        reference["note"] = crate::output_view::STORAGE_NOTE.into();
    }
    for (name, path, size) in [
        ("stdout", paths[0], sizes[0]),
        ("stderr", paths[1], sizes[1]),
    ] {
        if size == 0 {
            let _ = std::fs::remove_file(path);
        } else {
            reference[name] = crate::output_view::describe(path)?;
        }
    }
    Ok(reference.to_string())
}

fn terminate_command_group(pgid: u32, signal: nix::sys::signal::Signal) {
    use nix::sys::signal::killpg;
    use nix::unistd::Pid;
    match killpg(Pid::from_raw(pgid as i32), signal) {
        Ok(()) | Err(nix::errno::Errno::ESRCH) => {}
        Err(error) => robonix_scribe::warn!(
            "[executor] signal {:?} command pgid {} failed: {}",
            signal,
            pgid,
            error
        ),
    }
}

/// Stdout, then stderr, then the exit code, which search commands need: grep
/// and rg exit 1 for no match and higher for errors.
fn format_command_output(stdout: &[u8], stderr: &[u8], code: Option<i32>) -> String {
    let mut result = String::from_utf8_lossy(stdout).into_owned();
    if !stderr.is_empty() {
        if !result.is_empty() && !result.ends_with('\n') {
            result.push('\n');
        }
        result.push_str("stderr: ");
        result.push_str(&String::from_utf8_lossy(stderr));
    }
    if !result.is_empty() && !result.ends_with('\n') {
        result.push('\n');
    }
    result.push_str(&format!("exit code: {}", code.unwrap_or(-1)));
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    async fn run(command: &str, dir: &Path, inline: usize) -> anyhow::Result<String> {
        let args = serde_json::json!({ "command": command }).to_string();
        run_command_in(&args, &PlanRuntime::default(), "test", dir, inline).await
    }

    #[tokio::test]
    async fn short_output_is_inline_with_its_exit_code() {
        let dir = crate::output_view::tests::TempDir::new();
        let no_match = run("printf 'alpha\\n' | grep -n -F -e absent", &dir.0, 1024).await;
        assert_eq!(no_match.unwrap(), "exit code: 1");
        let failed = run("printf 'diagnostic\\n' >&2; exit 2", &dir.0, 1024)
            .await
            .unwrap();
        assert_eq!(failed, "stderr: diagnostic\nexit code: 2");
        assert_eq!(std::fs::read_dir(&dir.0).unwrap().count(), 0);
    }

    #[tokio::test]
    async fn long_output_is_written_to_disk_whole_and_the_command_runs_once() {
        let dir = crate::output_view::tests::TempDir::new();
        let count = dir.0.join("count");
        let command = format!(
            "echo run >> '{}'; head -c 20000000 /dev/zero | tr '\\0' x; echo TAIL-972",
            count.display()
        );
        let reference: serde_json::Value =
            serde_json::from_str(&run(&command, &dir.0, 1024).await.unwrap()).unwrap();
        assert_eq!(reference["exit_code"], 0);
        assert!(reference.get("stderr").is_none());
        assert_eq!(reference["stdout"]["bytes"], 20_000_009);
        let saved = std::fs::read(reference["stdout"]["file"].as_str().unwrap()).unwrap();
        assert_eq!(saved.len(), 20_000_009);
        assert!(saved.ends_with(b"xTAIL-972\n"));
        assert_eq!(std::fs::read_to_string(count).unwrap(), "run\n");
    }

    #[cfg(target_os = "linux")]
    #[tokio::test]
    async fn a_full_disk_is_reported_and_the_stream_is_still_drained() {
        let full = std::fs::OpenOptions::new()
            .write(true)
            .open("/dev/full")
            .unwrap();
        let data = vec![b'x'; 300_000];
        let failure = spool(Some(&data[..]), full).await.unwrap().unwrap();
        assert_eq!(failure.raw_os_error(), Some(28));
    }

    #[test]
    fn storage_failure_keeps_what_was_written_and_tells_the_model() {
        let dir = crate::output_view::tests::TempDir::new();
        let (stdout, stderr) = (dir.0.join("out"), dir.0.join("err"));
        std::fs::write(&stdout, "partial\n").unwrap();
        std::fs::write(&stderr, "").unwrap();
        let text = command_result(
            &[stdout.as_path(), stderr.as_path()],
            Some(0),
            Some(std::io::Error::from_raw_os_error(28)),
            1024,
        )
        .unwrap();
        let reference: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert!(!reference["storage_error"].as_str().unwrap().is_empty());
        assert_eq!(reference["stdout"]["bytes"], 8);
        assert!(stdout.exists());
    }

    /// Run environment-mutating tests in a child process so concurrent execution tests
    /// never inherit a workspace directory that another test removes.
    fn isolated_workspace_test(name: &str) -> bool {
        if std::env::var("RBNX_TEST_WORKSPACE_CASE").as_deref() == Ok(name) {
            return false;
        }
        let status = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                &format!("dispatch::builtin::tests::{name}"),
                "--nocapture",
            ])
            .env("RBNX_TEST_WORKSPACE_CASE", name)
            .status()
            .unwrap();
        assert!(status.success(), "isolated workspace test failed: {name}");
        true
    }

    /// Check workspace path handling with an isolated process environment.
    #[test]
    fn a_provider_without_a_manual_still_documents_its_capabilities() {
        // Most providers never hand-write a CAPABILITY.md, but every provider
        // registers its capabilities' descriptions. Answering only from the
        // manual left this builtin with nothing to return for those providers,
        // which is exactly when the planner asks.
        let provider = atlas_pb::CapabilityProvider {
            id: "memgraph".to_string(),
            capability_md: String::new(),
            capabilities: vec![atlas_pb::Capability {
                contract_id: "robonix/service/memory/hybrid_search".to_string(),
                description: "Search memory.\n\nRequest JSON: {...}".to_string(),
                ..Default::default()
            }],
            ..Default::default()
        };
        let doc = render_provider_doc(&provider);
        assert!(doc.contains("robonix/service/memory/hybrid_search"));
        assert!(doc.contains("Request JSON: {...}"));
    }

    #[test]
    fn a_provider_with_neither_manual_nor_descriptions_documents_nothing() {
        let provider = atlas_pb::CapabilityProvider {
            id: "bare".to_string(),
            ..Default::default()
        };
        assert!(render_provider_doc(&provider).is_empty());
    }

    #[test]
    fn path_traversal_dotdot_is_rejected() {
        if isolated_workspace_test("path_traversal_dotdot_is_rejected") {
            return;
        }
        // Set workspace to a temp dir so we have a known root
        let tmp = std::env::temp_dir().join("rbnx_test_ws");
        std::fs::create_dir_all(&tmp).unwrap();

        // ../../../etc/passwd must be rejected
        let result = safe_resolve_in(&tmp, "../../../etc/passwd");
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

    /// Check workspace path handling with an isolated process environment.
    #[test]
    fn path_traversal_absolute_outside_workspace_is_rejected() {
        if isolated_workspace_test("path_traversal_absolute_outside_workspace_is_rejected") {
            return;
        }
        let tmp = std::env::temp_dir().join("rbnx_test_ws2");
        std::fs::create_dir_all(&tmp).unwrap();

        // /etc/hostname is a real file outside workspace
        let result = safe_resolve_in(&tmp, "/etc/hostname");
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

    /// Check workspace path handling with an isolated process environment.
    #[test]
    fn path_resolve_within_workspace_is_allowed() {
        if isolated_workspace_test("path_resolve_within_workspace_is_allowed") {
            return;
        }
        let tmp = std::env::temp_dir().join("rbnx_test_ws3");
        std::fs::create_dir_all(&tmp).unwrap();
        // Create a test file inside workspace
        let test_file = tmp.join("allowed.txt");
        std::fs::write(&test_file, "hello").unwrap();

        let result = safe_resolve_in(&tmp, "allowed.txt");
        assert!(
            result.is_ok(),
            "path within workspace should be allowed: {:?}",
            result.err()
        );

        // Cleanup
        let _ = std::fs::remove_file(&test_file);
        let _ = std::fs::remove_dir(&tmp);
    }
}
