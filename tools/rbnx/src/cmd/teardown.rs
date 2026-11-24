// SPDX-License-Identifier: MulanPSL-2.0
// Shared boot-state persistence + tear-down logic for `rbnx boot` and
// `rbnx shutdown`. Boot writes `state.json`; shutdown reads it.
//
// Scope: package runtime shutdown. Each component record carries provider
// lifecycle metadata, optional package `stop`, and wrapper PGID. Teardown uses
// the same ordered contract for Ctrl-C, `rbnx shutdown`, boot failure cleanup,
// and restart recovery: Driver(CMD_SHUTDOWN) if reachable, then manifest stop,
// then TERM/KILL of the wrapper process group.
//
// Boot also doesn't currently kill its children on the `?`-error path,
// leaking atlas/pilot/executor as orphans. Persisted state + a separate
// `rbnx shutdown` command lets the user (or boot's own error path)
// always reach the same teardown helper.

use anyhow::{Context, Result};
use robonix_cli::launch::{PackageRuntimeRecord, shutdown_package_runtime_checked};
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::path::{Path, PathBuf};
use std::time::Duration;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BootState {
    pub manifest_path: String,
    pub boot_pid: u32,
    #[serde(default)]
    pub boot_start_time_ticks: Option<u64>,
    #[serde(default)]
    pub boot_id: String,
    pub started_at_ms: u64,
    pub atlas_endpoint: String,
    pub components: Vec<ComponentRecord>,
}

/// Backward-compatible name for persisted boot components. New fields are
/// optional in JSON so old state files still deserialize.
pub type ComponentRecord = PackageRuntimeRecord;

pub fn state_path(manifest_dir: &Path) -> PathBuf {
    manifest_dir.join("rbnx-boot").join("state.json")
}

pub fn write_state(path: &Path, state: &BootState) -> Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).with_context(|| format!("create {}", parent.display()))?;
    }
    let text = serde_json::to_string_pretty(state)?;
    let temp = path.with_extension(format!("json.{}.tmp", std::process::id()));
    std::fs::write(&temp, text).with_context(|| format!("write {}", temp.display()))?;
    std::fs::rename(&temp, path)
        .with_context(|| format!("replace {} with {}", path.display(), temp.display()))?;
    Ok(())
}

pub fn read_state(path: &Path) -> Result<BootState> {
    let raw = std::fs::read_to_string(path).with_context(|| format!("read {}", path.display()))?;
    let state: BootState =
        serde_json::from_str(&raw).with_context(|| format!("parse {}", path.display()))?;
    Ok(state)
}

/// Stop each component with the canonical runtime order. Idempotent: missing
/// providers, stop hooks, or PGIDs are treated as best-effort shutdown noise.
pub async fn teardown(
    state_path: &Path,
    atlas_endpoint: Option<&str>,
    components: &[ComponentRecord],
    boot_id: Option<&str>,
) -> bool {
    // Boot, explicit shutdown and its watchdog can all reach cleanup. Only
    // one may call Drivers or signal groups at a time.
    let _lock = match cleanup_lock(state_path).await {
        Ok(lock) => lock,
        Err(error) => {
            eprintln!("shutdown lock: {error:#}");
            return false;
        }
    };
    let owned = if let Some(id) = boot_id {
        match boot_processes(id, &[]) {
            Ok(processes) => processes,
            Err(error) => {
                eprintln!("cannot inspect boot children: {error:#}");
                return false;
            }
        }
    } else {
        Vec::new()
    };
    let mut complete = true;
    // Reverse order so services/skills/primitives die before pilot/atlas.
    for c in components.iter().rev() {
        robonix_cli::output::sub_step(&format!(
            "[shutdown] {} stopping (pid={}, pgid={})",
            c.name, c.pid, c.pgid
        ));
        match tokio::time::timeout(
            Duration::from_secs(5),
            shutdown_package_runtime_checked(atlas_endpoint, c, Duration::from_secs(2), boot_id),
        )
        .await
        {
            Ok(stopped) => complete &= stopped,
            Err(_) => {
                eprintln!(
                    "shutdown {}: 5s lifecycle budget exceeded; checking owned processes",
                    c.name
                );
                if boot_id.is_none() || !cfg!(target_os = "linux") {
                    complete = false;
                }
            }
        }
    }
    if let Some(id) = boot_id {
        match stop_boot_processes(id, &owned).await {
            Ok(()) => {}
            Err(error) => {
                eprintln!("shutdown residuals: {error:#}");
                complete = false;
            }
        }
    }
    complete
}

/// Hold the deploy's boot lock before clearing logs or touching persisted state.
pub fn lock_boot(manifest: &Path) -> Result<File> {
    let manifest = manifest.canonicalize()?;
    let state = state_path(manifest.parent().context("manifest directory missing")?);
    let owner = open_lock(&state.with_file_name("boot.owner.lock"))?;
    owner
        .try_lock()
        .context("a boot is running or still stopping; wait for shutdown to finish")?;
    let cleanup = open_lock(&state.with_file_name("teardown.lock"))?;
    cleanup
        .try_lock()
        .context("previous stack cleanup is still running; not starting another stack")?;
    if state.exists() {
        anyhow::bail!(
            "previous boot state remains at {}; run rbnx shutdown -f {} before boot (logs preserved)",
            state.display(),
            manifest.display()
        );
    }
    Ok(owner)
}

pub(super) fn open_lock(path: &Path) -> Result<File> {
    std::fs::create_dir_all(path.parent().context("lock directory missing")?)?;
    Ok(std::fs::OpenOptions::new()
        .create(true)
        .truncate(false)
        .read(true)
        .write(true)
        .open(path)?)
}

async fn cleanup_lock(state: &Path) -> Result<File> {
    let file = open_lock(&state.with_file_name("teardown.lock"))?;
    loop {
        match file.try_lock() {
            Ok(()) => return Ok(file),
            Err(std::fs::TryLockError::WouldBlock) => {
                tokio::time::sleep(Duration::from_millis(100)).await
            }
            Err(error) => return Err(error.into()),
        }
    }
}

/// Detached package children have their own PGID, but inherit the exact boot id.
#[cfg(target_os = "linux")]
fn boot_processes(boot_id: &str, known: &[(u32, u64)]) -> Result<Vec<(u32, u64)>> {
    use std::os::unix::fs::MetadataExt;
    let uid = std::fs::metadata("/proc/self")?.uid();
    let mut processes = std::collections::HashMap::new();
    let mut owned = std::collections::HashSet::new();
    let expected = format!("RBNX_BOOT_ID={boot_id}");
    for entry in std::fs::read_dir("/proc")? {
        let entry = entry?;
        let Some(pid) = entry
            .file_name()
            .to_str()
            .and_then(|name| name.parse::<u32>().ok())
        else {
            continue;
        };
        if pid == std::process::id() {
            continue;
        }
        let metadata = match entry.metadata() {
            Ok(metadata) => metadata,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => continue,
            Err(error) => return Err(error.into()),
        };
        if metadata.uid() != uid {
            continue;
        }
        let stat = match std::fs::read_to_string(entry.path().join("stat")) {
            Ok(stat) => stat,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => continue,
            Err(error) => return Err(error.into()),
        };
        let fields = stat
            .rsplit_once(')')
            .context("unreadable process metadata")?
            .1
            .split_whitespace()
            .collect::<Vec<_>>();
        if matches!(fields.first().copied(), Some("Z" | "X")) {
            continue;
        }
        let parent: u32 = fields.get(1).context("missing parent pid")?.parse()?;
        let start: u64 = fields.get(19).context("missing start time")?.parse()?;
        // An unreadable environment alone proves neither ownership nor absence.
        // Before stopping parents, retain exact identities of their descendants.
        let marker = std::fs::read(entry.path().join("environ"))
            .ok()
            .is_some_and(|environment| {
                environment
                    .split(|byte| *byte == 0)
                    .any(|item| item == expected.as_bytes())
            });
        if marker || known.contains(&(pid, start)) {
            owned.insert(pid);
        }
        processes.insert(pid, (start, parent));
    }
    loop {
        let descendants = processes
            .iter()
            .filter_map(|(&pid, &(_, parent))| {
                (owned.contains(&parent) && !owned.contains(&pid)).then_some(pid)
            })
            .collect::<Vec<_>>();
        if descendants.is_empty() {
            break;
        }
        owned.extend(descendants);
    }
    Ok(owned
        .into_iter()
        .map(|pid| (pid, processes[&pid].0))
        .collect())
}

#[cfg(target_os = "linux")]
async fn stop_boot_processes(boot_id: &str, known: &[(u32, u64)]) -> Result<()> {
    use nix::sys::signal::{Signal, kill};
    use nix::unistd::Pid;
    let mut owned = known.to_vec();
    for signal in [Signal::SIGTERM, Signal::SIGKILL] {
        let processes = boot_processes(boot_id, &owned)?;
        owned = processes.clone();
        if processes.is_empty() {
            return Ok(());
        }
        eprintln!(
            "shutdown: {:?} boot-owned residual PIDs {:?}",
            signal,
            processes.iter().map(|(pid, _)| *pid).collect::<Vec<_>>()
        );
        for (pid, start) in processes {
            // Recheck identity on the action side; never kill by executable name.
            if robonix_cli::launch::proc_start_time_ticks(pid) == Some(start) {
                match kill(Pid::from_raw(pid as i32), signal) {
                    Ok(()) | Err(nix::errno::Errno::ESRCH) => {}
                    Err(error) => return Err(error.into()),
                }
            }
        }
        let deadline = tokio::time::Instant::now() + Duration::from_secs(3);
        while tokio::time::Instant::now() < deadline {
            if boot_processes(boot_id, &owned)?.is_empty() {
                return Ok(());
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    }
    anyhow::bail!(
        "boot-owned processes still live: {:?}; keeping state.json",
        boot_processes(boot_id, &owned)?
    )
}

#[cfg(not(target_os = "linux"))]
async fn stop_boot_processes(_boot_id: &str, _known: &[(u32, u64)]) -> Result<()> {
    Ok(())
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
    use super::*;
    use std::process::{Child, Command, Stdio};
    static PROCESS_TEST_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

    fn fixture() -> (PathBuf, PathBuf) {
        let dir = std::env::temp_dir().join(format!("rbnx-cleanup-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&dir).unwrap();
        let manifest = dir.join("robonix_manifest.yaml");
        std::fs::write(&manifest, "name: cleanup-test\n").unwrap();
        (dir, manifest)
    }

    fn child(id: &str, ignore_term: bool) -> Child {
        Command::new("python3")
            .args(["-u", "-c", if ignore_term {
                "import os,signal,time; os.setsid(); signal.signal(signal.SIGTERM,signal.SIG_IGN); print('ready',flush=True); time.sleep(60)"
            } else {
                "import os,time; os.setsid(); print('ready',flush=True); time.sleep(60)"
            }])
            .env("RBNX_BOOT_ID", id)
            .stdout(Stdio::piped())
            .spawn().unwrap()
    }

    fn ready(child: &mut Child) {
        use std::io::BufRead;
        let mut line = String::new();
        std::io::BufReader::new(child.stdout.take().unwrap())
            .read_line(&mut line)
            .unwrap();
        assert_eq!(line.trim(), "ready");
    }

    #[test]
    fn duplicate_and_stale_boot_fail_before_logs_are_cleared() {
        let (dir, manifest) = fixture();
        let owner = lock_boot(&manifest).unwrap();
        assert!(lock_boot(&manifest).is_err());
        drop(owner);
        let state = state_path(&dir);
        std::fs::write(&state, "old state; must not overwrite").unwrap();
        let log = state.parent().unwrap().join("logs/atlas.log");
        std::fs::create_dir_all(log.parent().unwrap()).unwrap();
        std::fs::write(&log, "old failure evidence").unwrap();
        assert!(lock_boot(&manifest).is_err());
        assert_eq!(
            std::fs::read_to_string(log).unwrap(),
            "old failure evidence"
        );
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[tokio::test]
    async fn cleanup_lock_blocks_boot_and_serializes_shutdown() {
        let (dir, manifest) = fixture();
        let state = state_path(&dir);
        let first = cleanup_lock(&state).await.unwrap();
        assert!(lock_boot(&manifest).is_err());
        assert!(
            tokio::time::timeout(Duration::from_millis(150), cleanup_lock(&state))
                .await
                .is_err()
        );
        drop(first);
        let second = cleanup_lock(&state).await.unwrap();
        drop(second);
        assert!(lock_boot(&manifest).is_ok());
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[tokio::test]
    async fn detached_children_are_reaped_without_touching_another_boot() {
        let _serial = PROCESS_TEST_LOCK.lock().await;
        let id = format!("cleanup-{}", uuid::Uuid::new_v4());
        let other = format!("foreign-{}", uuid::Uuid::new_v4());
        let mut normal = child(&id, false);
        let mut stubborn = child(&id, true);
        let mut foreign = child(&other, false);
        ready(&mut normal);
        ready(&mut stubborn);
        ready(&mut foreign);
        let known = boot_processes(&id, &[]).unwrap();
        assert_eq!(known.len(), 2);
        let result = stop_boot_processes(&id, &known).await;
        // Reap fixtures before checking the shutdown result.
        let _ = normal.kill();
        let _ = stubborn.kill();
        normal.wait().unwrap();
        stubborn.wait().unwrap();
        let foreign_alive = foreign.try_wait().unwrap().is_none();
        foreign.kill().unwrap();
        foreign.wait().unwrap();
        result.unwrap();
        assert!(foreign_alive);
        assert!(boot_processes(&id, &[]).unwrap().is_empty());
    }
    #[tokio::test]
    async fn opaque_descendant_is_stopped_but_foreign_opaque_process_is_preserved() {
        let _serial = PROCESS_TEST_LOCK.lock().await;
        let id = format!("opaque-{}", uuid::Uuid::new_v4());
        let child_code = "import ctypes,os,time; os.setsid(); assert ctypes.CDLL(None).prctl(4,0,0,0,0)==0; print('ready',flush=True); time.sleep(60)";
        let mut wrapper = Command::new("python3").args(["-u", "-c",
            "import subprocess,sys,time; p=subprocess.Popen([sys.executable,'-u','-c',sys.argv[1]],stdout=subprocess.PIPE); assert p.stdout.readline().strip()==b'ready'; print(p.pid,flush=True); time.sleep(60)", child_code])
            .env("RBNX_BOOT_ID", &id).stdout(Stdio::piped()).spawn().unwrap();
        use std::io::BufRead;
        let mut line = String::new();
        std::io::BufReader::new(wrapper.stdout.take().unwrap())
            .read_line(&mut line)
            .unwrap();
        let opaque_pid: u32 = line.trim().parse().unwrap();
        let mut foreign = Command::new("python3")
            .args(["-u", "-c", child_code])
            .env("RBNX_BOOT_ID", "foreign-opaque")
            .stdout(Stdio::piped())
            .spawn()
            .unwrap();
        ready(&mut foreign);
        let known = boot_processes(&id, &[]).unwrap();
        let tracked = known.iter().any(|(pid, _)| *pid == opaque_pid);
        let result = stop_boot_processes(&id, &known).await;
        let _ = wrapper.kill();
        wrapper.wait().unwrap();
        let foreign_alive = foreign.try_wait().unwrap().is_none();
        foreign.kill().unwrap();
        foreign.wait().unwrap();
        assert!(tracked);
        result.unwrap();
        assert!(foreign_alive);
    }

    #[tokio::test]
    async fn timed_out_explicit_stop_hook_cannot_survive_cleanup() {
        let _serial = PROCESS_TEST_LOCK.lock().await;
        let (dir, _manifest) = fixture();
        let id = format!("hook-{}", uuid::Uuid::new_v4());
        let mut process = child(&id, false);
        ready(&mut process);
        let start = robonix_cli::launch::proc_start_time_ticks(process.id());
        let pid_file = dir.join("hook.pid");
        let child_file = dir.join("hook-child.pid");
        let record = ComponentRecord {
            name: "test-provider".into(),
            pid: process.id(),
            pgid: process.id(),
            package_dir: Some(dir.display().to_string()),
            stop: Some(format!(
                "echo $$ > {}; setsid sleep 60 & echo $! > {}; wait",
                pid_file.display(),
                child_file.display()
            )),
            ..Default::default()
        };
        let state = state_path(&dir);
        write_state(
            &state,
            &BootState {
                manifest_path: dir.display().to_string(),
                boot_pid: std::process::id(),
                boot_start_time_ticks: start,
                boot_id: id.clone(),
                started_at_ms: 0,
                atlas_endpoint: "127.0.0.1:1".into(),
                components: vec![record.clone()],
            },
        )
        .unwrap();
        let complete = teardown(&state, None, &[record], Some(&id)).await;
        let _ = process.kill();
        process.wait().unwrap();
        let hook_pid: u32 = std::fs::read_to_string(pid_file)
            .unwrap()
            .trim()
            .parse()
            .unwrap();
        let hook_live = std::fs::read_to_string(format!("/proc/{hook_pid}/stat"))
            .ok()
            .is_some_and(|stat| {
                !stat
                    .rsplit_once(')')
                    .unwrap()
                    .1
                    .trim_start()
                    .starts_with('Z')
            });
        assert!(complete);
        assert!(!hook_live, "stop hook survived cleanup");
        let child_pid: u32 = std::fs::read_to_string(child_file)
            .unwrap()
            .trim()
            .parse()
            .unwrap();
        let child_live = std::fs::read_to_string(format!("/proc/{child_pid}/stat"))
            .ok()
            .is_some_and(|stat| {
                !stat
                    .rsplit_once(')')
                    .unwrap()
                    .1
                    .trim_start()
                    .starts_with('Z')
            });
        assert!(!child_live, "detached stop-hook child survived cleanup");
        std::fs::remove_dir_all(dir).unwrap();
    }
}
