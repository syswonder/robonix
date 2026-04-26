// SPDX-License-Identifier: MulanPSL-2.0
// Run package commands: build, start (start blocks until process exits).
//
// Dev-packaging contract: one package has ONE top-level `start` shell body
// (not a list of nodes). `rbnx start` just executes that body at the
// package root — the body itself is responsible for spawning processes
// and registering capabilities with atlas. No node-id flag.

use super::build;
use anyhow::{Context, Result};
use robonix_cli::Config;
use robonix_cli::manifest;
use robonix_cli::output;
use robonix_cli::process::ProcessManager;
use std::path::{Path, PathBuf};

/// Directory against which relative `-p` is resolved: **the pwd of the command invocation**.
/// When `cargo run` runs from `robonix/rust`, the process cwd is not the user's shell cwd — wrappers
/// should `export RBNX_INVOCATION_CWD="$(pwd)"` before `cd`+`cargo run`. If unset, `std::env::current_dir()` is used.
pub(crate) const RBNX_INVOCATION_CWD: &str = "RBNX_INVOCATION_CWD";

/// POSIX-shell single-quoted escape, used when we synthesise `export FOO=...`
/// fragments to inject into a package's `start` body.
fn shell_escape(value: &str) -> String {
    format!("'{}'", value.replace('\'', "'\"'\"'"))
}

fn path_base_for_dash_p() -> Result<PathBuf> {
    if let Ok(s) = std::env::var(RBNX_INVOCATION_CWD) {
        Ok(PathBuf::from(s))
    } else {
        std::env::current_dir().context("Failed to get current directory")
    }
}

/// Resolve `-p` to a filesystem path before `canonicalize`: relative paths and `.` use
/// [`path_base_for_dash_p`] as the prefix (invocation pwd, or process cwd).
pub(crate) fn resolve_local_path_for_filesystem(p: &Path) -> Result<PathBuf> {
    if p.as_os_str() == "." || p.as_os_str() == "./" {
        return path_base_for_dash_p();
    }
    if p.is_absolute() {
        return Ok(p.to_path_buf());
    }
    Ok(path_base_for_dash_p()?.join(p))
}

/// Walk up from the invocation cwd looking for a directory that contains
/// a `package_manifest.yaml`. Returns the first match.
pub(crate) fn find_package_from_cwd() -> Result<PathBuf> {
    let start = path_base_for_dash_p()?;
    let mut cur: Option<&Path> = Some(&start);
    while let Some(d) = cur {
        if d.join(manifest::MANIFEST_FILE).is_file() {
            return d
                .canonicalize()
                .with_context(|| format!("Failed to canonicalize: {}", d.display()));
        }
        cur = d.parent();
    }
    anyhow::bail!(
        "no {} found in {} or any parent; pass -p <path> or `cd` into a package directory",
        manifest::MANIFEST_FILE,
        start.display()
    )
}

/// Resolve package path from -p (local path) or -g (system-installed name).
/// When neither is given, walk up from cwd to find a package manifest.
fn resolve_package_path(
    config: &Config,
    path: Option<PathBuf>,
    global: Option<String>,
) -> Result<PathBuf> {
    if let Some(p) = path {
        let p = resolve_local_path_for_filesystem(&p)?;
        let canonical = p
            .canonicalize()
            .with_context(|| format!("Failed to canonicalize: {}", p.display()))?;
        if canonical.join(manifest::MANIFEST_FILE).exists() {
            return Ok(canonical);
        }
        anyhow::bail!(
            "Path {} does not contain {}",
            canonical.display(),
            manifest::MANIFEST_FILE
        );
    }

    if let Some(name) = global {
        let db = robonix_cli::PackageDatabase::load(&config.package_storage_path)?;
        if let Some(pkg) = db.get_package(&name) {
            return Ok(pkg.path.clone());
        }
        anyhow::bail!(
            "Package '{}' not found in system storage ({})",
            name,
            config.package_storage_path.display()
        );
    }

    find_package_from_cwd()
}

/// Resolve package path for `start`: same `-p` rules as `build`, then system-installed name fallback.
fn resolve_package_path_for_start(config: &Config, spec: &str) -> Result<PathBuf> {
    let path = resolve_local_path_for_filesystem(Path::new(spec))?;
    if path.join(manifest::MANIFEST_FILE).is_file() {
        return path
            .canonicalize()
            .with_context(|| format!("Failed to canonicalize: {}", path.display()));
    }

    let db = robonix_cli::PackageDatabase::load(&config.package_storage_path)?;
    if let Some(pkg) = db.get_package(spec) {
        return Ok(pkg.path.clone());
    }

    anyhow::bail!(
        "Package '{}' not found at {} (relative -p uses {} or process cwd). Try -g <installed name> or export {}=\"$(pwd)\" before cargo run.",
        spec,
        path.display(),
        RBNX_INVOCATION_CWD,
        RBNX_INVOCATION_CWD
    )
}

pub async fn execute_build(
    config: Config,
    path: Option<PathBuf>,
    global: Option<String>,
    clean: bool,
) -> Result<()> {
    let package_root = resolve_package_path(&config, path, global)?;
    build::execute_local(package_root, clean).await
}

pub async fn execute_start(
    config: &Config,
    spec: Option<&str>,
    registry_endpoint: Option<&str>,
) -> Result<()> {
    let package_root = match spec {
        Some(s) => resolve_package_path_for_start(config, s)?,
        None => find_package_from_cwd()?,
    };
    let detected = manifest::detect_and_load(&package_root)?;
    let manifest = &detected.manifest;
    manifest.validate_and_summarize()?;

    let endpoint = registry_endpoint
        .map(String::from)
        .unwrap_or_else(|| "127.0.0.1:50051".to_string());

    let run_root = package_root
        .parent()
        .context("Package root has no parent")?;
    let log_dir = run_root.join("rbnx-deploy").join("logs");
    let process_manager = ProcessManager::new(log_dir)?;

    output::action("Running", &manifest.package.name);
    output::sub_step(&format!("Atlas endpoint: {}", endpoint));
    if !manifest.capabilities.is_empty() {
        output::sub_step(&format!(
            "Capabilities: {}",
            manifest
                .capabilities
                .iter()
                .map(|c| c.name.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }

    let mut env = std::collections::HashMap::new();
    env.insert("ROBONIX_ATLAS".to_string(), endpoint.clone());

    if !manifest.build.trim().is_empty() && !build::build_stamp_path(&package_root).exists() {
        output::sub_step("No rbnx-build/.rbnx-built — running package build first");
        build::build_local_package(&package_root, false)?;
    }

    let exports = env
        .iter()
        .map(|(k, v)| format!("export {}={}", k, shell_escape(v)))
        .collect::<Vec<_>>()
        .join("; ");
    let start_body = manifest.start.trim();
    let setup_bash = package_root
        .join("rbnx-build")
        .join("ws")
        .join("install")
        .join("setup.bash");
    let setup_source = if setup_bash.exists() {
        format!("source {}", shell_escape(&setup_bash.display().to_string()))
    } else {
        String::new()
    };
    let prefix_parts: Vec<String> = [setup_source, exports]
        .into_iter()
        .filter(|s| !s.is_empty())
        .collect();
    let start_command = if prefix_parts.is_empty() {
        start_body.to_string()
    } else {
        format!("{}; {start_body}", prefix_parts.join("; "))
    };

    let result = process_manager
        .start_process(
            &manifest.package.name,
            &manifest.package.name,
            "package",
            &package_root,
            &start_command,
        )
        .await?;
    output::check(&format!(
        "{} exited (PID {})",
        manifest.package.name, result.pid
    ));

    output::success(&format!("Package {} finished", manifest.package.name));
    Ok(())
}
