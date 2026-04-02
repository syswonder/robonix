// SPDX-License-Identifier: MulanPSL-2.0
// Run package commands: build, start (start blocks until process exits)

use super::build;
use super::launch_helpers;
use anyhow::{Context, Result};
use robonix_cli::Config;
use robonix_cli::manifest;
use robonix_cli::output;
use robonix_cli::process::ProcessManager;
use std::path::PathBuf;

/// Resolve package path from -p (local path) or -g (system-installed name).
fn resolve_package_path(
    config: &Config,
    path: Option<PathBuf>,
    global: Option<String>,
) -> Result<PathBuf> {
    if let Some(p) = path {
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

    anyhow::bail!("Specify -p <path> for local package or -g <name> for system-installed package")
}

/// Resolve package path for start: accepts path or name (looks in examples, cwd, system storage).
fn resolve_package_path_for_start(config: &Config, spec: &str) -> Result<PathBuf> {
    let path = PathBuf::from(spec);
    if path.exists() {
        let canonical = path
            .canonicalize()
            .with_context(|| format!("Failed to canonicalize: {}", path.display()))?;
        if canonical.join(manifest::MANIFEST_FILE).exists() {
            return Ok(canonical);
        }
    }

    let cwd = std::env::current_dir().context("Failed to get current directory")?;
    let candidates = [
        cwd.join("examples").join("packages").join(spec),
        cwd.join("examples").join(spec),
        cwd.join(spec),
        cwd.join("rust")
            .join("examples")
            .join("packages")
            .join(spec),
        cwd.join("rust").join("examples").join(spec),
    ];

    for candidate in &candidates {
        if candidate.join(manifest::MANIFEST_FILE).exists() {
            return candidate
                .canonicalize()
                .with_context(|| format!("Failed to canonicalize: {}", candidate.display()));
        }
    }

    let db = robonix_cli::PackageDatabase::load(&config.package_storage_path)?;
    if let Some(pkg) = db.get_package(spec) {
        return Ok(pkg.path.clone());
    }

    anyhow::bail!(
        "Package '{}' not found. Tried: {:?} and system storage",
        spec,
        candidates
            .iter()
            .map(|p| p.display().to_string())
            .collect::<Vec<_>>()
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
    spec: &str,
    node_id: &str,
    registry_endpoint: Option<&str>,
) -> Result<()> {
    let package_root = resolve_package_path_for_start(config, spec)?;
    let detected = manifest::detect_and_load(&package_root)?;
    let manifest = &detected.manifest;

    let node = manifest
        .nodes
        .iter()
        .find(|n| n.id == node_id)
        .ok_or_else(|| {
            anyhow::anyhow!(
                "Node '{}' not found in manifest. Available: {}",
                node_id,
                manifest
                    .nodes
                    .iter()
                    .map(|n| n.id.as_str())
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        })?;

    let endpoint = registry_endpoint
        .map(String::from)
        .or_else(|| std::env::var("ROBONIX_META_GRPC_ENDPOINT").ok())
        .unwrap_or_else(|| "127.0.0.1:50051".to_string());

    // Scan skills/ directory and pre-register with robonix-atlas
    let skills = manifest::scan_skills(&package_root);
    if !skills.is_empty() {
        output::sub_step(&format!("Discovered {} skill(s):", skills.len()));
        for s in &skills {
            output::sub_step(&format!("  - {} : {}", s.name, s.description));
        }
        let grpc_url = if endpoint.contains("://") {
            endpoint.clone()
        } else {
            format!("http://{}", endpoint)
        };
        match robonix_sdk::RobonixClient::connect(&grpc_url).await {
            Ok(mut sdk) => {
                let skill_items: Vec<robonix_sdk::SkillInfoItem> = skills
                    .iter()
                    .map(|s| robonix_sdk::SkillInfoItem {
                        name: s.name.clone(),
                        description: s.description.clone(),
                        path: s.path.display().to_string(),
                        metadata_json: s.metadata_json.clone(),
                    })
                    .collect();
                match sdk
                    .register_node_with_skills(&node.id, "", "", "", skill_items, "", "")
                    .await
                {
                    Ok(_) => output::sub_step("Skills registered with robonix-atlas"),
                    Err(e) => {
                        output::sub_step(&format!("Warning: failed to register skills: {e:#}"))
                    }
                }
            }
            Err(e) => {
                output::sub_step(&format!(
                    "Warning: could not connect to server for skill registration: {e:#}"
                ));
            }
        }
    }

    let run_root = package_root
        .parent()
        .context("Package root has no parent")?;
    let log_dir = run_root.join("rbnx-deploy").join("logs");
    let process_manager = ProcessManager::new(log_dir)?;

    output::action(
        "Running",
        &format!("node {} ({})", node_id, manifest.package.name),
    );
    output::sub_step(&format!("Runtime endpoint: {}", endpoint));

    let mut env = std::collections::HashMap::new();
    env.insert("ROBONIX_META_GRPC_ENDPOINT".to_string(), endpoint.clone());
    env.insert("ROBONIX_ATLAS".to_string(), endpoint.clone());
    if let Some(profile) = manifest
        .launch_profiles
        .as_ref()
        .and_then(|p| p.get("default"))
    {
        if let Some(launch) = profile.nodes.get(&node.id) {
            for (k, v) in &launch.env {
                env.insert(k.clone(), v.clone());
            }
        }
    }

    if !build::build_stamp_path(&package_root).exists() {
        output::sub_step("No rbnx-build/.rbnx-built — running package build script first");
        build::build_local_package(&package_root, false)?;
    }

    let exports = env
        .iter()
        .map(|(k, v)| format!("export {}={}", k, launch_helpers::shell_escape(v)))
        .collect::<Vec<_>>()
        .join("; ");
    let start_body = node.start.trim();
    if start_body.is_empty() {
        anyhow::bail!(
            "Node '{}' has empty `start` in {}",
            node.id,
            manifest::MANIFEST_FILE
        );
    }
    let start_command = if exports.is_empty() {
        start_body.to_string()
    } else {
        format!("{exports}; {start_body}")
    };

    let std_name = format!("{}.{}", manifest.package.name, node.id);
    let result = process_manager
        .start_process(
            &manifest.package.name,
            &std_name,
            "node",
            &package_root,
            &start_command,
        )
        .await?;
    output::check(&format!("{} exited (PID {})", std_name, result.pid));

    output::success(&format!("Node {} finished", node_id));
    Ok(())
}
