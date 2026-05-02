// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx codegen -p <package>` — one-shot codegen wrapper that replaces
// the copy-pasted robonix-codegen + grpc_tools.protoc boilerplate in each
// package's build.sh.
//
// For a given package root:
//   1. Stage the system-wide .proto files (IDL + capabilities) into
//      `<pkg>/rbnx-build/proto-staging/`. This dir is package-local and
//      transient — no committed artefacts elsewhere in the tree.
//   2. If --mcp: regenerate `<pkg>/robonix_mcp_types/`
//      (robonix-codegen --lang mcp).
//   3. Run grpc_tools.protoc on the staged + runtime protos, emitting
//      `<pkg>/proto_gen/`.
//   4. Write `<pkg>/rbnx-build/ws/install/setup.bash` so `rbnx start`
//      injects the right PYTHONPATH.
//
// TODO: union package-local capabilities (`<pkg>/capabilities/`) and
// package-local IDL (`<pkg>/interfaces/lib/`) into a staging dir before
// running codegen — currently those are a copy-pasted snippet in a few
// build.sh scripts.

use anyhow::{Context, Result};
use colored::*;
use robonix_cli::{Config, SourcePathKey};
use std::path::{Path, PathBuf};
use std::process::Command;

fn run_cmd(label: &str, cmd: &mut Command) -> Result<()> {
    log::debug!("[codegen] {}: {:?}", label, cmd);
    let status = cmd
        .status()
        .with_context(|| format!("failed to execute `{label}`"))?;
    if !status.success() {
        anyhow::bail!("{label} failed with {status}");
    }
    Ok(())
}

fn resolve_pkg_root(package: &Path) -> Result<PathBuf> {
    let abs = if package.is_absolute() {
        package.to_path_buf()
    } else {
        let base = std::env::var("RBNX_INVOCATION_CWD")
            .map(PathBuf::from)
            .unwrap_or(std::env::current_dir()?);
        base.join(package)
    };
    let abs = abs
        .canonicalize()
        .with_context(|| format!("package path not found: {}", abs.display()))?;
    if !abs.join("robonix_manifest.yaml").exists() && !abs.join("rbnx_manifest.yaml").exists() {
        eprintln!(
            "{}: {} has no robonix_manifest.yaml (continuing anyway)",
            "warn".yellow().bold(),
            abs.display()
        );
    }
    Ok(abs)
}

pub async fn execute(
    config: Config,
    package: Option<PathBuf>,
    mcp: bool,
    clean: bool,
    out_dir: Option<PathBuf>,
) -> Result<()> {
    let pkg_root = match package {
        Some(p) => resolve_pkg_root(&p)?,
        None => super::run_package::find_package_from_cwd()?,
    };
    let rust_root = config.resolve_source_path(SourcePathKey::RustRoot)?;
    let interfaces_lib = config.resolve_source_path(SourcePathKey::InterfacesLib)?;
    let capabilities_dir = config.resolve_source_path(SourcePathKey::Capabilities)?;
    let runtime_proto = config.resolve_source_path(SourcePathKey::RuntimeProto)?;

    // Where to place proto_gen/ and robonix_mcp_types/. Defaults to package root;
    // override for packages that want them inside a sub-dir (e.g. tiago_bridge/).
    let out_root = match out_dir {
        Some(d) if d.is_absolute() => d,
        Some(d) => pkg_root.join(d),
        None => pkg_root.clone(),
    };
    let proto_gen = out_root.join("proto_gen");
    let mcp_types = out_root.join("robonix_mcp_types");
    let rbnx_build = pkg_root.join("rbnx-build");
    // Per-invocation staging for the system-wide .proto files. No commits;
    // grpc_tools.protoc reads from here in step 3.
    let proto_staging = rbnx_build.join("proto-staging");

    if clean {
        for p in [&proto_gen, &mcp_types, &rbnx_build] {
            if p.exists() {
                std::fs::remove_dir_all(p).ok();
            }
        }
    }
    std::fs::create_dir_all(&proto_staging)?;

    let cargo_bin = if Path::new("/usr/bin/cargo").exists() {
        "/usr/bin/cargo".to_string()
    } else {
        std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string())
    };

    println!("{} package: {}", "[codegen]".bold(), pkg_root.display());
    println!(
        "{} robonix source: {}",
        "[codegen]".bold(),
        rust_root.display()
    );

    // 1. Stage system .proto into rbnx-build/proto-staging/.
    println!("{} robonix-codegen --lang proto ...", "[codegen]".bold());
    // TODO: support <pkg>/capabilities and <pkg>/interfaces/lib union.
    run_cmd(
        "robonix-codegen proto",
        Command::new(&cargo_bin)
            .args(["run", "-p", "robonix-codegen", "--manifest-path"])
            .arg(rust_root.join("Cargo.toml"))
            .args(["--", "--lang", "proto", "-I"])
            .arg(&interfaces_lib)
            .arg("--contracts")
            .arg(&capabilities_dir)
            .arg("-o")
            .arg(&proto_staging),
    )?;

    // 2. Optional: MCP dataclasses.
    if mcp {
        println!("{} robonix-codegen --lang mcp ...", "[codegen]".bold());
        std::fs::create_dir_all(&mcp_types).ok();
        run_cmd(
            "robonix-codegen mcp",
            Command::new(&cargo_bin)
                .args(["run", "-p", "robonix-codegen", "--manifest-path"])
                .arg(rust_root.join("Cargo.toml"))
                .args(["--", "--lang", "mcp", "-I"])
                .arg(&interfaces_lib)
                .arg("-o")
                .arg(&mcp_types),
        )?;
    }

    // 3. Package-local Python stubs via grpc_tools.protoc.
    println!(
        "{} grpc_tools.protoc → {}",
        "[codegen]".bold(),
        proto_gen.display()
    );
    std::fs::create_dir_all(&proto_gen)?;
    let proto_files: Vec<PathBuf> = std::fs::read_dir(&runtime_proto)?
        .chain(std::fs::read_dir(&proto_staging)?)
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("proto"))
        .collect();

    let mut protoc = Command::new("python3");
    protoc
        .args(["-m", "grpc_tools.protoc", "-I"])
        .arg(&runtime_proto)
        .arg("-I")
        .arg(&proto_staging)
        .arg(format!("--python_out={}", proto_gen.display()))
        .arg(format!("--grpc_python_out={}", proto_gen.display()));
    for f in &proto_files {
        protoc.arg(f);
    }
    // Treat as best-effort — some transitive imports may fail, that's OK.
    let _ = protoc.status();

    // 4. Write PYTHONPATH setup stub so `rbnx start` sees all the right paths.
    let ws_install = rbnx_build.join("ws").join("install");
    std::fs::create_dir_all(&ws_install)?;
    let mut py_parts: Vec<String> = vec![
        pkg_root.display().to_string(),
        proto_gen.display().to_string(),
    ];
    if mcp {
        py_parts.push(mcp_types.display().to_string());
    }
    let joined = py_parts.join(":");
    let setup_bash = ws_install.join("setup.bash");
    std::fs::write(
        &setup_bash,
        format!("#!/usr/bin/env bash\nexport PYTHONPATH=\"{joined}:${{PYTHONPATH:-}}\"\n"),
    )?;

    println!(
        "{} done — {}, {} setup.bash",
        "[codegen]".green().bold(),
        if mcp {
            "proto+mcp+stubs"
        } else {
            "proto+stubs"
        },
        if mcp_types.exists() {
            "mcp_types, "
        } else {
            ""
        }
    );
    Ok(())
}
