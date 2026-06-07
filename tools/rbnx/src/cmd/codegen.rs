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

pub(crate) fn run_cmd(label: &str, cmd: &mut Command) -> Result<()> {
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
    if !abs.join("package_manifest.yaml").exists()
        && !abs.join("robonix_manifest.yaml").exists()
        && !abs.join("rbnx_manifest.yaml").exists()
    {
        eprintln!(
            "{}: {} has no package_manifest.yaml (continuing anyway)",
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
    ros2: bool,
    clean: bool,
    out_dir: Option<PathBuf>,
) -> Result<()> {
    let pkg_root = match package {
        Some(p) => resolve_pkg_root(&p)?,
        None => super::run_package::find_package_from_cwd()?,
    };
    let rust_root = config.resolve_source_path(SourcePathKey::RustRoot)?;
    // <root>/capabilities — contract TOMLs (top level) + IDL under lib/.
    let capabilities_dir = config.resolve_source_path(SourcePathKey::Capabilities)?;
    // <root>/capabilities/lib — single canonical IDL search root.
    // msg/srv references in contract TOMLs (e.g. `demo/srv/Hello`)
    // resolve as `<root>/capabilities/lib/demo/srv/Hello.srv`.
    let interfaces_lib = config.resolve_source_path(SourcePathKey::InterfacesLib)?;
    let runtime_proto = config.resolve_source_path(SourcePathKey::RuntimeProto)?;
    // Per-package overlay: `<pkg>/capabilities/` mirrors the global
    // layout. When present we add it both as an IDL include
    // (`<pkg>/capabilities/lib/`) and as a contracts root, so packages
    // can ship their own msg/srv/contracts that merge with the global
    // set (symmetric with how atlas's contract registry merges roots).
    let pkg_caps = pkg_root.join("capabilities");
    let pkg_caps_lib: Option<PathBuf> = {
        let p = pkg_caps.join("lib");
        p.is_dir().then_some(p)
    };
    // <pkg>/capabilities/ also gets passed to --contracts so per-package
    // contracts merge with the global tree (atlas does the same merge
    // at the registry level — this keeps codegen consistent).
    let pkg_caps_root: Option<PathBuf> = pkg_caps.is_dir().then_some(pkg_caps);

    // Codegen output convention: every package gets
    // `<pkg>/rbnx-build/codegen/{proto_gen, robonix_mcp_types}`. Both robonix_api
    // and rbnx-cli rely on this exact layout, so packages don't need to plumb
    // paths anywhere — `rbnx codegen -p $PKG` is the whole story. The
    // `--out-dir` flag stays as an escape hatch for unusual layouts but
    // defaults to the convention.
    let rbnx_build = pkg_root.join("rbnx-build");
    let out_root = match out_dir {
        Some(d) if d.is_absolute() => d,
        Some(d) => pkg_root.join(d),
        None => rbnx_build.join("codegen"),
    };
    let proto_gen = out_root.join("proto_gen");
    let mcp_types = out_root.join("robonix_mcp_types");
    // ROS 2 canonical message overlay (a colcon workspace of source). Only
    // generated with --ros2; consumers colcon-build it and source
    // install/setup.bash so their rclpy types are Robonix's.
    let ros2_idl = out_root.join("ros2_idl");
    // Per-invocation staging for the system-wide .proto files. No commits;
    // grpc_tools.protoc reads from here in step 3.
    let proto_staging = rbnx_build.join("proto-staging");

    if clean {
        // Include ros2_idl explicitly: when --out-dir points outside
        // rbnx-build/, removing rbnx_build alone leaves the overlay behind.
        for p in [&proto_gen, &mcp_types, &ros2_idl, &rbnx_build] {
            if p.exists() {
                std::fs::remove_dir_all(p).ok();
            }
        }
    }
    std::fs::create_dir_all(&proto_staging)?;

    // Prefer the installed `robonix-codegen` binary on PATH (or in the
    // workspace target dir) — `cargo run -p robonix-codegen` rebuilds /
    // re-resolves the workspace on every invocation, which adds 100-300 ms
    // even when nothing changed and floods the boot log with `Compiling…`
    // lines that don't belong in a per-package codegen step. Falls back
    // to `cargo run` only if neither binary is available, so a fresh
    // checkout that hasn't done `cargo install` keeps working.
    let direct_codegen = locate_codegen_bin(&rust_root);
    let cargo_bin = if direct_codegen.is_none() {
        if Path::new("/usr/bin/cargo").exists() {
            Some("/usr/bin/cargo".to_string())
        } else {
            Some(std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string()))
        }
    } else {
        None
    };

    println!("{} package: {}", "[codegen]".bold(), pkg_root.display());
    println!(
        "{} robonix source: {}",
        "[codegen]".bold(),
        rust_root.display()
    );

    // 1. Stage system .proto into rbnx-build/proto-staging/.
    //    Includes: global <root>/capabilities/lib + (if present) per-package
    //    <pkg>/capabilities/lib. --contracts: global capabilities tree
    //    (per-package contracts merging through codegen is a follow-up).
    println!("{} robonix-codegen --lang proto ...", "[codegen]".bold());
    let mut proto_cmd =
        build_codegen_cmd(direct_codegen.as_ref(), cargo_bin.as_deref(), &rust_root);
    proto_cmd
        .args(["--lang", "proto", "-I"])
        .arg(&interfaces_lib);
    if let Some(p) = pkg_caps_lib.as_ref() {
        proto_cmd.arg("-I").arg(p);
    }
    proto_cmd.arg("--contracts").arg(&capabilities_dir);
    if let Some(p) = pkg_caps_root.as_ref() {
        proto_cmd.arg("--contracts").arg(p);
    }
    proto_cmd.arg("-o").arg(&proto_staging);
    run_cmd("robonix-codegen proto", &mut proto_cmd)?;

    // 2. Optional: MCP dataclasses.
    if mcp {
        println!("{} robonix-codegen --lang mcp ...", "[codegen]".bold());
        std::fs::create_dir_all(&mcp_types).ok();
        let mut mcp_cmd =
            build_codegen_cmd(direct_codegen.as_ref(), cargo_bin.as_deref(), &rust_root);
        mcp_cmd.args(["--lang", "mcp", "-I"]).arg(&interfaces_lib);
        // Include per-package <pkg>/capabilities/lib too — same merge
        // semantics as the proto step, so per-pkg srv files (e.g.
        // explore_rbnx's Explore.srv) emit their MCP Request/Response
        // dataclasses alongside the global ones.
        if let Some(p) = pkg_caps_lib.as_ref() {
            mcp_cmd.arg("-I").arg(p);
        }
        mcp_cmd.arg("-o").arg(&mcp_types);
        run_cmd("robonix-codegen mcp", &mut mcp_cmd)?;
    }

    // 3. Package-local Python stubs via grpc_tools.protoc.
    //
    // We shell out to the active python3's `grpcio-tools` package because
    // generating Python `_pb2.py` + `_pb2_grpc.py` from `.proto` is what
    // the standard protoc-with-grpc-python-plugin combo is designed to
    // do, and there's no maintained Rust crate that emits Python stubs.
    // Probe for both python3 and the module up front — the historical
    // silent-ignore on failure left packages with "0 generated Servicers"
    // at runtime and a debug session per missing dep.
    probe_python_grpc_tools()?;

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
    let status = protoc
        .status()
        .with_context(|| "failed to spawn python3 -m grpc_tools.protoc")?;
    if !status.success() {
        anyhow::bail!(
            "python3 -m grpc_tools.protoc failed with {status}. \
             Re-run with -v / RUST_LOG=debug to see protoc output."
        );
    }

    // 3b. Optional: ROS 2 canonical message overlay (source). Emitted next
    //     to proto_gen / robonix_mcp_types so it follows the same rbnx-build
    //     convention. It still needs `colcon build` in a ROS 2 environment;
    //     the package's build.sh does that (e.g. docker exec into the
    //     container) and start.sh sources <ros2_idl>/install/setup.bash.
    if ros2 {
        println!("{} robonix-codegen --lang ros2 ...", "[codegen]".bold());
        let mut ros2_cmd =
            build_codegen_cmd(direct_codegen.as_ref(), cargo_bin.as_deref(), &rust_root);
        ros2_cmd.args(["--lang", "ros2", "-I"]).arg(&interfaces_lib);
        if let Some(p) = pkg_caps_lib.as_ref() {
            ros2_cmd.arg("-I").arg(p);
        }
        ros2_cmd.arg("-o").arg(&ros2_idl);
        run_cmd("robonix-codegen ros2", &mut ros2_cmd)?;
    }

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

/// Locate a runnable `robonix-codegen` binary without going through cargo.
/// Search order:
///   1. `$ROBONIX_CODEGEN_BIN` (override for unusual layouts)
///   2. `$CARGO_HOME/bin/robonix-codegen` / `~/.cargo/bin/robonix-codegen`
///      — what `cargo install --path crates/robonix-codegen` puts there
///   3. `<rust_root>/target/release/robonix-codegen`
///   4. `<rust_root>/target/debug/robonix-codegen`
///   5. anything matching `robonix-codegen` on `$PATH`
///
/// Returns `None` only when none of those exist; callers fall back to
/// `cargo run -p robonix-codegen` which keeps a fresh-checkout workflow
/// alive even before the user has installed any binaries.
pub(crate) fn locate_codegen_bin(rust_root: &Path) -> Option<PathBuf> {
    if let Ok(s) = std::env::var("ROBONIX_CODEGEN_BIN")
        && !s.is_empty()
    {
        let p = PathBuf::from(s);
        if p.is_file() {
            return Some(p);
        }
    }
    let cargo_home = std::env::var("CARGO_HOME")
        .map(PathBuf::from)
        .ok()
        .or_else(|| dirs::home_dir().map(|h| h.join(".cargo")));
    if let Some(home) = cargo_home {
        let p = home.join("bin").join("robonix-codegen");
        if p.is_file() {
            return Some(p);
        }
    }
    for profile in ["release", "debug"] {
        let p = rust_root
            .join("target")
            .join(profile)
            .join("robonix-codegen");
        if p.is_file() {
            return Some(p);
        }
    }
    if let Ok(path_env) = std::env::var("PATH") {
        for dir in std::env::split_paths(&path_env) {
            let p = dir.join("robonix-codegen");
            if p.is_file() {
                return Some(p);
            }
        }
    }
    None
}

/// Probe the active python3 for `grpc_tools.protoc`. Bails with a single,
/// copy-pasteable install instruction if either is missing — this is the
/// only Python dep `rbnx codegen` reaches for, so making it explicit up
/// front is the entire UX cost of not vendoring protoc + grpc_python_plugin.
fn probe_python_grpc_tools() -> Result<()> {
    let py = Command::new("python3").arg("--version").output();
    if py.is_err() || !py.as_ref().unwrap().status.success() {
        anyhow::bail!(
            "python3 not found on PATH. `rbnx codegen` shells out to \
             `python3 -m grpc_tools.protoc` to emit Python gRPC stubs. \
             Install python3 (>=3.10) and re-run."
        );
    }
    let mod_probe = Command::new("python3")
        .args(["-c", "import grpc_tools.protoc"])
        .output()
        .with_context(|| "failed to spawn python3 for grpc_tools probe")?;
    if !mod_probe.status.success() {
        let py_path = Command::new("python3")
            .args(["-c", "import sys; print(sys.executable)"])
            .output()
            .ok()
            .and_then(|o| String::from_utf8(o.stdout).ok())
            .map(|s| s.trim().to_string())
            .unwrap_or_else(|| "python3".to_string());
        anyhow::bail!(
            "Python module 'grpc_tools' not importable from {py_path}.\n\
             `rbnx codegen` needs grpcio-tools to emit Python `_pb2.py` + `_pb2_grpc.py`.\n\
             Install into the python3 above:\n\
             \n    python3 -m pip install --user grpcio-tools\n"
        );
    }
    Ok(())
}

/// Build a fresh `Command` invoking `robonix-codegen` either directly
/// (preferred — picks up `$ROBONIX_CODEGEN_BIN` / installed bin / target/
/// in that order) or via `cargo run -p robonix-codegen` as a last
/// resort. Caller appends the actual `--lang … -I … -o …` args.
pub(crate) fn build_codegen_cmd(
    direct: Option<&PathBuf>,
    cargo: Option<&str>,
    rust_root: &Path,
) -> Command {
    if let Some(bin) = direct {
        Command::new(bin)
    } else {
        let cargo = cargo.expect("either direct codegen bin or cargo bin must be set");
        let mut cmd = Command::new(cargo);
        cmd.args(["run", "-p", "robonix-codegen", "--manifest-path"])
            .arg(rust_root.join("Cargo.toml"))
            .arg("--");
        cmd
    }
}
