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
//      `<pkg>/rbnx-build/codegen/proto_gen/`.
//
// Codegen owns only `rbnx-build/codegen/` and `rbnx-build/proto-staging/`.
// In particular, it must never create, overwrite, or remove the colcon
// workspace under `rbnx-build/ws/`; `rbnx start` injects generated Python
// paths directly and sources a real colcon overlay when one exists.
//
// TODO: union package-local capabilities (`<pkg>/capabilities/`) and
// package-local IDL (`<pkg>/interfaces/lib/`) into a staging dir before
// running codegen — currently those are a copy-pasted snippet in a few
// build.sh scripts.

use anyhow::{Context, Result};
use colored::*;
use robonix_cli::{Config, SourcePathKey};
use robonix_scribe::debug;
use sha2::{Digest, Sha256};
use std::path::{Path, PathBuf};
use std::process::{Command, Output};
use std::time::{SystemTime, UNIX_EPOCH};

pub(crate) fn run_cmd(label: &str, cmd: &mut Command) -> Result<()> {
    debug!("[codegen] {}: {:?}", label, cmd);
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
    python: Option<PathBuf>,
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
        clean_codegen_outputs([&proto_gen, &mcp_types, &ros2_idl, &proto_staging])?;
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
    let python_selection = select_codegen_python(python)?;
    let python = &python_selection.path;
    let python_info = probe_python_grpc_tools(python)?;
    println!(
        "{} Python: {} ({})",
        "[codegen]".bold(),
        python_info.executable,
        python_info.version
    );

    println!(
        "{} grpc_tools.protoc → {}",
        "[codegen]".bold(),
        proto_gen.display()
    );
    let proto_gen_pending = temporary_sibling(&proto_gen, "pending");
    if proto_gen_pending.exists() {
        std::fs::remove_dir_all(&proto_gen_pending)?;
    }
    std::fs::create_dir_all(&proto_gen_pending)?;
    let mut proto_inputs = collect_proto_inputs(&runtime_proto, "runtime")?;
    proto_inputs.extend(collect_proto_inputs(&proto_staging, "staged")?);
    proto_inputs.sort_by(|a, b| a.logical_path.cmp(&b.logical_path));

    let input_fingerprint = fingerprint_proto_inputs(&proto_inputs)?;
    let mut protoc = Command::new(python);
    protoc
        .args(["-m", "grpc_tools.protoc", "-I"])
        .arg(&runtime_proto)
        .arg("-I")
        .arg(&proto_staging)
        .arg(format!("--python_out={}", proto_gen_pending.display()))
        .arg(format!("--grpc_python_out={}", proto_gen_pending.display()));
    for input in &proto_inputs {
        protoc.arg(&input.physical_path);
    }
    let status = protoc
        .status()
        .with_context(|| format!("failed to spawn {} -m grpc_tools.protoc", python.display()))?;
    if !status.success() {
        std::fs::remove_dir_all(&proto_gen_pending).ok();
        anyhow::bail!(
            "{} -m grpc_tools.protoc failed with {status}. \
             Re-run with -v / RUST_LOG=debug to see protoc output.",
            python.display()
        );
    }
    if let Err(error) = validate_generated_imports(python, &proto_gen_pending) {
        std::fs::remove_dir_all(&proto_gen_pending).ok();
        return Err(error);
    }
    write_toolchain_metadata(
        &proto_gen_pending.join("codegen-toolchain.json"),
        &python_info,
        &python_selection.source,
        direct_codegen.as_deref(),
        &rust_root,
        &input_fingerprint,
    )?;
    publish_directory(&proto_gen_pending, &proto_gen)?;

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

    println!(
        "{} done — {}{}",
        "[codegen]".green().bold(),
        if mcp {
            "proto+mcp+stubs"
        } else {
            "proto+stubs"
        },
        if mcp_types.exists() {
            format!(" in {}", out_root.display())
        } else {
            String::new()
        }
    );
    Ok(())
}

fn clean_codegen_outputs<'a>(paths: impl IntoIterator<Item = &'a PathBuf>) -> Result<()> {
    for path in paths {
        if path.exists() {
            if path.is_dir() {
                std::fs::remove_dir_all(path)
            } else {
                std::fs::remove_file(path)
            }
            .with_context(|| format!("remove codegen output {}", path.display()))?;
        }
    }
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
#[derive(Debug)]
struct PythonInfo {
    executable: String,
    version: String,
    grpcio_tools: String,
    protobuf: String,
    grpcio: String,
    protoc_version: String,
}

#[derive(Debug, Clone)]
struct ProtoInput {
    logical_path: String,
    physical_path: PathBuf,
}

#[derive(Debug, PartialEq, Eq)]
struct PythonSelection {
    path: PathBuf,
    source: String,
}

fn resolve_python_selection(
    cli: Option<PathBuf>,
    environment: Option<std::ffi::OsString>,
) -> PythonSelection {
    if let Some(path) = cli {
        return PythonSelection {
            path,
            source: "cli".to_string(),
        };
    }
    if let Some(path) = environment
        && !path.is_empty()
    {
        return PythonSelection {
            path: PathBuf::from(path),
            source: "environment".to_string(),
        };
    }
    PythonSelection {
        path: PathBuf::from("python3"),
        source: "default".to_string(),
    }
}

fn select_codegen_python(cli: Option<PathBuf>) -> Result<PythonSelection> {
    let selection = resolve_python_selection(cli, std::env::var_os("RBNX_CODEGEN_PYTHON"));
    let candidate = &selection.path;
    let output = Command::new(candidate)
        .arg("--version")
        .output()
        .with_context(|| {
            format!(
                "cannot execute codegen Python selected by {}: {}",
                selection.source,
                candidate.display()
            )
        })?;
    if !output.status.success() {
        anyhow::bail!(
            "codegen Python selected by {} failed `--version`: {} ({})",
            selection.source,
            candidate.display(),
            output.status
        );
    }
    Ok(selection)
}

fn probe_python_grpc_tools(python: &Path) -> Result<PythonInfo> {
    const PROBE: &str = r#"
import importlib.metadata as m, json, sys
import grpc_tools.protoc
def v(name):
    try: return m.version(name)
    except m.PackageNotFoundError: return "unknown"
print(json.dumps({"executable": sys.executable, "version": sys.version.split()[0],
                  "grpcio_tools": v("grpcio-tools"), "protobuf": v("protobuf"),
                  "grpcio": v("grpcio")}, sort_keys=True))
"#;
    let mod_probe = Command::new(python)
        .args(["-c", PROBE])
        .output()
        .with_context(|| format!("failed to spawn {} for grpc_tools probe", python.display()))?;
    if !mod_probe.status.success() {
        anyhow::bail!(
            "Python module 'grpc_tools' not importable from {}.\n\
             `rbnx codegen` needs grpcio-tools to emit Python `_pb2.py` + `_pb2_grpc.py`.\n\
             Install into the python3 above:\n\
             \n    {} -m pip install grpcio-tools\n\
             Probe stderr: {}",
            python.display(),
            python.display(),
            String::from_utf8_lossy(&mod_probe.stderr).trim()
        );
    }
    let value: serde_json::Value = serde_json::from_slice(&mod_probe.stdout)
        .with_context(|| "codegen Python returned invalid toolchain probe metadata")?;
    let get = |key: &str| {
        value[key]
            .as_str()
            .map(str::to_owned)
            .with_context(|| format!("codegen Python probe omitted `{key}`"))
    };
    let protoc_version = Command::new(python)
        .args(["-m", "grpc_tools.protoc", "--version"])
        .output()
        .with_context(|| {
            format!(
                "failed to query grpc_tools.protoc version via {}",
                python.display()
            )
        })?;
    if !protoc_version.status.success() {
        anyhow::bail!(
            "failed to query bundled grpc_tools.protoc version via {}: {}",
            python.display(),
            String::from_utf8_lossy(&protoc_version.stderr).trim()
        );
    }
    Ok(PythonInfo {
        executable: get("executable")?,
        version: get("version")?,
        grpcio_tools: get("grpcio_tools")?,
        protobuf: get("protobuf")?,
        grpcio: get("grpcio")?,
        protoc_version: String::from_utf8_lossy(&protoc_version.stdout)
            .trim()
            .to_owned(),
    })
}

fn collect_proto_inputs(root: &Path, namespace: &str) -> Result<Vec<ProtoInput>> {
    let mut inputs = Vec::new();
    for entry in std::fs::read_dir(root)
        .with_context(|| format!("read proto input directory {}", root.display()))?
    {
        let path = entry?.path();
        if path.extension().and_then(|s| s.to_str()) != Some("proto") {
            continue;
        }
        let relative = path
            .strip_prefix(root)
            .with_context(|| format!("derive logical proto path for {}", path.display()))?;
        inputs.push(ProtoInput {
            logical_path: format!(
                "{namespace}/{}",
                relative.to_string_lossy().replace('\\', "/")
            ),
            physical_path: path,
        });
    }
    Ok(inputs)
}

fn fingerprint_proto_inputs(proto_inputs: &[ProtoInput]) -> Result<String> {
    let mut sorted = proto_inputs.to_vec();
    sorted.sort_by(|a, b| a.logical_path.cmp(&b.logical_path));
    let mut digest = Sha256::new();
    for input in sorted {
        let contents = std::fs::read(&input.physical_path).with_context(|| {
            format!(
                "read proto input for fingerprint: {}",
                input.physical_path.display()
            )
        })?;
        digest.update((input.logical_path.len() as u64).to_be_bytes());
        digest.update(input.logical_path.as_bytes());
        digest.update((contents.len() as u64).to_be_bytes());
        digest.update(contents);
    }
    Ok(format!("{:x}", digest.finalize()))
}

fn temporary_sibling(path: &Path, label: &str) -> PathBuf {
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    path.with_file_name(format!(
        ".{}.{}-{}-{nonce}",
        path.file_name().unwrap_or_default().to_string_lossy(),
        label,
        std::process::id()
    ))
}

fn validate_generated_imports(python: &Path, generated: &Path) -> Result<()> {
    const VALIDATE: &str = r#"
import importlib, pathlib, sys, traceback
root = pathlib.Path(sys.argv[1]).resolve()
sys.path.insert(0, str(root))
files = sorted(set(root.rglob("*_pb2.py")) | set(root.rglob("*_pb2_grpc.py")))
if not files:
    raise SystemExit("no generated *_pb2.py or *_pb2_grpc.py files found")
failed = []
for path in files:
    name = ".".join(path.relative_to(root).with_suffix("").parts)
    try:
        importlib.import_module(name)
    except Exception:
        failed.append((str(path.relative_to(root)), traceback.format_exc()))
if failed:
    for path, error in failed:
        print("IMPORT FAILED: " + path, file=sys.stderr)
        print(error, file=sys.stderr)
    raise SystemExit(f"{len(failed)} of {len(files)} generated modules failed import validation")
print(f"validated {len(files)} generated Python modules")
"#;
    let output = Command::new(python)
        .arg("-c")
        .arg(VALIDATE)
        .arg(generated)
        .output()
        .with_context(|| {
            format!(
                "failed to run generated-stub validation with {}",
                python.display()
            )
        })?;
    ensure_success("generated Python import validation", output)
}

fn ensure_success(label: &str, output: Output) -> Result<()> {
    if output.status.success() {
        println!(
            "{} {}",
            "[codegen]".bold(),
            String::from_utf8_lossy(&output.stdout).trim()
        );
        return Ok(());
    }
    anyhow::bail!(
        "{label} failed with {}:\n{}",
        output.status,
        String::from_utf8_lossy(&output.stderr).trim()
    )
}

fn publish_directory(pending: &Path, destination: &Path) -> Result<()> {
    let backup = temporary_sibling(destination, "previous");
    if destination.exists() {
        std::fs::rename(destination, &backup).with_context(|| {
            format!(
                "preserve previous generated stubs {}",
                destination.display()
            )
        })?;
    }
    if let Err(error) = std::fs::rename(pending, destination) {
        if backup.exists() {
            std::fs::rename(&backup, destination).ok();
        }
        return Err(error).with_context(|| {
            format!(
                "publish generated stubs atomically to {}",
                destination.display()
            )
        });
    }
    if backup.exists() {
        std::fs::remove_dir_all(backup)?;
    }
    Ok(())
}

fn write_toolchain_metadata(
    path: &Path,
    python: &PythonInfo,
    selection_source: &str,
    codegen_bin: Option<&Path>,
    rust_root: &Path,
    input_fingerprint: &str,
) -> Result<()> {
    let generator_version = codegen_bin.and_then(|bin| {
        Command::new(bin)
            .arg("--version")
            .output()
            .ok()
            .filter(|output| output.status.success())
            .map(|output| String::from_utf8_lossy(&output.stdout).trim().to_owned())
            .filter(|version| !version.is_empty())
    });
    let metadata = serde_json::json!({
        "schema_version": 1,
        "inputs": {
            "algorithm": "sha256",
            "proto_paths_and_contents": input_fingerprint
        },
        "generator": {
            "kind": if codegen_bin.is_some() { "binary" } else { "cargo-run" },
            "path": codegen_bin.map(|p| p.display().to_string()),
            "version": generator_version,
            "rust_root": rust_root.display().to_string()
        },
        "python": {
            "executable": python.executable,
            "selection_source": selection_source,
            "version": python.version,
            "grpcio_tools": python.grpcio_tools,
            "bundled_protoc": python.protoc_version,
            "protobuf": python.protobuf,
            "grpcio": python.grpcio
        }
    });
    std::fs::write(path, serde_json::to_vec_pretty(&metadata)?)
        .with_context(|| format!("write codegen metadata {}", path.display()))
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn temp_root(label: &str) -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        std::env::temp_dir().join(format!(
            "rbnx-codegen-{label}-{}-{nonce}",
            std::process::id()
        ))
    }

    #[test]
    fn clean_preserves_existing_colcon_workspace() {
        let root = temp_root("preserve-colcon");
        let codegen = root.join("rbnx-build/codegen");
        let staging = root.join("rbnx-build/proto-staging");
        let setup = root.join("rbnx-build/ws/install/setup.bash");
        let original = b"# generated by colcon\nCOLCON_CURRENT_PREFIX=/real/overlay\n";

        fs::create_dir_all(&codegen).unwrap();
        fs::create_dir_all(&staging).unwrap();
        fs::create_dir_all(setup.parent().unwrap()).unwrap();
        fs::write(codegen.join("generated.py"), "generated").unwrap();
        fs::write(staging.join("contracts.proto"), "staged").unwrap();
        fs::write(&setup, original).unwrap();

        clean_codegen_outputs([&codegen, &staging]).unwrap();

        assert!(!codegen.exists());
        assert!(!staging.exists());
        assert_eq!(fs::read(&setup).unwrap(), original);
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn publish_directory_replaces_previous_tree() {
        let root = temp_root("atomic-publish");
        let destination = root.join("proto_gen");
        let pending = root.join(".proto_gen.pending");
        fs::create_dir_all(&destination).unwrap();
        fs::create_dir_all(&pending).unwrap();
        fs::write(destination.join("old_pb2.py"), "old").unwrap();
        fs::write(pending.join("new_pb2.py"), "new").unwrap();

        publish_directory(&pending, &destination).unwrap();

        assert!(!pending.exists());
        assert!(!destination.join("old_pb2.py").exists());
        assert_eq!(
            fs::read_to_string(destination.join("new_pb2.py")).unwrap(),
            "new"
        );
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn validates_every_generated_python_module() {
        let root = temp_root("validate-imports");
        fs::create_dir_all(&root).unwrap();
        fs::write(root.join("hello_pb2.py"), "VALUE = 1\n").unwrap();
        fs::write(
            root.join("hello_pb2_grpc.py"),
            "import hello_pb2\nVALUE = hello_pb2.VALUE\n",
        )
        .unwrap();

        validate_generated_imports(Path::new("python3"), &root).unwrap();

        fs::write(root.join("broken_pb2.py"), "raise RuntimeError('broken')\n").unwrap();
        let error = validate_generated_imports(Path::new("python3"), &root).unwrap_err();
        assert!(error.to_string().contains("broken_pb2.py"));
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn python_selection_precedence_is_cli_then_environment_then_default() {
        let env = Some(std::ffi::OsString::from("/env/python"));
        assert_eq!(
            resolve_python_selection(Some(PathBuf::from("/cli/python")), env.clone()),
            PythonSelection {
                path: PathBuf::from("/cli/python"),
                source: "cli".to_string(),
            }
        );
        assert_eq!(
            resolve_python_selection(None, env),
            PythonSelection {
                path: PathBuf::from("/env/python"),
                source: "environment".to_string(),
            }
        );
        assert_eq!(
            resolve_python_selection(None, None),
            PythonSelection {
                path: PathBuf::from("python3"),
                source: "default".to_string(),
            }
        );
    }

    #[test]
    fn proto_fingerprint_is_order_independent_and_content_sensitive() {
        let root = temp_root("fingerprint");
        fs::create_dir_all(&root).unwrap();
        let first = root.join("a.proto");
        let second = root.join("b.proto");
        fs::write(&first, "syntax = \"proto3\";\n").unwrap();
        fs::write(&second, "message B {}\n").unwrap();

        let first_input = ProtoInput {
            logical_path: "runtime/a.proto".to_string(),
            physical_path: first.clone(),
        };
        let second_input = ProtoInput {
            logical_path: "staged/b.proto".to_string(),
            physical_path: second.clone(),
        };
        let forward =
            fingerprint_proto_inputs(&[first_input.clone(), second_input.clone()]).unwrap();
        let reverse =
            fingerprint_proto_inputs(&[second_input.clone(), first_input.clone()]).unwrap();
        assert_eq!(forward, reverse);
        assert_eq!(forward.len(), 64);

        fs::write(&second, "message B { string value = 1; }\n").unwrap();
        let changed = fingerprint_proto_inputs(&[first_input, second_input]).unwrap();
        assert_ne!(forward, changed);
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn proto_fingerprint_does_not_depend_on_clone_root() {
        let left = temp_root("fingerprint-left");
        let right = temp_root("fingerprint-right");
        fs::create_dir_all(&left).unwrap();
        fs::create_dir_all(&right).unwrap();
        let left_path = left.join("same.proto");
        let right_path = right.join("same.proto");
        fs::write(&left_path, "message Same {}\n").unwrap();
        fs::write(&right_path, "message Same {}\n").unwrap();

        let left_hash = fingerprint_proto_inputs(&[ProtoInput {
            logical_path: "runtime/same.proto".to_string(),
            physical_path: left_path,
        }])
        .unwrap();
        let right_hash = fingerprint_proto_inputs(&[ProtoInput {
            logical_path: "runtime/same.proto".to_string(),
            physical_path: right_path,
        }])
        .unwrap();

        assert_eq!(left_hash, right_hash);
        fs::remove_dir_all(left).unwrap();
        fs::remove_dir_all(right).unwrap();
    }

    #[test]
    fn metadata_records_selection_protoc_and_fingerprint() {
        let root = temp_root("metadata");
        fs::create_dir_all(&root).unwrap();
        let path = root.join("codegen-toolchain.json");
        let python = PythonInfo {
            executable: "/venv/bin/python".to_string(),
            version: "3.12.1".to_string(),
            grpcio_tools: "1.70.0".to_string(),
            protobuf: "5.29.0".to_string(),
            grpcio: "1.70.0".to_string(),
            protoc_version: "libprotoc 29.3".to_string(),
        };

        write_toolchain_metadata(
            &path,
            &python,
            "environment",
            None,
            &root,
            "0123456789abcdef",
        )
        .unwrap();

        let value: serde_json::Value = serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
        assert_eq!(value["python"]["selection_source"], "environment");
        assert_eq!(value["python"]["bundled_protoc"], "libprotoc 29.3");
        assert_eq!(
            value["inputs"]["proto_paths_and_contents"],
            "0123456789abcdef"
        );
        fs::remove_dir_all(root).unwrap();
    }
}
