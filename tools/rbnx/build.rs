// SPDX-License-Identifier: MulanPSL-2.0
// rbnx-cli proto codegen — same pattern as robonix-pilot/build.rs.
// Generates Rust stubs for RobonixSystemPilot (chat command needs SubmitTask) and
// reads atlas types via robonix_atlas::pb. All artefacts in OUT_DIR.

use std::collections::BTreeSet;
use std::path::PathBuf;

use robonix_codegen::codegen::{contract_gen, msg_parser, proto_gen};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    // tools/rbnx → up 2 = repo root.
    let repo_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .ok_or("could not locate repo root from CARGO_MANIFEST_DIR")?
        .to_path_buf();

    let idl_root = repo_root.join("capabilities/lib");
    let contracts_root = repo_root.join("capabilities");
    let proto_out = PathBuf::from(std::env::var("OUT_DIR")?);

    clean_generated_proto_dir(&proto_out)?;

    println!("cargo:rerun-if-changed={}", idl_root.display());
    println!("cargo:rerun-if-changed={}", contracts_root.display());
    println!("cargo:rerun-if-changed=build.rs");

    let mut resolver = msg_parser::MsgResolver::new(std::slice::from_ref(&idl_root))?;
    let mut idl_skips = 0usize;
    resolver.resolve_all_in_index(false, &mut idl_skips)?;
    resolver.resolve_all_srv(false, &mut idl_skips)?;

    let contract_srvs: BTreeSet<(String, String)> =
        contract_gen::collect_referenced_srvs(&contracts_root)?;
    proto_gen::generate(&resolver, &proto_out, Some(&contract_srvs), false)?;
    contract_gen::generate(
        &mut resolver,
        std::slice::from_ref(&contracts_root),
        &proto_out,
        false,
    )?;

    // Prefer a system protoc when present (needed on architectures like
    // LoongArch where protoc-bin-vendored does not ship a binary).
    let protoc = std::env::var_os("PROTOC")
        .map(PathBuf::from)
        .map(Ok)
        .unwrap_or_else(protoc_bin_vendored::protoc_bin_path)?;
    // SAFETY: build.rs is single-threaded.
    unsafe {
        std::env::set_var("PROTOC", protoc);
    }

    let proto_files: Vec<PathBuf> = std::fs::read_dir(&proto_out)?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().map(|x| x == "proto").unwrap_or(false))
        .collect();

    tonic_prost_build::configure()
        .build_server(false) // CLI is client-only
        .build_client(true)
        .compile_protos(&proto_files, std::slice::from_ref(&proto_out))?;

    emit_build_metadata(&repo_root)
}

/// Record build facts as compile-time env vars for `rbnx --version` and the
/// `rbnx boot` banner. vergen writes `VERGEN_*` (git, build date, rustc,
/// target) and falls back to a placeholder when a fact is unavailable, so a
/// build from a tarball without `.git` still succeeds.
fn emit_build_metadata(repo_root: &std::path::Path) -> Result<(), Box<dyn std::error::Error>> {
    use std::process::Command;
    use vergen_gitcl::{BuildBuilder, CargoBuilder, Emitter, GitclBuilder, RustcBuilder};

    Emitter::default()
        .add_instructions(&BuildBuilder::default().build_date(true).build()?)?
        .add_instructions(&CargoBuilder::default().target_triple(true).build()?)?
        .add_instructions(&RustcBuilder::default().semver(true).build()?)?
        .add_instructions(
            &GitclBuilder::default()
                .sha(false)
                .describe(true, true, None)
                .commit_date(true)
                .dirty(false)
                .build()?,
        )?
        .emit()?;

    // Build host: <user>@<hostname>. Both come from build env; clean
    // fallbacks for sandboxed builders that scrub them.
    let user = std::env::var("USER")
        .or_else(|_| std::env::var("USERNAME"))
        .unwrap_or_else(|_| "unknown".to_string());
    let host = Command::new("hostname")
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "unknown".to_string());
    let builder = format!("{user}@{host}");

    // Build time in the build host's LOCAL timezone (e.g.
    // 2026-06-29T09:09:22+0800), matching the usual kernel/dev banner — a bare
    // UTC `Z` is confusing on a machine running local time. SOURCE_DATE_EPOCH
    // (Reproducible Builds) still selects the instant when set.
    let build_time = local_build_time();

    println!("cargo:rustc-env=ROBONIX_BUILDER={builder}");
    println!("cargo:rustc-env=ROBONIX_BUILD_TIME={build_time}");

    // vergen watches HEAD and the loose branch ref; logs/HEAD also changes
    // when the branch ref is packed and has no loose file.
    let logs_head = repo_root.join(".git").join("logs").join("HEAD");
    if logs_head.exists() {
        println!("cargo:rerun-if-changed={}", logs_head.display());
    }
    println!("cargo:rerun-if-env-changed=SOURCE_DATE_EPOCH");
    Ok(())
}

/// Build timestamp in the host's local timezone, e.g.
/// `2026-06-29T09:09:22+0800`. Shells out to `date` so we get the host TZ
/// without a tz crate in the build-deps (build.rs already shells to `hostname`
/// / `rustc`). SOURCE_DATE_EPOCH selects the instant for reproducible builds;
/// otherwise "now". Falls back to UTC via `format_unix_utc` if `date` is
/// unavailable or non-GNU (e.g. the `-d@` form is GNU-specific).
fn local_build_time() -> String {
    let epoch = std::env::var("SOURCE_DATE_EPOCH")
        .ok()
        .and_then(|s| s.trim().parse::<i64>().ok());
    let mut cmd = std::process::Command::new("date");
    if let Some(secs) = epoch {
        cmd.arg(format!("-d@{secs}"));
    }
    cmd.arg("+%Y-%m-%dT%H:%M:%S%z");
    if let Some(out) = cmd.output().ok().filter(|o| o.status.success())
        && let Ok(s) = String::from_utf8(out.stdout)
    {
        let t = s.trim().to_string();
        if !t.is_empty() {
            return t;
        }
    }
    let secs = epoch.unwrap_or_else(|| {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs() as i64)
            .unwrap_or(0)
    });
    format_unix_utc(secs)
}

/// Format a unix epoch (seconds) as `YYYY-MM-DDTHH:MM:SSZ`. Lifted out
/// of build.rs into a free fn so we can keep it tiny — no chrono in the
/// build-script dep tree (chrono lives in the runtime deps already, but
/// pulling it into build deps adds 30s+ of cold compile per crate).
fn format_unix_utc(secs: i64) -> String {
    // Days from Unix epoch to civil date — Howard Hinnant's "chrono"
    // algorithm, public domain, ~10 lines. Handles negative epochs and
    // year boundaries correctly without a calendar table.
    let s = secs.rem_euclid(86_400);
    let days = secs.div_euclid(86_400);
    let (h, rem) = (s / 3600, s % 3600);
    let (m, sec) = (rem / 60, rem % 60);

    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m_civ = if mp < 10 { mp + 3 } else { mp - 9 };
    let y_civ = y + i64::from(m_civ <= 2);

    format!("{y_civ:04}-{m_civ:02}-{d:02}T{h:02}:{m:02}:{sec:02}Z")
}

/// Removes stale generated proto/Rust files from OUT_DIR before regenerating.
///
/// Side effect: deletes only files with `.proto` or `.rs` extensions in the
/// current crate build output directory. This keeps renamed IDL packages from
/// being compiled alongside their previous generated names during incremental
/// builds and rust-analyzer checks.
fn clean_generated_proto_dir(
    proto_out: &std::path::Path,
) -> Result<(), Box<dyn std::error::Error>> {
    if !proto_out.exists() {
        return Ok(());
    }
    for entry in std::fs::read_dir(proto_out)? {
        let path = entry?.path();
        let Some(ext) = path.extension().and_then(|x| x.to_str()) else {
            continue;
        };
        if matches!(ext, "proto" | "rs") {
            std::fs::remove_file(path)?;
        }
    }
    Ok(())
}
