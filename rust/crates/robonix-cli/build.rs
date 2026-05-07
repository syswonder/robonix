// SPDX-License-Identifier: MulanPSL-2.0
// rbnx-cli proto codegen — same pattern as robonix-pilot/build.rs.
// Generates Rust stubs for SystemPilot (chat command needs SubmitTask) and
// reads atlas types via robonix_atlas::pb. All artefacts in OUT_DIR.

use std::collections::BTreeSet;
use std::path::PathBuf;

use robonix_codegen::codegen::{contract_gen, msg_parser, proto_gen};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_rust = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .ok_or("could not locate workspace rust dir from CARGO_MANIFEST_DIR")?
        .to_path_buf();
    let repo_root = workspace_rust
        .parent()
        .ok_or("could not locate repo root from CARGO_MANIFEST_DIR")?
        .to_path_buf();

    let idl_root = workspace_rust.join("crates/robonix-interfaces/lib");
    let contracts_root = repo_root.join("capabilities");
    let proto_out = PathBuf::from(std::env::var("OUT_DIR")?);

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

    let protoc = protoc_bin_vendored::protoc_bin_path()?;
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

    emit_build_metadata(&repo_root);
    Ok(())
}

/// Inject Linux-kernel-style banner facts as compile-time env vars so
/// `rbnx boot` can print "robonix v0.1.0 (beb843f) built ... on host by
/// user with rustc 1.95.0 (target=x86_64-…)". Each fact has a graceful
/// fallback (e.g. "unknown") so a `cargo install` from a tarball without
/// a `.git` dir still builds.
fn emit_build_metadata(repo_root: &std::path::Path) {
    use std::process::Command;

    // git short SHA — if a `.git` exists at repo_root, hash it; otherwise
    // honour an externally-set ROBONIX_GIT_SHA (so CI / package builds
    // can inject it without needing the git dir).
    let git_sha = std::env::var("ROBONIX_GIT_SHA").ok().or_else(|| {
        let dot_git = repo_root.join(".git");
        if !dot_git.exists() {
            return None;
        }
        Command::new("git")
            .args([
                "-C",
                repo_root.to_str().unwrap_or(""),
                "rev-parse",
                "--short=7",
                "HEAD",
            ])
            .output()
            .ok()
            .filter(|o| o.status.success())
            .and_then(|o| String::from_utf8(o.stdout).ok())
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
    });
    let git_sha = git_sha.unwrap_or_else(|| "unknown".to_string());

    // git dirty flag — append "+" to the SHA when there are uncommitted
    // changes, mirroring `git describe --dirty` and the kernel's "+"
    // suffix on "make rpm-pkg" of a dirty tree.
    let dirty = Command::new("git")
        .args([
            "-C",
            repo_root.to_str().unwrap_or(""),
            "status",
            "--porcelain",
        ])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| !o.stdout.is_empty())
        .unwrap_or(false);
    let git_sha = if dirty && git_sha != "unknown" {
        format!("{git_sha}+")
    } else {
        git_sha
    };

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

    // Build time — UTC ISO-8601, second precision. SOURCE_DATE_EPOCH (the
    // Reproducible Builds standard) overrides the wallclock for repro
    // builds.
    let build_time = if let Ok(s) = std::env::var("SOURCE_DATE_EPOCH")
        && let Ok(secs) = s.parse::<i64>()
    {
        format_unix_utc(secs)
    } else {
        let secs = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs() as i64)
            .unwrap_or(0);
        format_unix_utc(secs)
    };

    // Compiler version — `rustc -V` (e.g. "rustc 1.95.0 (abc123 2026-04-15)").
    let rustc = std::env::var("RUSTC").unwrap_or_else(|_| "rustc".to_string());
    let rustc_ver = Command::new(&rustc)
        .arg("-V")
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_string())
        .unwrap_or_else(|| "rustc unknown".to_string());

    // Compile target triple is exposed by cargo as $TARGET in build.rs.
    let target = std::env::var("TARGET").unwrap_or_else(|_| "unknown".to_string());

    println!("cargo:rustc-env=ROBONIX_GIT_SHA={git_sha}");
    println!("cargo:rustc-env=ROBONIX_BUILDER={builder}");
    println!("cargo:rustc-env=ROBONIX_BUILD_TIME={build_time}");
    println!("cargo:rustc-env=ROBONIX_RUSTC={rustc_ver}");
    println!("cargo:rustc-env=ROBONIX_TARGET={target}");

    // Rerun if any of these inputs change. The repo's HEAD ref is the
    // most useful signal — covers commits and branch switches.
    let head = repo_root.join(".git").join("HEAD");
    if head.exists() {
        println!("cargo:rerun-if-changed={}", head.display());
    }
    println!("cargo:rerun-if-env-changed=ROBONIX_GIT_SHA");
    println!("cargo:rerun-if-env-changed=SOURCE_DATE_EPOCH");
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
