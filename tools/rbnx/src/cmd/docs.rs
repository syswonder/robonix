// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx docs` — regenerate the mdBook contract + ROS IDL reference
// (`docs/src/reference/{contracts,idl}.md`) from `capabilities/`.
//
// This is a developer-side step run inside the robonix source tree: it
// reads the live `capabilities/` (contracts + lib IDL) through the same
// loader codegen uses, and writes plain-markdown pages into the docs
// (robonix-book) submodule. Those pages are committed there, so the
// mdBook / GitHub Pages build compiles them with NO robonix environment —
// no rbnx, no Rust, no `capabilities/`. Re-run after changing a contract
// or IDL so the browsable reference stays in sync.

use anyhow::{Context, Result};
use colored::*;
use robonix_cli::{Config, SourcePathKey};
use std::path::{Path, PathBuf};
use std::process::Command;

use super::codegen::{build_codegen_cmd, locate_codegen_bin, run_cmd};

pub async fn execute(config: Config, out_dir: Option<PathBuf>) -> Result<()> {
    let root = config.resolve_source_path(SourcePathKey::Root)?;
    let rust_root = config.resolve_source_path(SourcePathKey::RustRoot)?;
    let capabilities_dir = config.resolve_source_path(SourcePathKey::Capabilities)?;
    let interfaces_lib = config.resolve_source_path(SourcePathKey::InterfacesLib)?;

    // Default into the docs submodule's reference dir. The generated files
    // are committed there; mdBook / Pages need only the markdown.
    let out = match out_dir {
        Some(d) if d.is_absolute() => d,
        Some(d) => root.join(d),
        None => root.join("docs").join("src").join("reference"),
    };
    std::fs::create_dir_all(&out)
        .with_context(|| format!("create reference dir {}", out.display()))?;

    let stamp = version_stamp(&root);

    let direct = locate_codegen_bin(&rust_root);
    let cargo = if direct.is_none() {
        Some(std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string()))
    } else {
        None
    };

    println!(
        "{} robonix-codegen --lang docs → {}",
        "[docs]".bold(),
        out.display()
    );
    println!("{} version stamp: {}", "[docs]".bold(), stamp);
    let mut cmd = build_codegen_cmd(direct.as_ref(), cargo.as_deref(), &rust_root);
    cmd.args(["--lang", "docs", "-I"])
        .arg(&interfaces_lib)
        .arg("--contracts")
        .arg(&capabilities_dir)
        .arg("-o")
        .arg(&out)
        .arg("--doc-stamp")
        .arg(&stamp);
    run_cmd("robonix-codegen docs", &mut cmd)?;

    println!(
        "{} wrote contracts.md + idl.md. Commit them into the docs repo — \
         the mdBook / Pages build needs no robonix environment.",
        "[docs]".green().bold()
    );
    Ok(())
}

/// `v<rbnx-version> · robonix commit <short-sha>[-dirty] · <commit-date>`,
/// read from git in `root`. The commit is what the reference was generated
/// from — always stated explicitly. Degrades gracefully when git or repo
/// metadata is unavailable so the reference still carries *some* version.
fn version_stamp(root: &Path) -> String {
    let git = |args: &[&str]| -> Option<String> {
        let out = Command::new("git")
            .arg("-C")
            .arg(root)
            .args(args)
            .output()
            .ok()?;
        if !out.status.success() {
            return None;
        }
        let s = String::from_utf8(out.stdout).ok()?.trim().to_string();
        (!s.is_empty()).then_some(s)
    };
    let sha = git(&["rev-parse", "--short", "HEAD"]).unwrap_or_else(|| "unknown".to_string());
    let dirty = git(&["status", "--porcelain"]).is_some_and(|s| !s.is_empty());
    let date = git(&["log", "-1", "--format=%cd", "--date=short"]).unwrap_or_default();
    let ver = env!("CARGO_PKG_VERSION");
    let dirty_tag = if dirty { "-dirty" } else { "" };
    let date_part = if date.is_empty() {
        String::new()
    } else {
        format!(" · {date}")
    };
    format!("v{ver} · commit {sha}{dirty_tag}{date_part}")
}
