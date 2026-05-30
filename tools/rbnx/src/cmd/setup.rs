// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx setup` — register the current robonix source tree so out-of-tree packages
// (their build.sh, codegen calls) can resolve paths via `rbnx path <key>`.

use anyhow::{Context, Result};
use colored::*;
use robonix_cli::Config;
use std::path::{Path, PathBuf};

/// Markers that identify a directory as a valid robonix repo root.
/// The Cargo workspace lives at the repo root; capabilities/ hosts the
/// contract TOMLs plus the IDL tree under capabilities/lib (used to be a
/// symlink into rust/crates/robonix-interfaces/lib; now hosted directly).
const ROBONIX_ROOT_MARKERS: &[&str] = &[
    "Cargo.toml",
    "capabilities",
    "capabilities/lib",
];

fn looks_like_robonix_root(dir: &Path) -> bool {
    ROBONIX_ROOT_MARKERS.iter().all(|m| dir.join(m).exists())
}

/// Find a robonix root by walking up from `start`.
fn find_root_upwards(start: &Path) -> Option<PathBuf> {
    let mut cur = start;
    loop {
        if looks_like_robonix_root(cur) {
            return Some(cur.to_path_buf());
        }
        match cur.parent() {
            Some(p) => cur = p,
            None => return None,
        }
    }
}

pub async fn execute(_config: Config, path: Option<PathBuf>) -> Result<()> {
    // Determine candidate: explicit arg, $RBNX_INVOCATION_CWD, then process cwd.
    let candidate = if let Some(p) = path {
        if p.is_absolute() {
            p
        } else {
            std::env::current_dir()?.join(p)
        }
    } else {
        let invocation_cwd = std::env::var("RBNX_INVOCATION_CWD").ok();
        invocation_cwd
            .map(PathBuf::from)
            .unwrap_or(std::env::current_dir()?)
    };

    let candidate = candidate
        .canonicalize()
        .with_context(|| format!("Failed to canonicalize {}", candidate.display()))?;

    let root = if looks_like_robonix_root(&candidate) {
        candidate.clone()
    } else if let Some(r) = find_root_upwards(&candidate) {
        r
    } else {
        eprintln!(
            "{}: {} does not look like a robonix source tree.",
            "error".red().bold(),
            candidate.display()
        );
        eprintln!(
            "Expected markers (at least one level up): {}",
            ROBONIX_ROOT_MARKERS.join(", ")
        );
        eprintln!("Run `rbnx setup` from inside a cloned robonix repo.");
        std::process::exit(1);
    };

    let mut config = Config::load()?;
    let previous = config.robonix_source_path.clone();
    config.robonix_source_path = Some(root.clone());
    config.save()?;

    println!("{} robonix source path registered:", "✓".green().bold());
    println!("  {}", root.display().to_string().bold().cyan());
    if let Some(prev) = previous.filter(|p| p != &root) {
        println!("  (previously: {})", prev.display().to_string().dimmed());
    }
    println!(
        "\nOther packages can now resolve paths via `{}`.",
        "rbnx path <key>".bold()
    );
    println!("Valid keys: root, rust, capabilities, interfaces-lib, runtime-proto, robonix-api");
    Ok(())
}
