// SPDX-License-Identifier: MulanPSL-2.0
// Generate the browsable contract + ROS IDL reference for the mdBook
// (`docs/src/reference/{contracts,idl}.md`). Reuses the same contract
// loader as proto generation (`contract_gen::load_contract_summaries`),
// so the reference can't drift from what codegen / atlas actually parse.
//
// Two pages, cross-linked:
//   contracts.md — every contract; its payload cell links into idl.md.
//   idl.md       — every .msg/.srv under the lib root(s), raw, anchored.

use anyhow::{Context, Result};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use super::contract_gen::load_contract_summaries;

/// One ROS IDL file discovered under a lib root.
struct IdlFile {
    /// lib-relative path with extension, e.g. `chassis/srv/ExecuteMoveCommand.srv`.
    rel: String,
    /// ROS package (the dir above `msg/`/`srv/`), e.g. `chassis`, `sensor_msgs`.
    pkg: String,
    /// Type name without extension, e.g. `ExecuteMoveCommand`.
    name: String,
    /// "msg" | "srv".
    kind: &'static str,
    /// Raw file content.
    body: String,
}

/// Emit `contracts.md` + `idl.md` into `out`. `stamp` is the version line
/// (e.g. `v0.1 @ abc1234 (2026-06-06)`) written verbatim into each header.
pub fn generate(
    contracts_dirs: &[PathBuf],
    lib_roots: &[PathBuf],
    out: &Path,
    stamp: Option<&str>,
    verbose: bool,
) -> Result<()> {
    let contracts = load_contract_summaries(contracts_dirs)?;
    let idl = collect_idl(lib_roots)?;
    let idl_rels: BTreeSet<&str> = idl.iter().map(|f| f.rel.as_str()).collect();
    let banner = stamp.unwrap_or("(version unknown)");

    let contracts_md = render_contracts(&contracts, contracts_dirs, &idl_rels, banner);
    let idl_md = render_idl(&idl, banner);

    fs::create_dir_all(out)?;
    let cpath = out.join("contracts.md");
    let ipath = out.join("idl.md");
    fs::write(&cpath, contracts_md).with_context(|| format!("write {}", cpath.display()))?;
    fs::write(&ipath, idl_md).with_context(|| format!("write {}", ipath.display()))?;

    eprintln!(
        "[robonix-codegen] docs: {} contracts, {} IDL files -> {}",
        contracts.len(),
        idl.len(),
        out.display()
    );
    let _ = verbose;
    Ok(())
}

/// Stable in-page anchor for a lib-relative IDL path. Both pages compute it
/// from the same string (contract `idl` field == idl file `rel`), so links
/// always resolve. `chassis/srv/ExecuteMoveCommand.srv` → `chassis-srv-executemovecommand-srv`.
fn idl_anchor(rel: &str) -> String {
    rel.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() {
                c.to_ascii_lowercase()
            } else {
                '-'
            }
        })
        .collect()
}

/// First namespace segment after `robonix/` — primitive / service / system / skill.
fn category_of(id: &str) -> &str {
    id.split('/').nth(1).unwrap_or("other")
}

fn render_contracts(
    contracts: &[super::contract_gen::ContractSummary],
    contracts_dirs: &[PathBuf],
    idl_rels: &BTreeSet<&str>,
    banner: &str,
) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "# 契约参考（自动生成）");
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "> 由 robonix {banner} 自动生成，请勿手改。重新生成：`rbnx docs`。"
    );
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "本页罗列 `capabilities/` 下的所有标准契约（共 {} 条）。",
        contracts.len()
    );
    let _ = writeln!(
        out,
        "载荷列链到对应的 [ROS IDL](idl.md)。概念与字段含义见 [接口目录](../interface-catalog/index.md)。"
    );
    let _ = writeln!(out);
    let _ = writeln!(out, "[toc]");

    // Stable category order; anything unexpected lands in "other".
    for cat in ["primitive", "service", "system", "skill", "other"] {
        let rows: Vec<&super::contract_gen::ContractSummary> = contracts
            .iter()
            .filter(|c| {
                let cc = category_of(&c.id);
                cc == cat
                    || (cat == "other"
                        && !matches!(cc, "primitive" | "service" | "system" | "skill"))
            })
            .collect();
        if rows.is_empty() {
            continue;
        }
        let _ = writeln!(out);
        let _ = writeln!(out, "## {cat}");
        let _ = writeln!(out);
        let _ = writeln!(out, "| 契约 ID | kind | mode | 载荷（IDL） | 契约 TOML |");
        let _ = writeln!(out, "|---|---|---|---|---|");
        for c in rows {
            let payload = if idl_rels.contains(c.idl.as_str()) {
                format!("[`{}`](idl.md#{})", c.idl, idl_anchor(&c.idl))
            } else {
                format!("`{}`", c.idl)
            };
            let toml_rel = rel_under(&c.toml_path, contracts_dirs);
            let _ = writeln!(
                out,
                "| `{}` | {} | `{}` | {} | `{}` |",
                c.id, c.kind, c.mode, payload, toml_rel
            );
        }
    }
    out
}

fn render_idl(idl: &[IdlFile], banner: &str) -> String {
    // Group by ROS package, then by file name (both sorted).
    let mut by_pkg: BTreeMap<&str, Vec<&IdlFile>> = BTreeMap::new();
    for f in idl {
        by_pkg.entry(f.pkg.as_str()).or_default().push(f);
    }
    for v in by_pkg.values_mut() {
        v.sort_by(|a, b| a.name.cmp(&b.name));
    }

    let mut out = String::new();
    let _ = writeln!(out, "# ROS IDL 参考（自动生成）");
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "> 由 robonix {banner} 自动生成，请勿手改。重新生成：`rbnx docs`。"
    );
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "本页是 `capabilities/lib/` 下全部 ROS IDL（`.msg` / `.srv`）的原文，按 ROS 包分组（共 {} 个文件）。[契约参考](contracts.md) 的载荷列链到这里对应的锚点。",
        idl.len()
    );
    let _ = writeln!(out);
    let _ = writeln!(out, "[toc]");

    for (pkg, files) in &by_pkg {
        let _ = writeln!(out);
        let _ = writeln!(out, "## {pkg}");
        for f in files {
            let _ = writeln!(out);
            // Explicit HTML anchor so the contract page's computed link
            // resolves regardless of mdBook's heading-slug rules.
            let _ = writeln!(out, "<a id=\"{}\"></a>", idl_anchor(&f.rel));
            let _ = writeln!(out, "### {} `{}`", f.name, f.kind);
            let _ = writeln!(out);
            let _ = writeln!(out, "`{}`", f.rel);
            let _ = writeln!(out);
            let _ = writeln!(out, "```rosidl");
            let _ = write!(out, "{}", f.body);
            if !f.body.ends_with('\n') {
                let _ = writeln!(out);
            }
            let _ = writeln!(out, "```");
        }
    }
    out
}

/// Path relative to whichever `dirs` prefix it lives under, `/`-normalised.
fn rel_under(p: &Path, dirs: &[PathBuf]) -> String {
    for d in dirs {
        if let Ok(r) = p.strip_prefix(d) {
            return r.to_string_lossy().replace('\\', "/");
        }
    }
    p.to_string_lossy().replace('\\', "/")
}

/// ROS package for a lib-relative path: the segment just above `msg/`/`srv/`,
/// else the first segment. `common_interfaces/sensor_msgs/msg/Image.msg`
/// → `sensor_msgs`; `chassis/srv/X.srv` → `chassis`.
fn ros_pkg_of(rel: &str) -> String {
    let parts: Vec<&str> = rel.split('/').collect();
    for (i, seg) in parts.iter().enumerate() {
        if (*seg == "msg" || *seg == "srv") && i > 0 {
            return parts[i - 1].to_string();
        }
    }
    parts.first().map(|s| s.to_string()).unwrap_or_default()
}

fn collect_idl(lib_roots: &[PathBuf]) -> Result<Vec<IdlFile>> {
    let mut by_rel: BTreeMap<String, IdlFile> = BTreeMap::new();
    for root in lib_roots {
        let mut files = Vec::new();
        walk_idl(root, &mut files)?;
        for p in files {
            let kind: &'static str = match p.extension().and_then(|e| e.to_str()) {
                Some("srv") => "srv",
                Some("msg") => "msg",
                _ => continue,
            };
            let rel = p
                .strip_prefix(root)
                .unwrap_or(&p)
                .to_string_lossy()
                .replace('\\', "/");
            let name = p
                .file_stem()
                .map(|s| s.to_string_lossy().to_string())
                .unwrap_or_default();
            let pkg = ros_pkg_of(&rel);
            let body = fs::read_to_string(&p).with_context(|| format!("read {}", p.display()))?;
            by_rel.entry(rel.clone()).or_insert(IdlFile {
                rel,
                pkg,
                name,
                kind,
                body,
            });
        }
    }
    Ok(by_rel.into_values().collect())
}

fn walk_idl(dir: &Path, out: &mut Vec<PathBuf>) -> Result<()> {
    if !dir.is_dir() {
        return Ok(());
    }
    for entry in fs::read_dir(dir).with_context(|| format!("read_dir {}", dir.display()))? {
        let p = entry?.path();
        if p.is_dir() {
            walk_idl(&p, out)?;
        } else if matches!(
            p.extension().and_then(|e| e.to_str()),
            Some("msg") | Some("srv")
        ) {
            out.push(p);
        }
    }
    Ok(())
}
