#!/usr/bin/env python3
"""
Scan .msg / .srv / .action under repos/<repo>/<distro>/, align by package and interface name
across distros, compare field differences and emit Markdown + JSON reports.
"""

from __future__ import annotations

import argparse
import json
import re
from collections import defaultdict
from pathlib import Path
from typing import Any

# --- Simple IDL parsing ---

def _strip_comment(line: str) -> str:
    i = line.find("#")
    return (line[:i] if i >= 0 else line).strip()

def _is_default_literal(token: str) -> bool:
    """True if token looks like a default value (bool, number, or quoted string)."""
    if token in ("true", "false"):
        return True
    if not token:
        return False
    # Integer or float
    t = token.lstrip("-")
    if t.replace(".", "", 1).isdigit():
        return True
    # Quoted string
    if len(token) >= 2 and token[0] == '"' and token[-1] == '"':
        return True
    return False


def _parse_msg_like_block(lines: list[str]) -> list[tuple[str, str]]:
    """Parse a .msg block or one section of .srv (request/response) or .action. Returns [(type, name), ...]"""
    fields: list[tuple[str, str]] = []
    for raw in lines:
        line = _strip_comment(raw)
        if not line:
            continue
        # Constant: UPPER_CASE= value
        if "=" in line and not line.strip().startswith(("int", "float", "string", "bool", "byte", "char", "wstring")):
            const_match = re.match(r"^([A-Z][A-Z0-9_]*)\s*=\s*(.*)$", line)
            if const_match:
                name, _ = const_match.group(1), const_match.group(2)
                fields.append(("const", name))
                continue
        parts = line.split()
        if len(parts) >= 2:
            # Optional default: "type name default" e.g. "bool read_only false" -> type=bool, name=read_only
            if len(parts) >= 3 and _is_default_literal(parts[-1]):
                name = parts[-2]
                type_part = " ".join(parts[:-2])
            else:
                name = parts[-1]
                type_part = " ".join(parts[:-1])
            if name and not name.startswith("#"):
                fields.append((type_part.strip(), name))
    return fields


def parse_msg_file(path: Path) -> list[tuple[str, str]]:
    text = path.read_text(encoding="utf-8", errors="replace")
    return _parse_msg_like_block(text.splitlines())


def parse_srv_file(path: Path) -> dict[str, list[tuple[str, str]]]:
    text = path.read_text(encoding="utf-8", errors="replace")
    parts = [p.strip() for p in text.split("---")]
    request = _parse_msg_like_block(parts[0].splitlines()) if parts else []
    response = _parse_msg_like_block(parts[1].splitlines()) if len(parts) > 1 else []
    return {"request": request, "response": response}


def parse_action_file(path: Path) -> dict[str, list[tuple[str, str]]]:
    text = path.read_text(encoding="utf-8", errors="replace")
    parts = [p.strip() for p in text.split("---")]
    goal = _parse_msg_like_block(parts[0].splitlines()) if parts else []
    result = _parse_msg_like_block(parts[1].splitlines()) if len(parts) > 1 else []
    feedback = _parse_msg_like_block(parts[2].splitlines()) if len(parts) > 2 else []
    return {"goal": goal, "result": result, "feedback": feedback}


def collect_idl_in_tree(root: Path) -> dict[str, dict[str, list[tuple[str, str]]]]:
    """
    Walk */(msg|srv|action)/* under root and return:
    { "package_name/kind/interface_name": { "request"|"response"|"goal"|"result"|"feedback"|"": [ (type, name), ... ] } }
    .msg uses block key ""; .srv uses "request"/"response"; .action uses "goal"/"result"/"feedback".
    Top-level key is pkg/kind/name, e.g. std_msgs/msg/Header, std_srvs/srv/SetBool.
    """
    out: dict[str, dict[str, list[tuple[str, str]]]] = {}
    for msg_dir in root.rglob("msg"):
        if not msg_dir.is_dir():
            continue
        pkg = msg_dir.parent.name
        for f in msg_dir.glob("*.msg"):
            key = f"{pkg}/msg/{f.stem}"
            out[key] = {"": parse_msg_file(f)}
    for srv_dir in root.rglob("srv"):
        if not srv_dir.is_dir():
            continue
        pkg = srv_dir.parent.name
        for f in srv_dir.glob("*.srv"):
            key = f"{pkg}/srv/{f.stem}"
            out[key] = parse_srv_file(f)
    for action_dir in root.rglob("action"):
        if not action_dir.is_dir():
            continue
        pkg = action_dir.parent.name
        for f in action_dir.glob("*.action"):
            key = f"{pkg}/action/{f.stem}"
            out[key] = parse_action_file(f)
    return out


def fields_signature(blocks: dict[str, list[tuple[str, str]]]) -> str:
    """Quick signature to check if two interfaces are identical."""
    parts = []
    for block_name in sorted(blocks):
        parts.append(block_name + ":" + ";".join(f"{t} {n}" for t, n in blocks[block_name]))
    return "|".join(parts)


def diff_blocks(
    a: dict[str, list[tuple[str, str]]], b: dict[str, list[tuple[str, str]]]
) -> dict[str, Any]:
    """Compare two interfaces' blocks and return a change description."""
    all_blocks = sorted(set(a) | set(b))
    changes: list[dict[str, Any]] = []
    for block in all_blocks:
        fa = {f[1]: f[0] for f in a.get(block, [])}
        fb = {f[1]: f[0] for f in b.get(block, [])}
        only_a = set(fa) - set(fb)
        only_b = set(fb) - set(fa)
        type_changes = [n for n in set(fa) & set(fb) if fa[n] != fb[n]]
        if only_a or only_b or type_changes:
            changes.append({
                "block": block or "(message)",
                "added": [{"name": n, "type": fb[n]} for n in sorted(only_b)],
                "removed": [{"name": n, "type": fa[n]} for n in sorted(only_a)],
                "type_changed": [{"name": n, "from": fa[n], "to": fb[n]} for n in sorted(type_changes)],
            })
    return {"blocks_changed": changes, "identical": len(changes) == 0}


# --- Main flow: scan repos, align by distro, progressive comparison, one table ---

# Chronological distro order (release date): older → newer. Comparison is progressive (each vs next).
# See README for full distro table. Foxy was the LTS before Humble (most-used pre-Humble).
DISTRO_CHRONOLOGICAL_ORDER = [
    "foxy",     # June 2020, LTS to June 2023 (most-used before Humble)
    "humble",   # May 2022, LTS to May 2027
    "iron",     # May 2023, EOL Nov 2024
    "jazzy",    # May 2024, LTS to May 2029
    "kilted",   # May 2025, LTS to Dec 2026
    "rolling",  # development
]


def _block_field_count(blocks: dict[str, list[tuple[str, str]]]) -> int:
    return sum(len(fields) for fields in blocks.values())


def _describe_diff(delta: dict[str, Any]) -> list[str]:
    """Turn diff_blocks output into short bullet lines for one transition (with inline code)."""
    parts: list[str] = []
    for block in delta.get("blocks_changed", []):
        block_name = block.get("block") or "(message)"
        for x in block.get("added", []):
            parts.append(f"+ `{block_name}`: `{x['type']}` `{x['name']}`")
        for x in block.get("removed", []):
            parts.append(f"- `{block_name}`: `{x['type']}` `{x['name']}`")
        for x in block.get("type_changed", []):
            parts.append(f"~ `{x['name']}`: `{x['from']}` → `{x['to']}`")
    return parts


def run_analysis(repos_root: Path, distros: list[str] | None = None) -> tuple[dict, list[dict[str, Any]]]:
    """
    repos_root: path to repos/ (under it: repo_name/distro_name/...).
    Compares distros in chronological order (progressive: each vs next).
    Returns (summary, rows) where each row has: interface, cells {distro: cell_text}, changes (progressive).
    """
    if distros is None:
        distros = []
    # Collect all IDLs per (repo, distro)
    data: dict[str, dict[str, dict[str, dict[str, list[tuple[str, str]]]]]] = defaultdict(lambda: defaultdict(dict))
    for repo_dir in repos_root.iterdir():
        if not repo_dir.is_dir() or repo_dir.name.startswith("."):
            continue
        for distro_dir in repo_dir.iterdir():
            if not distro_dir.is_dir():
                continue
            distro = distro_dir.name
            if distros and distro not in distros:
                continue
            idl_map = collect_idl_in_tree(distro_dir)
            for key, blocks in idl_map.items():
                data[repo_dir.name][distro][key] = blocks

    all_keys = set()
    for repo, by_distro in data.items():
        for distro, by_key in by_distro.items():
            for key in by_key:
                all_keys.add((repo, key))

    all_distros = set()
    for repo in data:
        all_distros.update(data[repo].keys())
    # Order distros by release chronology; append any extra (e.g. from repos) not in the list
    ordered_distros: list[str] = [d for d in DISTRO_CHRONOLOGICAL_ORDER if d in all_distros]
    for d in sorted(all_distros):
        if d not in ordered_distros:
            ordered_distros.append(d)

    summary: dict[str, Any] = {
        "repos": list({r for r, _ in all_keys}),
        "distros": ordered_distros,
        "total_interfaces": len(all_keys),
        "per_repo": defaultdict(lambda: {"total": 0, "only_in_some": 0, "with_progressive_diff": 0}),
    }
    rows: list[dict[str, Any]] = []

    for repo, key in sorted(all_keys):
        summary["per_repo"][repo]["total"] += 1
        by_distro = {d: data[repo][d].get(key) for d in data[repo]}
        present = [d for d in by_distro if by_distro[d] is not None]
        # Only in some = not in every distro *that this repo has* (incomplete repos: count only their distros)
        only_in_some = len(present) < len(by_distro)
        if only_in_some:
            summary["per_repo"][repo]["only_in_some"] += 1

        cells: dict[str, str] = {}
        for d in ordered_distros:
            if d not in data[repo]:
                # Branch not fetched for this repo (e.g. control_msgs has no foxy/rolling)
                cells[d] = "∅"
            else:
                blocks = by_distro.get(d)
                if blocks is None:
                    # Branch fetched, but this definition doesn't exist in that distro
                    cells[d] = "—"
                else:
                    n = _block_field_count(blocks)
                    cells[d] = f"✓ ({n})" if n else "✓"

        # Only compare adjacent distros that this repo actually has (no "only in X" for uncloned distros)
        repo_distros = [d for d in ordered_distros if d in data[repo]]
        change_parts: list[str] = []
        for i in range(len(repo_distros) - 1):
            old_d, new_d = repo_distros[i], repo_distros[i + 1]
            va = by_distro.get(old_d)
            vb = by_distro.get(new_d)
            if va is None and vb is None:
                continue
            if va is None:
                change_parts.append(f"`{old_d}`→`{new_d}`: only in `{new_d}`")
                summary["per_repo"][repo]["with_progressive_diff"] += 1
                continue
            if vb is None:
                change_parts.append(f"`{old_d}`→`{new_d}`: only in `{old_d}`")
                summary["per_repo"][repo]["with_progressive_diff"] += 1
                continue
            delta = diff_blocks(va, vb)
            if not delta["identical"]:
                summary["per_repo"][repo]["with_progressive_diff"] += 1
                desc = _describe_diff(delta)
                change_parts.append(f"`{old_d}`→`{new_d}`: " + "; ".join(desc[:5]) + (" …" if len(desc) > 5 else ""))

        rows.append({
            "interface": f"{repo}/{key}",
            "cells": cells,
            "changes": " | ".join(change_parts) if change_parts else "",
        })

    summary["per_repo"] = dict(summary["per_repo"])
    return summary, rows


def write_md_report(summary: dict, rows: list[dict[str, Any]], out_path: Path) -> None:
    ordered = summary.get("distros", [])
    # One big table: header = Interface | distro1 | distro2 | ... | Changes (progressive)
    header_cells = ["Interface"] + ordered + ["Changes (progressive)"]
    sep = "| " + " | ".join(["---"] * len(header_cells)) + " |"
    lines = [
        "# ROS 2 IDL diff report (progressive by distro order)",
        "",
        "Distros are compared in **chronological release order** (each vs next): " + " → ".join(f"`{d}`" for d in ordered) + ".",
        "",
        "## Summary",
        f"- Repos: {', '.join(f'`{r}`' for r in summary.get('repos', []))}",
        f"- Distros (order): {', '.join(f'`{d}`' for d in ordered)}",
        f"- Total interfaces: {summary.get('total_interfaces', 0)}",
        "",
        "### Per repo",
        "",
        "| Column | Meaning |",
        "|--------|--------|",
        "| **Interfaces** | Number of distinct definitions (e.g. `std_msgs/msg/Header` = 1). Same definition in several distros still counts as 1. |",
        "| **Only in some distros** | Among the distros **fetched for this repo**, how many definitions exist in only part of them. For repos with fewer distros (e.g. only humble+jazzy), only those count—not the full list. |",
        "| **Changed at some step** | Count of *steps* where a definition differs (each foxy→humble, humble→jazzy, jazzy→rolling). One interface can contribute more than once if it changes in multiple steps; so this number can be larger than Interfaces. |",
        "",
        "| Repo | Interfaces | Only in some distros | Changed at some step |",
        "|------|------------|----------------------|----------------------|",
    ]
    for repo, rec in summary.get("per_repo", {}).items():
        lines.append(f"| `{repo}` | {rec['total']} | {rec['only_in_some']} | {rec['with_progressive_diff']} |")
    lines.extend([
        "",
        "---",
        "",
        "## Interface table (one row per definition)",
        "",
        "Distro columns: `✓ (n)` = present; `—` = absent in that distro; `∅` = branch not fetched for this repo.",
        "",
        "| " + " | ".join(header_cells) + " |",
        sep,
    ])
    for row in rows:
        interface_cell = "`" + row["interface"].replace("`", "\\`") + "`"
        distro_cells = [row["cells"].get(d, "∅") for d in ordered]
        changes_raw = row.get("changes", "")
        # Split Changes into lines: each transition on its own line, items within separated too
        changes_md = changes_raw.replace(" | ", "<br><br>").replace("; ", "<br>") if changes_raw else ""
        cell_values = [interface_cell] + distro_cells + [changes_md]
        # Escape pipes in cell text for Markdown
        cell_values = [str(c).replace("|", "\\|") for c in cell_values]
        lines.append("| " + " | ".join(cell_values) + " |")
    lines.append("")
    out_path.write_text("\n".join(lines), encoding="utf-8")


def write_json_report(summary: dict, rows: list[dict[str, Any]], out_path: Path) -> None:
    payload = {"summary": summary, "rows": rows}
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze ROS 2 IDL differences across distros.")
    parser.add_argument("--repos-dir", type=Path, default=Path("repos"), help="Root of cloned repos (e.g. repos/)")
    parser.add_argument("--out-dir", type=Path, default=Path("reports"), help="Output directory for reports")
    parser.add_argument("--distros", nargs="*", help="Limit to these distros (default: all found)")
    args = parser.parse_args()

    root = args.repos_dir
    if not root.is_absolute():
        root = Path(__file__).resolve().parent.parent / root
    if not root.exists():
        print(f"Repos root does not exist: {root}", file=__import__("sys").stderr)
        return 1

    summary, rows = run_analysis(root, args.distros)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    md_path = args.out_dir / "idl_diff_report.md"
    json_path = args.out_dir / "idl_diff_report.json"
    write_md_report(summary, rows, md_path)
    write_json_report(summary, rows, json_path)
    print(f"Report written: {md_path}")
    print(f"JSON written:  {json_path}")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
