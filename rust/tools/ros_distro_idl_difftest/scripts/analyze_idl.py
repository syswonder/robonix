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
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# --- Simple IDL parsing ---

def _strip_comment(line: str) -> str:
    i = line.find("#")
    return (line[:i] if i >= 0 else line).strip()

def _parse_msg_like_block(lines: list[str]) -> list[tuple[str, str]]:
    """Parse a .msg block or one section of .srv (request/response) or .action. Returns [(type, name), ...]"""
    fields: list[tuple[str, str]] = []
    for raw in lines:
        line = _strip_comment(raw)
        if not line:
            continue
        # Constant: UPPER_CASE= value or field: type name [= default]
        if "=" in line and not line.strip().startswith(("int", "float", "string", "bool", "byte", "char", "wstring")):
            # Likely constant, format "NAME= value"
            const_match = re.match(r"^([A-Z][A-Z0-9_]*)\s*=\s*(.*)$", line)
            if const_match:
                name, _ = const_match.group(1), const_match.group(2)
                fields.append(("const", name))
                continue
        parts = line.split()
        if len(parts) >= 2:
            # Last token is name, rest is type (may include [])
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


# --- Main flow: scan repos, align by distro, generate report ---

@dataclass
class DistroDiff:
    distro_a: str
    distro_b: str
    key: str
    only_in_a: bool
    only_in_b: bool
    diff: dict[str, Any]


def run_analysis(repos_root: Path, distros: list[str] | None = None) -> tuple[dict, list[DistroDiff]]:
    """
    repos_root: path to repos/ (under it: repo_name/distro_name/...).
    Returns (summary, list of DistroDiff).
    """
    if distros is None:
        distros = []
    # Collect all IDLs per (repo, distro)
    # structure: data[repo][distro][key] = blocks
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

    # Union of all (repo, key)
    all_keys: set[tuple[str, str]] = set()
    for repo, by_distro in data.items():
        for distro, by_key in by_distro.items():
            for key in by_key:
                all_keys.add((repo, key))

    # For each (repo, key), compare every distro pair
    diffs: list[DistroDiff] = []
    summary: dict[str, Any] = {
        "repos": list({r for r, _ in all_keys}),
        "distros": distros or list(set(d for repo in data for d in data[repo])),
        "total_interfaces": len(all_keys),
        "per_repo": defaultdict(lambda: {"total": 0, "only_in_some": 0, "diff_count": 0}),
        "per_distro_pair": defaultdict(int),
    }

    all_distros = set()
    for repo in data:
        all_distros.update(data[repo].keys())
    distro_list_global = sorted(all_distros)

    for repo, key in sorted(all_keys):
        summary["per_repo"][repo]["total"] += 1
        by_distro = {d: data[repo][d].get(key) for d in data[repo]}
        present = [d for d in by_distro if by_distro[d] is not None]
        only_in_some = len(present) < len(by_distro)
        if only_in_some:
            summary["per_repo"][repo]["only_in_some"] += 1

        for i, da in enumerate(distro_list_global):
            for db in distro_list_global[i + 1 :]:
                va = by_distro.get(da)
                vb = by_distro.get(db)
                if va is None and vb is None:
                    continue
                if va is None:
                    diffs.append(DistroDiff(da, db, f"{repo}/{key}", False, True, {}))
                    summary["per_distro_pair"][(da, db)] += 1
                    continue
                if vb is None:
                    diffs.append(DistroDiff(da, db, f"{repo}/{key}", True, False, {}))
                    summary["per_distro_pair"][(da, db)] += 1
                    continue
                delta = diff_blocks(va, vb)
                if not delta["identical"]:
                    diffs.append(DistroDiff(da, db, f"{repo}/{key}", False, False, delta))
                    summary["per_repo"][repo]["diff_count"] += 1
                    summary["per_distro_pair"][(da, db)] += 1

    summary["per_repo"] = dict(summary["per_repo"])
    summary["per_distro_pair"] = {f"{a}_{b}": c for (a, b), c in summary["per_distro_pair"].items()}
    return summary, diffs


def write_md_report(summary: dict, diffs: list[DistroDiff], out_path: Path) -> None:
    lines = [
        "# ROS 2 IDL diff report",
        "",
        "## Summary",
        f"- Repos: {', '.join(summary.get('repos', []))}",
        f"- Distros: {', '.join(summary.get('distros', []))}",
        f"- Total interfaces (pkg+kind+name): {summary.get('total_interfaces', 0)}",
        "",
        "### Per repo",
        "| Repo | Interfaces | Only in some distros | With field/structure diff |",
        "|------|------------|----------------------|----------------------------|",
    ]
    for repo, rec in summary.get("per_repo", {}).items():
        lines.append(f"| {repo} | {rec['total']} | {rec['only_in_some']} | {rec['diff_count']} |")
    lines.extend([
        "",
        "### Per distro pair (interface diff count)",
        "| Distro A | Distro B | Diff count |",
        "|----------|----------|------------|",
    ])
    for pair, count in sorted(summary.get("per_distro_pair", {}).items()):
        a, b = pair.replace("_", " ").split(maxsplit=1) if "_" in pair else (pair, "")
        lines.append(f"| {a} | {b} | {count} |")
    lines.extend(["", "---", "", "## Details (interfaces with diffs)", ""])

    for d in diffs:
        lines.append(f"### {d.key} ({d.distro_a} vs {d.distro_b})")
        if d.only_in_a:
            lines.append("- Only in " + d.distro_a)
        elif d.only_in_b:
            lines.append("- Only in " + d.distro_b)
        else:
            for block in d.diff.get("blocks_changed", []):
                lines.append(f"- **{block['block']}**")
                for x in block.get("added", []):
                    lines.append(f"  - Added: `{x['type']} {x['name']}`")
                for x in block.get("removed", []):
                    lines.append(f"  - Removed: `{x['type']} {x['name']}`")
                for x in block.get("type_changed", []):
                    lines.append(f"  - Type changed: `{x['name']}` {x['from']} -> {x['to']}")
        lines.append("")

    out_path.write_text("\n".join(lines), encoding="utf-8")


def write_json_report(summary: dict, diffs: list[DistroDiff], out_path: Path) -> None:
    payload = {
        "summary": summary,
        "diffs": [
            {
                "distro_a": d.distro_a,
                "distro_b": d.distro_b,
                "key": d.key,
                "only_in_a": d.only_in_a,
                "only_in_b": d.only_in_b,
                "diff": d.diff,
            }
            for d in diffs
        ],
    }
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

    summary, diffs = run_analysis(root, args.distros)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    from datetime import datetime
    ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    md_path = args.out_dir / f"idl_diff_report_{ts}.md"
    json_path = args.out_dir / f"idl_diff_report_{ts}.json"
    write_md_report(summary, diffs, md_path)
    write_json_report(summary, diffs, json_path)
    print(f"Report written: {md_path}")
    print(f"JSON written:  {json_path}")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
