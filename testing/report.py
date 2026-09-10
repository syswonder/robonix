# SPDX-License-Identifier: MulanPSL-2.0
"""Generate an offline Webots CI report with an embedded log viewer."""

from __future__ import annotations

import argparse
import base64
import html
import json
import mimetypes
import os
import re
import shutil
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path
from urllib.parse import urlparse


ALLOWED_LOG_SUFFIXES = {
    ".json",
    ".jsonl",
    ".log",
    ".md",
    ".out",
    ".stderr",
    ".stdout",
    ".txt",
    ".yaml",
    ".yml",
}
ALLOWED_LOG_NAMES = {
    "Dockerfile",
}
REDACTIONS = [
    re.compile(r"(?i)(authorization:\s*bearer\s+)[^\s]+"),
    re.compile(r"(?i)((?:github_)?token\s*[:=]\s*)[^\s]+"),
    re.compile(r"(?i)((?:api[_-]?key|password|secret)\s*[:=]\s*)[^\s]+"),
]


BEIJING_TZ = timezone(timedelta(hours=8))
TIME_METADATA_SUFFIXES = ("_on", "_at", "_date")


def _format_beijing_time(raw: str) -> str:
    if not raw:
        return ""
    text = str(raw).replace("Z", "+00:00")
    try:
        dt = datetime.fromisoformat(text)
    except ValueError:
        return str(raw)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt.astimezone(BEIJING_TZ).isoformat(timespec="seconds")


def _read_json(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        return {}


def _redact_text(text: str) -> str:
    for pattern in REDACTIONS:
        text = pattern.sub(r"\1[REDACTED]", text)
    return text


def _safe_href(value: str) -> str:
    try:
        parsed = urlparse(value)
    except ValueError:
        return ""
    if parsed.scheme not in {"http", "https"} or not parsed.netloc:
        return ""
    host = parsed.netloc.lower()
    if host == "github.com" or host.endswith(".github.com"):
        return value
    if host == "github.io" or host.endswith(".github.io"):
        return value
    return ""


def _status_label(passed: bool) -> str:
    return "PASS" if passed else "FAIL"


def _code(value: object, class_name: str = "") -> str:
    cls = f" mono {class_name}".rstrip()
    return f'<span class="{cls}">{html.escape(str(value))}</span>'


def _button(label: object, log_name: str, class_name: str = "") -> str:
    cls = f" {class_name}" if class_name else ""
    return (
        f'<button class="link-button{cls}" data-log-name="{html.escape(str(log_name))}" '
        f'onclick="openLogByName(this.dataset.logName)">{html.escape(str(label))}</button>'
    )


def _code_lines(values: list[object], class_name: str = "") -> str:
    if not values:
        return '<span class="muted">-</span>'
    return "".join(f'<div class="mono-line">{_code(v, class_name)}</div>' for v in values)


def _failure_lines(values: list[object]) -> str:
    if not values:
        return '<span class="muted">-</span>'
    return "".join(f'<div class="failure-line">{_code(v, "failure-text")}</div>' for v in values)


def _compact_json(value: object) -> str:
    try:
        return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ": "))
    except TypeError:
        return str(value)


def _count_rtdl_leaves(node: object) -> int:
    if not isinstance(node, dict):
        return 0
    count = 1 if node.get("op") == "do" else 0
    return count + sum(_count_rtdl_leaves(child) for child in node.get("children", []) or [])


def _rtdl_node(node: object) -> str:
    if not isinstance(node, dict):
        return f'<div class="rtdl-line malformed">{_code(node)}</div>'

    op = str(node.get("op", ""))
    node_id = node.get("id", "")
    cap = node.get("cap", "")
    label = op
    if op == "do" and cap:
        label = f"do {cap}"
    elif node_id:
        label = f"{op} {node_id}"

    bits = [f'<span class="rtdl-op">{html.escape(label)}</span>']
    if op == "do" and node_id:
        bits.append(f'<span class="rtdl-id">{html.escape(str(node_id))}</span>')
    if "args" in node:
        bits.append(f'<span class="rtdl-args">args {_code(_compact_json(node.get("args")), "rtdl-json")}</span>')
    expect = node.get("expect")
    if isinstance(expect, dict):
        contract = expect.get("contract", "")
        success = expect.get("success", True)
        bits.append(
            '<span class="rtdl-expect">'
            f'expect {_code(contract, "contract")} '
            f'{"success" if success else "failure"}'
            "</span>"
        )

    children = "".join(_rtdl_node(child) for child in node.get("children", []) or [])
    child_block = f'<div class="rtdl-children">{children}</div>' if children else ""
    cls = "rtdl-node rtdl-leaf" if op == "do" else "rtdl-node"
    return f'<div class="{cls}"><div class="rtdl-line">{" ".join(bits)}</div>{child_block}</div>'


def _observed_round(round_data: dict | None) -> str:
    if not isinstance(round_data, dict):
        return '<div class="observed-round muted">No observed plan round was recorded for this step.</div>'
    calls = round_data.get("calls", [])
    if not calls:
        return '<div class="observed-round muted">Observed plan round had no calls.</div>'

    rows = []
    for call in calls:
        leaf = call.get("leaf_result", {}) if isinstance(call.get("leaf_result"), dict) else {}
        success = leaf.get("success")
        state = "PASS" if success is True else ("FAIL" if success is False else "PENDING")
        state_cls = "pass" if success is True else ("fail" if success is False else "")
        output = leaf.get("error") or leaf.get("output") or ""
        row_state = state_cls or "pending"
        row_cls = f"observed-call observed-{row_state} result-row-{row_state}"
        rows.append(
            f'<div class="{row_cls}">'
            f'<span class="status-badge {state_cls}">{state}</span>'
            f'{_code(call.get("call_id", ""), "path")}'
            f'{_code(call.get("contract", ""), "contract")}'
            f'<span class="rtdl-args">args {_code(_compact_json(call.get("args", {})), "rtdl-json")}</span>'
            f'<span class="observed-output">{html.escape(str(output))}</span>'
            "</div>"
        )
    return (
        f'<div class="observed-round"><div class="observed-title">Observed plan round {html.escape(str(round_data.get("index", "")))}</div>'
        f'{"".join(rows)}</div>'
    )


def _rtdl_step_block(step: dict, observed: dict | None) -> str:
    idx = step.get("index", "")
    desc = step.get("description") or step.get("content") or "RTDL planning round"
    leaves = _count_rtdl_leaves(step.get("rtdl"))
    summary = (
        f'<span class="rtdl-step-title">step {html.escape(str(idx))}</span>'
        f'<span class="rtdl-step-desc">{html.escape(str(desc))}</span>'
        f'<span class="rtdl-step-meta">{leaves} leaf{"s" if leaves != 1 else ""}</span>'
    )
    return (
        f'<details class="rtdl-step" open><summary>{summary}</summary>'
        f'{_rtdl_node(step.get("rtdl"))}'
        f'{_observed_round(observed)}'
        "</details>"
    )


def _scenario_rtdl_trees(summary: dict) -> str:
    sections = []
    for sc in summary.get("scenarios", []):
        steps = sc.get("rtdl_steps", [])
        if not steps:
            continue
        leaf_count = sum(_count_rtdl_leaves(step.get("rtdl")) for step in steps)
        has_multi_leaf_step = any(_count_rtdl_leaves(step.get("rtdl")) > 1 for step in steps)
        open_attr = " open" if (not sc.get("passed") or has_multi_leaf_step) else ""
        observed_rounds = sc.get("observed_rounds", [])
        step_html = "".join(
            _rtdl_step_block(
                step,
                observed_rounds[pos] if isinstance(observed_rounds, list) and pos < len(observed_rounds) else None,
            )
            for pos, step in enumerate(steps)
        )
        scenario_state = "pass" if sc.get("passed") else "fail"
        sections.append(
            f'<details class="rtdl-scenario result-row-{scenario_state}"{open_attr}>'
            "<summary>"
            f'<span class="status-badge {scenario_state}">{_status_label(bool(sc.get("passed")))}</span>'
            f'{_code(sc.get("family", ""), "suite")}/'
            f'{_code(sc.get("name", ""), "scenario")}'
            f'<span class="rtdl-step-meta">{len(steps)} step(s), {leaf_count} leaf call(s)</span>'
            "</summary>"
            f"{step_html}"
            "</details>"
        )
    if not sections:
        return '<p class="muted">No RTDL plan trees were recorded in the summary.</p>'
    return "\n".join(sections)


def _scenario_rows(summary: dict) -> str:
    rows = []
    for sc in summary.get("scenarios", []):
        passed = bool(sc.get("passed"))
        status = _status_label(passed)
        log = str(sc.get("log", ""))
        log_cell = _button(log or "-", log, "path-link") if log else '<span class="muted">-</span>'
        rows.append(
            f"<tr class=\"{'row-pass' if passed else 'row-fail'}\">"
            f"<td><span class=\"status-badge {'pass' if passed else 'fail'}\">{status}</span></td>"
            f"<td>{_code(sc.get('family', ''), 'suite')}</td>"
            f"<td>{_code(sc.get('name', ''), 'scenario')}</td>"
            f"<td class=\"number\">{html.escape(str(sc.get('rounds', '')))}</td>"
            f"<td>{_code_lines(sc.get('dispatched', []), 'contract')}</td>"
            f"<td>{_failure_lines(sc.get('failures', []))}</td>"
            f"<td>{log_cell}</td>"
            "</tr>"
        )
    return "\n".join(rows)



def _analysis_list(values: object) -> str:
    if not values:
        return '<p class="muted">-</p>'
    if isinstance(values, str):
        values = [values]
    if not isinstance(values, list):
        values = [values]
    items = []
    for value in values:
        if isinstance(value, dict):
            value = value.get("text") or _compact_json(value)
        items.append(f"<li>{html.escape(str(value))}</li>")
    return f"<ul>{''.join(items)}</ul>"


def _llm_analysis_section(analysis: dict | None) -> str:
    if not isinstance(analysis, dict) or not analysis:
        return ""
    title = str(analysis.get("title") or "LLM analysis")
    summary = str(analysis.get("summary") or "")
    available = analysis.get("available")
    badge = "available" if available else "unavailable"
    stage = str(analysis.get("stage") or "")
    confidence = str(analysis.get("confidence") or "")
    meta = []
    for key in ("provider", "model", "generated_on"):
        if analysis.get(key):
            meta.append(f"{_label_for_key(key)}: {html.escape(str(analysis[key]))}")
    if stage:
        meta.append(f"Stage: {html.escape(stage)}")
    if confidence:
        meta.append(f"Confidence: {html.escape(confidence)}")

    evidence_rows = []
    evidence = analysis.get("evidence")
    if isinstance(evidence, list):
        for item in evidence:
            if isinstance(item, dict):
                source = str(item.get("source") or "")
                line = item.get("line", "")
                excerpt = str(item.get("text") or "")
            else:
                source, line, excerpt = "", "", str(item)
            source_html = _button(source, Path(source).name, "path-link") if source else '<span class="muted">-</span>'
            evidence_rows.append(
                "<tr>"
                f"<td>{source_html}</td>"
                f"<td class=\"number\">{html.escape(str(line)) if line else '-'}</td>"
                f"<td>{html.escape(excerpt)}</td>"
                "</tr>"
            )
    evidence_html = ""
    if evidence_rows:
        evidence_html = (
            '<h3>Evidence</h3><table class="evidence-table">'
            '<thead><tr><th>Source</th><th>Line</th><th>Excerpt</th></tr></thead>'
            f"<tbody>{''.join(evidence_rows)}</tbody></table>"
        )

    sections = []
    field_labels = [
        ("pr_changes", "PR Changes"),
        ("test_result", "Test Result"),
        ("likely_root_cause", "Likely Root Cause"),
        ("suggested_fix", "Suggested Fix"),
        ("risks_or_watchouts", "Risks Or Watchouts"),
    ]
    for key, label in field_labels:
        value = analysis.get(key)
        if value:
            sections.append(f"<h3>{label}</h3>{_analysis_list(value)}")

    unavailable_note = "" if available else '<p class="muted">LLM analysis was not available; deterministic logs remain embedded below.</p>'
    meta_html = f"<p class=\"analysis-meta\">{' · '.join(meta)}</p>" if meta else ""
    return (
        '<h2>LLM-Assisted Analysis</h2>'
        f'<section class="analysis-card analysis-{html.escape(badge)}">'
        f'<div class="analysis-title">{html.escape(title)}</div>'
        f"{meta_html}"
        f'<p>{html.escape(summary)}</p>'
        f"{unavailable_note}"
        f"{''.join(sections)}"
        f"{evidence_html}"
        "</section>"
    )

def _coverage(summary: dict) -> str:
    rows = []
    for contract in summary.get("coverage", []):
        parts = str(contract).split("/")
        area = "/".join(parts[:3]) if len(parts) >= 3 else str(contract)
        rows.append(
            "<tr>"
            f"<td>{html.escape(area)}</td>"
            f"<td>{_code(contract, 'contract')}</td>"
            "</tr>"
        )
    if not rows:
        rows.append('<tr><td colspan="2"><span class="muted">No runtime contracts were recorded.</span></td></tr>')
    return (
        '<p class="section-note">'
        "Contracts observed by the scenario runner in executor leaf results. "
        "This shows which runtime contract ids were exercised by this run; it is not a full API inventory."
        "</p>"
        '<table class="coverage-table">'
        "<thead><tr><th>Area</th><th>Contract id</th></tr></thead>"
        f"<tbody>{''.join(rows)}</tbody>"
        "</table>"
    )


def _metadata_from_env() -> dict[str, str]:
    metadata = {"generated_on": datetime.now(timezone.utc).isoformat(timespec="seconds")}
    keys = {
        "workflow": "GITHUB_WORKFLOW",
        "run_id": "GITHUB_RUN_ID",
        "run_attempt": "GITHUB_RUN_ATTEMPT",
        "event": "GITHUB_EVENT_NAME",
        "repository": "GITHUB_REPOSITORY",
        "ref": "GITHUB_REF",
        "sha": "GITHUB_SHA",
        "actor": "GITHUB_ACTOR",
        "triggering_actor": "GITHUB_TRIGGERING_ACTOR",
        "runner_name": "RUNNER_NAME",
        "runner_os": "RUNNER_OS",
        "runner_arch": "RUNNER_ARCH",
    }
    metadata.update({label: os.environ[env] for label, env in keys.items() if os.environ.get(env)})
    server = os.environ.get("GITHUB_SERVER_URL")
    repo = os.environ.get("GITHUB_REPOSITORY")
    run_id = os.environ.get("GITHUB_RUN_ID")
    if server and repo and run_id:
        metadata["run_url"] = f"{server}/{repo}/actions/runs/{run_id}"
    if server and repo and metadata.get("sha"):
        metadata["commit_url"] = f"{server}/{repo}/commit/{metadata['sha']}"
    return metadata


def _parse_metadata(items: list[str], json_files: list[Path]) -> dict[str, str]:
    metadata = _metadata_from_env()
    for path in json_files:
        if not path.exists():
            continue
        try:
            data = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        if isinstance(data, dict):
            for key, value in data.items():
                key = str(key)
                if key == "generated_on" and metadata.get("generated_on"):
                    metadata["test_report_generated_on"] = str(value)
                else:
                    metadata[key] = str(value)
    for item in items:
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        key = key.strip()
        if key:
            metadata[key] = value.strip()
    return metadata


def _display_value(key: str, value: str, metadata: dict[str, str]) -> str:
    if not value:
        return '<span class="muted">-</span>'

    href = ""
    text = value
    if key.endswith(TIME_METADATA_SUFFIXES):
        text = _format_beijing_time(value)
    if key == "tested_commit":
        href = metadata.get("commit_url", "")
        text = value[:12]
    elif key == "sha":
        href = metadata.get("commit_url", "")
        text = value[:12]
    elif key == "pr_number":
        href = metadata.get("pr_url", "")
        text = f"#{value}"
    elif key == "run_id":
        href = metadata.get("run_url", "")
    elif key in {"run_url", "pr_url", "commit_url", "report_url", "origin_url"}:
        href = value

    escaped = html.escape(text)
    href = _safe_href(href)
    if href:
        return f'<a href="{html.escape(href, quote=True)}">{escaped}</a>'
    return escaped


def _label_for_key(key: str) -> str:
    if key.startswith("env_"):
        key = key[4:]
    return key.replace("_", " ").title()


def _table_for_keys(metadata: dict[str, str], keys: list[str], empty: str) -> str:
    if not keys:
        return f'<p class="muted">{html.escape(empty)}</p>'
    rows = []
    for key in keys:
        value = metadata.get(key, "")
        display = _display_value(key, value, metadata)
        rows.append(
            "<tr>"
            f"<th>{html.escape(_label_for_key(key))}</th>"
            f"<td>{display}</td>"
            "</tr>"
        )
    return f'<table class="metadata"><tbody>{"".join(rows)}</tbody></table>'


def _metadata_table(metadata: dict[str, str]) -> str:
    if not metadata:
        return '<p class="muted">No run metadata was provided.</p>'
    ordered = [
        "generated_on",
        "test_report_generated_on",
        "report_url",
        "run_id",
        "workflow",
        "event",
        "repository",
        "tested_commit",
        "tested_commit_subject",
        "tested_commit_author",
        "tested_commit_date",
        "pr_number",
        "pr_title",
        "pr_author",
        "pr_url",
        "pr_head_ref",
        "pr_head_repo",
        "triggered_by",
        "actor",
        "triggering_actor",
        "runner_name",
        "runner_os",
        "runner_arch",
        "origin_url",
    ]
    keys = [k for k in ordered if metadata.get(k) and not k.startswith("env_")]
    keys.extend(k for k in metadata if metadata.get(k) and k not in set(keys) and not k.endswith("_url") and not k.startswith("env_"))
    return _table_for_keys(metadata, keys, "No run metadata was provided.")


def _environment_table(metadata: dict[str, str]) -> str:
    ordered = [
        "env_hostname",
        "env_os_pretty_name",
        "env_kernel",
        "env_arch",
        "env_cpu_model",
        "env_cpu_cores",
        "env_mem_total",
        "env_gpu_summary",
        "env_nvidia_driver",
        "env_cuda_version",
        "env_docker_version",
        "env_docker_compose_version",
        "env_webots_version",
        "env_ros_distro",
        "env_ros_domain_id",
        "env_sim_container",
        "env_sim_image",
        "env_webots_stream_port",
        "env_webots_viewer_port",
        "env_python_version",
        "env_rustc_version",
        "env_rbnx_version",
    ]
    keys = [k for k in ordered if metadata.get(k)]
    keys.extend(k for k in metadata if metadata.get(k) and k.startswith("env_") and k not in set(keys))
    return _table_for_keys(metadata, keys, "No test environment metadata was provided.")


def write_metadata(metadata: dict[str, str], out: Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")


def _split_label_path(spec: str) -> tuple[str, Path]:
    if "=" in spec:
        label, path = spec.split("=", 1)
        return label.strip() or Path(path).name, Path(path)
    path = Path(spec)
    return path.name, path


def _language_for(path: Path) -> str:
    suffix = path.suffix.lower()
    if suffix == ".json":
        return "json"
    if suffix == ".jsonl":
        return "jsonl"
    if suffix in {".yaml", ".yml"}:
        return "yaml"
    if suffix == ".toml":
        return "toml"
    if suffix in {".py", ".rs", ".sh", ".bash", ".js", ".ts"}:
        return suffix[1:]
    if suffix in {".md", ".txt"}:
        return suffix[1:]
    return "log"


def _read_text(path: Path, max_bytes: int) -> tuple[str, bool, int]:
    original_size = path.stat().st_size
    with path.open("rb") as f:
        data = f.read(max_bytes if max_bytes > 0 else -1)
    truncated = max_bytes > 0 and original_size > max_bytes
    return _redact_text(data.decode("utf-8", errors="replace")), truncated, original_size


def _is_allowed_log_path(path: Path, root: Path | None = None) -> bool:
    if path.is_symlink():
        return False
    try:
        resolved = path.resolve(strict=True)
    except OSError:
        return False
    if not resolved.is_file():
        return False
    if root is not None:
        try:
            resolved.relative_to(root.resolve(strict=True))
        except (OSError, ValueError):
            return False
    return path.name in ALLOWED_LOG_NAMES or path.suffix.lower() in ALLOWED_LOG_SUFFIXES


def _collect_logs(
    log_roots: list[str],
    log_files: list[str],
    summary_json: Path,
    max_log_bytes: int,
    max_total_log_bytes: int,
) -> list[dict]:
    specs: list[tuple[str, Path, Path | None]] = []
    if not log_roots and not log_files:
        log_roots = [f"testing/logs={summary_json.parent}"]

    for spec in log_roots:
        label, root = _split_label_path(spec)
        if root.exists() and root.is_dir():
            for path in sorted(p for p in root.rglob("*") if _is_allowed_log_path(p, root)):
                specs.append((label, path, root))

    for spec in log_files:
        label, path = _split_label_path(spec)
        if _is_allowed_log_path(path):
            specs.append((label, path, None))

    entries = []
    seen: set[Path] = set()
    total_embedded = 0
    for label, path, root in specs:
        try:
            resolved = path.resolve()
        except OSError:
            resolved = path
        if resolved in seen:
            continue
        seen.add(resolved)

        if root is not None:
            try:
                rel = path.relative_to(root)
            except ValueError:
                rel = Path(path.name)
            display = f"{label}/{rel.as_posix()}"
        else:
            display = label

        try:
            size = path.stat().st_size
            remaining = max_total_log_bytes - total_embedded if max_total_log_bytes > 0 else 0
            if max_total_log_bytes > 0 and remaining <= 0:
                text = "<log omitted: total embedded log byte limit reached>"
                truncated = True
            else:
                caps = [cap for cap in (max_log_bytes, remaining) if cap > 0]
                read_cap = min(caps) if caps else 0
                text, truncated, size = _read_text(path, read_cap)
                if max_total_log_bytes > 0 and size > remaining:
                    truncated = True
                total_embedded += len(text.encode("utf-8", errors="replace"))
        except OSError as exc:
            text = f"<failed to read log: {exc}>"
            truncated = False
            size = 0

        entries.append(
            {
                "id": f"log-{len(entries)}",
                "name": path.name,
                "path": display,
                "size": size,
                "truncated": truncated,
                "language": _language_for(path),
                "content": text,
            }
        )
    return entries


def _render_log_tree_fallback(logs: list[dict]) -> str:
    if not logs:
        return '<p class="muted">No logs were embedded.</p>'

    tree: dict[str, object] = {}
    for entry in logs:
        parts = [part for part in str(entry["path"]).split("/") if part]
        cursor = tree
        for dirname in parts[:-1]:
            cursor = cursor.setdefault(dirname, {})  # type: ignore[assignment]
        cursor.setdefault("__files__", []).append(entry)  # type: ignore[union-attr]

    def render_node(node: dict[str, object]) -> str:
        out: list[str] = []
        files = sorted(node.get("__files__", []), key=lambda item: str(item["path"]))
        for entry in files:
            out.append(
                f'<button class="tree-file" data-log-id="{entry["id"]}" data-log-path="{html.escape(str(entry["path"]))}" '
                f'onclick="openLog(this.dataset.logId)">'
                f'<span class="tree-name">{html.escape(str(entry["name"]))}</span>'
                f'<span class="tree-meta">{html.escape(entry["language"])} · {entry["size"]} B</span>'
                "</button>"
            )
        for name, child in sorted((k, v) for k, v in node.items() if k != "__files__"):
            out.append(f'<details class="tree-group" open><summary>{html.escape(str(name))}</summary>')
            out.append(render_node(child))  # type: ignore[arg-type]
            out.append("</details>")
        return "\n".join(out)

    return render_node(tree)


def _log_tree_data(logs: list[dict]) -> list[dict]:
    nodes: list[dict] = []
    folders: dict[tuple[str, ...], str] = {}

    def folder_id(parts: tuple[str, ...]) -> str:
        existing = folders.get(parts)
        if existing:
            return existing
        node_id = f"tree-dir-{len(folders)}"
        folders[parts] = node_id
        nodes.append(
            {
                "id": node_id,
                "parent": folder_id(parts[:-1]) if len(parts) > 1 else "#",
                "text": parts[-1],
                "type": "folder",
                "state": {"opened": len(parts) <= 2},
            }
        )
        return node_id

    for entry in logs:
        parts = tuple(part for part in str(entry["path"]).split("/") if part)
        if not parts:
            continue
        parent = folder_id(parts[:-1]) if len(parts) > 1 else "#"
        nodes.append(
            {
                "id": entry["id"],
                "parent": parent,
                "text": entry["name"],
                "type": "file",
                "data": {"logId": entry["id"], "path": entry["path"]},
                "a_attr": {"title": f'{entry["path"]} ({entry["language"]}, {entry["size"]} B)'},
            }
        )
    return nodes


def _log_shortcuts(logs: list[dict]) -> str:
    if not logs:
        return ""
    specs = [
        ("Build", ("build-webots-deployment.log", "rbnx-build.log", "build.log")),
        ("rbnx boot", ("rbnx-boot.log",)),
        ("Scenario summary", ("summary.json",)),
        ("Scenario events", ("scenario.jsonl", "events.jsonl", "scenario-events.jsonl")),
        ("Scene", ("scene.log",)),
        ("Simulator", ("webots.log", "webots.stdout", "webots.stdout.log", "sim.log")),
    ]
    by_name: dict[str, dict] = {}
    by_path: dict[str, dict] = {}
    for entry in logs:
        by_name.setdefault(str(entry["name"]), entry)
        by_path.setdefault(str(entry["path"]), entry)
    parts = []
    for label, names in specs:
        found = None
        for name in names:
            found = by_name.get(name)
            if found:
                break
            found = next((entry for path, entry in by_path.items() if path.endswith('/' + name) or path.endswith(name)), None)
            if found:
                break
        if found:
            parts.append(
                f'<button class="log-chip" data-log-id="{found["id"]}" onclick="openLog(this.dataset.logId)">'
                f'{html.escape(label)}</button>'
            )
    if not parts:
        return ""
    return '<div class="log-shortcuts"><span>Quick logs:</span>' + "".join(parts) + "</div>"


def _preferred_log_id(logs: list[dict]) -> str:
    preferred = (
        "build-webots-deployment.log",
        "rbnx-boot.log",
        "summary.json",
        "scenario.jsonl",
        "events.jsonl",
        "scene.log",
    )
    for name in preferred:
        for entry in logs:
            if entry.get("name") == name or str(entry.get("path", "")).endswith('/' + name):
                return str(entry["id"])
    return str(logs[0]["id"]) if logs else ""


def _json_for_script(data: object) -> str:
    return json.dumps(data, ensure_ascii=False).replace("</", "<\\/")


CSS_URL_RE = re.compile(r"url\((['\"]?)([^)'\"]+)\1\)")


def _css_with_embedded_urls(path: Path) -> str:
    css = path.read_text(encoding="utf-8")
    base = path.parent

    def repl(match: re.Match[str]) -> str:
        raw = match.group(2).strip()
        if raw.startswith(("data:", "http://", "https://", "#")):
            return match.group(0)
        asset = (base / raw).resolve()
        try:
            data = asset.read_bytes()
        except OSError:
            return match.group(0)
        mime = mimetypes.guess_type(asset.name)[0] or "application/octet-stream"
        encoded = base64.b64encode(data).decode("ascii")
        return f'url("data:{mime};base64,{encoded}")'

    return CSS_URL_RE.sub(repl, css)


def _inline_styles(paths: list[Path]) -> str:
    blocks = []
    for path in paths:
        if not path or not path.exists():
            continue
        try:
            css = _css_with_embedded_urls(path)
        except OSError:
            continue
        blocks.append(f"<style data-inline-asset=\"{html.escape(path.name)}\">\n{css}\n</style>")
    return "\n".join(blocks)


def _inline_scripts(paths: list[Path]) -> str:
    blocks = []
    for path in paths:
        if not path or not path.exists():
            continue
        try:
            data = path.read_bytes()
        except OSError:
            continue
        encoded = base64.b64encode(data).decode("ascii")
        asset_name = html.escape(path.name)
        source_url = re.sub(r"[^A-Za-z0-9_.-]", "_", path.name)
        # Do not edit third-party JavaScript text before embedding it. Minified
        # bundles contain regex literals and strings where a blind </ replacement
        # can produce invalid JavaScript. The base64 wrapper keeps the HTML
        # self-contained while executing the original bytes in order.
        blocks.append(
            f'<script data-inline-asset="{asset_name}">\n'
            f'(0,eval)(atob("{encoded}") + "\\n//# sourceURL=inline-{source_url}");\n'
            f'</script>'
        )
    return "\n".join(blocks)


def _paths_from_env(name: str) -> list[Path]:
    raw = os.environ.get(name, "")
    return [Path(line.strip()) for line in raw.splitlines() if line.strip()]


def write_html(
    summary: dict,
    logs: list[dict],
    metadata: dict[str, str],
    analysis: dict | None,
    out: Path,
    inline_styles: list[Path] | None = None,
    inline_scripts: list[Path] | None = None,
    map_preview: bool = False,
) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    total = int(summary.get("total", 0) or 0)
    passed = int(summary.get("passed", 0) or 0)
    failed = int(summary.get("failed", 0) or 0)
    rate = summary.get("rate", 0)
    try:
        score = round(float(rate) * 100)
    except (TypeError, ValueError):
        score = 0
    verdict = "NO DATA" if total == 0 else ("PASS" if failed == 0 else "FAIL")
    result_class = "pass" if verdict == "PASS" else "fail"
    infrastructure_note = str(summary.get("infrastructure_note", "") or "").strip()
    infrastructure_section = (
        '<div class="infrastructure-note"><strong>Infrastructure:</strong> '
        f"{html.escape(infrastructure_note)}</div>"
        if infrastructure_note
        else ""
    )
    # slam-map.png is copied next to index.html by main(); reference it
    # relatively so the section works both on the published site and in a
    # downloaded artifact. Until now the map only reached summary.md, so the
    # report page carried no image at all.
    map_section = (
        '<h2>SLAM Map</h2>'
        '<p class="section-note">Occupancy grid saved by this run, rendered at the '
        'map\'s own resolution.</p>'
        '<div class="slam-map"><img src="slam-map.png" '
        'alt="SLAM occupancy map produced by this run"></div>'
        if map_preview
        else ""
    )
    generated_on = html.escape(_format_beijing_time(metadata.get("generated_on", "")))
    embedded_styles = _inline_styles(inline_styles or [])
    embedded_scripts = _inline_scripts(inline_scripts or [])
    log_tree_nodes_json = _json_for_script(_log_tree_data(logs))
    preferred_log_id = _preferred_log_id(logs)
    body = f"""<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\">
  <title>Robonix Webots CI Report</title>
  {embedded_styles}
  <style>
    :root {{
      --text: #111827;
      --muted: #6b7280;
      --line: #d1d5db;
      --head: #f3f4f6;
      --pass-bg: #ecfdf5;
      --pass-border: #a7f3d0;
      --pass-text: #065f46;
      --fail-bg: #fef2f2;
      --fail-border: #fecaca;
      --fail-text: #991b1b;
      --panel: #ffffff;
      --sans: Arial, Helvetica, "Liberation Sans", sans-serif;
      --mono: "JetBrains Mono", ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      background: #ffffff;
      color: var(--text);
      font-family: var(--sans);
      font-size: 13px;
      line-height: 1.35;
      margin: 24px;
    }}
    h1 {{ font-size: 22px; margin: 0 0 4px; }}
    .subtitle {{ color: var(--muted); font-size: 13px; margin: 0 0 16px; }}
    .report-nav {{ margin: 8px 0 14px; }}
    .report-nav a {{
      background: #e5e7eb;
      border: 2px solid #6b7280;
      color: #111827;
      display: inline-block;
      font-family: var(--mono);
      font-size: 12px;
      font-weight: 700;
      padding: 6px 10px;
      text-decoration: none;
      text-transform: uppercase;
    }}
    .report-nav a:hover {{ background: #d1d5db; text-decoration: underline; }}
    h2 {{ font-size: 16px; margin: 24px 0 10px; }}
    table {{
      border-collapse: collapse;
      border-spacing: 0;
      width: 100%;
      margin-top: 12px;
      border: 1px solid var(--line);
      border-radius: 0;
      overflow: visible;
    }}
    th, td {{
      border-bottom: 1px solid var(--line);
      padding: 6px 8px;
      text-align: left;
      vertical-align: top;
    }}
    th {{ background: var(--head); color: #374151; font-size: 11px; font-weight: 700; text-transform: uppercase; }}
    tr:last-child td {{ border-bottom: 0; }}
    .mono {{
      background: transparent;
      border: 0;
      border-radius: 0;
      font-family: var(--mono);
      font-size: 11px;
      padding: 0;
      white-space: pre-wrap;
      word-break: normal;
    }}
    .scenario-scroll {{
      overflow-x: auto;
      width: 100%;
    }}
    .scenario-table {{
      min-width: 1180px;
      table-layout: auto;
    }}
    .scenario-table th:nth-child(1),
    .scenario-table td:nth-child(1) {{ width: 78px; white-space: nowrap; }}
    .scenario-table th:nth-child(2),
    .scenario-table td:nth-child(2) {{ width: 92px; white-space: nowrap; }}
    .scenario-table th:nth-child(3),
    .scenario-table td:nth-child(3) {{ width: 210px; white-space: nowrap; }}
    .scenario-table th:nth-child(4),
    .scenario-table td:nth-child(4) {{ width: 72px; white-space: nowrap; }}
    .summary {{ display: flex; flex-wrap: wrap; gap: 8px; margin: 14px 0; }}
    .card {{ border: 1px solid var(--line); border-radius: 0; min-width: 140px; padding: 8px 10px; }}
    .card strong {{ color: var(--muted); display: block; font-size: 11px; margin-bottom: 6px; text-transform: uppercase; }}
    .metric {{ font-size: 16px; font-weight: 700; }}
    .score-metric {{
      font-family: var(--mono);
    }}
    .result-metric {{
      border: 1px solid currentColor;
      display: inline-block;
      font-family: var(--mono);
      font-size: 12px;
      padding: 2px 7px;
    }}
    .result-metric.pass {{ background: var(--pass-bg); color: var(--pass-text); }}
    .result-metric.fail {{ background: var(--fail-bg); color: var(--fail-text); }}
    .row-pass {{ background: var(--pass-bg); }}
    .row-fail {{ background: var(--fail-bg); }}
    .row-pass td {{ border-bottom-color: var(--pass-border); }}
    .row-fail td {{ border-bottom-color: var(--fail-border); }}
    .status-badge {{
      border-radius: 0;
      display: inline-block;
      font-family: var(--mono);
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0;
      min-width: 42px;
      padding: 2px 6px;
      text-align: center;
    }}
    .status-badge.pass {{ background: #d1fae5; color: var(--pass-text); }}
    .status-badge.fail {{ background: #fee2e2; color: var(--fail-text); }}
    .contract, .path, .suite, .scenario, .failure-text {{ font-weight: 500; }}
    .suite, .scenario, .contract, .path {{
      white-space: nowrap;
    }}
    .failure-text {{
      overflow-wrap: break-word;
      white-space: pre-wrap;
      word-break: normal;
    }}
    .mono-line {{
      max-width: 520px;
      overflow-x: auto;
      padding-bottom: 2px;
      white-space: nowrap;
    }}
    .rtdl-scenario {{
      border-top: 1px solid var(--line);
      padding: 12px 0;
    }}
    .rtdl-scenario:first-child {{ border-top: 0; }}
    .rtdl-scenario > summary {{
      align-items: center;
      cursor: pointer;
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
    }}
    .rtdl-step {{
      margin: 10px 0 0 24px;
    }}
    .rtdl-step > summary {{
      cursor: pointer;
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin-bottom: 6px;
    }}
    .rtdl-step-title, .rtdl-op, .rtdl-id, .rtdl-args, .rtdl-expect, .rtdl-json {{
      font-family: var(--mono);
      font-size: 11px;
    }}
    .rtdl-step-title, .rtdl-op {{
      color: #111827;
      font-weight: 650;
    }}
    .rtdl-step-desc, .rtdl-step-meta, .rtdl-id {{
      color: var(--muted);
      font-size: 12px;
    }}
    .rtdl-node {{
      margin: 5px 0 0 18px;
      padding-left: 12px;
      position: relative;
    }}
    .rtdl-node::before {{
      background: var(--line);
      content: "";
      height: 1px;
      left: 0;
      position: absolute;
      top: 9px;
      width: 8px;
    }}
    .rtdl-children {{
      border-left: 1px solid var(--line);
      margin-left: 4px;
      padding-left: 0;
    }}
    .rtdl-line {{
      align-items: baseline;
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      min-height: 18px;
    }}
    .rtdl-leaf .rtdl-line {{
      background: #f9fafb;
      border-radius: 0;
      padding: 2px 4px;
    }}
    .rtdl-expect {{
      color: #374151;
    }}
    .observed-round {{
      margin: 8px 0 0 30px;
    }}
    .observed-title {{
      color: var(--muted);
      font-size: 12px;
      margin-bottom: 4px;
    }}
    .observed-call {{
      align-items: baseline;
      background: var(--pass-bg);
      border-radius: 0;
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin: 3px 0;
      padding: 4px 6px;
    }}
    .observed-fail {{
      background: var(--fail-bg);
    }}
    .observed-output {{
      color: #374151;
      font-family: var(--mono);
      font-size: 11px;
      max-width: 100%;
      overflow-wrap: anywhere;
      white-space: pre-wrap;
    }}
    .failure-line {{ margin-bottom: 6px; }}
    .failure-line:last-child {{ margin-bottom: 0; }}
    .number {{
      color: #374151;
      font-family: var(--mono);
      text-align: right;
      white-space: nowrap;
    }}
    .muted {{ color: var(--muted); }}
    ul {{ padding-left: 20px; }}
    li {{ margin: 6px 0; }}
    .link-button {{
      appearance: none;
      background: transparent;
      border: 0;
      color: #1d4ed8;
      cursor: pointer;
      font: inherit;
      padding: 0;
      text-align: left;
      text-decoration: underline;
    }}
    .path-link {{
      font-family: var(--mono);
      font-size: 11px;
    }}
    .log-note {{
      color: var(--muted);
      font-size: 12px;
      margin: -4px 0 8px;
    }}
    .log-shortcuts {{
      align-items: center;
      display: flex;
      flex-wrap: wrap;
      gap: 6px;
      margin: 8px 0 10px;
    }}
    .log-shortcuts span {{ color: var(--muted); font-size: 12px; }}
    .log-chip {{
      appearance: none;
      background: #f3f4f6;
      border: 1px solid var(--line);
      border-radius: 4px;
      color: #111827;
      cursor: pointer;
      font-family: var(--mono);
      font-size: 11px;
      padding: 4px 8px;
    }}
    .log-chip:hover {{ background: #e5e7eb; }}
    .log-panel {{
      border: 1px solid var(--line);
      border-radius: 6px;
      display: grid;
      grid-template-columns: minmax(280px, 30%) 1fr;
      min-height: 580px;
      overflow: hidden;
    }}
    .log-tree {{
      background: #f9fafb;
      border-right: 1px solid var(--line);
      max-height: 820px;
      overflow: auto;
      padding: 10px;
    }}
    .log-tree-toolbar {{
      border-bottom: 1px solid var(--line);
      margin: -2px 0 8px;
      padding: 0 0 8px;
    }}
    .log-tree-search {{
      background: #ffffff;
      border: 1px solid var(--line);
      border-radius: 4px;
      box-sizing: border-box;
      color: var(--text);
      font-family: var(--mono);
      font-size: 11px;
      padding: 6px 8px;
      width: 100%;
    }}
    #log-tree-widget {{ font-family: var(--mono); font-size: 11px; }}
    #log-tree-widget .jstree-anchor {{ height: 22px; line-height: 22px; max-width: 100%; overflow: hidden; text-overflow: ellipsis; }}
    #log-tree-widget .jstree-clicked {{ background: #dbeafe; box-shadow: inset 0 0 0 1px #93c5fd; }}
    #log-tree-widget .jstree-hovered {{ background: #e5e7eb; box-shadow: none; }}
    .fallback-log-tree {{ display: block; }}
    .tree-group {{ margin: 0 0 8px 10px; }}
    .tree-group summary {{
      cursor: pointer;
      font-family: var(--mono);
      font-size: 11px;
      font-weight: 700;
      margin-bottom: 4px;
    }}
    .tree-file {{
      appearance: none;
      background: transparent;
      border: 0;
      border-radius: 4px;
      color: var(--text);
      cursor: pointer;
      display: block;
      padding: 5px 7px;
      text-align: left;
      width: 100%;
    }}
    .tree-file:hover, .tree-file.active {{ background: #e5e7eb; }}
    .tree-name {{
      display: block;
      font-family: var(--mono);
      font-size: 11px;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }}
    .tree-meta {{ color: var(--muted); display: block; font-size: 10px; margin-top: 1px; }}
    .viewer {{ min-width: 0; }}
    .viewer-head {{
      align-items: center;
      border-bottom: 1px solid var(--line);
      display: flex;
      gap: 10px;
      justify-content: space-between;
      padding: 9px 12px;
    }}
    .viewer-title {{
      font-family: var(--mono);
      font-size: 11px;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }}
    .viewer-meta {{ color: var(--muted); font-size: 11px; white-space: nowrap; }}
    .viewer-actions {{ align-items: center; display: flex; gap: 8px; }}
    .viewer-action {{
      appearance: none;
      background: transparent;
      border: 1px solid var(--line);
      border-radius: 4px;
      color: #374151;
      cursor: pointer;
      font-family: var(--mono);
      font-size: 11px;
      padding: 3px 7px;
    }}
    .viewer-action:hover {{ background: #f3f4f6; }}
    #ace-view {{
      height: 720px;
      min-height: 560px;
    }}
    .editor-loading {{
      background: #0b1020;
      color: #d1d5db;
      font-family: var(--mono);
      font-size: 11px;
      padding: 12px;
    }}
    .log-view {{
      background: #0b1020;
      color: #e5e7eb;
      display: none;
      font-family: var(--mono);
      font-size: 11px;
      line-height: 1.45;
      max-height: 720px;
      overflow: auto;
      padding: 8px 0;
    }}
    .log-line {{
      display: grid;
      grid-template-columns: 64px minmax(0, 1fr);
      min-height: 18px;
    }}
    .log-line:hover {{ background: rgba(255, 255, 255, 0.06); }}
    .line-no {{
      color: #6b7280;
      padding: 0 10px;
      text-align: right;
      user-select: none;
    }}
    .line-text {{
      background: transparent;
      border: 0;
      color: inherit;
      display: block;
      font: inherit;
      padding: 0 14px 0 0;
      white-space: pre-wrap;
      word-break: break-word;
    }}
    .tok-key {{ color: #93c5fd; }}
    .tok-string {{ color: #86efac; }}
    .tok-number {{ color: #fbbf24; }}
    .tok-bool {{ color: #c4b5fd; }}
    .tok-comment {{ color: #9ca3af; }}
    .tok-error {{ color: #fca5a5; font-weight: 700; }}
    .tok-warn {{ color: #fde68a; font-weight: 700; }}
    .tok-pass {{ color: #86efac; font-weight: 700; }}
    .analysis-card {{
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 14px 16px;
      margin: 12px 0 22px;
    }}
    .analysis-available {{ background: #f8fafc; }}
    .analysis-unavailable {{ background: #f9fafb; }}
    .analysis-title {{
      font-size: 16px;
      font-weight: 700;
      margin-bottom: 4px;
    }}
    .analysis-meta {{
      color: var(--muted);
      font-family: var(--mono);
      font-size: 11px;
      margin: 0 0 10px;
    }}
    .analysis-card h3 {{
      font-size: 12px;
      margin: 14px 0 6px;
      text-transform: uppercase;
    }}
    .evidence-table th:nth-child(1),
    .evidence-table td:nth-child(1) {{ width: 240px; }}
    .evidence-table th:nth-child(2),
    .evidence-table td:nth-child(2) {{ width: 80px; }}
    .metadata th {{
      text-transform: none;
      width: 180px;
    }}
    .metadata td {{
      font-family: var(--mono);
      font-size: 11px;
      word-break: break-word;
    }}
    .section-note {{
      color: var(--muted);
      margin: 6px 0 10px;
    }}
    .slam-map {{ margin: 12px 0 20px; }}
    /* Scaled up with nearest-neighbour: the grid is only a couple hundred
       pixels wide at its own resolution, and smoothing an occupancy grid
       invents wall edges the map does not actually have. */
    .slam-map img {{
      width: min(560px, 100%);
      image-rendering: pixelated;
      border: 1px solid #d0d0d0;
      background: #fff;
    }}
    .infrastructure-note {{
      background: #fffbeb;
      border: 1px solid #fcd34d;
      color: #92400e;
      margin: 0 0 16px;
      padding: 10px 12px;
    }}
    .coverage-table td:first-child {{
      color: #374151;
      font-family: var(--mono);
      font-size: 11px;
      white-space: nowrap;
      width: 220px;
    }}
  </style>
</head>
<body>
  <h1>Robonix Webots CI Report</h1>
  <div class=\"report-nav\"><a href=\"../../\">ALL REPORTS &gt;&gt;</a></div>
  <p class=\"subtitle\">Generated on {generated_on}</p>
  <div class=\"summary\">
    <div class=\"card\"><strong>Result</strong><span class=\"metric result-metric {result_class}\">{verdict}</span></div>
    <div class=\"card\"><strong>Scenarios</strong><span class=\"metric\">{passed}/{total} passed</span></div>
    <div class=\"card\"><strong>Failures</strong><span class=\"metric\">{failed}</span></div>
    <div class=\"card\"><strong>Score</strong><span class=\"metric score-metric\">{score}/100</span></div>
  </div>
  {infrastructure_section}
  {_llm_analysis_section(analysis)}
  {map_section}
  <h2>Run Metadata</h2>
  {_metadata_table(metadata)}
  <h2>Test Environment</h2>
  {_environment_table(metadata)}
  <h2>Scenarios</h2>
  <div class=\"scenario-scroll\">
  <table class=\"scenario-table\">
    <thead><tr><th>Status</th><th>Suite</th><th>Scenario</th><th>Rounds</th><th>Dispatched contracts</th><th>Failures</th><th>Log</th></tr></thead>
    <tbody>{_scenario_rows(summary)}</tbody>
  </table>
  </div>
  <h2>RTDL Plan Trees</h2>
  <p class=\"section-note\">Each scenario step is one VLM planning round. The tree below is the RTDL returned for that round; every <span class=\"mono\">do</span> leaf has its own expected runtime contract and output checks.</p>
  <div class=\"rtdl-tree-list\">{_scenario_rtdl_trees(summary)}</div>
  <h2>Logs</h2>
  <p class="log-note">Build stdout is embedded as <span class="mono">sim_logs/build-webots-deployment.log</span> when the build step starts. The deploy supervisor output with the Robonix boot banner is embedded as <span class="mono">rbnx-boot.log</span>.</p>
  {_log_shortcuts(logs)}
  <div class="log-panel">
    <aside class="log-tree">
      <div class="log-tree-toolbar"><input id="log-tree-search" class="log-tree-search" type="search" placeholder="Search logs..."></div>
      <div id="log-tree-widget"></div>
      <div id="fallback-log-tree" class="fallback-log-tree">{_render_log_tree_fallback(logs)}</div>
    </aside>
    <section class="viewer">
      <div class="viewer-head">
        <div id="viewer-title" class="viewer-title">Select a log</div>
        <div class="viewer-actions">
          <button id="copy-log-path" class="viewer-action" type="button">Copy path</button>
          <div id="viewer-meta" class="viewer-meta"></div>
        </div>
      </div>
      <div id="ace-view"><div class="editor-loading">Loading embedded Ace viewer; fallback log viewer is available if scripts are blocked.</div></div>
      <div id="log-view" class="log-view"><div class="log-line"><span class="line-no">-</span><code class="line-text">No log selected.</code></div></div>
    </section>
  </div>
  <h2>Runtime Contract Coverage</h2>
  {_coverage(summary)}
  {embedded_scripts}
  <script>
    const LOGS = {_json_for_script(logs)};
    const LOG_TREE_NODES = {log_tree_nodes_json};
    const PREFERRED_LOG_ID = "{html.escape(preferred_log_id)}";
    const LOG_BY_ID = new Map(LOGS.map((entry) => [entry.id, entry]));
    const LOG_BY_NAME = new Map();
    let currentLog = null;
    let aceEditor = null;
    let aceReady = false;
    let pendingEntry = null;
    let syncingTreeSelection = false;

    for (const entry of LOGS) {{
      if (!LOG_BY_NAME.has(entry.name)) LOG_BY_NAME.set(entry.name, entry);
      const pathTail = entry.path.split('/').slice(-1)[0];
      if (!LOG_BY_NAME.has(pathTail)) LOG_BY_NAME.set(pathTail, entry);
    }}

    function escapeHtml(value) {{
      return String(value)
        .replaceAll('&', '&amp;')
        .replaceAll('<', '&lt;')
        .replaceAll('>', '&gt;')
        .replaceAll('"', '&quot;')
        .replaceAll("'", '&#39;');
    }}

    function highlightLine(line, language) {{
      let out = escapeHtml(line);
      if (language === 'json' || language === 'jsonl') {{
        out = out.replace(/(&quot;[^&]*?&quot;)(\\s*:)/g, '<span class="tok-key">$1</span>$2');
        out = out.replace(/(:\\s*)(&quot;[^&]*?&quot;)/g, '$1<span class="tok-string">$2</span>');
        out = out.replace(/(:\\s*)(-?\\d+(?:\\.\\d+)?)/g, '$1<span class="tok-number">$2</span>');
        out = out.replace(/\\b(true|false|null)\\b/g, '<span class="tok-bool">$1</span>');
      }} else if (language === 'yaml' || language === 'toml') {{
        out = out.replace(/^([\\w.-]+)(\\s*[:=])/g, '<span class="tok-key">$1</span>$2');
        out = out.replace(/(#.*)$/g, '<span class="tok-comment">$1</span>');
      }} else if (language === 'py' || language === 'sh' || language === 'rs' || language === 'js' || language === 'ts') {{
        out = out.replace(/(#.*)$/g, '<span class="tok-comment">$1</span>');
        out = out.replace(/(\\/\\/.*)$/g, '<span class="tok-comment">$1</span>');
      }}
      out = out.replace(/\\b(ERROR|FAIL|FAILED|Traceback|Exception)\\b/g, '<span class="tok-error">$1</span>');
      out = out.replace(/\\b(WARN|WARNING)\\b/g, '<span class="tok-warn">$1</span>');
      out = out.replace(/\\b(PASS|SUCCESS|OK)\\b/g, '<span class="tok-pass">$1</span>');
      return out;
    }}

    function aceMode(language) {{
      const map = {{
        json: 'ace/mode/json',
        jsonl: 'ace/mode/json',
        yaml: 'ace/mode/yaml',
        yml: 'ace/mode/yaml',
        sh: 'ace/mode/sh',
        bash: 'ace/mode/sh',
        py: 'ace/mode/python',
        rs: 'ace/mode/rust',
        js: 'ace/mode/javascript',
        ts: 'ace/mode/typescript',
        md: 'ace/mode/markdown',
      }};
      return map[language] || 'ace/mode/text';
    }}

    function renderFallback(entry) {{
      const aceEl = document.getElementById('ace-view');
      const view = document.getElementById('log-view');
      if (aceEl) aceEl.style.display = 'none';
      view.style.display = 'block';
      const lines = entry.content.split(/\\r?\\n/);
      const html = lines.map((line, idx) => (
        '<div class="log-line"><span class="line-no">' + (idx + 1) + '</span><code class="line-text">' + highlightLine(line, entry.language) + '</code></div>'
      )).join('');
      view.innerHTML = html || '<div class="log-line"><span class="line-no">1</span><code class="line-text"></code></div>';
    }}

    function renderAce(entry) {{
      const aceEl = document.getElementById('ace-view');
      const view = document.getElementById('log-view');
      if (!aceReady || !aceEditor) {{
        renderFallback(entry);
        return;
      }}
      aceEl.style.display = 'block';
      view.style.display = 'none';
      aceEditor.session.setMode(aceMode(entry.language));
      aceEditor.session.setUseWorker(false);
      aceEditor.setValue(entry.content, -1);
      aceEditor.clearSelection();
      aceEditor.scrollToLine(1, true, true, function () {{}});
    }}

    function selectTreeNode(id) {{
      document.querySelectorAll('.tree-file').forEach((el) => {{
        el.classList.toggle('active', el.dataset.logId === id);
      }});
      if (window.jQuery) {{
        const tree = $('#log-tree-widget').jstree(true);
        if (tree && tree.get_node(id)) {{
          const selected = tree.get_selected();
          if (selected.length === 1 && selected[0] === id) {{
            tree.open_node(tree.get_parent(id));
            return;
          }}
          syncingTreeSelection = true;
          try {{
            tree.deselect_all(true);
            tree.select_node(id, true, true);
            tree.open_node(tree.get_parent(id));
          }} finally {{
            syncingTreeSelection = false;
          }}
        }}
      }}
    }}

    function renderLog(entry) {{
      currentLog = entry;
      pendingEntry = entry;
      document.getElementById('viewer-title').textContent = entry.path;
      const suffix = entry.truncated ? ' · truncated' : '';
      document.getElementById('viewer-meta').textContent = entry.language + ' · ' + entry.size + ' B' + suffix;
      selectTreeNode(entry.id);
      renderAce(entry);
    }}

    function openLog(id) {{
      const entry = LOG_BY_ID.get(id);
      if (entry) renderLog(entry);
    }}

    function openLogByName(name) {{
      const entry = LOG_BY_NAME.get(name);
      if (entry) {{
        renderLog(entry);
        document.getElementById('ace-view').scrollIntoView({{block: 'nearest'}});
      }} else {{
        document.getElementById('viewer-title').textContent = 'Log not embedded: ' + name;
        document.getElementById('viewer-meta').textContent = '';
        renderFallback({{content: 'No matching embedded log.', language: 'log', id: '', path: name, truncated: false, size: 0}});
      }}
    }}

    function initTree() {{
      if (window.jQuery && $.fn && $.fn.jstree) {{
        $('#fallback-log-tree').hide();
        $('#log-tree-widget')
          .jstree({{
            core: {{ data: LOG_TREE_NODES, multiple: false, themes: {{ stripes: false, dots: true, icons: true }} }},
            plugins: ['search', 'types', 'wholerow'],
            types: {{ folder: {{}}, file: {{ icon: 'jstree-file' }} }},
          }})
          .on('select_node.jstree', function (_event, data) {{
            if (syncingTreeSelection) return;
            const node = data.node;
            if (node && node.type === 'file') openLog(node.id);
          }});
        let searchTimer = null;
        $('#log-tree-search').on('input', function () {{
          clearTimeout(searchTimer);
          const value = this.value;
          searchTimer = setTimeout(function () {{ $('#log-tree-widget').jstree(true).search(value); }}, 180);
        }});
      }} else {{
        document.getElementById('log-tree-widget').style.display = 'none';
      }}
    }}

    function initAce() {{
      if (!window.ace) {{
        if (pendingEntry) renderFallback(pendingEntry);
        return;
      }}
      ace.config.set('basePath', '');
      ace.config.set('loadWorkerFromBlob', false);
      aceEditor = ace.edit('ace-view');
      aceEditor.setTheme('ace/theme/tomorrow_night_eighties');
      aceEditor.setOptions({{
        readOnly: true,
        highlightActiveLine: true,
        highlightGutterLine: true,
        showPrintMargin: false,
        fontFamily: 'JetBrains Mono, Menlo, Consolas, monospace',
        fontSize: '11px',
        wrap: false,
        useWorker: false,
      }});
      aceEditor.renderer.setScrollMargin(8, 8, 0, 0);
      aceReady = true;
      if (pendingEntry) renderAce(pendingEntry);
    }}

    document.getElementById('copy-log-path').addEventListener('click', function () {{
      if (!currentLog || !navigator.clipboard) return;
      navigator.clipboard.writeText(currentLog.path).catch(function () {{}});
    }});

    initTree();
    initAce();
    if (LOGS.length) openLog(PREFERRED_LOG_ID || LOGS[0].id);
  </script>
</body>
</html>
"""
    out.write_text(body)


def write_markdown(summary: dict, analysis: dict | None, out: Path, *,
                   map_preview: bool = False,
                   map_preview_url: str | None = None) -> None:
    # `map_preview` marks that slam-map.png sits next to this summary;
    # `map_preview_url` is where the published report site will serve it.
    out.parent.mkdir(parents=True, exist_ok=True)
    total = int(summary.get("total", 0) or 0)
    passed = int(summary.get("passed", 0) or 0)
    failed = int(summary.get("failed", 0) or 0)
    verdict = "NO DATA" if total == 0 else ("PASS" if failed == 0 else "FAIL")
    infrastructure_note = " ".join(
        str(summary.get("infrastructure_note", "") or "").split()
    ).replace("|", "\\|")
    lines = [
        "## Robonix Webots CI",
        "",
        f"**Result:** `{verdict}`",
        f"**Scenarios:** `{passed}/{total}`",
        f"**Failures:** `{failed}`",
    ]
    if infrastructure_note:
        lines.extend([f"**Infrastructure:** {infrastructure_note}"])
    lines.extend(
        [
            "",
            "| Status | Suite | Scenario | Rounds | Failures |",
            "| --- | --- | --- | ---: | --- |",
        ]
    )
    for sc in summary.get("scenarios", []):
        failures = "<br>".join(str(f).replace("|", "\\|") for f in sc.get("failures", [])) or "-"
        lines.append(
            f"| `{_status_label(bool(sc.get('passed')))}` | `{sc.get('family', '')}` | "
            f"`{sc.get('name', '')}` | {sc.get('rounds', '')} | {failures} |"
        )
    if map_preview:
        lines.extend(["", "### SLAM map", "",
                      f"![SLAM occupancy map from this run]({map_preview_url})" if map_preview_url
                      else "SLAM occupancy map: `slam-map.png` in the report artifact."])
    lines.extend(["", "HTML report with embedded log viewer: `testing/report/index.html` in the uploaded artifact."])
    out.write_text("\n".join(lines) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate Robonix Webots CI report")
    ap.add_argument("--summary-json", type=Path, default=Path("testing/logs/summary.json"))
    ap.add_argument("--out-dir", type=Path, default=Path("testing/report"))
    ap.add_argument("--log-root", action="append", default=[], help="label=directory to embed recursively")
    ap.add_argument("--log-file", action="append", default=[], help="label=path to embed as a single log")
    ap.add_argument("--metadata-json", action="append", type=Path, default=[], help="metadata JSON to merge")
    ap.add_argument("--metadata", action="append", default=[], help="key=value metadata to show in the report")
    ap.add_argument("--llm-analysis-json", type=Path, help="LLM-assisted diagnostic JSON to render")
    ap.add_argument("--map-preview", type=Path, help="SLAM occupancy PNG to ship as slam-map.png next to the report")
    ap.add_argument("--map-preview-url", default="", help="absolute URL where the published report site serves slam-map.png")
    ap.add_argument("--inline-style", action="append", type=Path, default=[], help="CSS file to embed directly into index.html")
    ap.add_argument("--inline-script", action="append", type=Path, default=[], help="JavaScript file to embed directly into index.html")
    ap.add_argument("--max-log-bytes", type=int, default=524288, help="per-log byte cap; 0 embeds complete files")
    ap.add_argument(
        "--max-total-log-bytes",
        type=int,
        default=12582912,
        help="total embedded log byte cap; 0 disables the total cap",
    )
    args = ap.parse_args()
    summary = _read_json(args.summary_json)
    logs = _collect_logs(
        args.log_root,
        args.log_file,
        args.summary_json,
        args.max_log_bytes,
        args.max_total_log_bytes,
    )
    metadata = _parse_metadata(args.metadata, args.metadata_json)
    analysis = _read_json(args.llm_analysis_json) if args.llm_analysis_json else {}
    write_metadata(metadata, args.out_dir / "metadata.json")
    inline_styles = list(args.inline_style) + _paths_from_env("ROBONIX_REPORT_INLINE_STYLES")
    inline_scripts = list(args.inline_script) + _paths_from_env("ROBONIX_REPORT_INLINE_SCRIPTS")
    # Copy the map before rendering: write_html needs to know whether
    # slam-map.png is there to decide if the section exists at all.
    map_preview = False
    if args.map_preview and args.map_preview.is_file():
        shutil.copyfile(args.map_preview, args.out_dir / "slam-map.png")
        map_preview = True
    elif args.map_preview:
        print(f"WARN map preview not found, skipping: {args.map_preview}", file=sys.stderr)
    write_html(summary, logs, metadata, analysis, args.out_dir / "index.html",
               inline_styles, inline_scripts, map_preview=map_preview)
    write_markdown(summary, analysis, args.out_dir / "summary.md",
                   map_preview=map_preview,
                   map_preview_url=args.map_preview_url or None)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
