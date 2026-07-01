# SPDX-License-Identifier: MulanPSL-2.0
"""Generate an offline Webots CI report with an embedded log viewer."""

from __future__ import annotations

import argparse
import html
import json
import os
import re
from datetime import datetime, timezone
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
    keys = [k for k in ordered if k in metadata and not k.startswith("env_")]
    keys.extend(k for k in metadata if k not in set(keys) and not k.endswith("_url") and not k.startswith("env_"))
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
    keys.extend(k for k in metadata if k.startswith("env_") and k not in set(keys))
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


def _log_tree(logs: list[dict]) -> str:
    if not logs:
        return '<p class="muted">No logs were embedded.</p>'

    groups: dict[str, list[dict]] = {}
    for entry in logs:
        root = str(entry["path"]).split("/", 1)[0]
        groups.setdefault(root, []).append(entry)

    parts = []
    for root, entries in groups.items():
        parts.append(f'<details class="tree-group" open><summary>{html.escape(root)}</summary>')
        for entry in entries:
            rel = str(entry["path"]).split("/", 1)[1] if "/" in str(entry["path"]) else str(entry["path"])
            parts.append(
                f'<button class="tree-file" data-log-id="{entry["id"]}" onclick="openLog(this.dataset.logId)">'
                f'<span class="tree-name">{html.escape(rel)}</span>'
                f'<span class="tree-meta">{html.escape(entry["language"])} · {entry["size"]} B</span>'
                "</button>"
            )
        parts.append("</details>")
    return "\n".join(parts)


def _json_for_script(data: object) -> str:
    return json.dumps(data, ensure_ascii=False).replace("</", "<\\/")


def write_html(summary: dict, logs: list[dict], metadata: dict[str, str], out: Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    total = int(summary.get("total", 0) or 0)
    passed = int(summary.get("passed", 0) or 0)
    failed = int(summary.get("failed", 0) or 0)
    rate = summary.get("rate", 0)
    verdict = "NO DATA" if total == 0 else ("PASS" if failed == 0 else "FAIL")
    generated_on = html.escape(metadata.get("generated_on", ""))
    body = f"""<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\">
  <title>Robonix Webots CI Report</title>
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
      --sans: "Noto Sans", ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      --mono: "JetBrains Mono", ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      color: var(--text);
      font-family: var(--sans);
      margin: 32px;
    }}
    h1 {{ margin: 0 0 6px; }}
    .subtitle {{ color: var(--muted); font-size: 13px; margin: 0 0 16px; }}
    h2 {{ margin: 28px 0 12px; }}
    table {{
      border-collapse: separate;
      border-spacing: 0;
      width: 100%;
      margin-top: 16px;
      border: 1px solid var(--line);
      border-radius: 6px;
      overflow: hidden;
    }}
    th, td {{
      border-bottom: 1px solid var(--line);
      padding: 10px 12px;
      text-align: left;
      vertical-align: top;
    }}
    th {{ background: var(--head); color: #374151; font-size: 11px; text-transform: uppercase; }}
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
    .summary {{ display: flex; flex-wrap: wrap; gap: 12px; margin: 16px 0; }}
    .card {{ border: 1px solid var(--line); padding: 12px 16px; border-radius: 6px; min-width: 140px; }}
    .card strong {{ color: var(--muted); display: block; font-size: 11px; margin-bottom: 6px; text-transform: uppercase; }}
    .metric {{ font-size: 18px; font-weight: 650; }}
    .row-pass {{ background: var(--pass-bg); }}
    .row-fail {{ background: var(--fail-bg); }}
    .row-pass td {{ border-bottom-color: var(--pass-border); }}
    .row-fail td {{ border-bottom-color: var(--fail-border); }}
    .status-badge {{
      border-radius: 999px;
      display: inline-block;
      font-family: var(--mono);
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0;
      min-width: 42px;
      padding: 3px 8px;
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
    .log-panel {{
      border: 1px solid var(--line);
      border-radius: 6px;
      display: grid;
      grid-template-columns: minmax(240px, 28%) 1fr;
      min-height: 520px;
      overflow: hidden;
    }}
    .log-tree {{
      background: #f9fafb;
      border-right: 1px solid var(--line);
      max-height: 760px;
      overflow: auto;
      padding: 12px;
    }}
    .tree-group {{ margin-bottom: 10px; }}
    .tree-group summary {{
      cursor: pointer;
      font-weight: 700;
      margin-bottom: 6px;
    }}
    .tree-file {{
      appearance: none;
      background: transparent;
      border: 0;
      border-radius: 4px;
      color: var(--text);
      cursor: pointer;
      display: block;
      padding: 6px 8px;
      text-align: left;
      width: 100%;
    }}
    .tree-file:hover, .tree-file.active {{ background: #e5e7eb; }}
    .tree-name {{
      display: block;
      font-family: var(--mono);
      font-size: 11px;
      overflow-wrap: anywhere;
    }}
    .tree-meta {{ color: var(--muted); display: block; font-size: 11px; margin-top: 2px; }}
    .viewer {{ min-width: 0; }}
    .viewer-head {{
      align-items: center;
      border-bottom: 1px solid var(--line);
      display: flex;
      gap: 10px;
      justify-content: space-between;
      padding: 10px 12px;
    }}
    .viewer-title {{
      font-family: var(--mono);
      font-size: 11px;
      overflow-wrap: anywhere;
    }}
    .viewer-meta {{ color: var(--muted); font-size: 11px; white-space: nowrap; }}
    .log-view {{
      background: #0b1020;
      color: #e5e7eb;
      font-family: var(--mono);
      font-size: 11px;
      line-height: 1.45;
      max-height: 700px;
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
  <p class=\"subtitle\">Generated on {generated_on}</p>
  <div class=\"summary\">
    <div class=\"card\"><strong>Result</strong><span class=\"metric {'pass' if verdict == 'PASS' else 'fail'}\">{verdict}</span></div>
    <div class=\"card\"><strong>Scenarios</strong><span class=\"metric\">{passed}/{total} passed</span></div>
    <div class=\"card\"><strong>Failures</strong><span class=\"metric\">{failed}</span></div>
    <div class=\"card\"><strong>Pass rate</strong><span class=\"metric\">{html.escape(str(rate))}</span></div>
  </div>
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
  <h2>Logs</h2>
  <div class=\"log-panel\">
    <aside class=\"log-tree\">{_log_tree(logs)}</aside>
    <section class=\"viewer\">
      <div class=\"viewer-head\">
        <div id=\"viewer-title\" class=\"viewer-title\">Select a log</div>
        <div id=\"viewer-meta\" class=\"viewer-meta\"></div>
      </div>
      <div id=\"log-view\" class=\"log-view\"><div class=\"log-line\"><span class=\"line-no\">-</span><code class=\"line-text\">No log selected.</code></div></div>
    </section>
  </div>
  <h2>Runtime Contract Coverage</h2>
  {_coverage(summary)}
  <script>
    const LOGS = {_json_for_script(logs)};
    const LOG_BY_ID = new Map(LOGS.map((entry) => [entry.id, entry]));
    const LOG_BY_NAME = new Map();
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
        out = out.replace(/(&quot;[^&]*?&quot;)(\\s*:)/g, '<span class=\"tok-key\">$1</span>$2');
        out = out.replace(/(:\\s*)(&quot;[^&]*?&quot;)/g, '$1<span class=\"tok-string\">$2</span>');
        out = out.replace(/(:\\s*)(-?\\d+(?:\\.\\d+)?)/g, '$1<span class=\"tok-number\">$2</span>');
        out = out.replace(/\\b(true|false|null)\\b/g, '<span class=\"tok-bool\">$1</span>');
      }} else if (language === 'yaml' || language === 'toml') {{
        out = out.replace(/^([\\w.-]+)(\\s*[:=])/g, '<span class=\"tok-key\">$1</span>$2');
        out = out.replace(/(#.*)$/g, '<span class=\"tok-comment\">$1</span>');
      }} else if (language === 'py' || language === 'sh' || language === 'rs' || language === 'js' || language === 'ts') {{
        out = out.replace(/(#.*)$/g, '<span class=\"tok-comment\">$1</span>');
        out = out.replace(/(\\/\\/.*)$/g, '<span class=\"tok-comment\">$1</span>');
      }}
      out = out.replace(/\\b(ERROR|FAIL|FAILED|Traceback|Exception)\\b/g, '<span class=\"tok-error\">$1</span>');
      out = out.replace(/\\b(WARN|WARNING)\\b/g, '<span class=\"tok-warn\">$1</span>');
      out = out.replace(/\\b(PASS|SUCCESS|OK)\\b/g, '<span class=\"tok-pass\">$1</span>');
      return out;
    }}

    function renderLog(entry) {{
      document.getElementById('viewer-title').textContent = entry.path;
      const suffix = entry.truncated ? ' · truncated' : '';
      document.getElementById('viewer-meta').textContent = `${{entry.language}} · ${{entry.size}} B${{suffix}}`;
      document.querySelectorAll('.tree-file').forEach((el) => {{
        el.classList.toggle('active', el.dataset.logId === entry.id);
      }});
      const lines = entry.content.split(/\\r?\\n/);
      const html = lines.map((line, idx) => (
        `<div class=\"log-line\"><span class=\"line-no\">${{idx + 1}}</span><code class=\"line-text\">${{highlightLine(line, entry.language)}}</code></div>`
      )).join('');
      document.getElementById('log-view').innerHTML = html || '<div class=\"log-line\"><span class=\"line-no\">1</span><code class=\"line-text\"></code></div>';
    }}

    function openLog(id) {{
      const entry = LOG_BY_ID.get(id);
      if (entry) renderLog(entry);
    }}

    function openLogByName(name) {{
      const entry = LOG_BY_NAME.get(name);
      if (entry) {{
        renderLog(entry);
        document.getElementById('log-view').scrollIntoView({{block: 'nearest'}});
      }} else {{
        document.getElementById('viewer-title').textContent = `Log not embedded: ${{name}}`;
        document.getElementById('viewer-meta').textContent = '';
        document.getElementById('log-view').innerHTML = '<div class=\"log-line\"><span class=\"line-no\">-</span><code class=\"line-text\">No matching embedded log.</code></div>';
      }}
    }}

    if (LOGS.length) renderLog(LOGS[0]);
  </script>
</body>
</html>
"""
    out.write_text(body)


def write_markdown(summary: dict, out: Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    total = int(summary.get("total", 0) or 0)
    passed = int(summary.get("passed", 0) or 0)
    failed = int(summary.get("failed", 0) or 0)
    verdict = "NO DATA" if total == 0 else ("PASS" if failed == 0 else "FAIL")
    lines = [
        "## Robonix Webots CI",
        "",
        f"**Result:** `{verdict}`",
        f"**Scenarios:** `{passed}/{total}`",
        f"**Failures:** `{failed}`",
        "",
        "| Status | Suite | Scenario | Rounds | Failures |",
        "| --- | --- | --- | ---: | --- |",
    ]
    for sc in summary.get("scenarios", []):
        failures = "<br>".join(str(f).replace("|", "\\|") for f in sc.get("failures", [])) or "-"
        lines.append(
            f"| `{_status_label(bool(sc.get('passed')))}` | `{sc.get('family', '')}` | "
            f"`{sc.get('name', '')}` | {sc.get('rounds', '')} | {failures} |"
        )
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
    write_metadata(metadata, args.out_dir / "metadata.json")
    write_html(summary, logs, metadata, args.out_dir / "index.html")
    write_markdown(summary, args.out_dir / "summary.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
