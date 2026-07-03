#!/usr/bin/env python3
"""Build the static GitHub Pages index for Robonix Webots CI reports."""

from __future__ import annotations

import argparse
import html
import json
import shutil
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any


def _parse_time(raw: Any) -> datetime | None:
    if not raw:
        return None
    text = str(raw).replace("Z", "+00:00")
    try:
        dt = datetime.fromisoformat(text)
    except ValueError:
        return None
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt.astimezone(timezone.utc)


def _read_json(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def collect_entries(site: Path, retention_days: int) -> list[dict[str, str]]:
    runs = site / "reports" / "runs"
    cutoff = datetime.now(timezone.utc) - timedelta(days=retention_days)
    entries: list[dict[str, str]] = []
    if not runs.exists():
        return entries

    for run_dir in sorted(p for p in runs.iterdir() if p.is_dir()):
        meta = _read_json(run_dir / "metadata.json")
        dt = _parse_time(meta.get("generated_on")) or _parse_time(meta.get("test_report_generated_on"))
        if dt and dt < cutoff:
            shutil.rmtree(run_dir)
            continue
        title = meta.get("pr_title") or meta.get("tested_commit_subject") or run_dir.name
        entries.append(
            {
                "run_id": run_dir.name,
                "generated_on": dt.isoformat() if dt else "",
                "title": str(title),
                "event": str(meta.get("event", "")),
                "workflow": str(meta.get("workflow", "")),
                "actor": str(meta.get("triggered_by") or meta.get("actor") or ""),
                "commit": str(meta.get("tested_commit", "")),
                "pr_number": str(meta.get("pr_number", "")),
                "url": f"runs/{run_dir.name}/",
            }
        )
    entries.sort(key=lambda x: x.get("generated_on") or x["run_id"], reverse=True)
    return entries


def write_index(site: Path, entries: list[dict[str, str]], retention_days: int) -> None:
    reports = site / "reports"
    reports.mkdir(parents=True, exist_ok=True)
    (reports / "index.json").write_text(json.dumps(entries, indent=2) + "\n", encoding="utf-8")

    rows = []
    for entry in entries:
        run_id = html.escape(entry["run_id"])
        title = html.escape(entry["title"])
        event = html.escape(entry.get("event") or "-")
        actor = html.escape(entry.get("actor") or "-")
        generated = html.escape(entry.get("generated_on") or "-")
        commit = html.escape(entry.get("commit")[:12] or "-")
        pr_number = entry.get("pr_number") or ""
        pr = f"#{html.escape(pr_number)}" if pr_number and pr_number != "-" else "-"
        url = html.escape(entry["url"], quote=True)
        rows.append(
            "<tr>"
            f'<td class="mono"><a href="{url}">{run_id}</a></td>'
            f'<td>{title}</td>'
            f'<td class="mono">{generated}</td>'
            f'<td class="mono">{event}</td>'
            f'<td class="mono">{pr}</td>'
            f'<td class="mono">{commit}</td>'
            f'<td class="mono">{actor}</td>'
            "</tr>"
        )
    rows_html = "\n".join(rows) or '<tr><td colspan="7">No reports published yet.</td></tr>'
    generated_on = datetime.now(timezone.utc).isoformat(timespec="seconds")
    reports.joinpath("index.html").write_text(
        f"""<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robonix Webots CI Reports</title>
<style>
  :root {{
    --text: #111827;
    --muted: #4b5563;
    --line: #d1d5db;
    --head: #f3f4f6;
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
  .subtitle {{ color: var(--muted); font-size: 13px; margin: 0 0 14px; }}
  .summary {{ display: flex; flex-wrap: wrap; gap: 8px; margin: 14px 0; }}
  .card {{ border: 1px solid var(--line); border-radius: 0; min-width: 140px; padding: 8px 10px; }}
  .card strong {{ color: var(--muted); display: block; font-size: 11px; margin-bottom: 6px; text-transform: uppercase; }}
  .metric {{ font-size: 16px; font-weight: 700; }}
  .table-scroll {{ overflow-x: auto; width: 100%; }}
  table {{ border-collapse: collapse; border-spacing: 0; border: 1px solid var(--line); margin-top: 12px; min-width: 980px; width: 100%; }}
  th, td {{ border-bottom: 1px solid var(--line); padding: 6px 8px; text-align: left; vertical-align: top; }}
  th {{ background: var(--head); color: #374151; font-size: 11px; font-weight: 700; text-transform: uppercase; white-space: nowrap; }}
  tr:last-child td {{ border-bottom: 0; }}
  a {{ color: #1d4ed8; text-decoration: none; }}
  a:hover {{ text-decoration: underline; }}
  .mono {{ font-family: var(--mono); font-size: 11px; white-space: nowrap; }}
</style>
<h1>Robonix Webots CI Reports</h1>
<p class="subtitle">Reports are retained for {retention_days} days. Generated on <span class="mono">{html.escape(generated_on)}</span>.</p>
<div class="summary">
  <div class="card"><strong>Total Reports</strong><div class="metric">{len(entries)}</div></div>
  <div class="card"><strong>Retention</strong><div class="metric">{retention_days} days</div></div>
</div>
<div class="table-scroll">
<table>
  <thead><tr><th>Run</th><th>Title</th><th>Generated</th><th>Event</th><th>PR</th><th>Commit</th><th>Actor</th></tr></thead>
  <tbody>
{rows_html}
  </tbody>
</table>
</div>
""",
        encoding="utf-8",
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="Build Robonix Webots report index")
    ap.add_argument("--site-dir", type=Path, required=True)
    ap.add_argument("--retention-days", type=int, default=90)
    args = ap.parse_args()
    entries = collect_entries(args.site_dir, args.retention_days)
    write_index(args.site_dir, entries, args.retention_days)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
