# SPDX-License-Identifier: MulanPSL-2.0
"""Build the canonical Webots CI HTML report from uploaded artifacts."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


def _find(root: Path, predicate) -> Path | None:
    if not root.exists():
        return None
    for path in sorted(root.rglob("*")):
        if predicate(path):
            return path
    return None


def _find_dir(root: Path, suffix: str) -> Path | None:
    return _find(root, lambda p: p.is_dir() and p.as_posix().endswith(suffix))


def _find_file(root: Path, name: str) -> Path | None:
    return _find(root, lambda p: p.is_file() and p.name == name)


def _run(args: list[str]) -> None:
    subprocess.run(args, check=True)


def _empty_summary(path: Path, reason: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                "total": 0,
                "passed": 0,
                "failed": 0,
                "rate": 0,
                "scenarios": [],
                "coverage": [],
                "infrastructure_note": reason,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return path


def _missing_summary_reason(artifact_root: Path) -> str:
    infrastructure = _find_file(artifact_root, "infrastructure.txt")
    if infrastructure is not None:
        try:
            reason = infrastructure.read_text(
                encoding="utf-8",
                errors="replace",
            ).strip()
        except OSError:
            reason = ""
        if reason:
            return reason[:2000]
    return (
        "No machine-readable scenario summary was produced; "
        "logs are still embedded below."
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="Build Robonix Webots report from artifacts")
    ap.add_argument("--artifact-root", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--metadata", action="append", default=[])
    ap.add_argument("--metadata-json", action="append", type=Path, default=[])
    ap.add_argument("--repo", type=Path, default=Path("."))
    ap.add_argument("--max-log-bytes", type=int, default=524288)
    ap.add_argument("--max-total-log-bytes", type=int, default=12582912)
    ap.add_argument("--llm", action="store_true", help="attempt LLM analysis when DEEPSEEK_API_KEY is present")
    ap.add_argument("--map-preview-url", default="", help="absolute URL where the published report site serves slam-map.png")
    args = ap.parse_args()

    here = Path(__file__).resolve().parent
    artifact_root = args.artifact_root
    artifact_root.mkdir(parents=True, exist_ok=True)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    summary_json = _find(artifact_root, lambda p: p.is_file() and p.as_posix().endswith("testing/logs/summary.json"))
    if summary_json is None:
        summary_json = _empty_summary(
            args.out_dir / "synthetic-summary.json",
            _missing_summary_reason(artifact_root),
        )

    metadata_jsons = list(args.metadata_json)
    report_metadata = _find_file(artifact_root, "report-metadata.json")
    if report_metadata:
        metadata_jsons.append(report_metadata)

    log_roots: list[str] = []
    for label, suffix in [
        ("testing/logs", "testing/logs"),
        ("boot_logs", "examples/webots/rbnx-boot/logs"),
        ("provider_logs", "examples/webots/logs"),
        ("scene_logs", "system/scene/logs"),
        ("cache_provider_logs", "cache-provider-logs"),
        ("sim_logs", "sim-logs"),
    ]:
        found = _find_dir(artifact_root, suffix)
        if found:
            log_roots.extend(["--log-root", f"{label}={found}"])

    log_files: list[str] = []
    for name in ["fake_vlm.log", "rbnx-boot.log", "remote-provenance.txt"]:
        found = _find_file(artifact_root, name)
        if found:
            log_files.extend(["--log-file", f"{name}={found}"])

    analysis_json = args.out_dir / "llm-analysis.json"
    context_json = args.out_dir / "diagnostic-context.json"
    collect_cmd = [
        sys.executable,
        str(here / "collect_diagnostic_context.py"),
        "--artifact-root",
        str(artifact_root),
        "--summary-json",
        str(summary_json),
        "--repo",
        str(args.repo),
        "--out",
        str(context_json),
    ]
    for meta in args.metadata:
        collect_cmd.extend(["--metadata", meta])
    for meta_json in metadata_jsons:
        collect_cmd.extend(["--metadata-json", str(meta_json)])
    _run(collect_cmd)

    if args.llm:
        _run([sys.executable, str(here / "llm_diagnose.py"), "--context-json", str(context_json), "--out", str(analysis_json)])
    else:
        analysis_json.write_text(
            json.dumps({"available": False, "title": "LLM analysis disabled", "summary": "LLM analysis was not requested for this report."}, indent=2)
            + "\n",
            encoding="utf-8",
        )

    report_cmd = [
        sys.executable,
        str(here / "report.py"),
        "--summary-json",
        str(summary_json),
        "--out-dir",
        str(args.out_dir),
        "--llm-analysis-json",
        str(analysis_json),
        "--max-log-bytes",
        str(args.max_log_bytes),
        "--max-total-log-bytes",
        str(args.max_total_log_bytes),
    ]
    for meta in args.metadata:
        report_cmd.extend(["--metadata", meta])
    for meta_json in metadata_jsons:
        report_cmd.extend(["--metadata-json", str(meta_json)])
    report_cmd.extend(log_roots)
    report_cmd.extend(log_files)
    map_preview = _find_file(artifact_root, "slam-map.png")
    if map_preview:
        report_cmd.extend(["--map-preview", str(map_preview)])
        if args.map_preview_url:
            report_cmd.extend(["--map-preview-url", args.map_preview_url])
    _run(report_cmd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
