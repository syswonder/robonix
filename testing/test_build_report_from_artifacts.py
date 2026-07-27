# SPDX-License-Identifier: MulanPSL-2.0
from pathlib import Path

from build_report_from_artifacts import _missing_summary_reason
from report import write_html, write_markdown


def test_missing_summary_uses_current_infrastructure_reason(tmp_path: Path):
    log_dir = tmp_path / "_temp" / "sim-logs"
    log_dir.mkdir(parents=True)
    (log_dir / "infrastructure.txt").write_text(
        "Runner GPU capacity gate failed before checkout.\n",
        encoding="utf-8",
    )

    assert (
        _missing_summary_reason(tmp_path)
        == "Runner GPU capacity gate failed before checkout."
    )


def test_missing_summary_falls_back_without_infrastructure_log(tmp_path: Path):
    reason = _missing_summary_reason(tmp_path)

    assert reason.startswith("No machine-readable scenario summary was produced")


def test_infrastructure_reason_is_visible_in_report_summary(tmp_path: Path):
    summary = {
        "total": 0,
        "passed": 0,
        "failed": 0,
        "rate": 0,
        "scenarios": [],
        "coverage": [],
        "infrastructure_note": "GPU capacity gate failed before checkout.",
    }
    html_path = tmp_path / "index.html"
    markdown_path = tmp_path / "summary.md"

    write_html(summary, [], {}, {}, html_path)
    write_markdown(summary, {}, markdown_path)

    assert "GPU capacity gate failed before checkout." in html_path.read_text()
    assert "GPU capacity gate failed before checkout." in markdown_path.read_text()
