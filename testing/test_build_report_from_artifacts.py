# SPDX-License-Identifier: MulanPSL-2.0
import tempfile
import unittest
from pathlib import Path

from build_report_from_artifacts import _missing_summary_reason
from report import write_html, write_markdown


class InfrastructureReportTests(unittest.TestCase):
    def test_missing_summary_uses_current_infrastructure_reason(self):
        with tempfile.TemporaryDirectory() as raw_dir:
            root = Path(raw_dir)
            log_dir = root / "_temp" / "sim-logs"
            log_dir.mkdir(parents=True)
            (log_dir / "infrastructure.txt").write_text(
                "Runner GPU capacity gate failed before checkout.\n",
                encoding="utf-8",
            )

            self.assertEqual(
                _missing_summary_reason(root),
                "Runner GPU capacity gate failed before checkout.",
            )

    def test_missing_summary_falls_back_without_infrastructure_log(self):
        with tempfile.TemporaryDirectory() as raw_dir:
            reason = _missing_summary_reason(Path(raw_dir))

        self.assertTrue(
            reason.startswith("No machine-readable scenario summary was produced")
        )

    def test_infrastructure_reason_is_visible_in_report_summary(self):
        summary = {
            "total": 0,
            "passed": 0,
            "failed": 0,
            "rate": 0,
            "scenarios": [],
            "coverage": [],
            "infrastructure_note": "GPU capacity gate failed before checkout.",
        }
        with tempfile.TemporaryDirectory() as raw_dir:
            root = Path(raw_dir)
            html_path = root / "index.html"
            markdown_path = root / "summary.md"

            write_html(summary, [], {}, {}, html_path)
            write_markdown(summary, {}, markdown_path)

            self.assertIn(
                "GPU capacity gate failed before checkout.",
                html_path.read_text(encoding="utf-8"),
            )
            self.assertIn(
                "GPU capacity gate failed before checkout.",
                markdown_path.read_text(encoding="utf-8"),
            )

    def test_markdown_embeds_map_preview_when_present(self):
        summary = {"total": 1, "passed": 1, "failed": 0, "scenarios": []}
        url = "https://ci-reports.example/reports/runs/1/slam-map.png"
        with tempfile.TemporaryDirectory() as raw_dir:
            markdown_path = Path(raw_dir) / "summary.md"

            write_markdown(summary, {}, markdown_path,
                           map_preview=True, map_preview_url=url)
            text = markdown_path.read_text(encoding="utf-8")
            self.assertIn("### SLAM map", text)
            self.assertIn(f"]({url})", text)

            write_markdown(summary, {}, markdown_path, map_preview=True)
            self.assertIn("`slam-map.png` in the report artifact",
                          markdown_path.read_text(encoding="utf-8"))

            write_markdown(summary, {}, markdown_path)
            self.assertNotIn("SLAM map",
                             markdown_path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
