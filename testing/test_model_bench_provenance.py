# SPDX-License-Identifier: MulanPSL-2.0
"""Regression tests for benchmark source and runtime identity."""

import hashlib
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from testing.model_bench import provenance


class ProvenanceTest(unittest.TestCase):
    def test_versions_identify_unsupported_runtime_without_fabricating_version(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "robonix-atlas"
            binary.write_text("#!/bin/sh\nprintf 'unsupported flag' >&2\nexit 2\n")
            binary.chmod(0o755)
            self.assertEqual(provenance._binary(str(binary)), {
                "version": None,
                "sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
            })
            binary.write_text("#!/bin/sh\nprintf 'robonix-atlas 1.2.3\\n'\n")
            self.assertEqual(provenance._binary(str(binary))["version"], "robonix-atlas 1.2.3")

    def test_probe_failures_are_explicitly_unknown_and_bounded(self):
        with patch.object(provenance.subprocess, "run", side_effect=subprocess.TimeoutExpired("version", 5)) as run:
            self.assertIsNone(provenance._run(["missing", "--version"]))
            self.assertEqual(run.call_args.kwargs["timeout"], 5)
        with patch.object(provenance.subprocess, "run", side_effect=FileNotFoundError):
            self.assertIsNone(provenance._run(["missing", "--version"]))

    def test_serving_facts_are_required_for_local_and_robot(self):
        facts = {"framework": "vllm", "version": "1.0", "hardware": "GPU",
                 "precision": "bf16", "max_context": 8192, "sampling": {"temperature": 0}}
        provenance.validate_serving("cloud", None)
        for kind in ("local", "robot"):
            provenance.validate_serving(kind, facts)
            with self.assertRaisesRegex(ValueError, "serving facts"):
                provenance.validate_serving(kind, None)
            with self.assertRaisesRegex(ValueError, "hardware"):
                provenance.validate_serving(kind, {**facts, "hardware": " "})
            with self.assertRaisesRegex(ValueError, "max_context"):
                provenance.validate_serving(kind, {**facts, "max_context": -1})

    def test_collect_records_full_source_identity_and_external_cases(self):
        repo = Path(__file__).resolve().parents[1]
        source_sha = "b" * 40
        facts = {"framework": "vllm", "version": "1.0", "hardware": "GPU",
                 "precision": "bf16", "max_context": 8192,
                 "sampling": {"temperature": 0, "stop": ["done"], "mode": "fake"}}
        supplied = {**facts, "environment": {"ROBONIX_TEST_SECRET": "hidden"},
                    "extra": "hidden", "ROBONIX_TEST_SECRET": "hidden",
                    "sampling": {**facts["sampling"], "extra": "hidden",
                                 "environment": {"ROBONIX_TEST_SECRET": "hidden"},
                                 "seed": {"ROBONIX_TEST_SECRET": "hidden"}}}
        with tempfile.TemporaryDirectory() as directory, patch.object(provenance.shutil, "which", return_value=None), \
                patch.object(provenance, "_git", side_effect=lambda repo, *args:
                             " M file" if args[0] == "status" else source_sha):
            cases = Path(directory) / "cases.yaml"
            cases.write_text("cases: []\n")
            result = provenance.collect(repo, cases, "model", "cloud", "board", None,
                                        "missing", 123.5, ["bench.py", "--model", "model"], supplied)
            self.assertEqual(result["planner"]["serving"], facts)
            self.assertEqual(set(result["system"]), {"os", "kernel", "python"})
            self.assertEqual(result["robonix"]["commit"], source_sha)
            self.assertIsInstance(result["robonix"]["dirty"], bool)
            self.assertRegex(result["bench"]["commit"], r"^[0-9a-f]{40}$")
            self.assertIsNone(result["bench"]["cases_commit"])
            self.assertIsNone(result["bench"]["cases_dirty"])
            self.assertEqual(result["board"]["cases_sha256"], hashlib.sha256(cases.read_bytes()).hexdigest())
            self.assertEqual(result["versions"]["runtime"]["pilot"], {"version": None, "sha256": None})
            self.assertEqual(result["start"], 123.5)
            self.assertIsNone(result["end"])

    def test_collect_tracks_cases_history_and_prefers_rbnx_siblings(self):
        repo = Path(__file__).resolve().parents[1]
        cases = repo / "testing/model_bench/cases.yaml"
        with tempfile.TemporaryDirectory() as directory:
            rbnx = Path(directory) / "rbnx"
            rbnx.write_text("#!/bin/sh\nprintf 'rbnx 1.2.3\\n'\n")
            rbnx.chmod(0o755)
            pilot = Path(directory) / "robonix-pilot"
            pilot.write_text("#!/bin/sh\nexit 2\n")
            pilot.chmod(0o755)
            with patch.object(provenance.shutil, "which", side_effect=lambda name: str(rbnx) if name == "rbnx" else None), \
                    patch.object(provenance, "_git", side_effect=lambda repo, *args:
                                 " M file" if args[0] == "status" else "b" * 40):
                result = provenance.collect(repo, cases, "model", "cloud", "board", "judge",
                                            "rbnx", 1, [])
            self.assertRegex(result["bench"]["cases_commit"], r"^[0-9a-f]{40}$")
            self.assertIsInstance(result["bench"]["cases_dirty"], bool)
            self.assertEqual(result["versions"]["rbnx"]["version"], "rbnx 1.2.3")
            self.assertEqual(result["versions"]["runtime"]["pilot"], {
                "version": None, "sha256": hashlib.sha256(pilot.read_bytes()).hexdigest()})


if __name__ == "__main__":
    unittest.main()
