# SPDX-License-Identifier: MulanPSL-2.0
"""Regression policy for venv-backed Python package code generation."""

from __future__ import annotations

from pathlib import Path
import re
import unittest


REPO_ROOT = Path(__file__).resolve().parents[2]
VENV_BACKED_BUILDS = (
    "services/memsearch/scripts/build.sh",
    "services/voiceprint/scripts/build.sh",
    "services/speech/scripts/build.sh",
    "services/memory/scripts/build.sh",
    "examples/webots/primitives/audio_driver/scripts/build.sh",
    "examples/webots/primitives/audio_client_bridge/scripts/build.sh",
    "examples/remote_liaison_demo/skills/status_skill/scripts/build.sh",
    "examples/remote_liaison_demo/skills/summary_skill/scripts/build.sh",
    "examples/remote_liaison_demo/skills/notes_skill/scripts/build.sh",
)
# Shared runners that own the venv their `rbnx codegen` call runs in.
CODEGEN_RUNNERS = (
    "examples/webots/scripts/run_python_codegen.sh",
    "system/scene/scripts/run_python_codegen.sh",
)
DIRECT_CODEGEN = re.compile(
    r"^[ \t]*(?:[A-Za-z_][A-Za-z0-9_]*=[^ \t\n]+[ \t]+)*"
    r"rbnx[ \t]+codegen\b",
    re.MULTILINE,
)
# Scene used to be exempt here: its build script called `rbnx codegen`
# directly and staged IDL/ROS artifacts on the host. It now goes through
# scripts/run_python_codegen.sh like every other Python package, so the
# policy applies to it unmodified and no exemption is left.
EXEMPT_DIRECT_CODEGEN: dict[str, str] = {}


def direct_codegen_calls(path: Path) -> list[re.Match[str]]:
    return list(DIRECT_CODEGEN.finditer(path.read_text(encoding="utf-8")))


class PythonCodegenPolicyTests(unittest.TestCase):
    def test_package_venv_is_ready_before_codegen(self):
        for relative in VENV_BACKED_BUILDS:
            with self.subTest(script=relative):
                text = (REPO_ROOT / relative).read_text(encoding="utf-8")
                calls = direct_codegen_calls(REPO_ROOT / relative)
                self.assertEqual(len(calls), 1)
                codegen = calls[0].start()
                self.assertLess(text.index("uv "), codegen)
                self.assertTrue(
                    "uv sync" in text[:codegen] or "uv pip install" in text[:codegen],
                    f"{relative} must synchronize runtime dependencies before codegen",
                )

    def test_codegen_selects_the_runtime_python_both_ways(self):
        scripts = VENV_BACKED_BUILDS + CODEGEN_RUNNERS
        for relative in scripts:
            with self.subTest(script=relative):
                text = (REPO_ROOT / relative).read_text(encoding="utf-8")
                calls = direct_codegen_calls(REPO_ROOT / relative)
                self.assertEqual(len(calls), 1)
                command = text[max(0, calls[0].start() - 180):calls[0].end()]
                self.assertIn("RBNX_CODEGEN_PYTHON=", command)
                self.assertIn("PATH=", command)

    def test_packages_smoke_test_generated_imports_after_codegen(self):
        for relative in VENV_BACKED_BUILDS:
            with self.subTest(script=relative):
                text = (REPO_ROOT / relative).read_text(encoding="utf-8")
                calls = direct_codegen_calls(REPO_ROOT / relative)
                self.assertEqual(len(calls), 1)
                codegen = calls[0].end()
                smoke = text[codegen:]
                self.assertIn("PYTHONPATH=", smoke)
                self.assertIn('"$VENV/bin/python"', smoke)
                self.assertIn("import ", smoke)

    def test_every_direct_python_codegen_call_is_managed_or_exempt(self):
        discovered: dict[str, list[re.Match[str]]] = {}
        for script in REPO_ROOT.rglob("*.sh"):
            calls = direct_codegen_calls(script)
            if calls:
                discovered[str(script.relative_to(REPO_ROOT))] = calls

        managed = set(VENV_BACKED_BUILDS) | set(CODEGEN_RUNNERS)
        self.assertEqual(set(discovered), managed | set(EXEMPT_DIRECT_CODEGEN))

        for relative, marker in EXEMPT_DIRECT_CODEGEN.items():
            with self.subTest(exemption=relative):
                text = (REPO_ROOT / relative).read_text(encoding="utf-8")
                self.assertIn(marker, text)
                self.assertEqual(len(discovered[relative]), 1)

    def test_audio_packages_pin_the_codegen_runtime_tuple(self):
        for relative in (
            "examples/webots/primitives/audio_driver/scripts/build.sh",
            "examples/webots/primitives/audio_client_bridge/scripts/build.sh",
        ):
            with self.subTest(script=relative):
                text = (REPO_ROOT / relative).read_text(encoding="utf-8")
                self.assertNotIn("--system-site-packages", text)
                self.assertIn('"grpcio==1.80.0"', text)
                self.assertIn('"grpcio-tools==1.76.0"', text)
                self.assertIn('"protobuf==6.33.6"', text)


if __name__ == "__main__":
    unittest.main()
