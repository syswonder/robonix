# SPDX-License-Identifier: MulanPSL-2.0
"""Regression tests for deployment-scoped interface proto discovery."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from testing import run_interfaces


def _complete_proto_tree(path: Path) -> Path:
    path.mkdir(parents=True)
    for name in run_interfaces.GENERATED_PROTO_FILES:
        (path / name).write_text(f"# {name}\n")
    return path.resolve()


class GeneratedProtoDirTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory()
        self.workspace = Path(self.tempdir.name)
        self.deployment = self.workspace / "deployments/selected"
        self.deployment.mkdir(parents=True)

    def tearDown(self) -> None:
        self.tempdir.cleanup()

    def _write_manifest(self, source: str) -> None:
        (self.deployment / "robonix_manifest.yaml").write_text(
            "primitive:\n"
            "  - name: camera\n"
            "    path: ./camera\n"
            "  - name: audio_driver\n"
            f"    {source}\n"
        )

    def test_manifest_url_selects_that_deployments_remote_cache(self) -> None:
        self._write_manifest("url: https://github.com/example/custom-audio.git")
        expected = _complete_proto_tree(
            self.deployment
            / "rbnx-boot/cache/custom-audio/rbnx-build/codegen/proto_gen"
        )

        actual = run_interfaces.generated_proto_dir(
            self.deployment, workspace=self.workspace
        )

        self.assertEqual(actual, expected)

    def test_ci_defaults_to_webots_deployment_cache(self) -> None:
        deployment = self.workspace / "examples/webots"
        deployment.mkdir(parents=True)
        (deployment / "robonix_manifest.yaml").write_text(
            "primitive:\n"
            "  - name: audio_driver\n"
            "    url: https://github.com/syswonder/primitive-audio-driver-rbnx\n"
        )
        expected = _complete_proto_tree(
            deployment
            / "rbnx-boot/cache/primitive-audio-driver-rbnx/rbnx-build/codegen/proto_gen"
        )

        with patch.dict(
            "os.environ",
            {
                "GITHUB_WORKSPACE": str(self.workspace),
                "ROBONIX_DEPLOYMENT_DIR": "",
            },
        ):
            actual = run_interfaces.generated_proto_dir()

        self.assertEqual(actual, expected)

    def test_selected_cache_precedes_complete_legacy_tree(self) -> None:
        self._write_manifest(
            "url: https://github.com/syswonder/primitive-audio-driver-rbnx"
        )
        expected = _complete_proto_tree(
            self.deployment
            / "rbnx-boot/cache/primitive-audio-driver-rbnx/rbnx-build/codegen/proto_gen"
        )
        _complete_proto_tree(
            self.deployment
            / "primitives/audio_driver/rbnx-build/codegen/proto_gen"
        )

        actual = run_interfaces.generated_proto_dir(
            self.deployment, workspace=self.workspace
        )

        self.assertEqual(actual, expected)

    def test_in_tree_legacy_layout_remains_supported(self) -> None:
        expected = _complete_proto_tree(
            self.deployment / "primitives/audio_driver/proto_gen"
        )

        actual = run_interfaces.generated_proto_dir(
            self.deployment, workspace=self.workspace
        )

        self.assertEqual(actual, expected)

    def test_partial_cache_does_not_shadow_complete_legacy_tree(self) -> None:
        self._write_manifest(
            "url: https://github.com/syswonder/primitive-audio-driver-rbnx"
        )
        partial = (
            self.deployment
            / "rbnx-boot/cache/primitive-audio-driver-rbnx/rbnx-build/codegen/proto_gen"
        )
        partial.mkdir(parents=True)
        (partial / "atlas_pb2.py").write_text("# incomplete\n")
        expected = _complete_proto_tree(
            self.deployment / "primitives/audio_driver/proto_gen"
        )

        actual = run_interfaces.generated_proto_dir(
            self.deployment, workspace=self.workspace
        )

        self.assertEqual(actual, expected)

    def test_error_names_deployment_and_missing_files(self) -> None:
        with self.assertRaisesRegex(
            FileNotFoundError,
            r"generated interface proto files not found.*audio_pb2.py",
        ):
            run_interfaces.generated_proto_dir(
                self.deployment, workspace=self.workspace
            )


if __name__ == "__main__":
    unittest.main()
