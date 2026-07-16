# SPDX-License-Identifier: MulanPSL-2.0
"""Checks for Scene's build and source-level capability contracts."""

import ast
import os
import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class SceneBuildConfigTest(unittest.TestCase):
    def test_ingest_uses_the_canonical_lidar3d_contract(self):
        tree = ast.parse((ROOT / "scene_service" / "service.py").read_text())
        contracts = None
        for node in ast.walk(tree):
            if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
                if node.target.id == "_SCENE_CONTRACTS":
                    contracts = ast.literal_eval(node.value)
                    break
        self.assertIsNotNone(contracts)
        by_kind = {
            kind: (contract_id, msg_type)
            for kind, contract_id, msg_type in contracts
        }
        self.assertEqual(
            by_kind["lidar3d"],
            ("robonix/primitive/lidar/lidar3d", "PointCloud2"),
        )

    def test_every_scene_mcp_tool_is_declared(self):
        tree = ast.parse((ROOT / "scene_service" / "service.py").read_text())
        tool_names = None
        for node in ast.walk(tree):
            if not isinstance(node, ast.Assign):
                continue
            if not any(
                isinstance(target, ast.Name) and target.id == "scene_tools"
                for target in node.targets
            ):
                continue
            tool_names = [
                item.attr
                for item in node.value.elts
                if isinstance(item, ast.Attribute)
                and isinstance(item.value, ast.Name)
                and item.value.id == "mcp_tools"
            ]
            break
        self.assertEqual(
            tool_names,
            [
                "list_objects",
                "goal_near",
                "goal_room",
                "get_scene_graph",
                "get_object_context",
                "get_robot_context",
                "list_relations",
            ],
        )

    def test_codegen_uses_exact_runtime_compatible_versions(self):
        helper = (ROOT / "scripts" / "run_python_codegen.sh").read_text()
        self.assertIn('PROTOBUF_VERSION="6.33.6"', helper)
        self.assertIn('GRPC_TOOLS_VERSION="1.76.0"', helper)
        self.assertIn('GRPCIO_VERSION="1.80.0"', helper)
        self.assertIn('PATH="$VENV/bin:$PATH" rbnx codegen', helper)
        self.assertIn('PYTHONPATH="$PROTO_ROOT:$MCP_ROOT"', helper)
        self.assertIn("verify_python_codegen.py", helper)

    def test_build_routes_codegen_through_versioned_helper(self):
        build = (ROOT / "scripts" / "build.sh").read_text()
        self.assertIn('bash "$PKG/scripts/run_python_codegen.sh"', build)
        self.assertNotIn('\nrbnx codegen -p "$PKG"', build)

    def test_runtime_dependency_sources_use_the_same_exact_versions(self):
        expected = (
            "grpcio==1.80.0",
            "protobuf==6.33.6",
        )
        sources = (
            ROOT / "pyproject.toml",
            ROOT / "docker" / "requirements" / "scene-base.txt",
            ROOT / "docker" / "requirements.txt",
        )
        for source in sources:
            contents = source.read_text()
            for requirement in expected:
                self.assertIn(requirement, contents, str(source))
            self.assertNotIn("grpcio-tools", contents, str(source))

        for source in (ROOT / "scene_service").rglob("*.py"):
            self.assertNotIn("import grpc_tools", source.read_text(), str(source))

    def test_native_core_runtime_install_is_fatal_and_verified(self):
        build = (ROOT / "scripts" / "build.sh").read_text()
        install = '"$PY" -m pip install --user -r "$BASE_REQ"'
        self.assertIn(install, build)
        install_line = next(line for line in build.splitlines() if install in line)
        self.assertNotIn("||", install_line)
        self.assertIn('"$PY" "$PKG/scripts/verify_python_codegen.py"', build)

    def test_executable_codegen_import_provenance_regression(self):
        script = ROOT / "tests" / "test_python_codegen.sh"
        self.assertTrue(os.access(script, os.X_OK), f"not executable: {script}")
        completed = subprocess.run(
            [str(script)],
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(
            completed.returncode,
            0,
            f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}",
        )


if __name__ == "__main__":
    unittest.main()
