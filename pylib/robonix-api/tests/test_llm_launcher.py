# SPDX-License-Identifier: MulanPSL-2.0
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


class LauncherTests(unittest.TestCase):
    def _run(self, catalog):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "probe.py").write_text(
                "import json, os, sys\n"
                "print(json.dumps({'env': dict(os.environ), 'args': sys.argv[1:]}))\n"
            )
            env = {
                "PATH": os.defpath,
                "PYTHONPATH": os.pathsep.join((str(root), str(Path(__file__).resolve().parents[1]))),
                "PYTHONDONTWRITEBYTECODE": "1",
                "MODEL_AUTH": "catalog-value",
                "UNUSED_PROVIDER_API_KEY": "unused",
                "UNRELATED_CREDENTIAL": "unused",
                "ROBONIX_LLM_KEY_ENVS": "UNRELATED_CREDENTIAL",
                "RBNX_BOOT_ID": "test-boot",
                "RBNX_INSTANCE_NAME": "probe",
                "ROS_DOMAIN_ID": "31",
                "SCENE_DUALMAP_SAM_WEIGHTS": "/models/sam.pt",
                "AGENT_MEMORY_DIR": "/memory",
            }
            if catalog is not None:
                path = root / "models.json"
                path.write_text(catalog if isinstance(catalog, str) else json.dumps(catalog))
                env["ROBONIX_LLM_CATALOG"] = str(path)
            return subprocess.run(
                [sys.executable, "-m", "robonix_api.llm", "probe", "--arg", "two words"],
                env=env, text=True, capture_output=True, timeout=10,
            )

    def test_catalog_keys_and_runtime_settings_reach_the_consumer(self):
        model = {
            "id": "local", "provider": "test", "model": "local",
            "base_url": "http://127.0.0.1:8000/v1", "api_key_env": "MODEL_AUTH",
        }
        result = self._run({"models": [model, {**model, "id": "other"}]})
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        env = data["env"]
        self.assertEqual(env["MODEL_AUTH"], "catalog-value")
        self.assertEqual(env["ROBONIX_LLM_KEY_ENVS"], "MODEL_AUTH")
        self.assertEqual(env["RBNX_BOOT_ID"], "test-boot")
        self.assertEqual(env["RBNX_INSTANCE_NAME"], "probe")
        self.assertEqual(env["ROS_DOMAIN_ID"], "31")
        self.assertEqual(env["SCENE_DUALMAP_SAM_WEIGHTS"], "/models/sam.pt")
        self.assertEqual(env["AGENT_MEMORY_DIR"], "/memory")
        self.assertNotIn("UNUSED_PROVIDER_API_KEY", env)
        self.assertNotIn("UNRELATED_CREDENTIAL", env)
        self.assertEqual(data["args"], ["--arg", "two words"])

    def test_keyless_catalog_forwards_no_credentials(self):
        result = self._run({"models": [{
            "id": "local", "provider": "test", "model": "local",
            "base_url": "http://127.0.0.1:8000/v1",
        }]})
        self.assertEqual(result.returncode, 0, result.stderr)
        env = json.loads(result.stdout)["env"]
        self.assertEqual(env["ROBONIX_LLM_KEY_ENVS"], "")
        for name in ("MODEL_AUTH", "UNUSED_PROVIDER_API_KEY", "UNRELATED_CREDENTIAL"):
            self.assertNotIn(name, env)

    def test_legacy_launch_keeps_the_environment(self):
        result = self._run(None)
        self.assertEqual(result.returncode, 0, result.stderr)
        env = json.loads(result.stdout)["env"]
        self.assertEqual(env["UNRELATED_CREDENTIAL"], "unused")

    def test_bad_catalog_fails_before_the_consumer_runs(self):
        result = self._run("{")
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, "")
        self.assertIn("JSONDecodeError", result.stderr)
