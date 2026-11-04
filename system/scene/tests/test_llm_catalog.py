# SPDX-License-Identifier: MulanPSL-2.0
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scene_service.scene_graph.llm_client import SceneGraphLLMClient


class SceneCatalogTests(unittest.TestCase):

    def _catalog(self, **fields):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "models.json"
        model = {
            "id": "test/vision", "provider": "vllm", "model": "test-vl",
            "base_url": "http://127.0.0.1:9999/v1", "features": {"vision": True},
            "latency_p95_ms": 100, **fields,
        }
        path.write_text(json.dumps({"models": [model]}), encoding="utf-8")
        return path
    def test_keyless_catalog_target_is_available(self):
        path = self._catalog(id="local/keyless", model="local")
        with patch.dict(os.environ, {"ROBONIX_LLM_CATALOG": str(path)}):
            client = SceneGraphLLMClient()
        self.assertTrue(client.available)
        self.assertFalse(client.requires_key)
        self.assertEqual(client.api_key, "")

    def test_catalog_load_error_is_not_hidden(self):
        with patch.dict(
            os.environ,
            {"ROBONIX_LLM_CATALOG": "/nonexistent/catalog.yaml"},
            clear=False,
        ):
            with self.assertRaises(FileNotFoundError):
                SceneGraphLLMClient()

    def test_catalog_overrides_the_vlm_env(self):
        with patch.dict(os.environ, {
            "ROBONIX_LLM_CATALOG": str(self._catalog(api_key_env="TEST_KEY")),
            "TEST_KEY": "secret", "VLM_BASE_URL": "http://legacy.invalid/v1",
            "VLM_MODEL": "legacy", "VLM_API_KEY": "legacy-key",
        }):
            client = SceneGraphLLMClient()
        self.assertEqual(client.base_url, "http://127.0.0.1:9999/v1")
        self.assertEqual(client.model, "test-vl")
        self.assertEqual(client.api_key, "secret")


if __name__ == "__main__":
    unittest.main()
