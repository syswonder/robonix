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
        with patch.dict(os.environ, {
            "ROBONIX_LLM_CATALOG": str(path),
            "VLM_BASE_URL": "", "VLM_API_KEY": "", "VLM_MODEL": "",
        }):
            client = SceneGraphLLMClient()
        self.assertTrue(client.available)
        self.assertFalse(client.requires_key)
        self.assertEqual(client.api_key, "")

    def test_catalog_load_error_is_not_hidden(self):
        with patch.dict(
            os.environ,
            {
                "ROBONIX_LLM_CATALOG": "/nonexistent/catalog.yaml",
                "VLM_BASE_URL": "", "VLM_API_KEY": "", "VLM_MODEL": "",
            },
            clear=False,
        ):
            with self.assertRaises(FileNotFoundError):
                SceneGraphLLMClient()

    def test_explicit_vlm_env_wins_over_catalog(self):
        with patch.dict(os.environ, {
            "ROBONIX_LLM_CATALOG": str(self._catalog(api_key_env="TEST_KEY")),
            "TEST_KEY": "secret", "VLM_BASE_URL": "http://legacy.invalid/v1",
            "VLM_MODEL": "legacy", "VLM_API_KEY": "legacy-key",
        }):
            client = SceneGraphLLMClient()
        self.assertEqual(client.base_url, "http://legacy.invalid/v1")
        self.assertEqual(client.model, "legacy")
        self.assertEqual(client.api_key, "legacy-key")


    def test_catalog_key_named_vlm_api_key_does_not_disable_it(self):
        # The launcher forwards catalog api_key_env names; a key alone is not
        # explicit endpoint configuration.
        with patch.dict(os.environ, {
            "ROBONIX_LLM_CATALOG": str(self._catalog(api_key_env="VLM_API_KEY")),
            "VLM_API_KEY": "secret", "VLM_BASE_URL": "", "VLM_MODEL": "",
        }):
            client = SceneGraphLLMClient()
        self.assertEqual(client.base_url, "http://127.0.0.1:9999/v1")
        self.assertEqual(client.api_key, "secret")

    def test_constructor_key_or_model_alone_stays_explicit(self):
        for kwargs in ({"api_key": "k"}, {"model": "m"}, {"api_key": "k", "model": "m"}):
            with self.subTest(kwargs=kwargs), patch.dict(os.environ, {
                "ROBONIX_LLM_CATALOG": str(self._catalog(api_key_env="TEST_KEY")),
                "TEST_KEY": "secret", "VLM_BASE_URL": "", "VLM_MODEL": "",
            }):
                client = SceneGraphLLMClient(**kwargs)
            self.assertEqual(client.base_url, "")

    def test_explicit_endpoint_argument_wins_over_the_catalog(self):
        with patch.dict(os.environ, {
            "ROBONIX_LLM_CATALOG": str(self._catalog(api_key_env="TEST_KEY")),
            "TEST_KEY": "secret",
        }):
            client = SceneGraphLLMClient(
                base_url="http://explicit.invalid/v1", api_key="k", model="m"
            )
        self.assertEqual(client.base_url, "http://explicit.invalid/v1")
        self.assertEqual(client.model, "m")


if __name__ == "__main__":
    unittest.main()
