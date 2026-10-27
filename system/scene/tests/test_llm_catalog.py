# SPDX-License-Identifier: MulanPSL-2.0
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scene_service.scene_graph.llm_client import SceneGraphLLMClient


class SceneCatalogTests(unittest.TestCase):
    def test_catalog_overrides_the_vlm_env(self):
        with tempfile.TemporaryDirectory() as tmp:
            catalog = Path(tmp) / "catalog.json"
            catalog.write_text(
                json.dumps(
                    {
                        "models": [
                            {
                                "id": "test/vision",
                                "provider": "vllm",
                                "base_url": "http://127.0.0.1:9999/v1",
                                "model": "test-vl",
                                "api_key_env": "TEST_KEY",
                                "capabilities": {"vision": True},
                                "latency_p95_ms": 100,
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            with patch.dict(
                os.environ,
                {
                    "ROBONIX_LLM_CATALOG": str(catalog),
                    "TEST_KEY": "secret",
                    "VLM_BASE_URL": "http://legacy.invalid/v1",
                    "VLM_MODEL": "legacy",
                    "VLM_API_KEY": "legacy-key",
                },
                clear=False,
            ):
                client = SceneGraphLLMClient()
        self.assertEqual(client.base_url, "http://127.0.0.1:9999/v1")
        self.assertEqual(client.model, "test-vl")
        self.assertEqual(client.api_key, "secret")


if __name__ == "__main__":
    unittest.main()
