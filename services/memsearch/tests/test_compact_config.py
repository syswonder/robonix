# SPDX-License-Identifier: MulanPSL-2.0
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

_HERE = Path(__file__).resolve().parent
_SVC = _HERE.parent
if str(_SVC) not in sys.path:
    sys.path.insert(0, str(_SVC))

from memsearch_service.service import _compact_config

_ENV_KEYS = (
    "ROBONIX_LLM_CATALOG",
    "VLM_BASE_URL",
    "VLM_API_KEY",
    "VLM_MODEL",
    "OPENAI_BASE_URL",
    "OPENAI_API_KEY",
    "OPENAI_MODEL",
)


class CompactConfigTests(unittest.TestCase):
    def setUp(self):
        for key in _ENV_KEYS:
            os.environ.pop(key, None)

    def tearDown(self):
        for key in _ENV_KEYS:
            os.environ.pop(key, None)

    def test_legacy_vlm_env(self):
        os.environ["VLM_BASE_URL"] = "https://legacy.example.com/v1"
        os.environ["VLM_API_KEY"] = "legacy"
        os.environ["VLM_MODEL"] = "legacy-model"
        self.assertEqual(
            _compact_config(),
            {
                "base_url": "https://legacy.example.com/v1",
                "api_key": "legacy",
                "model": "legacy-model",
            },
        )

    def test_catalog_overrides_legacy_env(self):
        with tempfile.TemporaryDirectory() as tmp:
            catalog = Path(tmp) / "catalog.json"
            catalog.write_text(
                json.dumps(
                    {
                        "models": [
                            {
                                "id": "test/summarizer",
                                "provider": "vllm",
                                "base_url": "http://127.0.0.1:9999/v1",
                                "model": "test-summarizer",
                                "api_key_env": "TEST_KEY",
                                "capabilities": {"json_object": True},
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
                    "VLM_BASE_URL": "https://legacy.invalid/v1",
                    "VLM_API_KEY": "legacy",
                },
                clear=False,
            ):
                config = _compact_config()
        self.assertEqual(
            config,
            {
                "base_url": "http://127.0.0.1:9999/v1",
                "api_key": "secret",
                "model": "test-summarizer",
            },
        )


if __name__ == "__main__":
    unittest.main()
