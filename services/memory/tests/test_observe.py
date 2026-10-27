"""Tests for VLM helpers — template summary, VLM config, image QA."""

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

from memory_service.core.types import ObjectCoord
from memory_service.core.observe import (
    _template_summary, _vlm_available, _vlm_config,
)


class TestTemplateSummary(unittest.TestCase):
    def test_with_objects_and_scene(self):
        objs = [
            ObjectCoord("o1", "red cup", 1.0, 2.0, 0.8),
            ObjectCoord("o2", "counter", 1.0, 2.0, 0.0),
        ]
        s = _template_summary(objs, "kitchen")
        assert "red cup" in s
        assert "counter" in s
        assert "kitchen" in s

    def test_no_objects(self):
        s = _template_summary([], "living_room")
        assert "living room" in s

    def test_no_scene_type(self):
        objs = [ObjectCoord("o1", "table", 0, 0, 0)]
        s = _template_summary(objs, "")
        assert "table" in s
        assert "current area" in s


class TestVLMConfig(unittest.TestCase):
    def _clear_env(self):
        for k in ("VLM_BASE_URL", "VLM_API_KEY", "VLM_MODEL",
                  "OPENAI_BASE_URL", "OPENAI_API_KEY", "OPENAI_MODEL",
                  "ROBONIX_LLM_CATALOG"):
            os.environ.pop(k, None)

    def test_no_creds_vlm_unavailable(self):
        self._clear_env()
        assert _vlm_available() is False

    def test_vlm_available_with_creds(self):
        self._clear_env()
        os.environ["VLM_BASE_URL"] = "https://vlm.example.com/v1"
        os.environ["VLM_API_KEY"] = "sk-test"
        assert _vlm_available() is True

    def test_openai_fallback(self):
        self._clear_env()
        os.environ["OPENAI_BASE_URL"] = "https://api.openai.com/v1"
        os.environ["OPENAI_API_KEY"] = "sk-openai"
        assert _vlm_available() is True

    def test_model_default(self):
        self._clear_env()
        os.environ["VLM_BASE_URL"] = "https://x/v1"
        os.environ["VLM_API_KEY"] = "sk"
        cfg = _vlm_config()
        assert cfg["model"] == "gpt-4.1"

    def test_catalog_overrides_legacy_config(self):
        self._clear_env()
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
                    "VLM_BASE_URL": "https://legacy.invalid/v1",
                    "VLM_API_KEY": "legacy",
                },
                clear=False,
            ):
                cfg = _vlm_config()
        assert cfg == {
            "base_url": "http://127.0.0.1:9999/v1",
            "api_key": "secret",
            "model": "test-vl",
        }


if __name__ == "__main__":
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromModule(__import__(__name__))
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
