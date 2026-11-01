# SPDX-License-Identifier: MulanPSL-2.0
import json
import tempfile
import unittest
from pathlib import Path

from robonix_api.llm import ModelCatalog

ROOT = Path(__file__).resolve().parent.parent
LLM = ROOT / "robonix_api" / "llm"


class ModelCatalogTests(unittest.TestCase):
    def test_load_example_yaml(self):
        catalog = ModelCatalog.load(LLM / "catalog.example.yaml")
        self.assertEqual(len(catalog.models), 3)
        deepseek = catalog.model("deepseek/flash")
        self.assertEqual(deepseek.base_url, "https://api.deepseek.com")
        self.assertTrue(deepseek.supports("vision"))
        self.assertFalse(deepseek.supports("json_schema"))

    def test_load_json(self):
        data = {
            "models": [
                {
                    "id": "local/test",
                    "provider": "vllm",
                    "base_url": "http://127.0.0.1:8000/v1",
                    "model": "test",
                    "api_key_env": "VLLM_API_KEY",
                    "features": {"vision": True},
                    "context": 8192,
                }
            ]
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "catalog.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            catalog = ModelCatalog.load(path)
        self.assertEqual(catalog.models[0].id, "local/test")
        self.assertEqual(catalog.models[0].protocol, "openai-chat")

    def test_api_key_env_is_optional(self):
        catalog = ModelCatalog.from_dict(
            {
                "models": [
                    {
                        "id": "local/keyless",
                        "provider": "vllm",
                        "base_url": "http://127.0.0.1:8000/v1",
                        "model": "local",
                    }
                ]
            }
        )
        self.assertIsNone(catalog.models[0].api_key_env)

    def test_rejects_empty_api_key_env(self):
        with self.assertRaises(ValueError):
            ModelCatalog.from_dict(
                {
                    "models": [
                        {
                            "id": "local/keyless",
                            "provider": "vllm",
                            "base_url": "http://127.0.0.1:8000/v1",
                            "model": "local",
                            "api_key_env": "",
                        }
                    ]
                }
            )

    def test_rejects_duplicate_ids(self):
        data = {
            "models": [
                {
                    "id": "same",
                    "provider": "vllm",
                    "base_url": "http://127.0.0.1:8000/v1",
                    "model": "same",
                    "api_key_env": "VLLM_API_KEY",
                },
                {
                    "id": "same",
                    "provider": "vllm",
                    "base_url": "http://127.0.0.1:8000/v1",
                    "model": "same",
                    "api_key_env": "VLLM_API_KEY",
                },
            ]
        }
        with self.assertRaises(ValueError):
            ModelCatalog.from_dict(data)

    def test_rejects_missing_field(self):
        with self.assertRaises(ValueError):
            ModelCatalog.from_dict(
                {"models": [{"id": "x", "provider": "vllm", "base_url": "http://x"}]}
            )

    def test_rejects_non_boolean_features(self):
        with self.assertRaises(ValueError):
            ModelCatalog.from_dict(
                {
                    "models": [
                        {
                            "id": "x",
                            "provider": "vllm",
                            "base_url": "http://x",
                            "model": "x",
                            "api_key_env": "VLLM_API_KEY",
                            "features": {"vision": "false"},
                        }
                    ]
                }
            )

    def test_unknown_model_is_an_error(self):
        catalog = ModelCatalog.load(LLM / "catalog.example.yaml")
        with self.assertRaises(ValueError):
            catalog.model("absent")
