# SPDX-License-Identifier: MulanPSL-2.0
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from uuid import uuid4

from robonix_api.llm import Requirements, api_key_for, resolve_from_env

MODELS = [
    {
        "id": "fast/model",
        "provider": "vllm",
        "base_url": "http://127.0.0.1:8000/v1",
        "model": "fast",
        "api_key_env": "FAST_KEY",
        "capabilities": {"vision": True},
        "latency_p95_ms": 100,
    },
    {
        "id": "slow/model",
        "provider": "vllm",
        "base_url": "http://127.0.0.1:8001/v1",
        "model": "slow",
        "api_key_env": "SLOW_KEY",
        "capabilities": {"vision": True},
        "latency_p95_ms": 200,
    },
]


def _catalog(models):
    directory = tempfile.TemporaryDirectory()
    path = Path(directory.name) / f"catalog-{uuid4().hex}.json"
    path.write_text(json.dumps({"models": models}), encoding="utf-8")
    return directory, path


class ResolveFromEnvTests(unittest.TestCase):
    def test_disabled_without_catalog_env(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertIsNone(resolve_from_env(Requirements(capabilities=("vision",))))

    def test_prefers_a_target_whose_key_is_set(self):
        directory, path = _catalog(MODELS)
        try:
            with tempfile.TemporaryDirectory() as keys:
                with patch.dict(
                    os.environ,
                    {"ROBONIX_LLM_CATALOG": str(path), "SLOW_KEY": "secret"},
                    clear=True,
                ):
                    target = resolve_from_env(Requirements(capabilities=("vision",)))
                    key = api_key_for(target)
            self.assertEqual(target.id, "slow/model")
            self.assertEqual(key, "secret")
        finally:
            directory.cleanup()

    def test_both_keys_prefer_the_faster_model(self):
        directory, path = _catalog(MODELS)
        try:
            with patch.dict(
                os.environ,
                {
                    "ROBONIX_LLM_CATALOG": str(path),
                    "FAST_KEY": "fast",
                    "SLOW_KEY": "slow",
                },
                clear=True,
            ):
                target = resolve_from_env(Requirements(capabilities=("vision",)))
            self.assertEqual(target.id, "fast/model")
        finally:
            directory.cleanup()

    def test_missing_keys_still_return_the_first_target(self):
        directory, path = _catalog(MODELS)
        try:
            with patch.dict(
                os.environ, {"ROBONIX_LLM_CATALOG": str(path)}, clear=True
            ):
                target = resolve_from_env(Requirements(capabilities=("vision",)))
            self.assertEqual(target.id, "fast/model")
            self.assertEqual(api_key_for(target), "")
        finally:
            directory.cleanup()

    def test_no_match_is_an_error(self):
        directory, path = _catalog(MODELS)
        try:
            with patch.dict(
                os.environ, {"ROBONIX_LLM_CATALOG": str(path)}, clear=True
            ):
                with self.assertRaises(RuntimeError):
                    resolve_from_env(Requirements(capabilities=("audio",)))
        finally:
            directory.cleanup()
