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
        "features": {"vision": True},
        "latency_p95_ms": 100,
    },
    {
        "id": "slow/model",
        "provider": "vllm",
        "base_url": "http://127.0.0.1:8001/v1",
        "model": "slow",
        "api_key_env": "SLOW_KEY",
        "features": {"vision": True},
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
            self.assertIsNone(resolve_from_env(Requirements(features=("vision",))))

    def test_prefers_a_target_whose_key_is_set(self):
        directory, path = _catalog(MODELS)
        try:
            with tempfile.TemporaryDirectory() as keys:
                with patch.dict(
                    os.environ,
                    {"ROBONIX_LLM_CATALOG": str(path), "SLOW_KEY": "secret"},
                    clear=True,
                ):
                    target = resolve_from_env(Requirements(features=("vision",)))
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
                target = resolve_from_env(Requirements(features=("vision",)))
            self.assertEqual(target.id, "fast/model")
        finally:
            directory.cleanup()

    def test_missing_keys_are_an_error(self):
        directory, path = _catalog(MODELS)
        try:
            with patch.dict(
                os.environ, {"ROBONIX_LLM_CATALOG": str(path)}, clear=True
            ):
                with self.assertRaises(RuntimeError) as ctx:
                    resolve_from_env(Requirements(features=("vision",)))
            self.assertIn("FAST_KEY", str(ctx.exception))
            self.assertIn("SLOW_KEY", str(ctx.exception))
        finally:
            directory.cleanup()

    def test_keyless_target_is_usable_without_a_key(self):
        model = dict(MODELS[0])
        model["id"] = "local/keyless"
        del model["api_key_env"]
        directory, path = _catalog([model])
        try:
            with patch.dict(
                os.environ, {"ROBONIX_LLM_CATALOG": str(path)}, clear=True
            ):
                target = resolve_from_env(Requirements(features=("vision",)))
                key = api_key_for(target)
            self.assertEqual(target.id, "local/keyless")
            self.assertEqual(key, "")
        finally:
            directory.cleanup()

    def test_keyless_target_beats_a_slower_keyed_target(self):
        keyless = dict(MODELS[0])
        keyless["id"] = "local/keyless"
        keyless["latency_p95_ms"] = 100
        del keyless["api_key_env"]
        keyed = dict(MODELS[1])
        keyed["id"] = "cloud/keyed"
        keyed["latency_p95_ms"] = 2000
        directory, path = _catalog([keyless, keyed])
        try:
            with patch.dict(
                os.environ,
                {"ROBONIX_LLM_CATALOG": str(path), "SLOW_KEY": "secret"},
                clear=True,
            ):
                target = resolve_from_env(Requirements(features=("vision",)))
            self.assertEqual(target.id, "local/keyless")
        finally:
            directory.cleanup()

    def test_no_match_is_an_error(self):
        directory, path = _catalog(MODELS)
        try:
            with patch.dict(
                os.environ, {"ROBONIX_LLM_CATALOG": str(path)}, clear=True
            ):
                with self.assertRaises(RuntimeError):
                    resolve_from_env(Requirements(features=("audio",)))
        finally:
            directory.cleanup()

    def test_missing_schema_key_allows_a_usable_json_fallback(self):
        schema = {**MODELS[0], "features": {"json_schema": True}}
        fallback = {**MODELS[1], "features": {"json_object": True}}
        del fallback["api_key_env"]
        directory, path = _catalog([schema, fallback])
        self.addCleanup(directory.cleanup)
        with patch.dict(os.environ, {"ROBONIX_LLM_CATALOG": str(path)}, clear=True):
            req = Requirements(features=("json_schema",))
            target = resolve_from_env(req)
            self.assertEqual(target.id, "slow/model")
            self.assertTrue(target.downgraded)
            with self.assertRaises(RuntimeError):
                resolve_from_env(Requirements(features=("json_schema",), allow_downgrade=False))
            os.environ["FAST_KEY"] = "dummy"
            target = resolve_from_env(req)
            self.assertEqual(target.id, "fast/model")
            self.assertFalse(target.downgraded)
