# SPDX-License-Identifier: MulanPSL-2.0
import json
import os
import tempfile
import unittest
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch

from robonix_api.llm import ModelCatalog, Requirements, api_key_for, resolve_from_env

def _catalog(*models):
    return ModelCatalog.from_dict({"models": list(models)})


def _model(model_id, *, latency=None, vision=True, json_schema=True):
    return {
        "id": model_id,
        "provider": "vllm",
        "base_url": "http://127.0.0.1:8000/v1",
        "model": model_id,
        "api_key_env": "VLLM_API_KEY",
        "features": {
            "vision": vision,
            "json_object": True,
            "json_schema": json_schema,
            "tools": True,
            "stream": True,
        },
        "latency_p95_ms": latency,
    }


class ResolveTests(unittest.TestCase):
    def test_feature_is_a_hard_filter(self):
        catalog = _catalog(_model("vision", vision=True), _model("text", vision=False))
        targets = catalog.resolve(Requirements(features=("vision",)))
        self.assertEqual([target.id for target in targets], ["vision"])

    def test_preference_is_latency_then_price_then_id(self):
        catalog = _catalog(
            _model("slow", latency=2000),
            _model("fast", latency=100),
            _model("also-fast", latency=100),
            {**_model("expensive", latency=100), "price_input_per_mtok": 1},
            _model("unknown"),
        )
        targets = catalog.resolve(Requirements())
        self.assertEqual(
            [target.id for target in targets],
            ["also-fast", "fast", "expensive", "slow", "unknown"],
        )

    def test_json_schema_requires_a_schema_model(self):
        catalog = _catalog(_model("object-only", json_schema=False, latency=100))
        targets = catalog.resolve(
            Requirements(features=("json_schema",)),
        )
        self.assertEqual(targets, [])


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


class ContractTests(unittest.TestCase):
    def test_shared_resolve_cases(self):
        path = Path(__file__).resolve().parent.parent / "robonix_api/llm/contract/resolve_cases.json"
        fixture = json.loads(path.read_text(encoding="utf-8"))
        catalog = ModelCatalog.from_dict(fixture["catalog"])
        for case in fixture["cases"]:
            with self.subTest(case=case["name"]):
                raw = dict(case["requirements"])
                raw["features"] = tuple(raw.get("features", ()))
                targets = catalog.resolve(Requirements(**raw))
                self.assertEqual([t.id for t in targets], case["expected_ids"])


@contextmanager
def _env_catalog(models, **env):
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "models.json"
        path.write_text(json.dumps({"models": models}), encoding="utf-8")
        with patch.dict(os.environ, {"ROBONIX_LLM_CATALOG": str(path), **env}, clear=True):
            yield


class ResolveFromEnvTests(unittest.TestCase):
    def test_disabled_without_catalog_env(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertIsNone(resolve_from_env(Requirements(features=("vision",))))

    def test_prefers_a_target_whose_key_is_set(self):
        with _env_catalog(MODELS, SLOW_KEY="secret"):
            target = resolve_from_env(Requirements(features=("vision",)))
            self.assertEqual(target.id, "slow/model")
            self.assertEqual(api_key_for(target), "secret")

    def test_both_keys_prefer_the_faster_model(self):
        with _env_catalog(MODELS, FAST_KEY="fast", SLOW_KEY="slow"):
            self.assertEqual(resolve_from_env(Requirements(features=("vision",))).id, "fast/model")

    def test_missing_keys_are_an_error(self):
        with _env_catalog(MODELS), self.assertRaises(RuntimeError) as ctx:
            resolve_from_env(Requirements(features=("vision",)))
        self.assertIn("FAST_KEY", str(ctx.exception))
        self.assertIn("SLOW_KEY", str(ctx.exception))

    def test_keyless_target_is_usable_without_a_key(self):
        model = {**MODELS[0], "id": "local/keyless"}
        del model["api_key_env"]
        with _env_catalog([model]):
            target = resolve_from_env(Requirements(features=("vision",)))
            self.assertEqual(target.id, "local/keyless")
            self.assertEqual(api_key_for(target), "")

    def test_keyless_target_beats_a_slower_keyed_target(self):
        keyless = {**MODELS[0], "id": "local/keyless", "latency_p95_ms": 100}
        del keyless["api_key_env"]
        keyed = {**MODELS[1], "id": "cloud/keyed", "latency_p95_ms": 2000}
        with _env_catalog([keyless, keyed], SLOW_KEY="secret"):
            self.assertEqual(resolve_from_env(Requirements(features=("vision",))).id, "local/keyless")

    def test_no_match_is_an_error(self):
        with _env_catalog(MODELS), self.assertRaises(RuntimeError):
            resolve_from_env(Requirements(features=("audio",)))

    def test_missing_schema_key_does_not_relax_features(self):
        schema = {**MODELS[0], "features": {"json_schema": True}}
        fallback = {**MODELS[1], "features": {"json_object": True}}
        del fallback["api_key_env"]
        with _env_catalog([schema, fallback]):
            req = Requirements(features=("json_schema",))
            with self.assertRaisesRegex(RuntimeError, "FAST_KEY"):
                resolve_from_env(req)
            os.environ["FAST_KEY"] = "dummy"
            target = resolve_from_env(req)
            self.assertEqual(target.id, "fast/model")
