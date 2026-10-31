# SPDX-License-Identifier: MulanPSL-2.0
import unittest

from robonix_api.llm import Catalog, Requirements


def _catalog(*models):
    return Catalog.from_dict({"models": list(models)})


def _model(model_id, *, latency=None, vision=True, json_schema=True, context=8192):
    return {
        "id": model_id,
        "provider": "vllm",
        "base_url": "http://127.0.0.1:8000/v1",
        "model": model_id,
        "api_key_env": "VLLM_API_KEY",
        "capabilities": {
            "vision": vision,
            "json_object": True,
            "json_schema": json_schema,
            "tools": True,
            "stream": True,
        },
        "context": context,
        "latency_p95_ms": latency,
    }


class ResolveTests(unittest.TestCase):
    def test_capability_is_a_hard_filter(self):
        catalog = _catalog(_model("vision", vision=True), _model("text", vision=False))
        targets = catalog.resolve(Requirements(capabilities=("vision",)))
        self.assertEqual([target.id for target in targets], ["vision"])

    def test_context_budget_filters_models(self):
        catalog = _catalog(
            _model("small", context=1000),
            _model("large", context=100000),
        )
        targets = catalog.resolve(
            Requirements(input_tokens=5000, max_output_tokens=1000),
        )
        self.assertEqual([target.id for target in targets], ["large"])

    def test_latency_budget_rejects_unknown_latency(self):
        catalog = _catalog(_model("known", latency=500), _model("unknown", latency=None))
        targets = catalog.resolve(Requirements(max_latency_ms=1000))
        self.assertEqual([target.id for target in targets], ["known"])

    def test_preference_is_latency_then_price_then_id(self):
        catalog = _catalog(
            _model("slow", latency=2000),
            _model("fast", latency=100),
            _model("also-fast", latency=100),
        )
        targets = catalog.resolve(Requirements())
        self.assertEqual(
            [target.id for target in targets], ["also-fast", "fast", "slow"]
        )

    def test_downgrade_marks_every_target(self):
        catalog = _catalog(_model("object-only", json_schema=False, latency=100))
        targets = catalog.resolve(
            Requirements(capabilities=("json_schema",), allow_downgrade=True),
        )
        self.assertEqual([target.id for target in targets], ["object-only"])
        self.assertTrue(targets[0].downgraded)

    def test_no_downgrade_keeps_the_strict_request(self):
        catalog = _catalog(_model("object-only", json_schema=False, latency=100))
        targets = catalog.resolve(
            Requirements(capabilities=("json_schema",), allow_downgrade=False),
        )
        self.assertEqual(targets, [])
