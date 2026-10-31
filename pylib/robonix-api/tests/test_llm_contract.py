# SPDX-License-Identifier: MulanPSL-2.0
import json
import unittest
from pathlib import Path

from robonix_api.llm import Catalog, Requirements

ROOT = Path(__file__).resolve().parent.parent
CONTRACT = json.loads(
    (ROOT / "robonix_api" / "llm" / "contract" / "resolve_cases.json").read_text(
        encoding="utf-8"
    )
)


class ContractTests(unittest.TestCase):
    def test_shared_resolve_cases(self):
        catalog = Catalog.from_dict(CONTRACT["catalog"])
        for case in CONTRACT["cases"]:
            with self.subTest(case=case["name"]):
                raw = case["requirements"]
                requirements = Requirements(
                    capabilities=tuple(raw.get("capabilities", ())),
                    input_tokens=raw.get("input_tokens", 0),
                    max_output_tokens=raw.get("max_output_tokens", 0),
                    max_latency_ms=raw.get("max_latency_ms"),
                    max_cost_usd_per_call=raw.get("max_cost_usd_per_call"),
                    allow_downgrade=raw.get("allow_downgrade", True),
                )
                targets = catalog.resolve(requirements)
                self.assertEqual(
                    [target.id for target in targets], case["expected_ids"]
                )
                if targets:
                    self.assertEqual(
                        targets[0].downgraded,
                        case.get("expected_downgraded", False),
                    )
