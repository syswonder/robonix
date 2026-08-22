# SPDX-License-Identifier: MulanPSL-2.0
"""Regression tests for deterministic fake-VLM history parsing."""

from __future__ import annotations

import json
import unittest

from testing.fake_vlm.server import _leaf_results_from_messages


class FakeVlmHistoryTests(unittest.TestCase):
    def test_wrapper_and_nested_leaf_are_counted_once(self) -> None:
        leaf = {
            "contract_id": "robonix/service/scene/list_objects",
            "success": True,
            "output": '{"objects":[]}',
        }
        messages = [
            {
                "role": "user",
                "content": json.dumps(
                    {"call_id": "scene-list-1", "leaf_result": leaf}
                ),
            }
        ]

        self.assertEqual(_leaf_results_from_messages(messages), [leaf])

    def test_replayed_history_with_same_call_id_is_counted_once(self) -> None:
        first = {
            "call_id": "scene-list-1",
            "leaf_result": {
                "contract_id": "robonix/service/scene/list_objects",
                "success": True,
                "output": '{"objects":[]}',
            },
        }
        second = {
            "call_id": "scene-list-2",
            "leaf_result": {
                "contract_id": "robonix/service/scene/list_objects",
                "success": True,
                "output": '{"objects":[]}',
            },
        }
        messages = [
            {"role": "user", "content": json.dumps(first)},
            {"role": "user", "content": json.dumps([first, second])},
        ]

        leaves = _leaf_results_from_messages(messages)

        self.assertEqual(len(leaves), 2)
        self.assertEqual(
            [leaf["contract_id"] for leaf in leaves],
            [
                "robonix/service/scene/list_objects",
                "robonix/service/scene/list_objects",
            ],
        )

    def test_legacy_leaf_without_call_id_is_not_double_counted(self) -> None:
        leaf = {
            "contract_id": "robonix/service/scene/clear_object",
            "success": True,
            "output": "{}",
        }
        messages = [
            {
                "role": "user",
                "content": json.dumps({"leaf_result": leaf}),
            }
        ]

        self.assertEqual(_leaf_results_from_messages(messages), [leaf])


if __name__ == "__main__":
    unittest.main()
