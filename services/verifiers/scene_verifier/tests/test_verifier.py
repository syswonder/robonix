"""Run with python -m unittest discover -s tests -v from the package root."""
import asyncio
import json
import math
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock

from scene_verifier.config import VerifierConfig, parse_config
from scene_verifier.core import (
    NAVIGATE_CONTRACT, NavigationGoal, RobotContext, parse_envelope,
    parse_goal, parse_robot_context, shortest_angle_error,
    verify_navigation_result,
)
from scene_verifier.scene_client import decode_mcp_response
from scene_verifier.service import verify_request


def payload():
    """Construct a complete PR #242 envelope for a yaw-zero map goal."""
    return {
        "target_provider_id": "nav2",
        "target_contract_id": NAVIGATE_CONTRACT,
        "target_description": "navigate",
        "target_args": {"goal": {
            "header": {"frame_id": "map"},
            "pose": {
                "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }},
        "target_output": {"state": "SUCCEEDED"},
        "verifier_args": {"scene_provider_id": "scene", "check_yaw": True},
    }


def context():
    return RobotContext(True, False, "floor-3", 1.0, 2.0, 0.0, "current")


class CoreTests(unittest.TestCase):
    def test_config_defaults(self):
        self.assertEqual(parse_config({}), VerifierConfig())

    def test_invalid_config(self):
        """Reject permissive numeric casts and invalid tolerance ranges."""
        for value in (True, "0.5", 0, -1, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_config({"distance_tolerance_m": value})
        with self.assertRaises(ValueError):
            parse_config({"yaw_tolerance_rad": 4.0})
        with self.assertRaises(ValueError):
            parse_config({"observation_timeout_s": 60})

    def test_envelope(self):
        envelope = parse_envelope(json.dumps(payload()))
        self.assertEqual(envelope.scene_provider_id, "scene")
        self.assertTrue(envelope.check_yaw)

    def test_bad_envelopes(self):
        """Reject malformed wire input and incorrect private option types."""
        for raw in ("{", "[]", '{"x":1,"x":2}', '{"x":NaN}'):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                parse_envelope(raw)
        for field, value in (("target_contract_id", "unsupported"),
                             ("target_args", "{}")):
            item = payload()
            item[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                parse_envelope(json.dumps(item))
        for options in ({}, {"scene_provider_id": "scene", "check_yaw": "false"}):
            item = payload()
            item["verifier_args"] = options
            with self.assertRaises(ValueError):
                parse_envelope(json.dumps(item))

    def test_parse_goal(self):
        goal = parse_goal(payload()["target_args"], True)
        self.assertEqual(goal, NavigationGoal("map", 1.0, 2.0, 0.0))

    def test_missing_position(self):
        args = payload()["target_args"]
        del args["goal"]["pose"]["position"]["x"]
        with self.assertRaises(ValueError):
            parse_goal(args, True)

    def test_heading_disabled_allows_omission(self):
        args = payload()["target_args"]
        del args["goal"]["pose"]["orientation"]
        self.assertIsNone(parse_goal(args, False).yaw)
        with self.assertRaises(ValueError):
            parse_goal(args, True)

    def test_zero_quaternion_rejected(self):
        args = payload()["target_args"]
        args["goal"]["pose"]["orientation"]["w"] = 0.0
        with self.assertRaises(ValueError):
            parse_goal(args, True)

    def test_unsupported_frame(self):
        args = payload()["target_args"]
        args["goal"]["header"]["frame_id"] = "odom"
        with self.assertRaises(ValueError):
            parse_goal(args, True)

    def test_malformed_observation(self):
        raw = vars(context()).copy()
        for field, bad in (("stale", "false"), ("x", float("nan")),
                           ("pose_known", 1)):
            with self.subTest(field=field), self.assertRaises(ValueError):
                parse_robot_context({**raw, field: bad})

    def test_pass_and_distance_boundary(self):
        goal = parse_goal(payload()["target_args"], True)
        self.assertTrue(verify_navigation_result(goal, context(), VerifierConfig())[0])
        self.assertFalse(verify_navigation_result(
            goal, replace(context(), x=1.5), VerifierConfig())[0])

    def test_bad_observation_verdicts(self):
        """Unknown, stale and unidentified maps never confirm completion."""
        goal = parse_goal(payload()["target_args"], True)
        for item in (replace(context(), pose_known=False),
                     replace(context(), stale=True), replace(context(), map_id="")):
            with self.subTest(item=item):
                self.assertFalse(verify_navigation_result(goal, item, VerifierConfig())[0])

    def test_map_identity_not_frame_name(self):
        goal = parse_goal(payload()["target_args"], True)
        self.assertTrue(verify_navigation_result(
            goal, context(), VerifierConfig(), "floor-3")[0])
        self.assertFalse(verify_navigation_result(
            goal, context(), VerifierConfig(), "floor-4")[0])

    def test_yaw_wrap_and_mismatch(self):
        self.assertLess(shortest_angle_error(3.13, -3.13), 0.03)
        goal = NavigationGoal("map", 1, 2, -3.13)
        self.assertTrue(verify_navigation_result(
            goal, replace(context(), yaw=3.13), VerifierConfig())[0])
        self.assertFalse(verify_navigation_result(goal, context(), VerifierConfig())[0])

    def test_yaw_boundary(self):
        goal = NavigationGoal("map", 1, 2, math.pi)
        self.assertFalse(verify_navigation_result(
            goal, context(), VerifierConfig(yaw_tolerance_rad=math.pi))[0])


class ResponseTests(unittest.TestCase):
    def test_structured_and_text(self):
        raw = vars(context()).copy()
        structured = SimpleNamespace(isError=False, structuredContent=raw, content=[])
        text = SimpleNamespace(isError=False, structuredContent=None, content=[
            SimpleNamespace(type="text", text=json.dumps(raw))])
        self.assertEqual(decode_mcp_response(structured), raw)
        self.assertEqual(decode_mcp_response(text), raw)

    def test_tool_error(self):
        result = SimpleNamespace(isError=True, structuredContent={}, content=[])
        with self.assertRaises(RuntimeError):
            decode_mcp_response(result)


class ServiceTests(unittest.IsolatedAsyncioTestCase):
    async def test_routes_exact_provider(self):
        fetch = AsyncMock(return_value=context())
        passed, _ = await verify_request("p:0", json.dumps(payload()),
                                        VerifierConfig(), "scene_verifier", fetch)
        self.assertTrue(passed)
        fetch.assert_awaited_once_with("scene_verifier", "scene", 5.0)

    async def test_invalid_request_does_not_observe(self):
        fetch = AsyncMock()
        with self.assertRaises(ValueError):
            await verify_request("p:0", "{}", VerifierConfig(), "v", fetch)
        fetch.assert_not_awaited()

    async def test_unavailable_is_exception(self):
        fetch = AsyncMock(side_effect=RuntimeError("Scene offline"))
        with self.assertLogs("scene_verifier", level="ERROR"):
            with self.assertRaisesRegex(RuntimeError, "Scene offline"):
                await verify_request("p:0", json.dumps(payload()),
                                     VerifierConfig(), "v", fetch)

    async def test_cancel_propagates(self):
        fetch = AsyncMock(side_effect=asyncio.CancelledError())
        with self.assertRaises(asyncio.CancelledError):
            await verify_request("p:0", json.dumps(payload()),
                                 VerifierConfig(), "v", fetch)

    async def test_output_cannot_override_observation(self):
        fetch = AsyncMock(return_value=replace(context(), stale=True))
        passed, _ = await verify_request("p:0", json.dumps(payload()),
                                        VerifierConfig(), "v", fetch)
        self.assertFalse(passed)


if __name__ == "__main__":
    unittest.main()