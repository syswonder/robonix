"""Package-local tests; run with unittest discover after build."""
import asyncio
import json
import os
from pathlib import Path
import stat
import tempfile
import unittest
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock, patch

from vlm_verifier import camera, core, vlm


def envelope(provider="wrist"):
    return dict(target_provider_id="pick", target_contract_id="robonix/skill/pick/pick",
                target_description="Pick up the red bottle",
                target_args={"object_name": "red bottle"}, target_output={"success": True},
                verifier_args={"camera_provider_id": provider})


class ParsingTests(unittest.TestCase):
    def test_wrist_camera_prompt_recognizes_partial_opposing_jaws(self):
        """Accept valid wrist-camera evidence without requiring the full gripper."""
        prompt = vlm.SYSTEM_PROMPT
        self.assertIn("wrist-mounted camera", prompt)
        self.assertIn("two opposing jaw tips", prompt)
        self.assertIn("do not require the complete gripper body", prompt)
        self.assertIn("without two identifiable opposing jaws is insufficient", prompt)

    def test_vlm_context_preserves_target_and_output_as_separate_fields(self):
        context = json.loads(vlm.describe_context(envelope()))
        for key in ("target_contract_id", "target_description", "target_args",
                    "target_provider_id", "target_output"):
            self.assertEqual(context[key], envelope()[key])

    def test_context_json_or_string(self):
        for value in ({}, [], "original text", None):
            data = envelope()
            data["target_output"] = value
            self.assertEqual(core.parse_envelope(json.dumps(data))["target_output"], value)

    def test_missing_fields_and_invalid_envelope(self):
        """Check missing fields and invalid envelope."""
        for field in envelope():
            data = envelope()
            del data[field]
            with self.subTest(field=field), self.assertRaises(ValueError):
                core.parse_envelope(json.dumps(data))
        for raw in ("[]", "null", "{", "{}"):
            with self.assertRaises(ValueError):
                core.parse_envelope(raw)

    def test_missing_camera_is_rejected(self):
        for args in ({}, None, {"camera_provider_id": " "}, {"camera_provider_id": 1},
                     {"camera_provider_ids": []}, {"camera_provider_ids": "head"},
                     {"camera_provider_ids": ["head", " "]},
                     {"camera_provider_ids": ["head", " head "]},
                     {"camera_provider_id": "head", "camera_provider_ids": ["wrist"]}):
            data = envelope()
            data["verifier_args"] = args
            with self.assertRaises(ValueError):
                core.parse_envelope(json.dumps(data))

    def test_single_and_multiple_camera_configuration(self):
        self.assertEqual(core.camera_provider_ids({"camera_provider_id": " wrist "}), ["wrist"])
        self.assertEqual(core.camera_provider_ids({"camera_provider_ids": [" head ", "wrist"]}),
                         ["head", "wrist"])

    def test_strict_verdict(self):
        """Check strict verdict."""
        for verdict in (True, False):
            self.assertEqual(vlm.parse_verdict(json.dumps(
                {"observable": True, "passed": verdict, "detail": " evidence "})),
                {"passed": verdict, "detail": "evidence"})
        for raw in ('{"observable":true,"passed":"false","detail":"x"}',
                    '{"observable":true,"passed":1,"detail":"x"}',
                    '{"observable":true,"passed":true,"detail":""}',
                    '{"observable":true,"passed":false,"detail":null}',
                    '{"observable":true,"passed":true,"passed":false,"detail":"x"}',
                    '{"observable":true,"passed":true,"detail":"x","confidence":1}',
                    '{"observable":"true","passed":false,"detail":"x"}',
                    '{"observable":false,"passed":true,"detail":"x"}',
                    '{"passed":true,"detail":"x"}',
                    '[]', 'null', 'not json', '```json\n{}\n```'):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                vlm.parse_verdict(raw)

    def test_unobservable_verdict_is_not_a_rejection(self):
        with self.assertRaises(vlm.InsufficientEvidence) as caught:
            vlm.parse_verdict(json.dumps({"observable": False, "passed": False,
                                         "detail": "occluded; secret-model-content"}))
        self.assertNotIn("secret-model-content", str(caught.exception))

    def test_vlm_configuration(self):
        """Check vlm configuration."""
        valid = dict(base_url="http://localhost:123/v1/", api_key="secret", model="vision")
        self.assertEqual(vlm.VlmConfig.parse(valid).base_url, "http://localhost:123/v1")
        for key in valid:
            invalid = dict(valid, **{key: ""})
            with self.assertRaises(ValueError):
                vlm.VlmConfig.parse(invalid)
        for url in ("file:///x", "http://user:secret@host/v1", "https://host/v1?key=secret"):
            with self.assertRaises(ValueError):
                vlm.VlmConfig.parse(dict(valid, base_url=url))

    def test_exact_camera_resolution(self):
        """Check exact camera resolution."""
        atlas = Mock()
        cap = SimpleNamespace(provider_id="wrist", contract_id=camera.RGB_CONTRACT)
        atlas.find_capability.return_value = [cap]
        self.assertIs(camera.resolve_camera(atlas, "verifier", "wrist"),
                      atlas.connect_capability.return_value)
        atlas.find_capability.assert_called_once_with(
            contract_id=camera.RGB_CONTRACT, transport="ros2", provider_id="wrist")
        for caps in ([], [cap, cap], [SimpleNamespace(provider_id="front", contract_id=camera.RGB_CONTRACT)]):
            atlas.find_capability.return_value = caps
            atlas.connect_capability.reset_mock()
            with self.assertRaises(RuntimeError):
                camera.resolve_camera(atlas, "verifier", "wrist")
            atlas.connect_capability.assert_not_called()

    def test_color_conversion(self):
        """Check color conversion."""
        import io
        from PIL import Image as PillowImage
        from sensor_msgs.msg import Image
        for encoding, pixel in (("rgb8", [255, 0, 0]), ("bgr8", [0, 0, 255])):
            msg = Image(height=4, width=4, encoding=encoding, step=12, data=pixel * 16)
            jpeg = camera.encode_jpeg(msg)
            decoded = PillowImage.open(io.BytesIO(jpeg)).getpixel((0, 0))
            self.assertGreater(decoded[0], 240)
            self.assertLess(decoded[2], 10)
        with self.assertRaises(ValueError):
            camera.encode_jpeg(Image())

    def test_verification_frame_is_private_and_call_id_is_sanitized(self):
        """Save the exact model JPEG without permitting call-id path traversal."""
        jpeg = bytes([255, 216, 255, 217])
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(os.environ, {"SCRIBE_LOG_DIR": directory}):
                path = camera.save_verification_frame("2:0/../../secret", jpeg)
            self.assertEqual(path.parent, Path(directory) / "vlm_verifier-frames")
            self.assertNotIn("/", path.name)
            self.assertIn("2_0_.._.._secret", path.name)
            self.assertEqual(path.read_bytes(), jpeg)
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)


    def test_padding_and_invalid_image_shapes(self):
        """Check padding and invalid image shapes."""
        from sensor_msgs.msg import Image
        padded = Image(height=2, width=2, encoding="rgb8", step=8,
                       data=([255, 0, 0] * 2 + [0, 0]) * 2)
        self.assertTrue(camera.encode_jpeg(padded).startswith(bytes([255, 216])))
        for bad in (
            Image(height=2, width=2, encoding="rgb8", step=5, data=[0] * 10),
            Image(height=2, width=2, encoding="rgb8", step=6, data=[0] * 11),
            Image(height=2, width=2, encoding="16UC1", step=4, data=[0] * 8),
        ):
            with self.assertRaises(ValueError):
                camera.encode_jpeg(bad)


class RequestTests(unittest.IsolatedAsyncioTestCase):
    async def test_outcomes_and_original_context(self):
        """Check outcomes and original context."""
        for passed in (True, False):
            observe = AsyncMock(return_value=b"image")
            evaluate = AsyncMock(return_value={"passed": passed, "detail": "visible evidence"})
            result = await core.verify("call", json.dumps(envelope()), observe, evaluate)
            self.assertIs(result["passed"], passed)
            observe.assert_awaited_once_with("wrist")
            evaluate.assert_awaited_once_with(envelope(), {"wrist": b"image"})

    async def test_insufficient_evidence_keeps_a_safe_unavailable_reason(self):
        with self.assertRaisesRegex(RuntimeError, "unavailable.*insufficient visual evidence") as caught:
            await core.verify("call", json.dumps(envelope()), AsyncMock(return_value=b"image"),
                              AsyncMock(side_effect=vlm.InsufficientEvidence("secret-model-content")))
        self.assertNotIn("secret-model-content", str(caught.exception))

    async def test_multiple_cameras_are_captured_concurrently_and_keep_identity(self):
        entered = set()
        both = asyncio.Event()

        async def observe(provider):
            entered.add(provider)
            if len(entered) == 2:
                both.set()
            await asyncio.wait_for(both.wait(), 1)
            return provider.encode()

        data = envelope()
        data["verifier_args"] = {"camera_provider_ids": ["head", "wrist"]}
        evaluate = AsyncMock(return_value={"passed": True, "detail": "visible"})
        await core.verify("call", json.dumps(data), observe, evaluate)
        evaluate.assert_awaited_once_with(data, {"head": b"head", "wrist": b"wrist"})

    async def test_camera_failure_cancels_other_captures_and_skips_model(self):
        entered = asyncio.Event()
        stopped = asyncio.Event()

        async def observe(provider):
            if provider == "missing":
                await entered.wait()
                raise RuntimeError("camera missing")
            entered.set()
            try:
                await asyncio.sleep(60)
            finally:
                stopped.set()

        data = envelope()
        data["verifier_args"] = {"camera_provider_ids": ["missing", "head"]}
        evaluate = AsyncMock()
        with self.assertRaisesRegex(RuntimeError, "unavailable"):
            await core.verify("call", json.dumps(data), observe, evaluate)
        self.assertTrue(stopped.is_set())
        evaluate.assert_not_called()

    async def test_observation_failure_skips_model_and_redacts(self):
        """Check observation failure skips model and redacts."""
        observe = AsyncMock(side_effect=RuntimeError("secret-key"))
        evaluate = AsyncMock()
        with self.assertRaisesRegex(RuntimeError, "unavailable") as caught:
            await core.verify("call", json.dumps(envelope()), observe, evaluate)
        self.assertNotIn("secret-key", str(caught.exception))
        evaluate.assert_not_called()

    async def test_timeouts_are_errors(self):
        """Check timeouts are errors."""
        for observe, evaluate in (
            (AsyncMock(side_effect=asyncio.TimeoutError), AsyncMock()),
            (AsyncMock(return_value=b"image"), AsyncMock(side_effect=asyncio.TimeoutError)),
        ):
            with self.assertRaisesRegex(RuntimeError, "unavailable"):
                await core.verify("call", json.dumps(envelope()), observe, evaluate)

    async def test_concurrent_requests_keep_images_separate(self):
        """Check concurrent requests keep images separate."""
        async def observe(provider):
            await asyncio.sleep(0.01 if provider == "a" else 0)
            return provider.encode()

        async def evaluate(context, images):
            provider = context["verifier_args"]["camera_provider_id"]
            self.assertEqual(images, {provider: provider.encode()})
            return {"passed": True, "detail": provider}

        results = await asyncio.gather(*(
            core.verify(p, json.dumps(envelope(p)), observe, evaluate) for p in ("a", "b")))
        self.assertEqual([r["detail"] for r in results], ["a", "b"])

    async def test_cancellation_propagates(self):
        """Check cancellation propagates."""
        entered = asyncio.Event()
        stopped = asyncio.Event()

        async def observe(provider):
            entered.set()
            try:
                await asyncio.sleep(60)
            finally:
                stopped.set()

        task = asyncio.create_task(core.verify("call", json.dumps(envelope()), observe, AsyncMock()))
        await entered.wait()
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task
        self.assertTrue(stopped.is_set())

    async def test_timeout_budgets(self):
        """Check timeout budgets."""
        original = asyncio.wait_for
        budgets = []

        async def track(awaitable, timeout):
            budgets.append(timeout)
            return await original(awaitable, timeout)

        with patch("vlm_verifier.core.asyncio.wait_for", side_effect=track):
            await core.verify("call", json.dumps(envelope()), AsyncMock(return_value=b"x"),
                              AsyncMock(return_value={"passed": True, "detail": "x"}))
        self.assertEqual(budgets, [50, 5, 40])


class ServicePhaseTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        from vlm_verifier import service
        self.service = service
        self.cache = {}
        self.config = vlm.VlmConfig("http://localhost/v1", "test-secret", "fake-vision")
        self.capture = AsyncMock(return_value=b"image")
        self.judge = AsyncMock(return_value={"passed": True, "detail": "visible"})
        stack = ExitStack()
        self.addCleanup(stack.close)
        for target, name, value in ((service, "_config", self.config),
                                    (service, "_before_images", self.cache),
                                    (service, "CAMERAS", SimpleNamespace(capture=self.capture)),
                                    (service.vlm, "judge", self.judge),
                                    (service.camera, "save_verification_frame", Mock(return_value=Path("frame.jpg")))):
            stack.enter_context(patch.object(target, name, value))

    async def request(self, phase, provider="wrist", call_id="call", cameras=None):
        data = envelope(provider)
        data["phase"] = phase
        if cameras is not None:
            data["verifier_args"] = {"camera_provider_ids": cameras}
        return await self.service.verify(self.service.Verify_Request(call_id=call_id, args_json=json.dumps(data)))

    async def test_before_and_after_send_one_ordered_pair(self):
        self.capture.side_effect = [b"before", b"after"]
        self.assertTrue((await self.request("before")).passed)
        self.judge.assert_not_called()
        self.assertTrue((await self.request("after")).passed)
        self.assertEqual(self.judge.call_args.args[2:], ({"wrist": b"before"}, {"wrist": b"after"}))
        self.judge.assert_awaited_once()
        self.assertFalse(self.cache)

    async def test_multiple_camera_pairs_are_labelled_and_kept_separate(self):
        self.capture.side_effect = [b"head-before", b"wrist-before", b"head-after", b"wrist-after"]
        await self.request("before", cameras=["head", "wrist"])
        await self.request("after", cameras=["head", "wrist"])
        self.assertEqual(self.judge.call_args.args[2:], (
            {"head": b"head-before", "wrist": b"wrist-before"},
            {"head": b"head-after", "wrist": b"wrist-after"}))
        self.assertFalse(self.cache)

    async def test_changed_camera_list_cannot_consume_another_view(self):
        await self.request("before", cameras=["head", "wrist"])
        with self.assertRaisesRegex(RuntimeError, "unavailable"):
            await self.request("after", cameras=["head"])
        self.judge.assert_not_called()

    async def test_unobservable_after_consumes_before_without_returning_a_verdict(self):
        await self.request("before")
        self.judge.side_effect = vlm.InsufficientEvidence()
        with self.assertRaisesRegex(RuntimeError, "unavailable.*insufficient visual evidence"):
            await self.request("after")
        self.assertFalse(self.cache)

    async def test_missing_duplicate_mismatched_and_consumed_before_fail_closed(self):
        with self.assertRaises(ValueError):
            await self.request(None)
        with self.assertRaisesRegex(RuntimeError, "unavailable"):
            await self.request("after")
        await self.request("before")
        with self.assertRaisesRegex(RuntimeError, "unavailable"):
            await self.request("before")
        with self.assertRaisesRegex(RuntimeError, "unavailable"):
            await self.request("after", "other")
        with self.assertRaisesRegex(RuntimeError, "unavailable"):
            await self.request("after")
        self.judge.assert_not_called()

    async def test_pending_cache_is_bounded_and_expired_images_are_removed(self):
        now = self.service.time.monotonic()
        self.cache.update({str(i): (now, {}, {"wrist": b"image"}) for i in range(64)})
        with self.assertRaisesRegex(RuntimeError, "unavailable"):
            await self.request("before")
        self.cache.update({str(i): (float("-inf"), {}, {"wrist": b"image"}) for i in range(64)})
        await self.request("before")
        self.assertEqual(list(self.cache), ["call"])

    async def test_pending_camera_bytes_are_bounded(self):
        with patch.object(self.service, "MAX_PENDING_BYTES", 8):
            await self.request("before", call_id="first")
            with self.assertRaisesRegex(RuntimeError, "unavailable"):
                await self.request("before", call_id="second")
        self.assertEqual(list(self.cache), ["first"])


if __name__ == "__main__":
    unittest.main()
