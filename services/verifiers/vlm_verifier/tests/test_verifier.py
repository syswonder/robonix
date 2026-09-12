"""Package-local tests; run with unittest discover after build."""
import asyncio
import json
import os
from pathlib import Path
import stat
import tempfile
import unittest
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

    def test_vlm_context_explains_the_verification_target(self):
        """Identify target-defining fields separately from supporting context."""
        text = vlm.describe_context(envelope())
        target_section, supporting_section = text.split("Supporting call context", 1)
        for field in ("target_contract_id", "target_description", "target_args"):
            self.assertIn(field, target_section)
        for field in ("target_provider_id", "target_output"):
            self.assertIn(field, supporting_section)
        self.assertIn("never as visual proof", supporting_section)
        self.assertIn(json.dumps(envelope()["target_args"]), text)

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
        for args in ({}, None, {"camera_provider_id": " "}, {"camera_provider_id": 1}):
            data = envelope()
            data["verifier_args"] = args
            with self.assertRaises(ValueError):
                core.parse_envelope(json.dumps(data))

    def test_strict_verdict(self):
        """Check strict verdict."""
        for verdict in (True, False):
            self.assertEqual(vlm.parse_verdict(json.dumps(
                {"passed": verdict, "detail": " evidence "})),
                {"passed": verdict, "detail": "evidence"})
        for raw in ('{"passed":"false","detail":"x"}', '{"passed":1,"detail":"x"}',
                    '{"passed":true,"detail":""}', '{"passed":false,"detail":null}',
                    '{"passed":true,"passed":false,"detail":"x"}',
                    '{"passed":true,"detail":"x","confidence":1}',
                    '[]', 'null', 'not json', '```json\n{}\n```'):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                vlm.parse_verdict(raw)

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
            evaluate.assert_awaited_once_with(envelope(), b"image")

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

        async def evaluate(context, jpeg):
            self.assertEqual(jpeg.decode(), context["verifier_args"]["camera_provider_id"])
            return {"passed": True, "detail": jpeg.decode()}

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


if __name__ == "__main__":
    unittest.main()
