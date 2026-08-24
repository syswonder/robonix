# SPDX-License-Identifier: MulanPSL-2.0
import base64
import os
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import grpc
from speech_service import service as service_module
from speech_service.service import (
    SpeechTtsServicer,
    SpeechTtsStreamServicer,
    _apply_cfg_to_env,
)
from speech_service.tencent_cloud import TencentTTSBackend
from speech_service.tts_errors import TTSInputError

_HAN_CHAR = "\N{CJK UNIFIED IDEOGRAPH-7532}"


class _OneShotBackend:
    def __init__(self, audio=b"\x00\x00", error=None):
        self.calls = []
        self.audio = audio
        self.error = error

    async def synthesize(self, text, voice="", speed=1.0):
        self.calls.append((text, voice, speed))
        if self.error is not None:
            raise self.error
        return self.audio


class _RejectingBackend:
    async def synthesize(self, text, voice="", speed=1.0):
        raise TTSInputError("invalid TTS text")

    async def synthesize_stream(self, text, voice="", speed=1.0):
        if text:
            raise TTSInputError("invalid TTS text")
        yield b""


class _UnexpectedValueErrorBackend:
    async def synthesize_stream(self, text, voice="", speed=1.0):
        if text:
            raise ValueError("backend configuration is invalid")
        yield b""


class _StreamingBackend:
    def __init__(self, *, fail_after_first=False):
        self.calls = []
        self.closed = False
        self.fail_after_first = fail_after_first

    async def synthesize_stream(self, text, voice="", speed=1.0):
        """Yield fixture PCM and record closure even after failure/cancellation."""
        self.calls.append((text, voice, speed))
        try:
            yield b"\x00\x00"
            if self.fail_after_first:
                raise RuntimeError("Tencent TTS segment 2/2 failed")
            yield b"\x01\x00"
        finally:
            self.closed = True


def _request(text="short text"):
    return SimpleNamespace(text=text, voice="", speed=0.0)


class SpeechTtsServicerTest(unittest.TestCase):
    def setUp(self):
        """Isolate TTS response metadata from the operator environment."""
        self._env = patch.dict(
            os.environ,
            {
                "SPEECH_TTS_OUTPUT_ENCODING": "pcm_s16le",
                "SPEECH_TTS_OUTPUT_SAMPLE_RATE": "16000",
            },
            clear=False,
        )
        self._env.start()
        self.addCleanup(self._env.stop)

    def test_runtime_config_applies_tencent_text_limits(self):
        """Driver config exports both limits consumed by Tencent TTS."""
        with patch.dict(os.environ, {}, clear=True):
            _apply_cfg_to_env(
                {
                    "tencent_tts_max_chars": 123,
                    "tencent_tts_max_total_chars": 4_567,
                }
            )
            self.assertEqual(os.environ["TENCENT_TTS_MAX_CHARS"], "123")
            self.assertEqual(os.environ["TENCENT_TTS_MAX_TOTAL_CHARS"], "4567")

    def test_one_shot_short_text_is_forwarded_unchanged(self):
        """The unary RPC keeps existing short-text and PCM metadata behavior."""
        backend = _OneShotBackend()
        response = SpeechTtsServicer(backend).Synthesize(_request(), Mock())

        self.assertEqual(backend.calls, [("short text", "", 1.0)])
        self.assertEqual(response.audio_data, b"\x00\x00")
        self.assertEqual(response.encoding, "pcm_s16le")
        self.assertEqual(response.sample_rate_hz, 16000)
        self.assertEqual(response.error, "")

    def test_oversized_audio_is_not_retained_in_cache(self):
        """A single large provider response cannot pin the TTS cache budget."""
        backend = _OneShotBackend()
        servicer = SpeechTtsServicer(backend)
        with patch.object(service_module, "_TTS_CACHE_MAX_BYTES", 1):
            servicer.Synthesize(_request(), Mock())
            servicer.Synthesize(_request(), Mock())

        self.assertEqual(len(backend.calls), 2)

    def test_cache_evicts_oldest_audio_to_stay_within_byte_budget(self):
        """Several small entries cannot collectively exceed the cache budget."""
        backend = _OneShotBackend()
        servicer = SpeechTtsServicer(backend)
        with patch.object(service_module, "_TTS_CACHE_MAX_BYTES", 3):
            servicer.Synthesize(_request("first"), Mock())
            servicer.Synthesize(_request("second"), Mock())
            servicer.Synthesize(_request("first"), Mock())

        self.assertEqual(len(backend.calls), 3)

    def test_invalid_metadata_fails_before_one_shot_backend_work(self):
        """Local response configuration is checked before provider work."""
        cases = (
            ({"SPEECH_TTS_OUTPUT_SAMPLE_RATE": "invalid"}, "invalid literal"),
            ({"SPEECH_TTS_OUTPUT_SAMPLE_RATE": "8000"}, "must be 16000"),
            ({"SPEECH_TTS_OUTPUT_ENCODING": "mp3"}, "must be pcm_s16le"),
            ({"SPEECH_TTS_OUTPUT_ENCODING": ""}, "must be pcm_s16le"),
        )
        for environment, detail in cases:
            with self.subTest(environment=environment):
                backend = _OneShotBackend()
                context = Mock()
                with patch.dict(os.environ, environment, clear=False):
                    response = SpeechTtsServicer(backend).Synthesize(
                        _request(), context
                    )

                self.assertEqual(backend.calls, [])
                self.assertIn(detail, response.error)
                context.set_code.assert_not_called()

    def test_one_shot_invalid_input_uses_contract_error_field(self):
        """Unary input errors retain the response-level failure contract."""
        context = Mock()
        response = SpeechTtsServicer(_RejectingBackend()).Synthesize(
            _request(), context
        )

        context.set_code.assert_not_called()
        context.set_details.assert_not_called()
        self.assertEqual(response.audio_data, b"")
        self.assertEqual(response.error, "invalid TTS text")

    def test_one_shot_provider_failure_uses_contract_error_field(self):
        """Unary provider failures retain the response-level failure contract."""
        context = Mock()
        response = SpeechTtsServicer(
            _OneShotBackend(error=RuntimeError("provider unavailable"))
        ).Synthesize(_request(), context)

        context.set_code.assert_not_called()
        context.set_details.assert_not_called()
        self.assertEqual(response.audio_data, b"")
        self.assertEqual(response.error, "provider unavailable")

    def test_one_shot_cancellation_stops_tencent_segment_fanout(self):
        """An inactive unary transport prevents the next paid provider request."""
        provider_payloads = []
        environment = {
            "TENCENTCLOUD_SECRET_ID": "test-id",
            "TENCENTCLOUD_SECRET_KEY": "test-key",
            "TENCENT_TTS_MAX_CHARS": "1",
        }
        with patch.dict(os.environ, environment, clear=True):
            backend = TencentTTSBackend()
        backend._post_json = lambda payload: (
            provider_payloads.append(payload)
            or {"Audio": base64.b64encode(b"\x00\x00").decode("ascii")}
        )
        context = Mock()
        context.is_active.side_effect = (True, False)

        response = SpeechTtsServicer(backend).Synthesize(
            _request(_HAN_CHAR * 3), context
        )

        self.assertEqual(len(provider_payloads), 1)
        self.assertIn("cancelled before segment 2/3", response.error)

    def test_stream_short_text_emits_audio_then_one_final_chunk(self):
        """A successful stream preserves chunk order and terminates explicitly."""
        backend = _StreamingBackend()
        context = Mock()
        responses = list(
            SpeechTtsStreamServicer(backend).SynthesizeStream(_request(), context)
        )

        self.assertEqual(backend.calls, [("short text", "", 1.0)])
        self.assertEqual(
            [response.chunk.data for response in responses[:-1]],
            [b"\x00\x00", b"\x01\x00"],
        )
        self.assertFalse(any(response.is_final for response in responses[:-1]))
        self.assertTrue(responses[-1].is_final)
        self.assertTrue(backend.closed)
        context.set_code.assert_not_called()

    def test_stream_failure_sets_rpc_error_without_false_final_chunk(self):
        """A later segment failure keeps earlier audio but never reports success."""
        backend = _StreamingBackend(fail_after_first=True)
        context = Mock()
        responses = list(
            SpeechTtsStreamServicer(backend).SynthesizeStream(_request(), context)
        )

        self.assertEqual([response.chunk.data for response in responses], [b"\x00\x00"])
        self.assertFalse(any(response.is_final for response in responses))
        context.set_code.assert_called_once_with(grpc.StatusCode.INTERNAL)
        context.set_details.assert_called_once_with("Tencent TTS segment 2/2 failed")
        self.assertTrue(backend.closed)

    def test_stream_invalid_input_sets_invalid_argument(self):
        """Streaming input errors use an explicit client-facing gRPC status."""
        context = Mock()
        responses = list(
            SpeechTtsStreamServicer(_RejectingBackend()).SynthesizeStream(
                _request(), context
            )
        )

        self.assertEqual(responses, [])
        context.set_code.assert_called_once_with(grpc.StatusCode.INVALID_ARGUMENT)
        context.set_details.assert_called_once_with("invalid TTS text")

    def test_stream_unexpected_value_error_sets_internal(self):
        """Backend ValueError does not mislabel an internal fault as bad input."""
        context = Mock()
        responses = list(
            SpeechTtsStreamServicer(_UnexpectedValueErrorBackend()).SynthesizeStream(
                _request(), context
            )
        )

        self.assertEqual(responses, [])
        context.set_code.assert_called_once_with(grpc.StatusCode.INTERNAL)
        context.set_details.assert_called_once_with("backend configuration is invalid")

    def test_invalid_metadata_fails_before_stream_backend_work(self):
        """Streaming validates local response metadata before provider work."""
        backend = _StreamingBackend()
        context = Mock()
        with patch.dict(
            os.environ, {"SPEECH_TTS_OUTPUT_SAMPLE_RATE": "invalid"}, clear=False
        ):
            responses = list(
                SpeechTtsStreamServicer(backend).SynthesizeStream(_request(), context)
            )

        self.assertEqual(responses, [])
        self.assertEqual(backend.calls, [])
        context.set_code.assert_called_once_with(grpc.StatusCode.INTERNAL)

    def test_client_cancellation_closes_backend_generator(self):
        """Closing the synchronous RPC iterator releases its async generator."""
        backend = _StreamingBackend()
        responses = SpeechTtsStreamServicer(backend).SynthesizeStream(
            _request(), Mock()
        )
        first = next(responses)
        self.assertEqual(first.chunk.data, b"\x00\x00")
        responses.close()
        self.assertTrue(backend.closed)

    def test_inactive_context_stops_before_requesting_another_chunk(self):
        """Server-side cancellation does not start additional backend work."""
        backend = _StreamingBackend()
        context = Mock()
        context.is_active.side_effect = (True, False)
        responses = list(
            SpeechTtsStreamServicer(backend).SynthesizeStream(_request(), context)
        )

        self.assertEqual([response.chunk.data for response in responses], [b"\x00\x00"])
        self.assertFalse(any(response.is_final for response in responses))
        self.assertTrue(backend.closed)


if __name__ == "__main__":
    unittest.main()
