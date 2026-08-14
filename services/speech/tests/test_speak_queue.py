# SPDX-License-Identifier: MulanPSL-2.0
import base64
import os
import threading
import unittest
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

from speech_service import service as service_module
from speech_service.service import _speaker_lock, speak
from speech_service.tencent_cloud import TencentTTSBackend

_HAN_CHAR = "\N{CJK UNIFIED IDEOGRAPH-7532}"
_SHORT_CJK_TEXT = _HAN_CHAR * 6


class SpeakerQueueTest(unittest.TestCase):
    def test_same_provider_serializes_callers(self):
        lock = _speaker_lock("robot-speaker")
        entered = threading.Event()

        def waiter():
            with _speaker_lock("robot-speaker"):
                entered.set()

        with lock:
            thread = threading.Thread(target=waiter)
            thread.start()
            self.assertFalse(entered.wait(0.05))
        self.assertTrue(entered.wait(0.5))
        thread.join(timeout=0.5)
        self.assertFalse(thread.is_alive())

    def test_different_providers_do_not_block_each_other(self):
        first = _speaker_lock("robot-speaker-a")
        second = _speaker_lock("robot-speaker-b")
        self.assertIsNot(first, second)
        with first:
            self.assertTrue(second.acquire(blocking=False))
            second.release()


class SpeakTencentTest(unittest.TestCase):
    def _run_speak(self, text):
        """Run speech/speak through Tencent TTS and capture provider/playback data."""
        provider_payloads = []
        played_frames = []
        environment = {
            "TENCENTCLOUD_SECRET_ID": "test-id",
            "TENCENTCLOUD_SECRET_KEY": "test-key",
            "TENCENT_TTS_MAX_CHARS": "140",
        }

        def provider_response(payload):
            if len(payload["Text"]) > 140:
                raise RuntimeError("UnsupportedOperation.TextTooLong")
            provider_payloads.append(payload)
            pcm = b"\x00\x00" * len(payload["Text"])
            return {"Audio": base64.b64encode(pcm).decode("ascii")}

        speaker_stub = Mock()
        speaker_stub.Speaker.side_effect = lambda frames: played_frames.extend(frames)
        speaker_channel = MagicMock()
        capability = SimpleNamespace(provider_id="test-speaker")
        connection = nullcontext(SimpleNamespace(endpoint="test-endpoint"))

        with patch.dict(os.environ, environment, clear=True):
            backend = TencentTTSBackend()
            backend._post_json = provider_response
            with (
                patch.object(
                    service_module.ATLAS, "find_capability", return_value=[capability]
                ),
                patch.object(service_module._tts_servicer, "tts_backend", backend),
                patch.object(
                    service_module.speech, "connect_capability", return_value=connection
                ),
                patch.object(
                    service_module.grpc,
                    "insecure_channel",
                    return_value=speaker_channel,
                ),
                patch.object(
                    service_module.contracts_grpc,
                    "RobonixPrimitiveAudioSpeakerStub",
                    return_value=speaker_stub,
                ),
            ):
                response = speak(SimpleNamespace(text=text, target="test-speaker"))

        speaker_channel.__enter__.assert_called_once_with()
        speaker_channel.__exit__.assert_called_once()

        return provider_payloads, played_frames, response

    def test_speak_plays_every_long_text_segment_in_order(self):
        """speech/speak forwards complete PCM joined from safe Tencent requests."""
        text = _HAN_CHAR * 244
        provider_payloads, played_frames, response = self._run_speak(text)

        sent = [payload["Text"] for payload in provider_payloads]
        self.assertEqual([len(segment) for segment in sent], [140, 104])
        self.assertEqual("".join(sent), text)
        self.assertEqual(
            b"".join(frame.data for frame in played_frames), b"\x00\x00" * 244
        )
        self.assertTrue(response.ok)

    def test_speak_keeps_short_text_single_request_behavior(self):
        """speech/speak still sends a short utterance once and plays it fully."""
        text = _SHORT_CJK_TEXT
        provider_payloads, played_frames, response = self._run_speak(text)

        self.assertEqual([payload["Text"] for payload in provider_payloads], [text])
        self.assertEqual(
            b"".join(frame.data for frame in played_frames),
            b"\x00\x00" * len(text),
        )
        self.assertTrue(response.ok)


if __name__ == "__main__":
    unittest.main()
