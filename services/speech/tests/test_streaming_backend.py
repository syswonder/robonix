# SPDX-License-Identifier: MulanPSL-2.0
import os
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from speech_service.service import SherpaStreamingBackend


class StreamingProviderTest(unittest.TestCase):
    def test_provider_selection(self):
        for provider in (None, "cpu", "cuda", "xnnpack"):
            with self.subTest(provider=provider):
                recognizer = Mock()
                sherpa = SimpleNamespace(OnlineRecognizer=recognizer)
                env = {k: v for k, v in os.environ.items() if k != "SHERPA_ASR_PROVIDER"}
                if provider is not None:
                    env["SHERPA_ASR_PROVIDER"] = provider
                with patch.dict(os.environ, env, clear=True), patch.dict(
                    sys.modules, {"sherpa_onnx": sherpa, "numpy": np},
                ):
                    backend = SherpaStreamingBackend()
                self.assertEqual(
                    recognizer.from_paraformer.call_args.kwargs["provider"],
                    provider or "cpu",
                )
                self.assertIs(backend._recognizer, recognizer.from_paraformer.return_value)


class StreamingTranscriptTest(unittest.TestCase):
    def backend(self, results):
        backend = SherpaStreamingBackend.__new__(SherpaStreamingBackend)
        backend._np = np
        stream = Mock()
        backend._recognizer = Mock()
        backend._recognizer.create_stream.return_value = stream
        backend._recognizer.is_ready.return_value = False
        backend._recognizer.get_result.side_effect = results
        return backend

    def test_final_is_complete_with_and_without_revision(self):
        for final in ("open the kitchen", "open the bedroom", ""):
            with self.subTest(final=final):
                backend = self.backend(["open the ", "open the kitchen", final])
                events = list(backend.recognize_stream([b"\0\0", b"\0\0"]))
                self.assertEqual([event["text"] for event in events[:-1]], ["open the ", "kitchen"])
                self.assertTrue(events[-1]["is_final"])
                self.assertEqual(events[-1]["text"], final)
                stream = backend._recognizer.create_stream.return_value
                tail = stream.accept_waveform.call_args.args[1]
                self.assertGreater(tail.size, 0)
                self.assertTrue(np.all(tail == 0))
                self.assertEqual(stream.mock_calls[-1][0], "input_finished")

    def test_revised_transcript_over_real_grpc(self):
        from concurrent.futures import ThreadPoolExecutor
        import grpc
        import asr_pb2
        import audio_pb2
        import robonix_contracts_pb2_grpc as contracts
        from speech_service.service import SpeechAsrStreamServicer
        backend = self.backend(["open the kitchen", "open the bedroom"])
        server = grpc.server(ThreadPoolExecutor(max_workers=1))
        contracts.add_RobonixServiceSpeechAsrStreamServicer_to_server(
            SpeechAsrStreamServicer(backend), server)
        port = server.add_insecure_port("127.0.0.1:0")
        server.start()
        try:
            with grpc.insecure_channel(f"127.0.0.1:{port}",
                                       options=(("grpc.enable_http_proxy", 0),)) as channel:
                stub = contracts.RobonixServiceSpeechAsrStreamStub(channel)
                events = list(stub.RecognizeStream(iter([asr_pb2.AsrAudioChunk(
                    chunk=audio_pb2.AudioChunk(data=b"\0\0", duration_s=1/16000))]), timeout=5))
            self.assertEqual(events[-1].text, "open the bedroom")
            self.assertTrue(events[-1].is_final)
            self.assertFalse(any(e.error for e in events))
        finally:
            server.stop(0).wait()

    def test_manifest_configuration_overrides_environment(self):
        from speech_service.service import _apply_cfg_to_env
        with patch.dict(os.environ, {"SHERPA_ASR_PROVIDER": "cuda"}):
            _apply_cfg_to_env({"sherpa_asr_dir": "/models/test", "sherpa_asr_threads": 3,
                               "sherpa_asr_provider": "cpu"})
            self.assertEqual(os.environ["SHERPA_ASR_PROVIDER"], "cpu")
            self.assertEqual(os.environ["SHERPA_ASR_THREADS"], "3")
            self.assertEqual(os.environ["SHERPA_ASR_DIR"], "/models/test")
