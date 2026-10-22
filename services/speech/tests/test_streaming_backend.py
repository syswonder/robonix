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
