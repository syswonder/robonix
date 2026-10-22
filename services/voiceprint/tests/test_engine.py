# SPDX-License-Identifier: MulanPSL-2.0
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


class EmbeddingProviderTest(unittest.TestCase):
    def test_provider_selection(self):
        source = Path(__file__).resolve().parents[1] / "voiceprint_service/engine.py"
        sherpa = SimpleNamespace(
            SpeakerEmbeddingExtractorConfig=Mock(), SpeakerEmbeddingExtractor=Mock(),
        )
        spec = importlib.util.spec_from_file_location("embedding_engine_test", source)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"sherpa_onnx": sherpa}):
            spec.loader.exec_module(module)
        with tempfile.NamedTemporaryFile() as model:
            for provider in (None, "cpu", "cuda", "coreml"):
                with self.subTest(provider=provider):
                    env = {k: v for k, v in os.environ.items() if k != "VOICEPRINT_PROVIDER"}
                    if provider is not None:
                        env["VOICEPRINT_PROVIDER"] = provider
                    with patch.dict(os.environ, env, clear=True):
                        engine = module.SpeakerEmbeddingEngine(Path(model.name))
                    self.assertEqual(
                        sherpa.SpeakerEmbeddingExtractorConfig.call_args.kwargs["provider"],
                        provider or "cpu",
                    )
                    self.assertIs(engine.extractor, sherpa.SpeakerEmbeddingExtractor.return_value)
