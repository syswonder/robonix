# SPDX-License-Identifier: MulanPSL-2.0
"""CAM++ embeddings; model weights are downloaded at build time."""
from __future__ import annotations

import io
import logging
import os
import wave
from pathlib import Path
from typing import Optional

import numpy as np
import sherpa_onnx
import soundfile as sf

log = logging.getLogger(__name__)

_DEFAULT_MODEL = (
    Path(__file__).resolve().parent.parent / "rbnx-build" / "models" / "campplus_zh.onnx"
)


class SpeakerEmbeddingEngine:
    """sherpa-onnx CAM++ speaker-embedding model (192-dim output)."""

    def __init__(self, model_path: Optional[Path] = None) -> None:
        path = Path(model_path or os.environ.get("VOICEPRINT_MODEL", "") or _DEFAULT_MODEL)
        if not path.is_file():
            raise FileNotFoundError(f"speaker-embedding model not found: {path}")
        provider = os.environ.get("VOICEPRINT_PROVIDER", "cpu")
        config = sherpa_onnx.SpeakerEmbeddingExtractorConfig(
            model=str(path),
            num_threads=1,
            provider=provider,
        )
        self.model_path = path
        self.extractor = sherpa_onnx.SpeakerEmbeddingExtractor(config)
        log.info(
            "CAM++ speaker embedding loaded from %s (dim=%d, requested provider=%s)",
            path, self.extractor.dim, provider,
        )

    # -- public extraction -------------------------------------------------

    def extract_from_file(self, path: str) -> np.ndarray:
        samples, sample_rate = sf.read(path, dtype="float32", always_2d=True)
        if samples.shape[1] > 1:
            samples = samples.mean(axis=1)
        else:
            samples = samples[:, 0]
        return self._embed(samples, sample_rate)

    def extract_from_pcm(
        self,
        pcm_bytes: bytes,
        encoding: str = "pcm_s16le",
        sample_rate: int = 16000,
    ) -> np.ndarray:
        samples, sample_rate = self._decode(
            pcm_bytes, encoding or "pcm_s16le", sample_rate or 16000,
        )
        return self._embed(samples, sample_rate)

    def close(self) -> None:
        """Release the model reference; ONNX Runtime frees its arena with it."""
        if hasattr(self, "extractor"):
            del self.extractor
        log.info("CAM++ speaker embedding released")

    # -- helpers -----------------------------------------------------------

    def _embed(self, samples: np.ndarray, sample_rate: int) -> np.ndarray:
        stream = self.extractor.create_stream()
        stream.accept_waveform(sample_rate, samples)
        stream.input_finished()
        embedding = self.extractor.compute(stream)
        return np.asarray(embedding, dtype=np.float32)

    @staticmethod
    def _decode(data: bytes, encoding: str, sample_rate: int) -> tuple[np.ndarray, int]:
        # The sherpa-onnx stream resamples; unknown encodings retain the
        # raw s16le fallback for compatibility.
        if encoding == "wav":
            with wave.open(io.BytesIO(data), "rb") as wf:
                raw = wf.readframes(wf.getnframes())
                sample_rate = wf.getframerate()
                samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
        else:
            samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
        return samples, sample_rate
