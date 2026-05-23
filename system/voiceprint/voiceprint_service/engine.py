# SPDX-License-Identifier: MulanPSL-2.0
"""ECAPA-TDNN embedding extractor.

Wraps SpeechBrain's ``SpeakerRecognition`` recipe (model
``speechbrain/spkrec-ecapa-voxceleb``) to produce a 192-dimension speaker
embedding from either a file path or a raw PCM byte buffer. The model is
fetched on first use; ``scripts/build.sh`` pre-warms the cache during the
package build so runtime startup never reaches out to HuggingFace.
"""
from __future__ import annotations

import io
import logging
import os
import wave
from pathlib import Path
from typing import Optional

import numpy as np
import torch
import torchaudio

log = logging.getLogger(__name__)

_DEFAULT_SAVEDIR = (
    Path(__file__).resolve().parent.parent / "rbnx-build" / "models" / "spkrec-ecapa-voxceleb"
)


def _resolve_device(pref: Optional[str] = None) -> str:
    if pref:
        return pref
    env = os.environ.get("VOICEPRINT_DEVICE")
    if env:
        return env
    return "cuda:0" if torch.cuda.is_available() else "cpu"


class EcapaTdnnEngine:
    """SpeechBrain ECAPA-TDNN speaker-embedding model."""

    def __init__(self, device: Optional[str] = None, savedir: Optional[Path] = None) -> None:
        # Imported lazily so unit tests that monkey-patch the engine do not
        # need to drag in speechbrain just to import this module.
        from speechbrain.inference.speaker import SpeakerRecognition

        self.device = _resolve_device(device)
        self.savedir = Path(savedir) if savedir else _DEFAULT_SAVEDIR
        self.savedir.mkdir(parents=True, exist_ok=True)
        log.info("Loading ECAPA-TDNN on %s (savedir=%s)", self.device, self.savedir)
        self.model = SpeakerRecognition.from_hparams(
            source="speechbrain/spkrec-ecapa-voxceleb",
            savedir=str(self.savedir),
            run_opts={"device": self.device},
        )
        log.info("ECAPA-TDNN loaded (embedding_dim=192)")

    # -- public extraction -------------------------------------------------

    def extract_from_file(self, path: str) -> np.ndarray:
        signal, _ = torchaudio.load(path)
        emb = self.model.encode_batch(signal.to(self.device))
        return emb.cpu().numpy().flatten()

    def extract_from_pcm(
        self,
        pcm_bytes: bytes,
        encoding: str = "pcm_s16le",
        sample_rate: int = 16000,
    ) -> np.ndarray:
        audio_tensor = self._decode(pcm_bytes, encoding or "pcm_s16le", sample_rate or 16000)
        emb = self.model.encode_batch(audio_tensor.unsqueeze(0).to(self.device))
        return emb.cpu().numpy().flatten()

    # -- helpers -----------------------------------------------------------

    @staticmethod
    def _decode(data: bytes, encoding: str, sample_rate: int) -> torch.Tensor:
        """Decode raw bytes into a 16 kHz mono float32 torch tensor."""
        if encoding == "wav":
            with wave.open(io.BytesIO(data), "rb") as wf:
                raw = wf.readframes(wf.getnframes())
                sample_rate = wf.getframerate()
                audio = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
        else:
            # pcm_s16le (default) and any unknown encoding fall through to the
            # raw s16le interpretation; mirrors the prototype's permissive
            # behaviour rather than rejecting unknown labels.
            audio = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

        tensor = torch.from_numpy(audio)
        if sample_rate != 16000:
            tensor = torchaudio.functional.resample(tensor, sample_rate, 16000)
        return tensor
