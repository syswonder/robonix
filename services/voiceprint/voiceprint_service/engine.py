# SPDX-License-Identifier: MulanPSL-2.0
"""ECAPA-TDNN embedding extractor.

Wraps SpeechBrain's ``SpeakerRecognition`` recipe (model
``speechbrain/spkrec-ecapa-voxceleb``) to produce a 192-dimension speaker
embedding from either a file path or a raw PCM byte buffer. The model is
resolved through ModelScope; ``scripts/build.sh`` pre-warms its cache during
the package build so normal runtime startup does not need a network download.
"""
from __future__ import annotations

import io
import logging
import os
import wave
from pathlib import Path
from typing import Optional

# ModelScope repo holding a complete mirror of speechbrain/spkrec-ecapa-voxceleb
# (hyperparams.yaml + all .ckpt + label_encoder.txt). We fetch the weights from
# ModelScope rather than HuggingFace because huggingface_hub's metadata HEAD only
# follows same-host redirects, and the reachable hf-mirror.com bounces
# resolve/main to huggingface.co with a cross-host 308 that no hf_hub version
# follows — so some HF paths fail in restricted runner environments. ModelScope's SDK has no such
# issue (it's also where the speech service pulls FunASR). Pulled once into the
# local ModelScope cache; build.sh pre-warms it so runtime startup is offline.
_MODELSCOPE_ID = "speechbrain/spkrec-ecapa-voxceleb"

import numpy as np
import soundfile as sf
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
        # need to drag in speechbrain / modelscope just to import this module.
        from speechbrain.inference.speaker import SpeakerRecognition

        self.device = _resolve_device(device)
        self.savedir = Path(savedir) if savedir else _DEFAULT_SAVEDIR
        self.savedir.mkdir(parents=True, exist_ok=True)
        # Resolve the model from ModelScope (returns a local snapshot dir with
        # the full repo). The upstream hyperparams file still names its
        # Hugging Face repository in `pretrained_path`, so pass an explicit
        # override as well as the local source. Without it SpeechBrain fetches
        # checkpoint metadata from Hugging Face despite the local snapshot.
        source = self._resolve_source()
        log.info("Loading ECAPA-TDNN on %s (source=%s, savedir=%s)", self.device, source, self.savedir)
        self.model = SpeakerRecognition.from_hparams(
            source=source,
            savedir=str(self.savedir),
            overrides={"pretrained_path": str(source)},
            run_opts={"device": self.device},
        )
        log.info("ECAPA-TDNN loaded (embedding_dim=192)")

    @staticmethod
    def _resolve_source() -> str:
        # Returns a local directory holding the ECAPA-TDNN repo, fetched from
        # ModelScope (cached under ~/.cache/modelscope). speechbrain's
        # from_hparams reads weights directly from this dir, so no HF fetch.
        from modelscope.hub.snapshot_download import snapshot_download

        return snapshot_download(_MODELSCOPE_ID)

    # -- public extraction -------------------------------------------------

    def extract_from_file(self, path: str) -> np.ndarray:
        # Do not use torchaudio.load here: current torchaudio releases route
        # it through optional torchcodec, which is not needed for the WAV/FLAC
        # files supported by this service. soundfile is already a declared
        # runtime dependency and keeps build-time model verification sealed.
        samples, _ = sf.read(path, dtype="float32", always_2d=True)
        signal = torch.from_numpy(samples.T)
        if signal.shape[0] > 1:
            signal = signal.mean(dim=0, keepdim=True)
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

    def close(self) -> None:
        """Release model references and return unused CUDA cache to Torch."""
        if hasattr(self, "model"):
            del self.model
        if self.device.startswith("cuda") and torch.cuda.is_available():
            torch.cuda.empty_cache()
        log.info("ECAPA-TDNN released from %s", self.device)

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
