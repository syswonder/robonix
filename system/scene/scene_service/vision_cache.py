# SPDX-License-Identifier: MulanPSL-2.0
"""Small, dependency-light helpers for suppressing duplicate vision calls."""

from __future__ import annotations

import hashlib
import io
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
from PIL import Image

_SAMPLE_SIZE = (32, 32)
_BLOCK_SIZE = 4
_RESAMPLE = getattr(Image, "Resampling", Image).BILINEAR


@dataclass(frozen=True)
class FrameFingerprint:
    """Exact and perceptual representations of one camera frame."""

    exact_digest: bytes
    perceptual: Optional[tuple[bytes, int, int]]


@dataclass
class InferenceCounters:
    """Counters shared by the scene vision inference paths."""

    processed: int = 0
    skipped: int = 0
    retried: int = 0
    failed: int = 0

    def as_dict(self) -> dict[str, int]:
        """Return a stable, log-friendly snapshot of all inference counts."""
        return {
            "processed": self.processed,
            "skipped": self.skipped,
            "retried": self.retried,
            "failed": self.failed,
        }


def _image_sample(image: Image.Image) -> tuple[bytes, int, int]:
    """Downsample an image for noise-tolerant comparison and keep its shape."""
    width, height = image.size
    if width <= 0 or height <= 0:
        raise ValueError("empty image")
    sample = image.convert("RGB").resize(_SAMPLE_SIZE, _RESAMPLE)
    return sample.tobytes(), width, height


def fingerprint_jpeg(payload: bytes) -> FrameFingerprint:
    """Fingerprint encoded image bytes, tolerating decode failures."""

    exact = hashlib.sha256(payload).digest()
    try:
        with Image.open(io.BytesIO(payload)) as image:
            perceptual = _image_sample(image)
    except Exception:  # noqa: BLE001
        perceptual = None
    return FrameFingerprint(exact, perceptual)


def fingerprint_bgr(frame: np.ndarray) -> FrameFingerprint:
    """Fingerprint an OpenCV-style BGR frame."""

    array = np.asarray(frame)
    exact = hashlib.sha256(
        repr((array.shape, array.dtype.str)).encode("ascii")
        + np.ascontiguousarray(array).tobytes()
    ).digest()
    try:
        if array.ndim != 3 or array.shape[2] < 3:
            raise ValueError("expected a BGR image")
        rgb = np.ascontiguousarray(array[..., :3][..., ::-1])
        if rgb.dtype != np.uint8:
            rgb = np.clip(rgb, 0, 255).astype(np.uint8)
        perceptual = _image_sample(Image.fromarray(rgb))
    except Exception:  # noqa: BLE001
        perceptual = None
    return FrameFingerprint(exact, perceptual)


def frames_equivalent(
    left: Optional[FrameFingerprint],
    right: Optional[FrameFingerprint],
    *,
    threshold: float,
) -> bool:
    """Return true for exact matches and low-level compression/noise changes.

    The 32x32 sample is divided into 4x4 blocks and the largest normalized RGB
    RMS distance is compared. Downsampling absorbs JPEG artefacts and sensor
    noise, while the block maximum keeps small local objects from disappearing
    into a whole-frame average.
    """

    if left is None or right is None:
        return False
    if left.exact_digest == right.exact_digest:
        return True
    if left.perceptual is None or right.perceptual is None:
        return False
    left_sample, left_width, left_height = left.perceptual
    right_sample, right_width, right_height = right.perceptual
    if (left_width, left_height) != (right_width, right_height):
        return False
    expected_sample_length = _SAMPLE_SIZE[0] * _SAMPLE_SIZE[1] * 3
    if (
        len(left_sample) != expected_sample_length
        or len(right_sample) != expected_sample_length
    ):
        return False
    left_array = np.frombuffer(left_sample, dtype=np.uint8).astype(np.float32)
    right_array = np.frombuffer(right_sample, dtype=np.uint8).astype(np.float32)
    diff = left_array.reshape(
        _SAMPLE_SIZE[1], _SAMPLE_SIZE[0], 3
    ) - right_array.reshape(_SAMPLE_SIZE[1], _SAMPLE_SIZE[0], 3)
    blocks = np.square(diff).reshape(
        _SAMPLE_SIZE[1] // _BLOCK_SIZE,
        _BLOCK_SIZE,
        _SAMPLE_SIZE[0] // _BLOCK_SIZE,
        _BLOCK_SIZE,
        3,
    )
    block_mse = blocks.mean(axis=(1, 3, 4))
    rms = math.sqrt(float(block_mse.max())) / 255.0
    bounded_threshold = threshold if math.isfinite(threshold) else 0.0
    return rms <= min(1.0, max(0.0, bounded_threshold))
