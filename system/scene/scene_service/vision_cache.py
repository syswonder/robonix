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

    The normalized RGB RMS distance is deliberately measured on a 32x32
    sample. JPEG artefacts and sensor noise disappear at that scale while
    object motion and camera-view changes remain visible.
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
    if not left_sample or len(left_sample) != len(right_sample):
        return False
    squared = sum((a - b) * (a - b) for a, b in zip(left_sample, right_sample))
    rms = math.sqrt(squared / len(left_sample)) / 255.0
    bounded_threshold = threshold if math.isfinite(threshold) else 0.0
    return rms <= min(1.0, max(0.0, bounded_threshold))
