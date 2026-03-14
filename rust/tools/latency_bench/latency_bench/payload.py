# SPDX-License-Identifier: MulanPSL-2.0
"""Shared payload generator for latency benchmark."""

import random
from typing import Optional

_SEED = 42
_SIZES = (64, 256, 1024)


def get_payload(size: int = 64) -> bytes:
    """Generate deterministic binary payload of given size (bytes)."""
    if size not in _SIZES:
        raise ValueError(f"Payload size must be one of {_SIZES}, got {size}")
    rng = random.Random(_SEED)
    return bytes(rng.randbytes(size))


def validate_response(request: bytes, response: bytes) -> bool:
    """Verify server echoed the request correctly."""
    return request == response
