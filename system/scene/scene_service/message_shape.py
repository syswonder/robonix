# SPDX-License-Identifier: MulanPSL-2.0
"""Whether an upstream image or occupancy grid is self-consistent.

ROS delivers a type-valid message whose buffer does not match its declared
size, and the reshape that follows raises. Every decode site asks here first
(issues #226, #227).
"""

from __future__ import annotations

# Bytes per pixel for the encodings Scene decodes.
_BYTES_PER_PIXEL = {
    "rgb8": 3,
    "bgr8": 3,
    "rgba8": 4,
    "bgra8": 4,
    "mono8": 1,
    "32fc1": 4,
    "16uc1": 2,
}


def occupancy_cells_expected(width: int, height: int) -> int:
    """How many cells a grid of this size must carry."""
    return int(width) * int(height)


def occupancy_grid_is_well_formed(width: int, height: int, data_len: int) -> bool:
    """Whether a grid reshapes to its declared, non-zero size."""
    if int(width) <= 0 or int(height) <= 0:
        return False
    return int(data_len) == occupancy_cells_expected(width, height)


def image_bytes_expected(width: int, height: int, encoding: str) -> int | None:
    """Expected buffer length, or None for an encoding Scene does not decode."""
    per_pixel = _BYTES_PER_PIXEL.get((encoding or "").lower())
    if per_pixel is None:
        return None
    return int(width) * int(height) * per_pixel


def image_is_well_formed(
    width: int, height: int, encoding: str, data_len: int
) -> bool:
    """Whether the buffer length matches; True for an encoding with no
    defined length (the decoder rejects those itself)."""
    if int(width) <= 0 or int(height) <= 0:
        return False
    expected = image_bytes_expected(width, height, encoding)
    if expected is None:
        return True
    return int(data_len) == expected
