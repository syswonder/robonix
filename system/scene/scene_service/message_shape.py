# SPDX-License-Identifier: MulanPSL-2.0
"""What makes an upstream image or occupancy grid well formed.

ROS 2 does not check that a message is self-consistent. An `OccupancyGrid`
whose `data` is shorter than `width * height`, or an `Image` whose buffer does
not match its declared size and encoding, is type-valid and will be delivered
to every subscriber. Both then reach code that reshapes the buffer, and
`reshape` raises.

That mattered because the raising happened inside request handlers: the Scene
debug UI answered 500 on `/api/state` and `/api/camera` for as long as the
malformed message stayed the latest one, and the `goal_near` capability failed
the same way through the MCP path. Anyone able to publish on `/map` could hold
the interface down.

The rule belongs in one place rather than at each decode site, so it cannot be
applied at one and forgotten at another. These functions are pure and import
nothing, so they can be tested without a ROS or robonix runtime.

Reported as issues #226 and #227.
"""

from __future__ import annotations

# Bytes per pixel for the encodings Scene decodes. An encoding that is absent
# is one Scene does not render, and its buffer length is not this module's
# business.
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
    """Whether a grid can be reshaped to its declared size.

    A zero dimension is rejected too: it is not decodable, and treating it as
    "empty but fine" pushes an empty array into code that assumes cells exist.
    """
    if int(width) <= 0 or int(height) <= 0:
        return False
    return int(data_len) == occupancy_cells_expected(width, height)


def image_bytes_expected(width: int, height: int, encoding: str) -> int | None:
    """How many bytes an image of this size and encoding must carry.

    `None` for an encoding Scene does not decode — the caller rejects those on
    their own grounds, and guessing a size for them would be inventing one.
    """
    per_pixel = _BYTES_PER_PIXEL.get((encoding or "").lower())
    if per_pixel is None:
        return None
    return int(width) * int(height) * per_pixel


def image_is_well_formed(
    width: int, height: int, encoding: str, data_len: int
) -> bool:
    """Whether an image buffer matches its declared size and encoding.

    An unsupported encoding returns True: this function answers "is the buffer
    the right length", and for an encoding with no defined length that question
    does not apply. The decoder still rejects it.
    """
    if int(width) <= 0 or int(height) <= 0:
        return False
    expected = image_bytes_expected(width, height, encoding)
    if expected is None:
        return True
    return int(data_len) == expected
