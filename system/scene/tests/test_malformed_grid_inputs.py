# SPDX-License-Identifier: MulanPSL-2.0
"""Scene must survive a structurally inconsistent upstream message.

Nothing in ROS 2 rejects an `OccupancyGrid` whose `data` length disagrees with
`width * height`, or an `Image` whose buffer does not match its declared size
and encoding. Both are type-valid and are delivered to every subscriber. Both
used to reach code that reshaped the buffer, and `reshape` raises.

The raising happened inside request handlers, which is what made it matter: the
debug UI answered 500 on `/api/state` and `/api/camera` for as long as the bad
message stayed the latest one, and `goal_near` failed the same way through the
MCP path. Anyone able to publish on `/map` could hold the interface down.

Both entry points now ask `message_shape` the same question, so these tests
cover the rule itself and the fact that each caller applies it.

Reported as issues #226 and #227.
"""

from types import SimpleNamespace

import numpy as np
import pytest

from scene_service import goal_planner
from scene_service.message_shape import (
    image_bytes_expected,
    image_is_well_formed,
    occupancy_grid_is_well_formed,
)


def _grid(*, width=8, height=6, resolution=0.05, cells=None):
    """An OccupancyGrid whose `data` length is independent of its declared
    size, which is the whole point of these tests."""
    if cells is None:
        cells = width * height
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
        ),
        data=np.zeros(cells, dtype=np.int8).tobytes(),
    )


# --- the rule ---------------------------------------------------------------


def test_a_grid_is_well_formed_only_at_its_declared_size():
    assert occupancy_grid_is_well_formed(8, 6, 48)
    assert not occupancy_grid_is_well_formed(8, 6, 47)
    assert not occupancy_grid_is_well_formed(8, 6, 49)


@pytest.mark.parametrize(("width", "height"), [(0, 6), (8, 0), (-1, 6)])
def test_a_grid_without_extent_is_not_well_formed(width, height):
    assert not occupancy_grid_is_well_formed(width, height, 0)


@pytest.mark.parametrize(
    ("encoding", "per_pixel"),
    [("rgb8", 3), ("bgr8", 3), ("rgba8", 4), ("bgra8", 4), ("mono8", 1),
     ("32fc1", 4), ("16uc1", 2)],
)
def test_each_decoded_encoding_states_its_own_length(encoding, per_pixel):
    assert image_bytes_expected(4, 3, encoding) == 4 * 3 * per_pixel
    assert image_is_well_formed(4, 3, encoding, 4 * 3 * per_pixel)
    assert not image_is_well_formed(4, 3, encoding, 4 * 3 * per_pixel - 1)


def test_an_encoding_scene_does_not_decode_has_no_length_to_check():
    # The decoder rejects it on its own grounds; inventing a size for it here
    # would be guessing.
    assert image_bytes_expected(4, 3, "yuv422") is None
    assert image_is_well_formed(4, 3, "yuv422", 7)


# --- the planner path (#227) ------------------------------------------------


def test_zero_resolution_is_rejected_rather_than_dividing_by_zero():
    with pytest.raises(ValueError, match="resolution"):
        goal_planner._grid_metadata(_grid(resolution=0.0))


@pytest.mark.parametrize("resolution", [-0.05, float("nan"), float("inf")])
def test_a_resolution_that_cannot_scale_anything_is_rejected(resolution):
    with pytest.raises(ValueError, match="resolution"):
        goal_planner._grid_metadata(_grid(resolution=resolution))


@pytest.mark.parametrize(("width", "height"), [(0, 6), (8, 0)])
def test_an_empty_dimension_is_rejected(width, height):
    with pytest.raises(ValueError):
        goal_planner._grid_metadata(_grid(width=width, height=height))


@pytest.mark.parametrize("cells", [10, 100])
def test_a_mismatched_buffer_is_rejected_before_reshape(cells):
    with pytest.raises(ValueError, match="cells"):
        goal_planner._grid_array(_grid(width=8, height=6, cells=cells),
                                 width=8, height=6)


def test_a_well_formed_grid_still_decodes():
    grid = _grid(width=8, height=6)
    assert goal_planner._grid_array(grid, width=8, height=6).shape == (6, 8)
