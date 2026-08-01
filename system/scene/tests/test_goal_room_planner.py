# SPDX-License-Identifier: MulanPSL-2.0
import math
from types import SimpleNamespace

import numpy as np

from scene_service import mcp_tools


def _grid(width=220, height=220, resolution=0.05):
    """Build the OccupancyGrid subset consumed by the room planner."""
    data = np.zeros((height, width), dtype=np.int8)
    return SimpleNamespace(
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-1.5, y=-1.5),
            ),
        ),
        data=data.tobytes(),
    )


def test_room_goal_stops_after_first_centroid_safe_cell(monkeypatch):
    """A large free room must not trigger exhaustive geometry checks."""
    room = [(-1.0, -1.0), (9.0, -1.0), (9.0, 9.0), (-1.0, 9.0)]
    geometry_checks = 0
    original = mcp_tools.disc_inside_polygon

    def counted_disc_inside_polygon(*args, **kwargs):
        """Count geometry checks while preserving the production result."""
        nonlocal geometry_checks
        geometry_checks += 1
        return original(*args, **kwargs)

    monkeypatch.setattr(
        mcp_tools,
        "disc_inside_polygon",
        counted_disc_inside_polygon,
    )
    result = mcp_tools._occupancy_room_goal(_grid(), room)

    assert result is not None
    x, y = result
    assert math.hypot(x - 4.0, y - 4.0) <= 0.05
    assert geometry_checks == 1
