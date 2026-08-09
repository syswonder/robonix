# SPDX-License-Identifier: MulanPSL-2.0
import math
from types import SimpleNamespace

import numpy as np

from scene_service import goal_planner
from scene_service.robot_geometry import RobotFootprint


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
    original = goal_planner._footprint_clear

    def counted_footprint_clear(*args, **kwargs):
        """Count geometry checks while preserving the production result."""
        nonlocal geometry_checks
        geometry_checks += 1
        return original(*args, **kwargs)

    monkeypatch.setattr(
        goal_planner,
        "_footprint_clear",
        counted_footprint_clear,
    )
    footprint = RobotFootprint(
        points=((-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1)),
        base_frame="fixture_base",
        inscribed_radius_m=0.1,
        circumscribed_radius_m=math.sqrt(0.02),
    )
    result = goal_planner.room_goal(
        _grid(), room, footprint, yaw_candidates=(0.0,)
    )

    assert result is not None
    x, y, _yaw = result
    assert math.hypot(x - 4.0, y - 4.0) <= 0.05
    assert geometry_checks == 1
