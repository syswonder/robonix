# SPDX-License-Identifier: MulanPSL-2.0
from types import SimpleNamespace

import numpy as np

from scene_service.goal_planner import object_goal, room_goal
from scene_service.robot_geometry import RobotFootprint


def _grid(width=30, height=30, resolution=0.1, fill=0):
    """Build the OccupancyGrid subset consumed by the pure planner."""
    data = np.full((height, width), fill, dtype=np.int8)
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


def _footprint(half_x, half_y):
    """Return a rectangular Soma footprint fixture."""
    return RobotFootprint(
        points=(
            (-half_x, -half_y),
            (half_x, -half_y),
            (half_x, half_y),
            (-half_x, half_y),
        ),
        base_frame="fixture_base",
        inscribed_radius_m=min(half_x, half_y),
        circumscribed_radius_m=(half_x**2 + half_y**2) ** 0.5,
    )


def test_room_goal_changes_with_soma_footprint():
    """The same room must accept a small robot and reject an oversized one."""
    grid = _grid()
    room = [(-0.3, -0.5), (0.3, -0.5), (0.3, 0.5), (-0.3, 0.5)]
    assert room_goal(grid, room, _footprint(0.1, 0.1), yaw=0.0) is not None
    assert room_goal(grid, room, _footprint(0.4, 0.1), yaw=0.0) is None


def test_object_goal_uses_complete_polygon():
    """A returned object approach pose keeps the real footprint in bounds."""
    grid = _grid(width=12, height=12)
    result = object_goal(
        grid,
        target_x=0.0,
        target_y=0.0,
        preferred_approach_yaw=0.0,
        minimum_standoff_m=0.4,
        footprint=_footprint(0.2, 0.1),
    )
    assert result is not None
    x, y, yaw = result
    assert (x * x + y * y) ** 0.5 >= 0.4
    assert np.isfinite([x, y, yaw]).all()


def test_room_goal_accepts_ros_signed_int8_sequence():
    """ROS OccupancyGrid commonly exposes unknown cells as integer -1 values."""
    grid = _grid(fill=-1)
    grid.data = [-1] * (grid.info.width * grid.info.height)
    room = [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]
    assert room_goal(grid, room, _footprint(0.1, 0.1), yaw=0.0) is None
