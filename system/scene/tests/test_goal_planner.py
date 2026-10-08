# SPDX-License-Identifier: MulanPSL-2.0
import math
from types import SimpleNamespace

import numpy as np

from scene_service import goal_planner
from scene_service.goal_planner import object_goal, room_goal, room_yaw_candidates
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
    headings = room_yaw_candidates(room)
    assert room_goal(
        grid, room, _footprint(0.1, 0.1), yaw_candidates=headings
    ) is not None
    assert room_goal(
        grid, room, _footprint(0.6, 0.6), yaw_candidates=headings
    ) is None


def test_room_goal_rotates_asymmetric_go2_footprint_to_fit_corridor():
    """An elongated Go2-like base must not be rejected at a fixed yaw of zero."""
    grid = _grid()
    room = [(-0.18, -0.5), (0.18, -0.5), (0.18, 0.5), (-0.18, 0.5)]
    result = room_goal(
        grid,
        room,
        _footprint(0.4, 0.1),
        yaw_candidates=room_yaw_candidates(room),
    )
    assert result is not None
    _, _, yaw = result
    assert abs(abs(yaw) - math.pi / 2.0) < 1e-9


def test_room_goal_stops_after_first_centroid_safe_pose(monkeypatch):
    """A large free room must not trigger exhaustive footprint checks."""
    grid = _grid(width=220, height=220, resolution=0.05)
    room = [(-1.0, -1.0), (9.0, -1.0), (9.0, 9.0), (-1.0, 9.0)]
    footprint_checks = 0
    original = goal_planner._footprint_clear

    def counted_footprint_clear(*args, **kwargs):
        """Count collision checks while preserving the production result."""
        nonlocal footprint_checks
        footprint_checks += 1
        return original(*args, **kwargs)

    monkeypatch.setattr(goal_planner, "_footprint_clear", counted_footprint_clear)
    result = room_goal(
        grid,
        room,
        _footprint(0.3, 0.2),
        yaw_candidates=room_yaw_candidates(room),
    )

    assert result is not None
    x, y, _yaw = result
    assert math.hypot(x - 4.0, y - 4.0) <= grid.info.resolution
    assert footprint_checks == 1


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


def test_object_goal_approaches_from_the_robot_side_of_a_wall():
    """An object detected on a wall is approached from where the robot is.

    Unexplored cells stay usable, so the unknown space behind the wall also
    fits the footprint and can be a little nearer the object. Taking it sends
    the robot looking for a way round the wall.
    """
    grid = _grid()
    data = np.zeros((30, 30), dtype=np.int8)
    data[20, :] = 100  # the wall: y in [0.5, 0.6)
    data[21:, :] = -1  # never observed behind it
    grid.data = data.tobytes()
    target_x, target_y = 0.0, 0.58  # on the wall, nearer its far face
    robot_x, robot_y = 0.0, -1.0
    result = object_goal(
        grid,
        target_x=target_x,
        target_y=target_y,
        preferred_approach_yaw=math.atan2(target_y - robot_y, target_x - robot_x),
        minimum_standoff_m=0.19,
        footprint=_footprint(0.1, 0.1),
    )
    assert result is not None
    _x, y, _yaw = result
    assert y < 0.5


def test_object_goal_falls_back_to_the_far_side_when_the_near_side_is_full():
    """With no room on the robot's side the far side is still returned."""
    grid = _grid()
    data = np.zeros((30, 30), dtype=np.int8)
    data[:21, :] = 100  # the wall and everything on the robot's side of it
    grid.data = data.tobytes()
    target_x, target_y = 0.0, 0.58
    result = object_goal(
        grid,
        target_x=target_x,
        target_y=target_y,
        preferred_approach_yaw=math.atan2(target_y + 1.0, target_x),
        minimum_standoff_m=0.19,
        footprint=_footprint(0.1, 0.1),
    )
    assert result is not None
    _x, y, _yaw = result
    assert y > 0.6


def test_room_goal_accepts_ros_signed_int8_sequence():
    """ROS OccupancyGrid commonly exposes unknown cells as integer -1 values."""
    grid = _grid(fill=-1)
    grid.data = [-1] * (grid.info.width * grid.info.height)
    room = [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]
    assert room_goal(
        grid,
        room,
        _footprint(0.1, 0.1),
        yaw_candidates=room_yaw_candidates(room),
    ) is None
