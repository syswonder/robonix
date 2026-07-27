# SPDX-License-Identifier: MulanPSL-2.0
"""Robot-footprint-aware occupancy planning for Scene spatial goals."""
from __future__ import annotations

import math
from collections.abc import Iterable, Sequence

from .geometry import point_in_polygon, polygon_centroid
from .robot_geometry import RobotFootprint

Point = tuple[float, float]


def _point_on_segment(point: Point, start: Point, end: Point) -> bool:
    px, py = point
    ax, ay = start
    bx, by = end
    cross = (px - ax) * (by - ay) - (py - ay) * (bx - ax)
    if abs(cross) > 1e-9:
        return False
    dot = (px - ax) * (bx - ax) + (py - ay) * (by - ay)
    if dot < -1e-9:
        return False
    return dot <= (bx - ax) ** 2 + (by - ay) ** 2 + 1e-9


def _contains_or_boundary(point: Point, polygon: Sequence[Point]) -> bool:
    edges = list(zip(polygon, polygon[1:] + polygon[:1]))
    return any(_point_on_segment(point, start, end) for start, end in edges) or (
        point_in_polygon(point[0], point[1], polygon)
    )


def _orientation(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    values = (
        _orientation(a, b, c),
        _orientation(a, b, d),
        _orientation(c, d, a),
        _orientation(c, d, b),
    )
    if values[0] * values[1] < 0.0 and values[2] * values[3] < 0.0:
        return True
    return (
        (abs(values[0]) <= 1e-9 and _point_on_segment(c, a, b))
        or (abs(values[1]) <= 1e-9 and _point_on_segment(d, a, b))
        or (abs(values[2]) <= 1e-9 and _point_on_segment(a, c, d))
        or (abs(values[3]) <= 1e-9 and _point_on_segment(b, c, d))
    )


def polygons_intersect(first: Sequence[Point], second: Sequence[Point]) -> bool:
    if any(_contains_or_boundary(point, second) for point in first):
        return True
    if any(_contains_or_boundary(point, first) for point in second):
        return True
    first_edges = zip(first, first[1:] + first[:1])
    second_edges = list(zip(second, second[1:] + second[:1]))
    return any(
        _segments_intersect(a, b, c, d)
        for a, b in first_edges
        for c, d in second_edges
    )


def polygon_inside_polygon(inner: Sequence[Point], outer: Sequence[Point]) -> bool:
    """Return true only when the complete inner polygon lies in outer."""
    if not inner or not outer:
        return False
    if not all(_contains_or_boundary(point, outer) for point in inner):
        return False
    outer_edges = list(zip(outer, outer[1:] + outer[:1]))
    for start, end in zip(inner, inner[1:] + inner[:1]):
        for outer_start, outer_end in outer_edges:
            if _segments_intersect(start, end, outer_start, outer_end):
                if not (
                    _point_on_segment(start, outer_start, outer_end)
                    or _point_on_segment(end, outer_start, outer_end)
                ):
                    return False
    return True


def transformed_footprint(
    footprint: RobotFootprint,
    x: float,
    y: float,
    yaw: float,
) -> list[Point]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return [
        (
            x + cosine * point_x - sine * point_y,
            y + sine * point_x + cosine * point_y,
        )
        for point_x, point_y in footprint.points
    ]


def _grid_metadata(grid_msg):
    info = grid_msg.info
    width = int(info.width)
    height = int(info.height)
    resolution = float(info.resolution)
    if width <= 0 or height <= 0 or not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("occupancy grid dimensions and resolution must be positive")
    return (
        width,
        height,
        resolution,
        float(info.origin.position.x),
        float(info.origin.position.y),
    )


def _grid_array(grid_msg, *, width: int, height: int):
    """Decode ROS signed int8 occupancy values from either bytes or sequences."""
    import numpy as np

    data = grid_msg.data
    if isinstance(data, (bytes, bytearray, memoryview)):
        grid = np.frombuffer(data, dtype=np.int8)
    else:
        grid = np.asarray(data, dtype=np.int8)
    if grid.size != width * height:
        raise ValueError(
            f"occupancy grid contains {grid.size} cells; expected {width * height}"
        )
    return grid.reshape(height, width)


def _footprint_clear(
    blocked,
    *,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    polygon: Sequence[Point],
) -> bool:
    min_gx = math.floor((min(x for x, _ in polygon) - origin_x) / resolution)
    max_gx = math.floor((max(x for x, _ in polygon) - origin_x) / resolution)
    min_gy = math.floor((min(y for _, y in polygon) - origin_y) / resolution)
    max_gy = math.floor((max(y for _, y in polygon) - origin_y) / resolution)
    if min_gx < 0 or min_gy < 0 or max_gx >= width or max_gy >= height:
        return False
    for gy in range(min_gy, max_gy + 1):
        y0 = origin_y + gy * resolution
        y1 = y0 + resolution
        for gx in range(min_gx, max_gx + 1):
            if not bool(blocked[gy, gx]):
                continue
            x0 = origin_x + gx * resolution
            x1 = x0 + resolution
            cell = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
            if polygons_intersect(polygon, cell):
                return False
    return True


def room_goal(
    grid_msg,
    room_points: Sequence[Point],
    footprint: RobotFootprint,
    *,
    yaw: float,
) -> tuple[float, float] | None:
    """Choose the centroid-nearest known-free pose fitting the real footprint."""
    room = [(float(x), float(y)) for x, y in room_points]
    if len(room) < 3:
        return None
    width, height, resolution, origin_x, origin_y = _grid_metadata(grid_msg)
    grid = _grid_array(grid_msg, width=width, height=height)
    blocked = (grid < 0) | (grid > 50)
    centroid_x, centroid_y = polygon_centroid(room)
    min_gx = max(0, math.floor((min(x for x, _ in room) - origin_x) / resolution))
    max_gx = min(
        width - 1,
        math.floor((max(x for x, _ in room) - origin_x) / resolution),
    )
    min_gy = max(0, math.floor((min(y for _, y in room) - origin_y) / resolution))
    max_gy = min(
        height - 1,
        math.floor((max(y for _, y in room) - origin_y) / resolution),
    )
    candidates: list[tuple[float, float, float]] = []
    for gy in range(min_gy, max_gy + 1):
        y = origin_y + (gy + 0.5) * resolution
        for gx in range(min_gx, max_gx + 1):
            x = origin_x + (gx + 0.5) * resolution
            candidate = transformed_footprint(footprint, x, y, yaw)
            if not polygon_inside_polygon(candidate, room):
                continue
            if _footprint_clear(
                blocked,
                width=width,
                height=height,
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
                polygon=candidate,
            ):
                candidates.append(
                    ((x - centroid_x) ** 2 + (y - centroid_y) ** 2, x, y)
                )
    if not candidates:
        return None
    _, x, y = min(candidates)
    return x, y


def _ring_cells(center_x: int, center_y: int, radius: int) -> Iterable[tuple[int, int]]:
    if radius == 0:
        yield center_x, center_y
        return
    for x in range(center_x - radius, center_x + radius + 1):
        yield x, center_y - radius
        yield x, center_y + radius
    for y in range(center_y - radius + 1, center_y + radius):
        yield center_x - radius, y
        yield center_x + radius, y


def object_goal(
    grid_msg,
    *,
    target_x: float,
    target_y: float,
    preferred_approach_yaw: float | None,
    minimum_standoff_m: float,
    footprint: RobotFootprint,
) -> tuple[float, float, float] | None:
    """Find the nearest map cell whose real footprint can approach an object."""
    width, height, resolution, origin_x, origin_y = _grid_metadata(grid_msg)
    grid = _grid_array(grid_msg, width=width, height=height)
    # Preserve the existing goal_near policy: unexplored cells are allowed,
    # but known occupied cells are not. Nav2 remains the final motion authority.
    blocked = grid > 50
    target_gx = math.floor((target_x - origin_x) / resolution)
    target_gy = math.floor((target_y - origin_y) / resolution)
    max_ring = max(
        abs(target_gx),
        abs(target_gy),
        abs(target_gx - (width - 1)),
        abs(target_gy - (height - 1)),
    )
    minimum_standoff_sq = minimum_standoff_m**2
    for ring in range(max_ring + 1):
        candidates: list[tuple[float, float, float, float]] = []
        for gx, gy in _ring_cells(target_gx, target_gy, ring):
            if not (0 <= gx < width and 0 <= gy < height):
                continue
            x = origin_x + (gx + 0.5) * resolution
            y = origin_y + (gy + 0.5) * resolution
            distance_sq = (x - target_x) ** 2 + (y - target_y) ** 2
            if distance_sq + 1e-12 < minimum_standoff_sq:
                continue
            yaw = math.atan2(target_y - y, target_x - x)
            angle_error = (
                abs(
                    math.atan2(
                        math.sin(yaw - preferred_approach_yaw),
                        math.cos(yaw - preferred_approach_yaw),
                    )
                )
                if preferred_approach_yaw is not None
                else 0.0
            )
            candidates.append((distance_sq, angle_error, x, y))
        for _distance_sq, _angle_error, x, y in sorted(candidates):
            yaw = math.atan2(target_y - y, target_x - x)
            candidate = transformed_footprint(footprint, x, y, yaw)
            if _footprint_clear(
                blocked,
                width=width,
                height=height,
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
                polygon=candidate,
            ):
                return x, y, yaw
    return None
