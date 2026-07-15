# SPDX-License-Identifier: MulanPSL-2.0
"""Geometry helpers shared by Scene room rendering and navigation."""
from __future__ import annotations

import math
from collections.abc import Sequence

Point = tuple[float, float]


def polygon_centroid(points: Sequence[Point]) -> Point:
    """Return the area-weighted centroid of a non-self-intersecting polygon."""
    pts = [(float(x), float(y)) for x, y in points]
    if not pts:
        return 0.0, 0.0
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts.pop()
    if len(pts) < 3:
        return (
            sum(x for x, _ in pts) / len(pts),
            sum(y for _, y in pts) / len(pts),
        )

    twice_area = 0.0
    cx = 0.0
    cy = 0.0
    for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]):
        cross = x1 * y2 - x2 * y1
        twice_area += cross
        cx += (x1 + x2) * cross
        cy += (y1 + y2) * cross
    if abs(twice_area) < 1e-9:
        return (
            sum(x for x, _ in pts) / len(pts),
            sum(y for _, y in pts) / len(pts),
        )
    return cx / (3.0 * twice_area), cy / (3.0 * twice_area)


def point_in_polygon(x: float, y: float, points: Sequence[Point]) -> bool:
    """Return whether a point lies inside a polygon using even-odd crossings."""
    pts = [(float(px), float(py)) for px, py in points]
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts.pop()
    if len(pts) < 3:
        return False
    inside = False
    for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]):
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
    return inside


def disc_inside_polygon(
    x: float,
    y: float,
    radius: float,
    points: Sequence[Point],
    samples: int = 16,
) -> bool:
    """Require a circular robot footprint, not just its centre, inside a room."""
    if not point_in_polygon(x, y, points):
        return False
    return all(
        point_in_polygon(
            x + radius * math.cos(2.0 * math.pi * i / samples),
            y + radius * math.sin(2.0 * math.pi * i / samples),
            points,
        )
        for i in range(samples)
    )
