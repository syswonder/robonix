# SPDX-License-Identifier: MulanPSL-2.0
"""The 3D page's default eye sees the whole occupancy grid."""
import math

import pytest

from scene_service.rerun_sink import _overview


@pytest.mark.parametrize("bounds", [(-12.2, -5.0, 2.3, 9.4), (0, 0, 40, 5), (0, 0, 1, 1)])
def test_overview_sees_every_corner(bounds):
    x0, y0, x1, y1 = bounds
    eye, target = _overview(*bounds)
    forward = [t - e for t, e in zip(target, eye)]
    norm = math.hypot(*forward)
    forward = [v / norm for v in forward]
    right = [1.0, 0.0, 0.0]
    up = [right[1] * forward[2] - right[2] * forward[1],
          right[2] * forward[0] - right[0] * forward[2],
          right[0] * forward[1] - right[1] * forward[0]]
    # rerun's default vertical field of view is 55 degrees; assume a square page.
    limit = math.tan(math.radians(55 / 2))
    for corner in ((x0, y0, 0), (x1, y0, 0), (x0, y1, 0), (x1, y1, 0)):
        ray = [c - e for c, e in zip(corner, eye)]
        depth = sum(r * f for r, f in zip(ray, forward))
        assert depth > 0
        assert abs(sum(r * a for r, a in zip(ray, right))) / depth < limit
        assert abs(sum(r * a for r, a in zip(ray, up))) / depth < limit
