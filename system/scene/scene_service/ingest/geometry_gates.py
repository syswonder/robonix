# SPDX-License-Identifier: MulanPSL-2.0
"""Things the world cannot be, decided by the map rather than by a threshold.

Both perception backends produce objects that could not exist where they are
claimed to be. The checks that reject them belong to neither backend: they
are statements about the world -- a robot cannot have seen an object in a cell
it never observed -- and living inside one backend meant the other shipped
without them (objects standing outside the walls on the concept_graphs rows).
"""
from __future__ import annotations

import logging
from typing import Any, Optional

log = logging.getLogger("scene.gates")


class KnownGround:
    """The occupancy grid as a "has the robot looked here" mask, cached per map.

    `hub.latest("occupancy_grid")` is read on demand; the mask is rebuilt only
    when the grid's stamp changes, which is only when the map does.
    """

    def __init__(self, hub: Any):
        self._hub = hub
        self._stamp = 0.0
        self._cache = None

    def current(self):
        """(mask, origin_xy, resolution), or None when there is no grid yet."""
        hub = self._hub
        if hub is None or not hub.has("occupancy_grid"):
            return None
        msg, stamp, _count = hub.latest("occupancy_grid")
        if msg is None or stamp <= 0.0:
            return None
        if self._stamp == stamp:
            return self._cache
        try:
            import numpy as np

            info = msg.info
            grid = np.asarray(msg.data, dtype=np.int16).reshape(int(info.height), int(info.width))
            # Occupied or free are both "the robot has looked here"; -1 is not.
            self._cache = (grid >= 0,
                           (float(info.origin.position.x), float(info.origin.position.y)),
                           float(info.resolution))
        except Exception as e:  # noqa: BLE001
            log.warning("cannot read the occupancy grid: %s", e)
            self._cache = None
        self._stamp = stamp
        return self._cache


def fraction_on_known_ground(points_xy, known) -> Optional[float]:
    """Share of these XY points that fall on cells the map has observed.

    None when the map has no opinion (no grid, or nothing to test) -- the caller
    keeps the object in that case rather than rejecting it on no evidence. A
    point outside the grid entirely counts as unobserved.
    """
    import numpy as np

    mask, (ox, oy), res = known
    if res <= 0.0:
        return None
    pts = np.asarray(points_xy, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0:
        return None
    cols = ((pts[:, 0] - ox) / res).astype(np.int32)
    rows = ((pts[:, 1] - oy) / res).astype(np.int32)
    inside = ((cols >= 0) & (cols < mask.shape[1]) & (rows >= 0) & (rows < mask.shape[0]))
    if not inside.any():
        return 0.0
    seen = mask[rows[inside], cols[inside]]
    # Points off the grid are unobserved by definition; count them in the denominator.
    return float(seen.sum()) / float(pts.shape[0])


def footprint_samples(x: float, y: float, size_x: float, size_y: float, n: int = 5):
    """A small grid of XY points across a box footprint, for objects that only
    carry a centre and a size rather than a point cloud."""
    import numpy as np

    xs = np.linspace(x - size_x / 2.0, x + size_x / 2.0, max(2, n))
    ys = np.linspace(y - size_y / 2.0, y + size_y / 2.0, max(2, n))
    gx, gy = np.meshgrid(xs, ys)
    return np.column_stack([gx.ravel(), gy.ravel()])
