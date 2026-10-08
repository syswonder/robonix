# SPDX-License-Identifier: MulanPSL-2.0
"""What the bounding boxes alone say about a pair of scene-graph nodes, and a
jitter-stable signature of that geometry for the relation cache key."""
from __future__ import annotations

import math

from .types import GeometryHint, SceneGraphNode

CONF_HIGH = 0.95
CONF_MED = 0.7

# `a` rests on `b` when a's bottom face is within this gap of b's top face.
_CONTACT_GAP_M = 0.05

# Signature buckets: fine enough that a real move invalidates the cache,
# coarse enough that EMA pose jitter does not.
_DIST_BUCKET_M = 0.25
_OVERLAP_BUCKET = 0.1

# Strict gates: plain AABB tests call a pillow "inside" a bed and two mugs on
# one table stacked.
_SUPPORT_FOOTPRINT_MIN = 0.5      # ≥ half of the top object's footprint over the base
_SUPPORT_MAX_SIZE_RATIO = 1.0     # the supported object may not be larger than the base
_CONTAIN_MAX_VOLUME_RATIO = 0.5   # an inside object is at most half the container's volume
_NEAR_STRICT_RADIUS_M = 1.5


def _half(node: SceneGraphNode) -> tuple[float, float, float]:
    e = node.bbox_extent
    return e[0] * 0.5, e[1] * 0.5, e[2] * 0.5


def _l2(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _xy_overlap(a: SceneGraphNode, b: SceneGraphNode) -> bool:
    ahx, ahy, _ = _half(a)
    bhx, bhy, _ = _half(b)
    ax, ay = a.bbox_center[0], a.bbox_center[1]
    bx, by = b.bbox_center[0], b.bbox_center[1]
    return (
        ax + ahx >= bx - bhx and bx + bhx >= ax - ahx
        and ay + ahy >= by - bhy and by + bhy >= ay - ahy
    )


def _rests_on(a: SceneGraphNode, b: SceneGraphNode) -> bool:
    """a's bottom touches b's top and their footprints overlap."""
    a_bottom = a.bbox_center[2] - _half(a)[2]
    b_top = b.bbox_center[2] + _half(b)[2]
    if abs(a_bottom - b_top) > _CONTACT_GAP_M:
        return False
    return _xy_overlap(a, b)


def _center_inside(inner: SceneGraphNode, outer: SceneGraphNode) -> bool:
    ic = inner.bbox_center
    oc = outer.bbox_center
    oh = _half(outer)
    return (
        oc[0] - oh[0] <= ic[0] <= oc[0] + oh[0]
        and oc[1] - oh[1] <= ic[1] <= oc[1] + oh[1]
        and oc[2] - oh[2] <= ic[2] <= oc[2] + oh[2]
    )


def _footprint_overlap_ratio(a: SceneGraphNode, b: SceneGraphNode) -> float:
    """Fraction of `a`'s XY footprint covered by `b`'s footprint (0..1)."""
    ahx, ahy, _ = _half(a)
    bhx, bhy, _ = _half(b)
    ax, ay = a.bbox_center[0], a.bbox_center[1]
    bx, by = b.bbox_center[0], b.bbox_center[1]
    ix = max(0.0, min(ax + ahx, bx + bhx) - max(ax - ahx, bx - bhx))
    iy = max(0.0, min(ay + ahy, by + bhy) - max(ay - ahy, by - bhy))
    return (ix * iy) / max(1e-6, 4.0 * ahx * ahy)


def _volume(n: SceneGraphNode) -> float:
    e = n.bbox_extent
    return max(1e-9, e[0] * e[1] * e[2])


def strict_geometric_relations(
    a: SceneGraphNode, b: SceneGraphNode, near_radius_m: float = _NEAR_STRICT_RADIUS_M,
) -> list[tuple[str, float, float]]:
    """`[(relation, confidence, distance_m)]` from `a` to `b`, strongest
    first: on_top_of / under, inside / contains, else `near` within
    `near_radius_m`; empty when the boxes say nothing."""
    dist = _l2(a.bbox_center, b.bbox_center)

    def _supports(top: SceneGraphNode, base: SceneGraphNode) -> bool:
        if not _rests_on(top, base):
            return False
        if _footprint_overlap_ratio(top, base) < _SUPPORT_FOOTPRINT_MIN:
            return False
        th, bh = _half(top), _half(base)
        return (th[0] * th[1]) <= _SUPPORT_MAX_SIZE_RATIO * (bh[0] * bh[1])

    if _supports(a, b):
        return [("on_top_of", CONF_HIGH, dist)]
    if _supports(b, a):
        return [("under", CONF_HIGH, dist)]
    a_in_b = _center_inside(a, b) and _volume(a) <= _CONTAIN_MAX_VOLUME_RATIO * _volume(b)
    b_in_a = _center_inside(b, a) and _volume(b) <= _CONTAIN_MAX_VOLUME_RATIO * _volume(a)
    if a_in_b and not b_in_a:
        return [("inside", CONF_HIGH, dist)]
    if b_in_a and not a_in_b:
        return [("contains", CONF_HIGH, dist)]
    if dist <= near_radius_m:
        return [("near", CONF_MED, dist)]
    return []


def geometry_signature(hint: GeometryHint) -> str:
    """Bucketed distance/overlap plus the discrete cues: equal for jitter,
    different after a real move."""
    d = round(hint.distance / _DIST_BUCKET_M)
    o = round(hint.xy_overlap / _OVERLAP_BUCKET)
    return f"d{d}_o{o}_{hint.vertical_order}_{hint.containment}"
