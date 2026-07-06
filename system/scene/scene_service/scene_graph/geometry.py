# SPDX-License-Identifier: MulanPSL-2.0
"""Deterministic geometric relation predicates and cache-invalidation
signatures, shared by the scene-graph builder/store.

This is the single source of truth for "what the bounding boxes alone
say" — the contact/containment judgements that were historically split
between ``state/relations.py`` (the 1 Hz RelationEngine) and the
``GeometryHint`` cues in ``scene_graph/relations.py``. Operating on
``SceneGraphNode.bbox_center`` / ``bbox_extent`` keeps it usable from the
scene-graph layer without importing the state-layer structs.

Two roles:
  * ``geometric_relation`` — when geometry alone is decisive, name the
    relation (so the builder can later skip the LLM); otherwise return
    ``None`` and leave it to the LLM.
  * ``geometry_signature`` — a coarse, jitter-stable fingerprint of a
    pair's *relational* geometry, used as part of the relation cache key
    so a cached edge invalidates when the spatial configuration actually
    changes (not on every EMA position wobble).
"""
from __future__ import annotations

import math
from typing import Optional

from .types import RELATION_TYPES, GeometryHint, SceneGraphEdge, SceneGraphNode

# Geometric confidence levels. A contact/containment judgement from the
# bounding boxes is near-certain; a bare proximity ("near") is softer.
CONF_HIGH = 0.95
CONF_MED = 0.7

# `a` rests on `b` when a's bottom face is within this gap of b's top
# face. Ported verbatim from RelationEngine._on (state/relations.py).
_CONTACT_GAP_M = 0.05
# Default proximity radius for the soft `near` predicate. Matches
# RelationEngine's _DEFAULT_NEAR_THRESHOLD_M; per-class tuning is left to
# the state layer and not reproduced here.
_NEAR_RADIUS_M = 1.0

# Signature bucket widths. These trade invalidation sensitivity against
# stability under EMA pose jitter (object_registry.update_object_pose,
# alpha=0.3): too fine and a stationary pair's signature flips every tick
# (the bug that made the old coord-keyed cache useless); too coarse and a
# real move fails to invalidate. A pair sitting exactly on a bucket
# boundary may still recompute occasionally — that is acceptable (one
# extra geometry/LLM evaluation), unlike the old "never recompute".
_DIST_BUCKET_M = 0.25
_OVERLAP_BUCKET = 0.1


# ── helpers ──────────────────────────────────────────────────────────────────

def _half(node: SceneGraphNode) -> tuple[float, float, float]:
    e = node.bbox_extent
    return e[0] * 0.5, e[1] * 0.5, e[2] * 0.5


def _l2(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _xy_overlap(a: SceneGraphNode, b: SceneGraphNode) -> bool:
    """Axis-aligned XY overlap between two bboxes centered on their poses.
    Boolean form, matching RelationEngine._xy_overlap (any positive
    overlap counts)."""
    ahx, ahy, _ = _half(a)
    bhx, bhy, _ = _half(b)
    ax, ay = a.bbox_center[0], a.bbox_center[1]
    bx, by = b.bbox_center[0], b.bbox_center[1]
    return (
        ax + ahx >= bx - bhx and bx + bhx >= ax - ahx
        and ay + ahy >= by - bhy and by + bhy >= ay - ahy
    )


def _rests_on(a: SceneGraphNode, b: SceneGraphNode) -> bool:
    """`a` rests on top of `b`: a's bottom within _CONTACT_GAP_M of b's
    top, with overlapping XY projections. Order matters. Uses true
    contact (bottom-vs-top), not a center-z comparison, so a hovering
    object is not mistaken for a resting one."""
    a_bottom = a.bbox_center[2] - _half(a)[2]
    b_top = b.bbox_center[2] + _half(b)[2]
    if abs(a_bottom - b_top) > _CONTACT_GAP_M:
        return False
    return _xy_overlap(a, b)


def _center_inside(inner: SceneGraphNode, outer: SceneGraphNode) -> bool:
    """Center of `inner` lies within `outer`'s axis-aligned bbox on all
    three axes. Ported from RelationEngine._inside."""
    ic = inner.bbox_center
    oc = outer.bbox_center
    oh = _half(outer)
    return (
        oc[0] - oh[0] <= ic[0] <= oc[0] + oh[0]
        and oc[1] - oh[1] <= ic[1] <= oc[1] + oh[1]
        and oc[2] - oh[2] <= ic[2] <= oc[2] + oh[2]
    )


# ── public API ───────────────────────────────────────────────────────────────

def geometric_relation(
    a: SceneGraphNode, b: SceneGraphNode
) -> Optional[tuple[str, float]]:
    """Return ``(relation, confidence)`` from ``a`` to ``b`` when the
    bounding boxes alone decide it, else ``None`` (defer to the LLM).

    Decision order (strongest first):
      * ``a`` rests on ``b``                    → ``on_top_of``  (high)
      * ``b`` rests on ``a``                    → ``under``      (high)
      * ``a``'s center inside ``b`` (only)      → ``inside``     (high)
      * ``b``'s center inside ``a`` (only)      → ``contains``   (high)
      * mutual containment / ambiguous nesting  → ``None`` (LLM disambiguates)
      * centers within proximity radius         → ``near``       (medium)
      * otherwise                               → ``None``

    Returns geometric-spatial relations only; semantic relations
    (``attached_to`` / ``part_of`` / ``same_object``) are never produced
    here and remain the LLM's job.
    """
    if _rests_on(a, b):
        return ("on_top_of", CONF_HIGH)
    if _rests_on(b, a):
        return ("under", CONF_HIGH)

    a_in_b = _center_inside(a, b)
    b_in_a = _center_inside(b, a)
    if a_in_b and not b_in_a:
        return ("inside", CONF_HIGH)
    if b_in_a and not a_in_b:
        return ("contains", CONF_HIGH)
    if a_in_b and b_in_a:
        # Boxes nest both ways (similar sizes); direction is ambiguous.
        return None

    if _l2(a.bbox_center, b.bbox_center) <= _NEAR_RADIUS_M:
        return ("near", CONF_MED)
    return None


# Defensive: relation names produced above must be valid wire values.
assert all(r in RELATION_TYPES for r in
           ("on_top_of", "under", "inside", "contains", "near"))


# Short, deterministic explanations per geometric relation — gives the
# emitted edges explainability parity with the LLM edges' `reason` field
# (the LLM path always fills one). Keyed from a→b semantics.
_GEOMETRIC_REASON = {
    "on_top_of": "geometry: source base within contact gap of target top, xy overlap",
    "under": "geometry: target base within contact gap of source top, xy overlap",
    "inside": "geometry: source center inside target bounding box",
    "contains": "geometry: target center inside source bounding box",
}


def compute_geometric_edges(
    nodes: list[SceneGraphNode],
) -> list[SceneGraphEdge]:
    """Deterministic contact/containment edges over a node list.

    For each unordered pair (i<j) ask ``geometric_relation`` and emit a
    ``method="geometric"`` edge only when geometry decides a contact or
    containment relation (``on_top_of``/``under``/``inside``/``contains``).
    ``near`` and ambiguous (``None``) pairs are skipped: proximity is
    served separately (``get_object_context.nearby_objects``) and
    ambiguous nesting is left to the LLM. One edge per pair (the inverse
    direction is implied — consumers read edges touching either endpoint).

    ``reachable_by`` is NOT produced here: it needs the gripper object,
    which lives in the registry, not in this node set — the fast loop adds
    it. O(N²) over a small node set (N<80); each call is cheap.
    """
    edges: list[SceneGraphEdge] = []
    n = len(nodes)
    for i in range(n):
        for j in range(i + 1, n):
            a, b = nodes[i], nodes[j]
            result = geometric_relation(a, b)
            if result is None:
                continue
            relation, conf = result
            if relation == "near":
                continue
            edges.append(SceneGraphEdge(
                source_id=a.object_id,
                target_id=b.object_id,
                relation=relation,
                confidence=conf,
                method="geometric",
                reason=_GEOMETRIC_REASON.get(relation, "geometry"),
            ))
    return edges


def geometry_signature(hint: GeometryHint) -> str:
    """Coarse, jitter-stable fingerprint of a pair's relational geometry.

    Quantizes the continuous cues (distance, xy_overlap) into buckets and
    keeps the already-discrete cues (vertical_order, containment) as-is.
    Same spatial configuration → same string (cache hit, no recompute);
    a real change → different string (cache miss → re-evaluate). Unlike a
    raw-coordinate key it does not drift on sub-bucket EMA jitter.
    """
    d = round(hint.distance / _DIST_BUCKET_M)
    o = round(hint.xy_overlap / _OVERLAP_BUCKET)
    return f"d{d}_o{o}_{hint.vertical_order}_{hint.containment}"
