# SPDX-License-Identifier: MulanPSL-2.0
"""Edge candidate generation and LLM-based relation inference."""
from __future__ import annotations

import logging
import math
import time
from typing import Optional

from .llm_client import SceneGraphLLMClient
from .prompts import RELATION_SYSTEM_PROMPT, build_relation_user_prompt
from .types import (
    RELATION_TYPES,
    GeometryHint,
    SceneGraphEdge,
    SceneGraphNode,
)

log = logging.getLogger(__name__)


# ── geometry helpers ─────────────────────────────────────────────────────────

def _l2(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _xy_overlap_ratio(a: SceneGraphNode, b: SceneGraphNode) -> float:
    """Intersection-over-min-area in the XY plane (axis-aligned)."""
    ax, ay = a.bbox_center[0], a.bbox_center[1]
    bx, by = b.bbox_center[0], b.bbox_center[1]
    a_hx, a_hy = a.bbox_extent[0] / 2, a.bbox_extent[1] / 2
    b_hx, b_hy = b.bbox_extent[0] / 2, b.bbox_extent[1] / 2

    ix = max(0.0, min(ax + a_hx, bx + b_hx) - max(ax - a_hx, bx - b_hx))
    iy = max(0.0, min(ay + a_hy, by + b_hy) - max(ay - a_hy, by - b_hy))
    inter = ix * iy
    if inter == 0.0:
        return 0.0

    area_a = a.bbox_extent[0] * a.bbox_extent[1]
    area_b = b.bbox_extent[0] * b.bbox_extent[1]
    min_area = min(area_a, area_b)
    if min_area <= 0.0:
        return 0.0
    return inter / min_area


def _maybe_containment(a: SceneGraphNode, b: SceneGraphNode) -> str:
    """Check if one bbox roughly contains the other. Returns
    ``"a_inside_b"`` / ``"b_inside_a"`` / ``"none"``."""
    def _inside(inner: SceneGraphNode, outer: SceneGraphNode) -> bool:
        for i in range(3):
            ic = inner.bbox_center[i]
            ie = inner.bbox_extent[i] / 2
            oc = outer.bbox_center[i]
            oe = outer.bbox_extent[i] / 2
            if ic - ie < oc - oe - 0.05 or ic + ie > oc + oe + 0.05:
                return False
        return True

    if _inside(a, b):
        return "a_inside_b"
    if _inside(b, a):
        return "b_inside_a"
    return "none"


def _vertical_order(a: SceneGraphNode, b: SceneGraphNode) -> str:
    az = a.bbox_center[2]
    bz = b.bbox_center[2]
    diff = az - bz
    if diff > 0.05:
        return "a_above_b"
    if diff < -0.05:
        return "b_above_a"
    return "same_level"


def compute_geometry_hint(a: SceneGraphNode, b: SceneGraphNode) -> GeometryHint:
    return GeometryHint(
        distance=_l2(a.bbox_center, b.bbox_center),
        xy_overlap=_xy_overlap_ratio(a, b),
        vertical_order=_vertical_order(a, b),
        containment=_maybe_containment(a, b),
    )


# ── edge candidate generation ───────────────────────────────────────────────

def generate_edge_candidates(
    nodes: list[SceneGraphNode],
    *,
    max_distance: float = 2.0,
    min_xy_overlap: float = 0.15,
    max_candidates: int = 200,
) -> list[tuple[SceneGraphNode, SceneGraphNode, GeometryHint]]:
    """Return candidate pairs with precomputed geometry hints.

    A pair is a candidate if any of these hold:
      - center distance < max_distance
      - XY overlap ratio > min_xy_overlap
      - one bbox contains the other
    """
    candidates: list[tuple[float, SceneGraphNode, SceneGraphNode, GeometryHint]] = []

    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            a, b = nodes[i], nodes[j]
            hint = compute_geometry_hint(a, b)

            if (
                hint.distance < max_distance
                or hint.xy_overlap > min_xy_overlap
                or hint.containment != "none"
            ):
                candidates.append((hint.distance, a, b, hint))

    # Sort by distance (closer pairs first), truncate.
    candidates.sort(key=lambda t: t[0])
    candidates = candidates[:max_candidates]

    return [(a, b, h) for _, a, b, h in candidates]


# ── LLM-based relation inference ────────────────────────────────────────────

class RelationInferer:
    """Infer the spatial relation between two objects via LLM."""

    def __init__(self, llm_client: SceneGraphLLMClient) -> None:
        self.llm_client = llm_client

    async def infer_relation(
        self,
        source: SceneGraphNode,
        target: SceneGraphNode,
        hint: GeometryHint,
    ) -> SceneGraphEdge:
        """Call LLM to infer the relation from *source* to *target*.

        Returns a SceneGraphEdge. On LLM failure the edge has
        ``relation="unknown"`` and ``method="llm_fail"``.
        """
        user_msg = build_relation_user_prompt(source, target, hint)
        raw = await self.llm_client.chat_json(
            system_prompt=RELATION_SYSTEM_PROMPT,
            user_message=user_msg,
            timeout=20,
        )

        if not raw:
            return SceneGraphEdge(
                source_id=source.object_id,
                target_id=target.object_id,
                relation="unknown",
                confidence=0.0,
                method="llm_fail",
                reason="LLM call returned empty",
            )

        relation = raw.get("relation", "unknown")
        if relation not in RELATION_TYPES:
            log.debug(
                "[scene-graph] LLM returned unknown relation '%s'; "
                "falling back to 'unknown'",
                relation,
            )
            relation = "unknown"

        confidence = 0.0
        try:
            confidence = float(raw.get("confidence", 0.0))
        except (TypeError, ValueError):
            pass

        reason = str(raw.get("reason", ""))

        return SceneGraphEdge(
            source_id=source.object_id,
            target_id=target.object_id,
            relation=relation,
            confidence=max(0.0, min(1.0, confidence)),
            method="llm",
            reason=reason,
        )
