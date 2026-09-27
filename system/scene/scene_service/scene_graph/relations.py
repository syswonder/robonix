# SPDX-License-Identifier: MulanPSL-2.0
"""Edge candidate generation and LLM-based relation inference."""
from __future__ import annotations

import math
import re

from .llm_client import SceneGraphLLMClient
from .prompts import (
    RELATION_SYSTEM_PROMPT,
    SEMANTIC_RELATION_SYSTEM_PROMPT,
    SEMANTIC_RELATIONS,
    build_relation_user_prompt,
    build_semantic_relation_user_prompt,
)
from .types import (
    RELATION_TYPES,
    GeometryHint,
    SceneGraphEdge,
    SceneGraphNode,
)

# Generous for a reasoning model; VLM_REASONING_EFFORT=minimal answers in ~2 s.
_LLM_TIMEOUT_SEC = 60.0


def _normalize_relation(relation: object) -> str:
    """"On-Top of" → "on_top_of"; callers reject what is not in the vocabulary."""
    return re.sub(r"[\s\-]+", "_", str(relation).strip().lower())


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
    min_area = min(a.bbox_extent[0] * a.bbox_extent[1],
                   b.bbox_extent[0] * b.bbox_extent[1])
    if min_area <= 0.0:
        return 0.0
    return inter / min_area


def _maybe_containment(a: SceneGraphNode, b: SceneGraphNode) -> str:
    """"a_inside_b" / "b_inside_a" / "none", with a 5 cm tolerance."""
    def _inside(inner: SceneGraphNode, outer: SceneGraphNode) -> bool:
        for i in range(3):
            ic, ie = inner.bbox_center[i], inner.bbox_extent[i] / 2
            oc, oe = outer.bbox_center[i], outer.bbox_extent[i] / 2
            if ic - ie < oc - oe - 0.05 or ic + ie > oc + oe + 0.05:
                return False
        return True

    if _inside(a, b):
        return "a_inside_b"
    if _inside(b, a):
        return "b_inside_a"
    return "none"


def _vertical_order(a: SceneGraphNode, b: SceneGraphNode) -> str:
    diff = a.bbox_center[2] - b.bbox_center[2]
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


def generate_edge_candidates(
    nodes: list[SceneGraphNode],
    *,
    max_distance: float = 2.0,
    min_xy_overlap: float = 0.15,
    max_candidates: int = 200,
) -> list[tuple[SceneGraphNode, SceneGraphNode, GeometryHint]]:
    """Pairs that are close, overlap in XY, or nest, nearest first."""
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
    candidates.sort(key=lambda t: t[0])
    return [(a, b, h) for _, a, b, h in candidates[:max_candidates]]


class RelationInferer:
    """Infer the relation between two objects via LLM."""

    def __init__(self, llm_client: SceneGraphLLMClient) -> None:
        self.llm_client = llm_client

    async def infer_relation(
        self,
        source: SceneGraphNode,
        target: SceneGraphNode,
        hint: GeometryHint,
    ) -> SceneGraphEdge:
        """Full relation; an off-vocabulary answer becomes "unknown"."""
        return await self._ask(
            source, target, RELATION_SYSTEM_PROMPT,
            build_relation_user_prompt(source, target, hint),
            RELATION_TYPES, "unknown",
        )

    async def infer_semantic_relation(
        self,
        source: SceneGraphNode,
        target: SceneGraphNode,
        hint: GeometryHint,
        known_relation: str,
    ) -> SceneGraphEdge:
        """Semantic relation for a pair geometry already placed; anything
        spatial or off-vocabulary becomes "none"."""
        return await self._ask(
            source, target, SEMANTIC_RELATION_SYSTEM_PROMPT,
            build_semantic_relation_user_prompt(source, target, hint, known_relation),
            SEMANTIC_RELATIONS, "none",
        )

    async def _ask(self, source, target, system_prompt, user_msg,
                   allowed, fallback: str) -> SceneGraphEdge:
        """One LLM call. An empty reply is `llm_fail`/"unknown" so the builder
        retries the pair instead of caching a non-answer."""
        raw = await self.llm_client.chat_json(
            system_prompt=system_prompt,
            user_message=user_msg,
            timeout=_LLM_TIMEOUT_SEC,
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
        relation = _normalize_relation(raw.get("relation", fallback))
        if relation not in allowed:
            relation = fallback
        try:
            confidence = float(raw.get("confidence", 0.0))
        except (TypeError, ValueError):
            confidence = 0.0
        return SceneGraphEdge(
            source_id=source.object_id,
            target_id=target.object_id,
            relation=relation,
            confidence=max(0.0, min(1.0, confidence)),
            method="llm",
            reason=str(raw.get("reason", "")),
        )
