# SPDX-License-Identifier: MulanPSL-2.0
"""LLM prompts for scene graph relation inference."""
from __future__ import annotations

import json
from typing import Any

from .types import RELATION_TYPES, GeometryHint, SceneGraphNode

RELATION_SYSTEM_PROMPT = """\
You infer spatial relationships between two objects in a robot's 3D scene graph.

Return ONLY valid JSON.

Allowed relation values:
- "near": objects are spatially close but no stronger relation is clear.
- "on_top_of": source object is physically on top of target object.
- "under": source object is under target object.
- "inside": source object is inside target object.
- "contains": source object contains target object.
- "attached_to": source object is attached to or mounted on target object.
- "part_of": source object is a part/component of target object.
- "same_object": both detections likely refer to the same physical object.
- "none": no meaningful relation.
- "unknown": insufficient information.

Use geometry first. Use captions and common sense only as secondary evidence.
Do not invent objects not present in the input.

Output schema:
{
  "relation": "...",
  "confidence": 0.0,
  "reason": "brief explanation"
}
"""


# Semantic relations geometry cannot determine — the only valid answers
# (besides "none") when geometry has already fixed the spatial relation.
SEMANTIC_RELATIONS = ("attached_to", "part_of", "same_object")

SEMANTIC_RELATION_SYSTEM_PROMPT = """\
You decide whether a SEMANTIC relationship holds between two objects in a
robot's 3D scene graph. Their spatial relation (contact / containment) has
ALREADY been determined by geometry — do NOT report spatial relations
(near / on_top_of / under / inside / contains).

Return ONLY valid JSON.

Allowed relation values:
- "attached_to": source is attached to or mounted on target (not merely resting on it).
- "part_of": source is a part/component of target (e.g. a handle of a door).
- "same_object": both detections likely refer to the same physical object.
- "none": no semantic relation beyond the geometry already known.

Decide from the labels/captions and common sense. A plain resting or
containing arrangement with no attachment or part/whole meaning is "none".

Output schema:
{
  "relation": "...",
  "confidence": 0.0,
  "reason": "brief explanation"
}
"""


def _pair_payload(a: SceneGraphNode, b: SceneGraphNode, hint: GeometryHint) -> dict[str, Any]:
    """Shared object-pair + geometry block used by both relation prompts."""
    return {
        "object_a": {
            "id": a.object_id,
            "label": a.label,
            "caption": a.caption or a.label,
            "bbox_center": list(a.bbox_center),
            "bbox_extent": list(a.bbox_extent),
        },
        "object_b": {
            "id": b.object_id,
            "label": b.label,
            "caption": b.caption or b.label,
            "bbox_center": list(b.bbox_center),
            "bbox_extent": list(b.bbox_extent),
        },
        "geometry_hint": {
            "distance": round(hint.distance, 3),
            "xy_overlap": round(hint.xy_overlap, 3),
            "vertical_order": hint.vertical_order,
            "containment": hint.containment,
        },
    }


def build_relation_user_prompt(
    a: SceneGraphNode,
    b: SceneGraphNode,
    hint: GeometryHint,
) -> str:
    """Build the user message payload for one object pair (full inference)."""
    payload = _pair_payload(a, b, hint)
    payload["instruction"] = (
        "Infer the relation from object_a to object_b. "
        "If object_a is on object_b, return relation='on_top_of'. "
        "If object_a is inside object_b, return relation='inside'. "
        "Return exactly one JSON object with keys: relation, confidence, reason."
    )
    return json.dumps(payload, ensure_ascii=False)


def build_semantic_relation_user_prompt(
    a: SceneGraphNode,
    b: SceneGraphNode,
    hint: GeometryHint,
    known_relation: str,
) -> str:
    """Build the user message for a *semantic-only* query on a pair whose
    spatial relation geometry has already fixed (passed as known_relation)."""
    payload = _pair_payload(a, b, hint)
    payload["known_spatial_relation"] = known_relation
    payload["instruction"] = (
        f"Geometry already classified the spatial relation as '{known_relation}'. "
        "Decide ONLY whether a semantic relation (attached_to, part_of, "
        "same_object) also holds from object_a to object_b; otherwise return "
        "'none'. Return exactly one JSON object with keys: relation, confidence, reason."
    )
    return json.dumps(payload, ensure_ascii=False)
