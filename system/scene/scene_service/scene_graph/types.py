# SPDX-License-Identifier: MulanPSL-2.0
"""Core data structures for the Scene Graph layer."""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional


# Allowed relation values — kept as a list so prompt and validation
# can reference the same source of truth.
RELATION_TYPES = [
    "near",
    "on_top_of",
    "under",
    "inside",
    "contains",
    "attached_to",
    "part_of",
    "same_object",
    "none",
    "unknown",
]

# Inverse mapping for directed relations.
INVERSE_RELATIONS: dict[str, str] = {
    "on_top_of": "under",
    "under": "on_top_of",
    "inside": "contains",
    "contains": "inside",
    "near": "near",
    "attached_to": "attached_to",
    "part_of": "contains",
    "same_object": "same_object",
    "none": "none",
    "unknown": "unknown",
}


@dataclass
class SceneGraphNode:
    """One node in the scene graph, mapped 1:1 to an ObjectRegistry record."""
    object_id: str
    label: str
    bbox_center: tuple[float, float, float]
    bbox_extent: tuple[float, float, float]
    yaw: float = 0.0
    caption: Optional[str] = None
    caption_updated_at: Optional[float] = None
    confidence: float = 0.0
    observation_count: int = 0
    last_seen: Optional[float] = None


@dataclass
class GeometryHint:
    """Spatial cues passed to the LLM alongside the object pair."""
    distance: float
    xy_overlap: float
    vertical_order: str   # "a_above_b" | "b_above_a" | "same_level"
    containment: str      # "a_inside_b" | "b_inside_a" | "none"


@dataclass
class SceneGraphEdge:
    """Directed relation between two scene graph nodes."""
    source_id: str
    target_id: str
    relation: str
    confidence: float = 0.0
    method: str = "llm"       # "llm" | "cached" | "llm_fail"
    reason: str = ""
    updated_at: float = field(default_factory=time.time)


@dataclass
class SceneGraphSnapshot:
    """Immutable snapshot of the full scene graph at a point in time."""
    nodes: dict[str, SceneGraphNode] = field(default_factory=dict)
    edges: list[SceneGraphEdge] = field(default_factory=list)
    updated_at: float = field(default_factory=time.time)
