# SPDX-License-Identifier: MulanPSL-2.0
"""Scene Graph — LLM-enhanced object relation layer over ObjectRegistry.

Runs as an async low-frequency background task (default every 30 s).
Reads ObjectRegistry snapshots, generates captions and edge relations
via configurable LLM, and exposes the result through SceneGraphStore
for MCP tools and web visualization.
"""
from .types import (
    RELATION_TYPES,
    GeometryHint,
    SceneGraphEdge,
    SceneGraphNode,
    SceneGraphSnapshot,
)

__all__ = [
    "RELATION_TYPES",
    "GeometryHint",
    "SceneGraphEdge",
    "SceneGraphNode",
    "SceneGraphSnapshot",
]
