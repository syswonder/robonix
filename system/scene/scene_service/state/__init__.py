# SPDX-License-Identifier: MulanPSL-2.0
"""scene_service.state — object registry, data association, relations,
snapshot scoping. All shared mutable state lives here behind a single
asyncio.Lock owned by ObjectRegistry."""

from .object_registry import (
    SceneObject,
    SceneSurface,
    Pose3D,
    BBox3D,
    ObjectRegistry,
    OBJECT_ATTRIBUTE_KEYS,
)
from .data_assoc import Detection, associate
from .relations import RelationEngine, RELATION_PREDICATES
from .snapshot import scope_snapshot

__all__ = [
    "SceneObject",
    "SceneSurface",
    "Pose3D",
    "BBox3D",
    "ObjectRegistry",
    "OBJECT_ATTRIBUTE_KEYS",
    "Detection",
    "associate",
    "RelationEngine",
    "RELATION_PREDICATES",
    "scope_snapshot",
]
