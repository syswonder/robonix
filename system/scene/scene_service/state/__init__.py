# SPDX-License-Identifier: MulanPSL-2.0
"""scene_service.state — object registry and data association.
All shared mutable state lives here behind a single asyncio.Lock owned
by ObjectRegistry. Geometric relations moved to
``scene_graph/geometry.py`` (judgment) + ``scene_graph/geometric_loop.py``
(runtime), which feed the Pilot-facing scene-graph store."""

from .object_registry import (
    SceneObject,
    SceneSurface,
    Pose3D,
    BBox3D,
    ObjectRegistry,
    OBJECT_ATTRIBUTE_KEYS,
)
from .data_assoc import Detection, associate

__all__ = [
    "SceneObject",
    "SceneSurface",
    "Pose3D",
    "BBox3D",
    "ObjectRegistry",
    "OBJECT_ATTRIBUTE_KEYS",
    "Detection",
    "associate",
]
