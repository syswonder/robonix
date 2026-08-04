# SPDX-License-Identifier: MulanPSL-2.0
"""Epoch-checked coordination for Scene's derived-object mutations."""

from __future__ import annotations

import asyncio
import copy
import math
from typing import Any

from .ingest.perception_vlm import _canon_class
from .state import BBox3D, ObjectRegistry, Pose3D, SceneObject


class ObjectMutationCoordinator:
    """Keep registry, detector, graph cache, and saved snapshot coherent."""

    def __init__(
        self,
        *,
        registry: ObjectRegistry,
        detector: Any,
        scene_graph_store: Any,
        live_binding: dict,
        ops_lock: asyncio.Lock,
        semantic_hold: dict,
        object_store: Any = None,
        map_meta: Any = None,
    ) -> None:
        self.registry = registry
        self.detector = detector
        self.scene_graph_store = scene_graph_store
        self.live_binding = live_binding
        self.ops_lock = ops_lock
        self.semantic_hold = semantic_hold
        self.object_store = object_store
        self.map_meta = map_meta

    def current_epoch(self) -> tuple[str, int]:
        map_id = str(self.live_binding.get("map_id") or "")
        generation = self.live_binding.get("generation")
        return map_id, -1 if generation is None else int(generation)

    async def snapshot_objects(self) -> tuple[dict, str, int]:
        """Return registry objects and their epoch under the map-ops lock."""
        async with self.ops_lock:
            objects, _surfaces = await self.registry.snapshot()
            map_id, generation = self.current_epoch()
            return objects, map_id, generation

    def _assert_epoch(self, map_id: str, generation: int) -> tuple[str, int]:
        current_map_id, current_generation = self.current_epoch()
        if str(map_id) != current_map_id or int(generation) != current_generation:
            raise RuntimeError(
                "Scene map epoch changed: requested "
                f"({map_id!r}, {generation}), current "
                f"({current_map_id!r}, {current_generation}); refresh "
                "list_objects and retry"
            )
        hold_reason = self.semantic_hold.get("reason")
        if hold_reason:
            raise RuntimeError(
                "Scene semantic state is not mutation-safe: "
                f"{hold_reason}; finish or retry the map operation first"
            )
        return current_map_id, current_generation

    def _snapshot_partition(self, map_id: str) -> str:
        if self.object_store is None or self.map_meta is None or not map_id:
            raise RuntimeError(
                "no committed semantic snapshot is available; save the map "
                "before requesting persist_to_snapshot"
            )
        meta = self.map_meta.read(map_id)
        if meta is None:
            raise RuntimeError(
                f"map {map_id!r} has no semantic snapshot; save it before "
                "requesting persist_to_snapshot"
            )
        return str(meta.object_partition)

    async def _persist_one(self, obj: SceneObject, partition: str) -> None:
        written = int(
            await asyncio.to_thread(
                self.object_store.persist,
                [(obj, None)],
                partition=partition,
            )
        )
        if written != 1:
            raise RuntimeError("semantic snapshot write returned no object row")

    async def _restore_snapshot_rows(
        self,
        objects: list[SceneObject],
        partition: str,
    ) -> bool:
        if not objects:
            return True
        written = int(
            await asyncio.to_thread(
                self.object_store.persist,
                [(obj, None) for obj in objects],
                partition=partition,
            )
        )
        return written == len(objects)

    async def update_label(
        self,
        *,
        object_id: str,
        label: str,
        clear_override: bool = False,
        expected_map_id: str,
        expected_generation: int,
        persist_to_snapshot: bool,
    ) -> tuple[SceneObject, bool, str, int]:
        normalized = _canon_class(str(label or ""))
        if not clear_override and (
            not normalized or len(normalized) > 128
        ):
            raise ValueError("label must contain 1 to 128 normalized characters")
        async with self.ops_lock:
            map_id, generation = self._assert_epoch(
                expected_map_id,
                expected_generation,
            )
            partition = (
                self._snapshot_partition(map_id)
                if persist_to_snapshot
                else None
            )
            async with self.registry.lock():
                current = self.registry.get_object(object_id)
                if current is None:
                    raise KeyError(f"unknown Scene object {object_id!r}")
                if current.attributes.get("is_robot"):
                    raise ValueError("the robot self-object label cannot be edited")
                old_label = current.cls
                old_attributes = copy.deepcopy(current.attributes)

            previous_override = str(
                old_attributes.get("operator_label", "") or ""
            )
            update_detector = getattr(self.detector, "update_object_label", None)
            clear_detector = getattr(
                self.detector,
                "clear_object_label_override",
                None,
            )
            if clear_override:
                if not previous_override:
                    raise ValueError(
                        f"Scene object {object_id!r} has no operator "
                        "label override"
                    )
                if clear_detector is not None:
                    await clear_detector(object_id)
                async with self.registry.lock():
                    updated = self.registry.clear_object_label_override(
                        object_id
                    )
            else:
                if update_detector is not None:
                    await update_detector(object_id, normalized)
                async with self.registry.lock():
                    updated = self.registry.update_object_label(
                        object_id,
                        normalized,
                    )

            persisted = False
            if partition is not None:
                try:
                    await self._persist_one(updated, partition)
                except Exception as exc:
                    async with self.registry.lock():
                        updated.cls = old_label
                        updated.attributes = old_attributes
                    if previous_override and update_detector is not None:
                        await update_detector(object_id, previous_override)
                    else:
                        if clear_detector is not None:
                            await clear_detector(object_id)
                    raise RuntimeError(
                        "failed to persist the label update; runtime state "
                        "was rolled back"
                    ) from exc
                persisted = True

            self.scene_graph_store.clear_derived_state()
            return updated, persisted, map_id, generation

    async def update_geometry(
        self,
        *,
        object_id: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
        size_x: float,
        size_y: float,
        size_z: float,
        frame_id: str,
        expected_map_id: str,
        expected_generation: int,
        persist_to_snapshot: bool,
    ) -> tuple[SceneObject, bool, str, int]:
        values = tuple(
            float(value)
            for value in (x, y, z, yaw, size_x, size_y, size_z)
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("geometry values must all be finite")
        if any(value <= 0.0 for value in (size_x, size_y, size_z)):
            raise ValueError("bounding-box sizes must be positive metres")
        requested_frame = str(frame_id or "").strip()
        if not requested_frame:
            raise ValueError("frame_id is required")
        normalized_yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        async with self.ops_lock:
            map_id, generation = self._assert_epoch(
                expected_map_id,
                expected_generation,
            )
            partition = (
                self._snapshot_partition(map_id)
                if persist_to_snapshot
                else None
            )
            async with self.registry.lock():
                current = self.registry.get_object(object_id)
                if current is None:
                    raise KeyError(f"unknown Scene object {object_id!r}")
                if current.attributes.get("is_robot"):
                    raise ValueError(
                        "the robot self-object geometry cannot be edited"
                    )
                current_frame = str(current.pose.frame_id or "").strip()
                bbox_frame = str(current.bbox.frame_id or current_frame).strip()
                if (
                    not current_frame
                    or requested_frame != current_frame
                    or requested_frame != bbox_frame
                ):
                    raise ValueError(
                        "geometry frame mismatch: requested "
                        f"{requested_frame!r}, object pose={current_frame!r}, "
                        f"bbox={bbox_frame!r}"
                    )
                old_pose = copy.deepcopy(current.pose)
                old_bbox = copy.deepcopy(current.bbox)
                old_attributes = copy.deepcopy(current.attributes)

            update_detector = getattr(
                self.detector,
                "update_object_geometry_override",
                None,
            )
            if update_detector is not None:
                await update_detector(object_id)
            async with self.registry.lock():
                updated = self.registry.update_object_geometry(
                    object_id,
                    Pose3D(
                        float(x),
                        float(y),
                        float(z),
                        normalized_yaw,
                        requested_frame,
                    ),
                    BBox3D(
                        float(size_x),
                        float(size_y),
                        float(size_z),
                        normalized_yaw,
                        requested_frame,
                    ),
                )

            persisted = False
            if partition is not None:
                try:
                    await self._persist_one(updated, partition)
                except Exception as exc:
                    async with self.registry.lock():
                        updated.pose = old_pose
                        updated.bbox = old_bbox
                        updated.attributes = old_attributes
                    clear_override = getattr(
                        self.detector,
                        "clear_object_geometry_override",
                        None,
                    )
                    if clear_override is not None:
                        await clear_override(object_id)
                    raise RuntimeError(
                        "failed to persist the geometry update; runtime state "
                        "was rolled back"
                    ) from exc
                persisted = True

            self.scene_graph_store.clear_derived_state()
            return updated, persisted, map_id, generation

    async def delete_object(
        self,
        *,
        object_id: str,
        expected_map_id: str,
        expected_generation: int,
        persist_to_snapshot: bool,
    ) -> tuple[str, bool, str, int]:
        async with self.ops_lock:
            map_id, generation = self._assert_epoch(
                expected_map_id,
                expected_generation,
            )
            partition = (
                self._snapshot_partition(map_id)
                if persist_to_snapshot
                else None
            )
            async with self.registry.lock():
                obj = self.registry.get_object(object_id)
                if obj is None:
                    raise KeyError(f"unknown Scene object {object_id!r}")
                if obj.attributes.get("is_robot"):
                    raise ValueError("the robot self-object cannot be deleted")
                object_backup = copy.deepcopy(obj)

            persisted = False
            if partition is not None:
                await asyncio.to_thread(
                    self.object_store.delete_object,
                    object_id,
                    partition=partition,
                )
                persisted = True

            try:
                delete_detector = getattr(self.detector, "delete_object", None)
                if delete_detector is not None:
                    await delete_detector(object_id)
                async with self.registry.lock():
                    if self.registry.get_object(object_id) is not None:
                        self.registry.delete_derived_object(object_id)
            except Exception as exc:
                rollback_ok = True
                if partition is not None:
                    rollback_ok = await self._restore_snapshot_rows(
                        [object_backup],
                        partition,
                    )
                raise RuntimeError(
                    "failed to delete the runtime object; "
                    + (
                        "the saved snapshot was rolled back"
                        if rollback_ok
                        else "the saved snapshot rollback also failed"
                    )
                ) from exc
            self.scene_graph_store.clear_derived_state()
            return object_id, persisted, map_id, generation

    async def flush_objects(
        self,
        *,
        expected_map_id: str,
        expected_generation: int,
        persist_to_snapshot: bool,
    ) -> tuple[int, bool, str, int]:
        async with self.ops_lock:
            map_id, generation = self._assert_epoch(
                expected_map_id,
                expected_generation,
            )
            partition = (
                self._snapshot_partition(map_id)
                if persist_to_snapshot
                else None
            )
            async with self.registry.lock():
                runtime_backup = [
                    copy.deepcopy(obj)
                    for obj in self.registry.all_objects()
                    if not obj.attributes.get("is_robot")
                ]
            persisted = False
            if partition is not None:
                await asyncio.to_thread(
                    self.object_store.delete_map,
                    partition,
                )
                persisted = True

            try:
                reset = getattr(self.detector, "reset_derived_state", None)
                if reset is not None:
                    await reset()
                async with self.registry.lock():
                    deleted_count = self.registry.clear_derived_objects()
            except Exception as exc:
                rollback_ok = True
                if partition is not None:
                    rollback_ok = await self._restore_snapshot_rows(
                        runtime_backup,
                        partition,
                    )
                raise RuntimeError(
                    "failed to flush runtime derived state; "
                    + (
                        "the saved snapshot was rolled back"
                        if rollback_ok
                        else "the saved snapshot rollback also failed"
                    )
                ) from exc
            self.scene_graph_store.clear_derived_state()
            return deleted_count, persisted, map_id, generation
