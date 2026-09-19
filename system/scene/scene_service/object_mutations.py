# SPDX-License-Identifier: MulanPSL-2.0
"""Epoch-checked coordination for Scene's derived-object mutations."""

from __future__ import annotations

import asyncio
import copy
import logging
import math
from typing import Any, Optional

from .state import BBox3D, ObjectRegistry, Pose3D, SceneObject

log = logging.getLogger("scene.object_mutations")


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

    def generation_supported(self) -> bool:
        """Whether the live mapping binding exposes a real generation counter.

        When it does not, the epoch check compares map_id alone and no longer
        orders two edits racing inside one map. Callers are told rather than
        left to infer it from the -1 sentinel."""
        return self.live_binding.get("generation") is not None

    async def snapshot_objects(self) -> tuple[dict, str, int, bool]:
        """Return registry objects and their epoch under the map-ops lock."""
        async with self.ops_lock:
            objects, _surfaces = await self.registry.snapshot()
            map_id, generation = self.current_epoch()
            return objects, map_id, generation, self.generation_supported()

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

    @staticmethod
    def _record_note(obj: SceneObject, note: str) -> None:
        """Store the caller's reason alongside the correction it explains.

        Operator corrections outrank perception and never expire, so the
        record has to be able to say why it exists. An empty note clears any
        earlier one rather than leaving a stale reason attached to a new edit."""
        text = str(note or "").strip()[:512]
        if text:
            obj.attributes["operator_note"] = text
        else:
            obj.attributes.pop("operator_note", None)

    def _invalidate_graph(self, object_id: str) -> None:
        """Drop only the caption/relation cache entries touching one
        object. A single edit must not wipe the whole map's LLM-derived
        cache; the full clear is reserved for flush_objects."""
        invalidate = getattr(self.scene_graph_store, "invalidate_object", None)
        if invalidate is not None:
            invalidate(object_id)
        else:  # pragma: no cover - legacy store
            self.scene_graph_store.clear_derived_state()

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

    # ── The entry points every protocol goes through ──────────────────────
    # MCP, the web API and gRPC differ in how a request arrives and how a
    # reply is shaped. They must not differ in what the operation does, so
    # they all land here rather than each assembling a call to the mechanism
    # below.

    def _persist_default(self, map_id: str, requested) -> bool:
        """Whether to write this correction into the map's snapshot.

        None -- the unasked case -- means "if there is one". A map that has
        never been saved has no snapshot to write into, and that is not a
        reason to refuse the edit: the session is editable, the edit simply
        lives as long as the session does, which is what the page already
        tells the reader about a temporary map.
        """
        if requested is not None:
            return bool(requested)
        meta = getattr(self, "map_meta", None)
        if meta is None:
            return False
        try:
            return meta.read(map_id) is not None
        except Exception:  # noqa: BLE001
            return False

    def resolve_epoch(
        self,
        expected_map_id: str = "",
        expected_generation: Optional[int] = None,
    ) -> tuple[str, int]:
        """The map epoch an edit is aimed at.

        A caller that names one is asserting "the object I saw, on the map I
        saw it on" -- the guard that stops an edit crossing a map switch. A
        caller that names none gets the current epoch, which is right for a
        script acting on what it just read and is exactly what an interactive
        client must not do.
        """
        current_id, current_generation = self.current_epoch()
        map_id = str(expected_map_id or "").strip()
        if not map_id:
            return current_id, current_generation
        try:
            generation = int(expected_generation)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            generation = current_generation
        return map_id, generation

    async def apply_label_correction(
        self,
        *,
        object_id: str,
        label: str,
        clear_override: bool = False,
        expected_map_id: str = "",
        expected_generation: Optional[int] = None,
        persist_to_snapshot: Optional[bool] = None,
        note: str = "",
    ):
        """Rename one object, or clear a previous rename. The one entry point.

        Persistence defaults to "if this map has a snapshot": a temporary
        session has none, and being unsaved is not a reason to refuse an edit
        on it.
        """
        map_id, generation = self.resolve_epoch(
            expected_map_id, expected_generation)
        persist = self._persist_default(map_id, persist_to_snapshot)
        return await self.update_label(
            object_id=object_id,
            label=label,
            clear_override=clear_override,
            expected_map_id=map_id,
            expected_generation=generation,
            persist_to_snapshot=persist,
            note=note,
        )

    async def remove_object(
        self,
        *,
        object_id: str,
        expected_map_id: str = "",
        expected_generation: Optional[int] = None,
        persist_to_snapshot: Optional[bool] = None,
        note: str = "",
    ):
        """Delete one object. The one entry point."""
        map_id, generation = self.resolve_epoch(
            expected_map_id, expected_generation)
        persist = self._persist_default(map_id, persist_to_snapshot)
        return await self.delete_object(
            object_id=object_id,
            expected_map_id=map_id,
            expected_generation=generation,
            persist_to_snapshot=persist,
            note=note,
        )

    async def update_label(
        self,
        *,
        object_id: str,
        label: str,
        clear_override: bool = False,
        expected_map_id: str,
        expected_generation: int,
        persist_to_snapshot: bool,
        note: str = "",
    ) -> tuple[SceneObject, bool, str, int]:
        # Operator labels are stored verbatim. Canonicalizing here silently
        # rewrote corrections ("desk" became "table") — the human said desk,
        # the record shows desk.
        normalized = str(label or "").strip()
        # Length is checked in both modes. `label` is ignored when the caller
        # clears the override, but accepting an unbounded string there just
        # because it will be dropped invites a caller to believe it was used.
        if len(normalized) > 128:
            raise ValueError("label must not exceed 128 characters")
        if not clear_override and not normalized:
            raise ValueError("label must contain 1 to 128 characters")
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
                    self._record_note(updated, "")
            else:
                if update_detector is not None:
                    await update_detector(object_id, normalized)
                async with self.registry.lock():
                    updated = self.registry.update_object_label(
                        object_id,
                        normalized,
                    )
                    self._record_note(updated, note)

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

            self._invalidate_graph(object_id)
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
        note: str = "",
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
                self._record_note(updated, note)

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

            self._invalidate_graph(object_id)
            return updated, persisted, map_id, generation

    async def delete_object(
        self,
        *,
        object_id: str,
        expected_map_id: str,
        expected_generation: int,
        persist_to_snapshot: bool,
        note: str = "",
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
            self._invalidate_graph(object_id)
            if str(note or "").strip():
                log.info(
                    "operator deleted %s (persisted=%s): %s",
                    object_id, persisted, str(note).strip()[:512],
                )
            return object_id, persisted, map_id, generation

    async def flush_objects(
        self,
        *,
        expected_map_id: str,
        expected_generation: int,
        persist_to_snapshot: bool,
        note: str = "",
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
            if str(note or "").strip():
                log.info(
                    "operator flushed %d objects (persisted=%s): %s",
                    deleted_count, persisted, str(note).strip()[:512],
                )
            return deleted_count, persisted, map_id, generation
