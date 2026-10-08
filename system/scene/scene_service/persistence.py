# SPDX-License-Identifier: MulanPSL-2.0
"""Scene objects in an embedded milvus-lite DB owned by the scene process.

Rows are partitioned by the `map_id` field, because a pose only means
something in the map frame it was observed in. Each Save writes a fresh
partition token (`"<map_id>__s<seq>"`, from `map_meta`) so two saves of a
same-named map never mix frames; the primary key is `"{partition}::{object_id}"`.
Each row carries a caption embedding, or a placeholder vector when no
embedder is wired.
"""
from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Callable, Optional

from .map_binding import sanitize_map_id as _sanitize_map_id
from .state.object_registry import BBox3D, Pose3D, SceneObject

log = logging.getLogger(__name__)

_COLLECTION = "scene_objects"

# open_clip ViT-B-32 feature size.
_DIM = 512

# A unit vector, not zeros, so COSINE distance stays defined.
_PLACEHOLDER_VEC = [1.0] + [0.0] * (_DIM - 1)

EmbedFn = Callable[[list[str]], Optional[list[list[float]]]]


class ObjectStore:
    """Latest state of each object per partition. Synchronous: callers run
    it off the event loop."""

    def __init__(self, db_path: str, *, map_id: str = "default", dim: int = _DIM) -> None:
        from pymilvus import MilvusClient

        self._dim = dim
        self._map_id = _sanitize_map_id(map_id)
        self._embed: Optional[EmbedFn] = None
        resolved = str(Path(db_path).expanduser())
        Path(resolved).parent.mkdir(parents=True, exist_ok=True)
        self._uri = resolved
        self._client = MilvusClient(uri=resolved)
        self._ensure_collection()

    @property
    def map_id(self) -> str:
        return self._map_id

    def _partition(self, partition: Optional[str]) -> str:
        """The explicit partition, else the store's own map id."""
        return self._map_id if partition is None else _sanitize_map_id(partition)

    # ── schema ───────────────────────────────────────────────────────────
    def _ensure_collection(self) -> None:
        """Create the collection, or load an existing current one (milvus-lite
        3.x reopens collections released). A collection from before per-map
        partitioning cannot hold valid snapshots and is recreated."""
        if self._client.has_collection(_COLLECTION):
            if self._schema_current():
                self._client.load_collection(_COLLECTION)
                return
            log.warning(
                "[scene-persist] recreating %s — schema is outdated (missing "
                "the composite primary key or a current field); the warm-"
                "restore cache cold-starts once", _COLLECTION,
            )
            self._client.drop_collection(_COLLECTION)
        from pymilvus import DataType

        schema = self._client.create_schema(
            enable_dynamic_field=True,
            description="scene object current-state for warm restart",
        )
        schema.add_field("pk", DataType.VARCHAR, max_length=384, is_primary=True)
        schema.add_field("object_id", DataType.VARCHAR, max_length=128)
        schema.add_field("map_id", DataType.VARCHAR, max_length=128)
        schema.add_field("embedding", DataType.FLOAT_VECTOR, dim=self._dim)
        # The object's `label`; the old column name keeps saved maps loadable.
        schema.add_field("cls", DataType.VARCHAR, max_length=128)
        schema.add_field("caption", DataType.VARCHAR, max_length=4096)
        schema.add_field("x", DataType.FLOAT)
        schema.add_field("y", DataType.FLOAT)
        schema.add_field("z", DataType.FLOAT)
        schema.add_field("yaw", DataType.FLOAT)
        schema.add_field("frame_id", DataType.VARCHAR, max_length=64)
        # The bbox shares the pose's frame_id.
        schema.add_field("size_x", DataType.FLOAT)
        schema.add_field("size_y", DataType.FLOAT)
        schema.add_field("size_z", DataType.FLOAT)
        schema.add_field("bbox_yaw", DataType.FLOAT)
        schema.add_field("confidence", DataType.FLOAT)
        schema.add_field("first_seen", DataType.DOUBLE)
        schema.add_field("last_seen", DataType.DOUBLE)
        schema.add_field("observation_count", DataType.INT64)
        schema.add_field("version", DataType.INT64)
        schema.add_field("attributes", DataType.VARCHAR, max_length=4096)  # JSON

        index_params = self._client.prepare_index_params()
        index_params.add_index(
            field_name="embedding", index_type="FLAT", metric_type="COSINE"
        )
        self._client.create_collection(
            collection_name=_COLLECTION,
            schema=schema,
            index_params=index_params,
        )
        log.info("[scene-persist] created collection %s at %s", _COLLECTION, self._uri)

    def _schema_current(self) -> bool:
        return self._has_composite_pk() and self._has_field("bbox_yaw")

    def _has_composite_pk(self) -> bool:
        try:
            desc = self._client.describe_collection(_COLLECTION)
        except Exception:  # noqa: BLE001
            return False
        for f in desc.get("fields", []):
            if f.get("is_primary") or f.get("is_primary_key"):
                return f.get("name") == "pk"
        return False

    def _has_field(self, name: str) -> bool:
        try:
            desc = self._client.describe_collection(_COLLECTION)
        except Exception:  # noqa: BLE001
            return False
        return any(f.get("name") == name for f in desc.get("fields", []))

    # ── embedder wiring ──────────────────────────────────────────────────
    def set_embedder(self, embed_text: Optional[EmbedFn]) -> None:
        self._embed = embed_text

    def _embed_captions(self, captions: list[str]) -> list[list[float]]:
        if self._embed is None:
            return [_PLACEHOLDER_VEC] * len(captions)
        try:
            vecs = self._embed(captions)
        except Exception as e:  # noqa: BLE001
            log.warning("[scene-persist] embed failed, using placeholder: %s", e)
            vecs = None
        if not vecs or len(vecs) != len(captions):
            return [_PLACEHOLDER_VEC] * len(captions)
        return vecs

    # ── write ────────────────────────────────────────────────────────────
    def persist(self, pairs: list[tuple[SceneObject, Optional[str]]],
                *, partition: Optional[str] = None) -> int:
        """Upsert `(object, caption)` pairs; a None caption falls back to the
        label. Returns the rows written, 0 on a (logged) milvus error, so the
        Save path can compare it with `len(pairs)`."""
        if not pairs:
            return 0
        target = self._partition(partition)
        # An object's own caption is the one to keep; the pair's is only a
        # search text for one that has none.
        captions = [obj.caption if obj.caption_source else (cap or obj.label)
                    for obj, cap in pairs]
        vecs = self._embed_captions(captions)
        rows = []
        for (obj, _cap), caption, vec in zip(pairs, captions, vecs):
            rows.append(
                {
                    "pk": f"{target}::{obj.object_id}",
                    "object_id": obj.object_id,
                    "map_id": target,
                    "embedding": vec,
                    "cls": obj.label,
                    "caption": caption,
                    "x": obj.pose.x,
                    "y": obj.pose.y,
                    "z": obj.pose.z,
                    "yaw": obj.pose.yaw,
                    "frame_id": obj.pose.frame_id,
                    "size_x": obj.bbox.size_x,
                    "size_y": obj.bbox.size_y,
                    "size_z": obj.bbox.size_z,
                    "bbox_yaw": obj.bbox.yaw,
                    "confidence": obj.confidence,
                    "first_seen": obj.first_seen,
                    "last_seen": obj.last_seen,
                    "observation_count": obj.observation_count,
                    "version": obj.observation_count,
                    # Who wrote the caption, so a person's survives a reload.
                    "attributes": json.dumps(
                        {**obj.attributes, "caption_source": obj.caption_source},
                        ensure_ascii=False),
                }
            )
        try:
            self._client.upsert(collection_name=_COLLECTION, data=rows)
        except Exception as e:  # noqa: BLE001
            log.warning("[scene-persist] upsert failed: %s", e)
            return 0
        return len(rows)

    # ── read ─────────────────────────────────────────────────────────────
    def load_all(self, *, partition: Optional[str] = None,
                 limit: int = 16384, strict: bool = False) -> list[SceneObject]:
        """One partition's objects, restored as `missing` until re-observed.

        A query error yields [] with an error log, or raises with `strict`
        (Load), where "unreadable" must not look like "empty"."""
        target = self._partition(partition)
        try:
            # Partitions are sanitized, so interpolation is injection-safe.
            rows = self._client.query(
                collection_name=_COLLECTION,
                filter=f'map_id == "{target}"',
                output_fields=[
                    "object_id", "cls", "caption", "x", "y", "z", "yaw",
                    "frame_id", "size_x", "size_y", "size_z", "bbox_yaw",
                    "confidence", "first_seen", "last_seen",
                    "observation_count", "attributes",
                ],
                limit=limit,
            )
        except Exception as e:  # noqa: BLE001
            if strict:
                raise RuntimeError(f"load_all({target}) failed: {e}") from e
            log.error("[scene-persist] load_all failed — restoring nothing: %s", e)
            return []

        objs: list[SceneObject] = []
        for r in rows:
            frame_id = str(r.get("frame_id") or "").strip()
            if not frame_id:
                log.warning(
                    "[scene-persist] skipping %s: stored spatial frame is missing",
                    r.get("object_id", "<unknown>"),
                )
                continue
            try:
                attrs = json.loads(r.get("attributes") or "{}")
            except (TypeError, ValueError):
                attrs = {}
            objs.append(
                SceneObject(
                    object_id=r["object_id"],
                    label=r["cls"],
                    pose=Pose3D(
                        x=r["x"], y=r["y"], z=r["z"], yaw=r["yaw"],
                        frame_id=frame_id,
                    ),
                    bbox=BBox3D(
                        size_x=r["size_x"], size_y=r["size_y"], size_z=r["size_z"],
                        yaw=r.get("bbox_yaw", 0.0),
                        frame_id=frame_id,
                    ),
                    confidence=r["confidence"],
                    first_seen=r["first_seen"],
                    last_seen=r["last_seen"],
                    observation_count=int(r["observation_count"]),
                    missing=True,
                    attributes=attrs,
                )
            )
            source = attrs.pop("caption_source", "")
            if source:  # rows saved before captions were kept have none
                objs[-1].caption = str(r.get("caption") or "")
                objs[-1].caption_source = source
        return objs

    def delete_map(self, map_id: str) -> int:
        """Delete one map's rows; returns how many were found (capped count)."""
        target = _sanitize_map_id(map_id)
        try:
            return self._delete_where(f'map_id == "{target}"')
        except Exception as e:  # noqa: BLE001
            raise RuntimeError(f"delete_map({target}) failed: {e}") from e

    def delete_object(
        self,
        object_id: str,
        *,
        partition: Optional[str] = None,
    ) -> bool:
        """Delete one object's row from one partition; True when it existed."""
        target = self._partition(partition)
        predicate = f"pk == {json.dumps(f'{target}::{object_id}')}"
        try:
            return bool(self._delete_where(predicate, limit=1))
        except Exception as e:  # noqa: BLE001
            raise RuntimeError(
                f"delete_object({target}, {object_id}) failed: {e}"
            ) from e

    def purge_live_partitions(self) -> int:
        """Delete rows old builds left under `.live*` partitions; 0 on error,
        since cleanup must never block a boot."""
        try:
            return self._delete_where('map_id like ".live%"')
        except Exception as e:  # noqa: BLE001
            log.warning("[scene-persist] live-partition cleanup failed: %s", e)
            return 0

    def _delete_where(self, predicate: str, limit: int = 16384) -> int:
        rows = self._client.query(
            collection_name=_COLLECTION, filter=predicate,
            output_fields=["pk"], limit=limit,
        )
        if rows:
            self._client.delete(collection_name=_COLLECTION, filter=predicate)
        return len(rows)

    def close(self) -> None:
        """Close the client and release the milvus-lite server so the next
        boot can reopen the same `.db` file."""
        try:
            self._client.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            from milvus_lite.server_manager import server_manager_instance

            server_manager_instance.release_server(self._uri)
        except Exception:  # noqa: BLE001
            pass
