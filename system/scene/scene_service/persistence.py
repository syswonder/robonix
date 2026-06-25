# SPDX-License-Identifier: MulanPSL-2.0
"""Object persistence — survives scene restarts so the registry warm-starts
instead of re-accumulating every stable object through the `min_observations`
filter from scratch.

Backed by an embedded milvus-lite DB owned solely by the scene process (one
`.db` file, single writer — no sharing with system/memory's memsearch DB).
Writes happen at the low-frequency scene-graph builder cadence; reads happen
once at boot. Each row carries a caption embedding so a future G3 object-search
layer can reuse the same table; when no embedder is wired (e.g. the VLM-fallback
perception path that has no CLIP), a fixed placeholder vector is stored so the
collection stays well-formed.

Rows are partitioned by `map_id`: an object's pose is only meaningful in the
`map` frame of the SLAM map it was observed on, so warm-restore loads exactly
the current map's objects and never mixes two maps. The primary key is the
composite `"{map_id}::{object_id}"` so the same `object_id` may exist on
different maps without colliding on upsert. Until mapping emits a real map
identity, `map_id` is a deploy-controlled string (`SCENE_MAP_ID`, default
`"default"`) — this stays internal to scene and touches no atlas/MCP contract.
"""
from __future__ import annotations

import json
import logging
import re
from pathlib import Path
from typing import Callable, Optional

from .state.object_registry import BBox3D, Pose3D, SceneObject

log = logging.getLogger(__name__)

_COLLECTION = "scene_objects"

# map_id is interpolated into a milvus filter expression, so restrict it to a
# safe identifier charset (no quotes / spaces) to keep the predicate injection
# -free. Anything else is squashed to "_"; empty falls back to "default".
_MAP_ID_UNSAFE = re.compile(r"[^A-Za-z0-9._\-]")


def _sanitize_map_id(raw: Optional[str]) -> str:
    cleaned = _MAP_ID_UNSAFE.sub("_", (raw or "").strip())
    return cleaned or "default"
# open_clip ViT-B-32 text/image features are 512-d (see
# ingest/perception_concept_graphs.py). Object image features already live in
# this space, so caption-text vectors are directly comparable for G3.
_DIM = 512

# Stored when no real embedder is available. A unit vector (not zeros) keeps
# COSINE distance well-defined.
_PLACEHOLDER_VEC = [1.0] + [0.0] * (_DIM - 1)

EmbedFn = Callable[[list[str]], Optional[list[list[float]]]]


class ObjectStore:
    """milvus-lite-backed store for `SceneObject` current-state.

    Upsert is keyed by the composite `"{map_id}::{object_id}"` primary key, so
    a row holds the latest state of an object on a given map (no version
    history in v1 — `version` mirrors `observation_count`).
    All methods are synchronous; callers run them off the asyncio loop or
    inside an already-held registry lock as appropriate.
    """

    def __init__(self, db_path: str, *, map_id: str = "default", dim: int = _DIM) -> None:
        """Open (or create) the milvus-lite DB at `db_path` and ensure the
        `scene_objects` collection exists. Creates parent dirs.

        `map_id` scopes every read/write of this store to one SLAM map; it is
        sanitized to a safe identifier (see `_sanitize_map_id`)."""
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
        """The sanitized map id this store is scoped to."""
        return self._map_id

    # ── schema ───────────────────────────────────────────────────────────
    def _ensure_collection(self) -> None:
        """Create the collection + index on first run; no-op when it already
        exists with the current composite-pk schema.

        Mirrors the field/index pattern used by system/memory's MilvusStore,
        but with an object-state schema instead of text chunks. A collection
        left by an earlier draft (legacy `object_id` primary key, before
        per-map partitioning) is dropped and recreated: that schema keys upsert
        on `object_id` alone, so two maps' identical ids would silently
        overwrite each other. The store is a best-effort warm-restore cache, so
        recreating it just forces a one-time cold start — never a boot failure."""
        if self._client.has_collection(_COLLECTION):
            if self._schema_current():
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
        # Composite primary key so the same object_id can live on two maps
        # without colliding on upsert. object_id + map_id are kept as their
        # own scalar fields for restore / filtering.
        schema.add_field("pk", DataType.VARCHAR, max_length=384, is_primary=True)
        schema.add_field("object_id", DataType.VARCHAR, max_length=128)
        schema.add_field("map_id", DataType.VARCHAR, max_length=128)
        schema.add_field("embedding", DataType.FLOAT_VECTOR, dim=self._dim)
        schema.add_field("cls", DataType.VARCHAR, max_length=128)
        schema.add_field("caption", DataType.VARCHAR, max_length=4096)
        # Pose (map frame).
        schema.add_field("x", DataType.FLOAT)
        schema.add_field("y", DataType.FLOAT)
        schema.add_field("z", DataType.FLOAT)
        schema.add_field("yaw", DataType.FLOAT)
        schema.add_field("frame_id", DataType.VARCHAR, max_length=64)
        # Bounding box: extents + yaw (orientation about z, same map frame as
        # the pose). bbox frame_id is not stored separately — it always equals
        # the pose frame_id, so restore reuses that.
        schema.add_field("size_x", DataType.FLOAT)
        schema.add_field("size_y", DataType.FLOAT)
        schema.add_field("size_z", DataType.FLOAT)
        schema.add_field("bbox_yaw", DataType.FLOAT)
        # Tracking metadata.
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
        """True iff the existing collection matches what we'd create now: the
        composite `pk` primary key AND the `bbox_yaw` field (added after the
        first persistence draft). A collection failing either check is dropped
        and recreated — cheap, since the store is a best-effort warm-restore
        cache and a recreate just forces a one-time cold start."""
        return self._has_composite_pk() and self._has_field("bbox_yaw")

    def _has_composite_pk(self) -> bool:
        """True iff the existing collection's primary key is the composite
        `pk` field. A legacy collection (primary key = `object_id`) returns
        False so the caller recreates it. Any describe error is treated as
        legacy (False) — safer to recreate than to write under a bad schema."""
        try:
            desc = self._client.describe_collection(_COLLECTION)
        except Exception:  # noqa: BLE001
            return False
        for f in desc.get("fields", []):
            if f.get("is_primary") or f.get("is_primary_key"):
                return f.get("name") == "pk"
        return False

    def _has_field(self, name: str) -> bool:
        """True iff the existing collection declares a field named `name`. A
        describe error is treated as absent (False) so the caller recreates."""
        try:
            desc = self._client.describe_collection(_COLLECTION)
        except Exception:  # noqa: BLE001
            return False
        return any(f.get("name") == name for f in desc.get("fields", []))

    # ── embedder wiring ──────────────────────────────────────────────────
    def set_embedder(self, embed_text: Optional[EmbedFn]) -> None:
        """Inject the caption→vector function (wired from perception's loaded
        CLIP text encoder). Safe to pass None — persistence then stores the
        placeholder vector."""
        self._embed = embed_text

    def _embed_captions(self, captions: list[str]) -> list[list[float]]:
        """Embed caption strings, falling back to the placeholder vector on a
        missing or failing embedder so a write never blocks on the model."""
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
    def persist(self, pairs: list[tuple[SceneObject, Optional[str]]]) -> int:
        """Upsert `(SceneObject, caption)` pairs under this store's `map_id`
        (composite pk `"{map_id}::{object_id}"`). Caption falls back to the
        class name when None. Returns the number of rows written;
        a milvus error is swallowed (logged) so a bad write never kills the
        builder tick."""
        if not pairs:
            return 0
        captions = [(cap or obj.cls) for obj, cap in pairs]
        vecs = self._embed_captions(captions)
        rows = []
        for (obj, _cap), caption, vec in zip(pairs, captions, vecs):
            rows.append(
                {
                    "pk": f"{self._map_id}::{obj.object_id}",
                    "object_id": obj.object_id,
                    "map_id": self._map_id,
                    "embedding": vec,
                    "cls": obj.cls,
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
                    "attributes": json.dumps(obj.attributes, ensure_ascii=False),
                }
            )
        try:
            self._client.upsert(collection_name=_COLLECTION, data=rows)
        except Exception as e:  # noqa: BLE001
            log.warning("[scene-persist] upsert failed: %s", e)
            return 0
        return len(rows)

    # ── read ─────────────────────────────────────────────────────────────
    def load_all(self, *, limit: int = 16384) -> list[SceneObject]:
        """Return this map's persisted objects as `SceneObject`s for warm
        restore (rows of other maps are filtered out by `map_id`).

        Restored objects come back `missing=True` (known but not currently
        observed) with their persisted `last_seen`; re-observation flips them
        back via data association. The embedding column is intentionally not
        read back — restore needs only scalar state. A query *error* is logged
        at error level and yields an empty list rather than failing boot — this
        is distinct from a genuinely empty DB, which returns `[]` silently."""
        try:
            # map_id is sanitized to a quote-free identifier in __init__, so
            # interpolating it into the predicate is injection-safe.
            rows = self._client.query(
                collection_name=_COLLECTION,
                filter=f'map_id == "{self._map_id}"',
                output_fields=[
                    "object_id", "cls", "caption", "x", "y", "z", "yaw",
                    "frame_id", "size_x", "size_y", "size_z", "bbox_yaw",
                    "confidence", "first_seen", "last_seen",
                    "observation_count", "attributes",
                ],
                limit=limit,
            )
        except Exception as e:  # noqa: BLE001
            # Not a cold start: the DB may hold rows we failed to read (bad
            # path, missing volume, schema mismatch). Surface it as an error so
            # a silently-empty graph isn't mistaken for "nothing persisted yet".
            log.error("[scene-persist] load_all failed — restoring nothing: %s", e)
            return []

        objs: list[SceneObject] = []
        for r in rows:
            try:
                attrs = json.loads(r.get("attributes") or "{}")
            except (TypeError, ValueError):
                attrs = {}
            objs.append(
                SceneObject(
                    object_id=r["object_id"],
                    cls=r["cls"],
                    pose=Pose3D(
                        x=r["x"], y=r["y"], z=r["z"], yaw=r["yaw"],
                        frame_id=r.get("frame_id") or "map",
                    ),
                    bbox=BBox3D(
                        size_x=r["size_x"], size_y=r["size_y"], size_z=r["size_z"],
                        yaw=r.get("bbox_yaw", 0.0),
                        frame_id=r.get("frame_id") or "map",
                    ),
                    confidence=r["confidence"],
                    first_seen=r["first_seen"],
                    last_seen=r["last_seen"],
                    observation_count=int(r["observation_count"]),
                    missing=True,
                    attributes=attrs,
                )
            )
        return objs

    # ── lifecycle ────────────────────────────────────────────────────────
    def close(self) -> None:
        """Close the client and release the milvus-lite server so the next
        boot can re-open the same `.db` file (mirrors memsearch's cleanup)."""
        try:
            self._client.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            from milvus_lite.server_manager import server_manager_instance

            server_manager_instance.release_server(self._uri)
        except Exception:  # noqa: BLE001
            pass
