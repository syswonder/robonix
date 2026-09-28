# SPDX-License-Identifier: MulanPSL-2.0
"""Live scene graph plus the on-disk relation cache.

Two asyncio writers own disjoint slices: the geometric loop writes nodes and
geometric edges, the builder writes semantic edges. The relation cache is
persisted as JSON so a restart does not re-ask the LLM about every pair; its
I/O failures only cost a redundant call.
"""
from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import Optional

from .geometry import geometry_signature
from .types import GeometryHint, SceneGraphEdge, SceneGraphNode, SceneGraphSnapshot

log = logging.getLogger(__name__)


class SceneGraphStore:
    """In-memory live graph (two writer slices) + on-disk relation cache."""

    def __init__(
        self,
        cache_dir: str = "/data/robonix/scene_graph/cache",
        map_id: str | None = None,
    ) -> None:
        # Cached answers are only valid in the map frame they were computed
        # in, so each map gets its own cache directory.
        self._nodes: dict[str, SceneGraphNode] = {}
        self._geometric_edges: list[SceneGraphEdge] = []
        self._semantic_edges: list[SceneGraphEdge] = []
        self._geometric_updated_at: float = 0.0
        self._semantic_updated_at: float = 0.0
        base = Path(cache_dir)
        if map_id:
            from ..persistence import _sanitize_map_id

            base = base / _sanitize_map_id(map_id)
        self._cache_dir = base
        self._relation_cache: dict[str, SceneGraphEdge] = {}
        self._load_caches()

    # ── live graph slices ────────────────────────────────────────────────

    def set_geometric(
        self, nodes: list[SceneGraphNode], edges: list[SceneGraphEdge]
    ) -> None:
        self._nodes = {n.object_id: n for n in nodes}
        self._geometric_edges = edges
        self._geometric_updated_at = time.time()

    def set_semantic_edges(self, edges: list[SceneGraphEdge]) -> None:
        self._semantic_edges = edges
        self._semantic_updated_at = time.time()

    def get_semantic_edges(self) -> list[SceneGraphEdge]:
        return self._semantic_edges

    def get_geometric_edges(self) -> list[SceneGraphEdge]:
        return self._geometric_edges

    def get_snapshot(self) -> Optional[SceneGraphSnapshot]:
        """Both slices composed for readers, or None before anything was
        published. A (source, target, relation) in both slices is kept once,
        geometric first."""
        if not (self._nodes or self._geometric_edges or self._semantic_edges):
            return None
        edges: list[SceneGraphEdge] = []
        seen: set[tuple[str, str, str]] = set()
        for e in list(self._geometric_edges) + list(self._semantic_edges):
            key = (e.source_id, e.target_id, e.relation)
            if key in seen:
                continue
            seen.add(key)
            edges.append(e)
        return SceneGraphSnapshot(
            nodes=dict(self._nodes),
            edges=edges,
            updated_at=max(self._geometric_updated_at, self._semantic_updated_at),
        )

    # ── relation cache ───────────────────────────────────────────────────

    @staticmethod
    def _relation_key(
        a: SceneGraphNode, b: SceneGraphNode, hint: GeometryHint,
        semantic_only: bool = False,
    ) -> str:
        # The bucketed geometry signature survives pose jitter but changes
        # when an object really moves. The mode suffix keeps semantic-only
        # and full answers for the same pair apart.
        mode = "s" if semantic_only else "f"
        return f"{a.object_id}__{b.object_id}__{geometry_signature(hint)}__{mode}"

    def get_cached_relation(
        self, a: SceneGraphNode, b: SceneGraphNode, hint: GeometryHint,
        semantic_only: bool = False,
    ) -> Optional[SceneGraphEdge]:
        return self._relation_cache.get(self._relation_key(a, b, hint, semantic_only))

    def put_cached_relation(
        self, a: SceneGraphNode, b: SceneGraphNode, hint: GeometryHint,
        edge: SceneGraphEdge, semantic_only: bool = False,
    ) -> None:
        # One entry per pair: a pair drifting through signature buckets would
        # otherwise leave one stale entry per bucket.
        prefix = f"{a.object_id}__{b.object_id}__"
        for k in [k for k in self._relation_cache if k.startswith(prefix)]:
            del self._relation_cache[k]
        self._relation_cache[self._relation_key(a, b, hint, semantic_only)] = edge

    def invalidate_object(self, object_id: str) -> int:
        """Drop the cached relations that mention one object; returns how
        many. Callers decide when a disk flush is worth it."""
        endpoint = f"{object_id}__"
        infix = f"__{object_id}__"
        stale = [
            k for k in self._relation_cache
            if k.startswith(endpoint) or infix in k
        ]
        for key in stale:
            del self._relation_cache[key]
        return len(stale)

    def clear_derived_state(self) -> None:
        """Clear both live slices and the cache (flush-scale operations)."""
        self._nodes.clear()
        self._geometric_edges.clear()
        self._semantic_edges.clear()
        self._geometric_updated_at = 0.0
        self._semantic_updated_at = 0.0
        self._relation_cache.clear()
        self.flush_caches()

    def prune_relations(self, live_object_ids: set[str]) -> int:
        """Drop cached edges with an endpoint no longer in the registry."""
        stale = [
            k for k, e in self._relation_cache.items()
            if e.source_id not in live_object_ids
            or e.target_id not in live_object_ids
        ]
        for k in stale:
            del self._relation_cache[k]
        return len(stale)

    # ── persistence ──────────────────────────────────────────────────────

    def _load_caches(self) -> None:
        for k, v in self._read_json("relations.json", default={}).items():
            try:
                self._relation_cache[k] = SceneGraphEdge(**v)
            except (TypeError, KeyError):
                pass

    def flush_caches(self) -> None:
        self._write_json("relations.json", {
            k: {
                "source_id": edge.source_id,
                "target_id": edge.target_id,
                "relation": edge.relation,
                "confidence": edge.confidence,
                "method": edge.method,
                "reason": edge.reason,
                "updated_at": edge.updated_at,
            }
            for k, edge in self._relation_cache.items()
        })

    def _read_json(self, filename: str, default: dict) -> dict:
        path = self._cache_dir / filename
        if not path.exists():
            return default
        try:
            with open(path, "r") as f:
                return json.load(f)
        except Exception:  # noqa: BLE001
            log.debug("[scene-graph-store] failed to read %s", path)
            return default

    def _write_json(self, filename: str, data: dict) -> None:
        try:
            self._cache_dir.mkdir(parents=True, exist_ok=True)
            with open(self._cache_dir / filename, "w") as f:
                json.dump(data, f, ensure_ascii=False, indent=1)
        except Exception:  # noqa: BLE001
            log.debug("[scene-graph-store] failed to write %s", filename)
