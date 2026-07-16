# SPDX-License-Identifier: MulanPSL-2.0
"""Live scene graph and persistent caption/relation caches.

Two single-threaded asyncio writers own disjoint slices:

* the geometric loop updates nodes and deterministic geometric edges; and
* the model builder updates semantic edges that geometry cannot decide.

``get_snapshot()`` combines both slices for MCP and web consumers.

Cache persistence uses simple JSON files so the system can survive
restarts without re-calling the LLM for every object pair. Only the
caption + relation *caches* are persisted; the live node/edge slices are
rebuilt each tick. Cache I/O failures are swallowed — the worst case is a
redundant LLM call.
"""
from __future__ import annotations

import json
import logging
import os
import time
from pathlib import Path
from typing import Optional

from .geometry import geometry_signature
from .types import GeometryHint, SceneGraphEdge, SceneGraphNode, SceneGraphSnapshot

log = logging.getLogger(__name__)


class SceneGraphStore:
    """In-memory live graph (two writer slices) + on-disk JSON caches."""

    def __init__(
        self,
        cache_dir: str = "/data/robonix/scene_graph/cache",
        map_id: str | None = None,
    ) -> None:
        """Build the live graph + open the on-disk JSON caches.

        When ``map_id`` is given, the caption/relation caches are nested under
        ``cache_dir/<sanitized map_id>/`` so two SLAM maps never share a cache
        (caption/relation answers are only valid within the map frame they were
        computed in — the same per-map isolation the object store gets from its
        ``"{map_id}::{object_id}"`` composite key). The id is run through
        ``map_binding.sanitize_map_id`` (the one sanitize rule shared by every
        map_id-partitioned store; reached via persistence's alias, imported
        lazily so this stays importable without the milvus backend) so it is a
        safe path component. With no ``map_id`` the
        path is unchanged (legacy/"default" behaviour)."""
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
        self._caption_cache: dict[str, str] = {}
        self._relation_cache: dict[str, SceneGraphEdge] = {}
        self._load_caches()

    # ── live graph slices ────────────────────────────────────────────────

    def set_geometric(
        self, nodes: list[SceneGraphNode], edges: list[SceneGraphEdge]
    ) -> None:
        """Replace the geometric slice (nodes + contact/containment edges).
        The dict/list references are swapped in one step, so a concurrent
        get_snapshot() never sees a half-updated geometric layer. Called by
        the fast geometric loop each tick."""
        self._nodes = {n.object_id: n for n in nodes}
        self._geometric_edges = edges
        self._geometric_updated_at = time.time()

    def set_semantic_edges(self, edges: list[SceneGraphEdge]) -> None:
        """Replace the semantic (LLM-inferred) edge slice. Called by the
        builder once per rebuild, after residual inference + hysteresis."""
        self._semantic_edges = edges
        self._semantic_updated_at = time.time()

    def get_semantic_edges(self) -> list[SceneGraphEdge]:
        """Current semantic edges — the builder reads these to carry edges
        forward across rebuilds (hysteresis)."""
        return self._semantic_edges

    def get_geometric_edges(self) -> list[SceneGraphEdge]:
        """Current geometric (contact/containment) edges, owned by the fast
        loop. The builder reads these to skip pairs geometry already
        decided, spending LLM budget only on the residual."""
        return self._geometric_edges

    def get_snapshot(self) -> Optional[SceneGraphSnapshot]:
        """Compose the live graph for read-only consumers (MCP, web).

        Nodes come from the geometric loop; edges are the geometric slice
        followed by the semantic slice. The builder skips pairs geometry
        already decided, so the slices are disjoint by construction; the
        dedup on (source, target, relation) — geometric wins — only guards
        the ~1 s startup window before the geometric debounce emits, and
        keeps distinct relations on the same pair (e.g. geometric on_top_of
        + semantic attached_to). Returns None before the first geometric
        tick has populated anything, preserving the prior "no snapshot yet"
        contract for callers."""
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

    # ── caption cache ────────────────────────────────────────────────────

    @staticmethod
    def _caption_key(node: SceneGraphNode) -> str:
        return f"{node.object_id}:{node.label}:{node.observation_count // 5}"

    def get_cached_caption(self, node: SceneGraphNode) -> Optional[str]:
        return self._caption_cache.get(self._caption_key(node))

    def put_cached_caption(self, node: SceneGraphNode) -> None:
        if node.caption is not None:
            self._caption_cache[self._caption_key(node)] = node.caption

    # ── relation cache ───────────────────────────────────────────────────

    @staticmethod
    def _relation_key(
        a: SceneGraphNode, b: SceneGraphNode, hint: GeometryHint,
        semantic_only: bool = False,
    ) -> str:
        # Key on object identity PLUS a coarse geometry signature. Raw
        # label/caption/coords made the key drift on every float-level
        # position jitter (near-100% miss, redundant LLM calls, edge
        # flicker); keying on object_id alone went to the other extreme —
        # a pair's relation was computed once and frozen forever, never
        # invalidating when an object moved. The bucketed signature from
        # geometry_signature() is the middle ground: stable under EMA
        # jitter, but changes when the spatial configuration actually
        # changes, so a moved object's edge cache-misses and recomputes.
        # The mode suffix ('s'/'f') keeps a semantic-only answer (for a
        # geometry-owned pair) from being served to a full-inference query
        # of the same pair+signature, and vice versa.
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
        # Keep one entry per object-pair: the latest geometry signature.
        # The signature is in the key so a moved object cache-misses and
        # recomputes (the whole point of the geometry-signature key), but
        # without this eviction a pair that drifts through many signature
        # buckets would leave one stale entry per visited bucket forever.
        # A pair's prior spatial answer is superseded by the current one,
        # so dropping the old-signature entries is lossless in practice
        # (an object that returns to an old configuration just recomputes
        # once). prune_relations handles the orthogonal case of an
        # endpoint leaving the registry entirely.
        prefix = f"{a.object_id}__{b.object_id}__"
        for k in [k for k in self._relation_cache if k.startswith(prefix)]:
            del self._relation_cache[k]
        self._relation_cache[self._relation_key(a, b, hint, semantic_only)] = edge

    def prune_relations(self, live_object_ids: set[str]) -> int:
        """Drop cached edges with an endpoint no longer in the registry.
        Returns how many were evicted. Complements put_cached_relation's
        keep-latest-per-pair eviction: that bounds growth across an
        object's *moves*, this bounds it across object *departures*.
        Called by the builder each rebuild before flushing to disk."""
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
        self._caption_cache = self._read_json("captions.json", default={})
        rel_raw = self._read_json("relations.json", default={})
        for k, v in rel_raw.items():
            try:
                self._relation_cache[k] = SceneGraphEdge(**v)
            except (TypeError, KeyError):
                pass

    def flush_caches(self) -> None:
        self._write_json("captions.json", self._caption_cache)
        rel_out: dict[str, dict] = {}
        for k, edge in self._relation_cache.items():
            rel_out[k] = {
                "source_id": edge.source_id,
                "target_id": edge.target_id,
                "relation": edge.relation,
                "confidence": edge.confidence,
                "method": edge.method,
                "reason": edge.reason,
                "updated_at": edge.updated_at,
            }
        self._write_json("relations.json", rel_out)

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
            path = self._cache_dir / filename
            with open(path, "w") as f:
                json.dump(data, f, ensure_ascii=False, indent=1)
        except Exception:  # noqa: BLE001
            log.debug("[scene-graph-store] failed to write %s", filename)
