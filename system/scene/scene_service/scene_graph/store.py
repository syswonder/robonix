# SPDX-License-Identifier: MulanPSL-2.0
"""Scene graph store — holds the latest snapshot and manages caches.

Cache persistence uses simple JSON files so the system can survive
restarts without re-calling the LLM for every object pair. Cache
I/O failures are swallowed — the worst case is a redundant LLM call.
"""
from __future__ import annotations

import json
import logging
import os
from pathlib import Path
from typing import Optional

from .types import SceneGraphEdge, SceneGraphNode, SceneGraphSnapshot

log = logging.getLogger(__name__)


def _round_tuple(t: tuple[float, ...], decimals: int = 1) -> str:
    return ",".join(f"{v:.{decimals}f}" for v in t)


class SceneGraphStore:
    """In-memory snapshot + on-disk JSON cache."""

    def __init__(self, cache_dir: str = "/data/robonix/scene_graph/cache") -> None:
        self._snapshot: Optional[SceneGraphSnapshot] = None
        self._cache_dir = Path(cache_dir)
        self._caption_cache: dict[str, str] = {}
        self._relation_cache: dict[str, SceneGraphEdge] = {}
        self._load_caches()

    # ── snapshot access ──────────────────────────────────────────────────

    def get_snapshot(self) -> Optional[SceneGraphSnapshot]:
        return self._snapshot

    def save_snapshot(self, snapshot: SceneGraphSnapshot) -> None:
        self._snapshot = snapshot

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
    def _relation_key(a: SceneGraphNode, b: SceneGraphNode) -> str:
        a_sig = f"{a.object_id}:{a.caption or a.label}:{_round_tuple(a.bbox_center)}"
        b_sig = f"{b.object_id}:{b.caption or b.label}:{_round_tuple(b.bbox_center)}"
        return f"{a_sig}__{b_sig}"

    def get_cached_relation(
        self, a: SceneGraphNode, b: SceneGraphNode
    ) -> Optional[SceneGraphEdge]:
        return self._relation_cache.get(self._relation_key(a, b))

    def put_cached_relation(
        self, a: SceneGraphNode, b: SceneGraphNode, edge: SceneGraphEdge
    ) -> None:
        self._relation_cache[self._relation_key(a, b)] = edge

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
