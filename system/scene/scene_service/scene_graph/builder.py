# SPDX-License-Identifier: MulanPSL-2.0
"""SceneGraphBuilder — async rebuild loop that reads ObjectRegistry,
generates captions, infers relations via LLM, and stores a snapshot.

Runs as a low-frequency background task (default 30 s) and never
blocks the real-time perception tick.
"""
from __future__ import annotations

import asyncio
import logging
import os
import time
from typing import Optional

from ..state.object_registry import ObjectRegistry, SceneObject
from .captioner import NodeCaptioner
from .relations import RelationInferer, generate_edge_candidates
from .store import SceneGraphStore
from .types import SceneGraphEdge, SceneGraphNode, SceneGraphSnapshot

log = logging.getLogger(__name__)


# ── config ───────────────────────────────────────────────────────────────────

def _env_int(key: str, default: int) -> int:
    try:
        return int(os.environ.get(key, str(default)))
    except ValueError:
        return default


def _env_float(key: str, default: float) -> float:
    try:
        return float(os.environ.get(key, str(default)))
    except ValueError:
        return default


def _env_bool(key: str, default: bool) -> bool:
    val = os.environ.get(key, str(default)).lower()
    return val in ("true", "1", "yes")


class SceneGraphConfig:
    """Pull scene-graph config from environment variables."""

    def __init__(self) -> None:
        self.interval_sec = _env_float("SCENE_GRAPH_INTERVAL_SEC", 30.0)
        self.min_observations = _env_int("SCENE_GRAPH_MIN_OBSERVATIONS", 2)
        self.max_objects = _env_int("SCENE_GRAPH_MAX_OBJECTS", 80)
        self.max_candidate_edges = _env_int("SCENE_GRAPH_MAX_CANDIDATE_EDGES", 200)
        self.max_llm_relations_per_cycle = _env_int(
            "SCENE_GRAPH_MAX_LLM_RELATIONS_PER_CYCLE", 20
        )
        self.caption_enabled = _env_bool("SCENE_GRAPH_CAPTION_ENABLED", True)
        self.relation_enabled = _env_bool("SCENE_GRAPH_RELATION_ENABLED", True)


# ── helpers ──────────────────────────────────────────────────────────────────

def _is_stable(obj: SceneObject, min_obs: int) -> bool:
    return (
        obj.observation_count >= min_obs
        and not obj.missing
        and not obj.attributes.get("is_robot", False)
        and obj.bbox.size_x > 0
        and obj.bbox.size_y > 0
        and obj.bbox.size_z > 0
    )


def _object_to_node(obj: SceneObject) -> SceneGraphNode:
    return SceneGraphNode(
        object_id=obj.object_id,
        label=obj.cls,
        bbox_center=(obj.pose.x, obj.pose.y, obj.pose.z),
        bbox_extent=(obj.bbox.size_x, obj.bbox.size_y, obj.bbox.size_z),
        yaw=obj.pose.yaw,
        confidence=obj.confidence,
        observation_count=obj.observation_count,
        last_seen=obj.last_seen,
    )


# ── builder ──────────────────────────────────────────────────────────────────

class SceneGraphBuilder:
    """Reads ObjectRegistry, generates captions and relations, stores result."""

    def __init__(
        self,
        registry: ObjectRegistry,
        captioner: NodeCaptioner,
        relation_inferer: RelationInferer,
        store: SceneGraphStore,
        config: Optional[SceneGraphConfig] = None,
    ) -> None:
        self.registry = registry
        self.captioner = captioner
        self.relation_inferer = relation_inferer
        self.store = store
        self.cfg = config or SceneGraphConfig()

    async def rebuild_once(self) -> SceneGraphSnapshot:
        t0 = time.monotonic()

        # 1. Snapshot registry.
        objs_dict, _ = await self.registry.snapshot()

        # 2. Filter stable objects.
        stable = [
            o for o in objs_dict.values()
            if _is_stable(o, self.cfg.min_observations)
        ]
        # Sort by observation_count desc, truncate.
        stable.sort(key=lambda o: -o.observation_count)
        stable = stable[: self.cfg.max_objects]

        # 3. Convert to SceneGraphNodes.
        nodes = [_object_to_node(o) for o in stable]

        # 4. Caption.
        if self.cfg.caption_enabled:
            for node in nodes:
                cached = self.store.get_cached_caption(node)
                if cached:
                    node.caption = cached
                else:
                    try:
                        await self.captioner.caption_node(node)
                    except Exception:  # noqa: BLE001
                        node.caption = node.label
                    self.store.put_cached_caption(node)
        else:
            for node in nodes:
                node.caption = node.label

        # 5. Generate candidate edges.
        edges: list[SceneGraphEdge] = []
        if self.cfg.relation_enabled and len(nodes) >= 2:
            candidates = generate_edge_candidates(
                nodes,
                max_candidates=self.cfg.max_candidate_edges,
            )

            # 6. Infer relations (from cache or LLM).
            llm_calls = 0
            for a, b, hint in candidates:
                cached_edge = self.store.get_cached_relation(a, b)
                if cached_edge is not None:
                    if cached_edge.relation not in ("none", "unknown"):
                        edges.append(cached_edge)
                    continue

                # Rate-limit LLM calls per cycle.
                if llm_calls >= self.cfg.max_llm_relations_per_cycle:
                    continue

                try:
                    edge = await self.relation_inferer.infer_relation(a, b, hint)
                except Exception as e:  # noqa: BLE001
                    log.warning("[scene-graph] relation inference error: %s", e)
                    edge = SceneGraphEdge(
                        source_id=a.object_id,
                        target_id=b.object_id,
                        relation="unknown",
                        method="llm_fail",
                        reason=str(e),
                    )
                llm_calls += 1

                self.store.put_cached_relation(a, b, edge)
                if edge.relation not in ("none", "unknown"):
                    edges.append(edge)

        # 7. Build and save snapshot.
        snapshot = SceneGraphSnapshot(
            nodes={n.object_id: n for n in nodes},
            edges=edges,
            updated_at=time.time(),
        )
        self.store.save_snapshot(snapshot)
        self.store.flush_caches()

        dt = time.monotonic() - t0
        log.info(
            "[scene-graph] rebuild: %d nodes, %d edges, %.1fs",
            len(nodes),
            len(edges),
            dt,
        )
        return snapshot


# ── async loop ───────────────────────────────────────────────────────────────

async def scene_graph_loop(
    builder: SceneGraphBuilder,
    stop: asyncio.Event,
) -> None:
    """Background loop that rebuilds the scene graph periodically."""
    interval = builder.cfg.interval_sec
    log.info(
        "[scene-graph] loop started (interval=%.0fs, min_obs=%d, max_obj=%d)",
        interval,
        builder.cfg.min_observations,
        builder.cfg.max_objects,
    )
    while not stop.is_set():
        try:
            await builder.rebuild_once()
        except Exception:  # noqa: BLE001
            log.exception("[scene-graph] rebuild failed")
        try:
            await asyncio.wait_for(stop.wait(), timeout=interval)
        except asyncio.TimeoutError:
            pass
