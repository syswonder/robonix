# SPDX-License-Identifier: MulanPSL-2.0
"""Low-frequency rebuild of the semantic edges from the ObjectRegistry."""
from __future__ import annotations

import asyncio
import logging
import os
import time
from typing import Any, Optional

from ..state.object_registry import ObjectRegistry, SceneObject
from .image_relations import ImageRelationInferer, ImageRelationResult
from .relations import RelationInferer, generate_edge_candidates
from .store import SceneGraphStore
from .types import SceneGraphEdge, SceneGraphNode, SceneGraphSnapshot

log = logging.getLogger(__name__)


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
    return os.environ.get(key, str(default)).lower() in ("true", "1", "yes")


class SceneGraphConfig:
    """Scene-graph settings from SCENE_GRAPH_* environment variables."""

    def __init__(self) -> None:
        self.interval_sec = _env_float("SCENE_GRAPH_INTERVAL_SEC", 30.0)
        self.min_observations = _env_int("SCENE_GRAPH_MIN_OBSERVATIONS", 2)
        self.max_objects = _env_int("SCENE_GRAPH_MAX_OBJECTS", 80)
        self.max_candidate_edges = _env_int("SCENE_GRAPH_MAX_CANDIDATE_EDGES", 200)
        self.max_llm_relations_per_cycle = _env_int(
            "SCENE_GRAPH_MAX_LLM_RELATIONS_PER_CYCLE", 20
        )
        self.relation_enabled = _env_bool("SCENE_GRAPH_RELATION_ENABLED", True)
        # Off forces the text-only per-pair inference.
        self.image_relations_enabled = _env_bool("SCENE_GRAPH_IMAGE_RELATIONS", True)
        # Rebuilds an unconfirmed edge survives, so one empty round does not
        # wipe the graph.
        self.max_stale_rounds = _env_int("SCENE_GRAPH_MAX_STALE_ROUNDS", 2)


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
        label=obj.label,
        bbox_center=(obj.pose.x, obj.pose.y, obj.pose.z),
        bbox_extent=(obj.bbox.size_x, obj.bbox.size_y, obj.bbox.size_z),
        yaw=obj.pose.yaw,
        confidence=obj.confidence,
        observation_count=obj.observation_count,
        last_seen=obj.last_seen,
        caption=obj.caption or obj.label,
    )


class SceneGraphBuilder:
    """Reads the registry, infers semantic edges, publishes them to the store."""

    def __init__(
        self,
        registry: ObjectRegistry,
        relation_inferer: RelationInferer,
        store: SceneGraphStore,
        config: Optional[SceneGraphConfig] = None,
        object_store: Any = None,
        perception: Any = None,
    ) -> None:
        self.registry = registry
        self.relation_inferer = relation_inferer
        self.store = store
        self.cfg = config or SceneGraphConfig()
        # Only set in the legacy SCENE_RESTORE_ON_START mode: each rebuild
        # upserts the stable objects so the registry can warm-restore at boot.
        self.object_store = object_store
        # A detector with latest_frame_bundle() enables the image relation pass.
        self.perception = perception
        self.image_inferer = (
            ImageRelationInferer(relation_inferer.llm_client)
            if self.cfg.image_relations_enabled
            else None
        )

    async def rebuild_once(self) -> SceneGraphSnapshot:
        t0 = time.monotonic()
        objs_dict = await self.registry.snapshot()
        # Includes `missing` objects: an edge dies when an endpoint leaves the
        # registry, not when it flickers for a tick.
        registry_ids = set(objs_dict.keys())

        stable = [
            o for o in objs_dict.values()
            if _is_stable(o, self.cfg.min_observations)
        ]
        stable.sort(key=lambda o: -o.observation_count)
        stable = stable[: self.cfg.max_objects]

        # Fewer than two stable objects is usually a perception blip, not an
        # empty scene: keep the last edges instead of wiping the graph.
        if len(stable) < 2:
            prev = self.store.get_semantic_edges()
            kept = [
                e for e in prev
                if e.source_id in registry_ids and e.target_id in registry_ids
            ]
            if len(kept) != len(prev):
                self.store.set_semantic_edges(kept)
            log.info(
                "[scene-graph] rebuild: %d stable node(s) (<2), preserved "
                "%d semantic edge(s)", len(stable), len(kept),
            )
            return SceneGraphSnapshot(nodes={}, edges=kept, updated_at=time.time())

        nodes = [_object_to_node(o) for o in stable]

        # Each path returns (edges, pairs it had an authoritative answer for);
        # prior edges of the other pairs survive through hysteresis below.
        edges: list[SceneGraphEdge] = []
        confirmed_pairs: set[tuple[str, str]] = set()
        if self.cfg.relation_enabled:
            image_result = await self._maybe_image_edges(nodes)
            if image_result is not None:
                edges, confirmed_pairs = self._accept_image_result(
                    image_result, registry_ids
                )
            else:
                edges, confirmed_pairs = await self._infer_text_edges(nodes)

        if self.cfg.max_stale_rounds > 0:
            for old in self.store.get_semantic_edges():
                if (old.source_id, old.target_id) in confirmed_pairs:
                    continue
                if (old.source_id not in registry_ids
                        or old.target_id not in registry_ids):
                    continue
                if old.stale_rounds + 1 > self.cfg.max_stale_rounds:
                    continue
                old.stale_rounds += 1
                edges.append(old)

        self.store.set_semantic_edges(edges)
        self.store.prune_relations(registry_ids)
        self.store.flush_caches()
        snapshot = SceneGraphSnapshot(
            nodes={n.object_id: n for n in nodes},
            edges=edges,
            updated_at=time.time(),
        )

        if self.object_store is not None:
            pairs = list(zip(stable, (n.caption for n in nodes)))
            written = await asyncio.get_running_loop().run_in_executor(
                None, self.object_store.persist, pairs
            )
            log.debug("[scene-graph] persisted %d objects", written)

        log.info(
            "[scene-graph] rebuild: %d nodes, %d edges, %.1fs",
            len(nodes), len(edges), time.monotonic() - t0,
        )
        return snapshot

    def _accept_image_result(
        self,
        result: ImageRelationResult,
        registry_ids: set[str],
    ) -> tuple[list[SceneGraphEdge], set[tuple[str, str]]]:
        """A backoff round observed nothing, so it keeps the live prior edges
        without aging them; a failed call ages them normally."""
        if result.outcome == "backoff":
            edges = [
                edge for edge in self.store.get_semantic_edges()
                if edge.source_id in registry_ids and edge.target_id in registry_ids
            ]
        else:
            edges = list(result.edges)
        return edges, {(edge.source_id, edge.target_id) for edge in edges}

    async def _maybe_image_edges(
        self, nodes: list[SceneGraphNode]
    ) -> Optional[ImageRelationResult]:
        """The image relation pass, or None when it cannot run (no frame)."""
        if self.image_inferer is None or self.perception is None:
            return None
        get_bundle = getattr(self.perception, "latest_frame_bundle", None)
        if get_bundle is None:
            return None
        try:
            bundle = get_bundle()
            if bundle is None:
                return None
            return await self.image_inferer.infer(nodes, bundle)
        except Exception as e:  # noqa: BLE001
            log.warning("[scene-graph] image relation pass failed: %s", e)
            return ImageRelationResult((), "failed")

    async def _infer_text_edges(
        self, nodes: list[SceneGraphNode]
    ) -> tuple[list[SceneGraphEdge], set[tuple[str, str]]]:
        """Per-pair text inference, used when there is no camera frame."""
        edges: list[SceneGraphEdge] = []
        confirmed_pairs: set[tuple[str, str]] = set()
        geo_rel: dict[frozenset[str], str] = {
            frozenset((e.source_id, e.target_id)): e.relation
            for e in self.store.get_geometric_edges()
        }
        candidates = generate_edge_candidates(
            nodes, max_candidates=self.cfg.max_candidate_edges,
        )
        llm_calls = 0
        for a, b, hint in candidates:
            pair_key = frozenset((a.object_id, b.object_id))
            # Geometry already fixed the spatial relation; ask only for a
            # semantic one.
            semantic_only = pair_key in geo_rel
            cached_edge = self.store.get_cached_relation(
                a, b, hint, semantic_only=semantic_only
            )
            if cached_edge is not None:
                confirmed_pairs.add((a.object_id, b.object_id))
                if cached_edge.relation not in ("none", "unknown"):
                    cached_edge.stale_rounds = 0
                    edges.append(cached_edge)
                continue
            # Pairs past the per-cycle budget stay unconfirmed, so their prior
            # edges survive.
            if llm_calls >= self.cfg.max_llm_relations_per_cycle:
                continue
            try:
                if semantic_only:
                    edge = await self.relation_inferer.infer_semantic_relation(
                        a, b, hint, geo_rel[pair_key]
                    )
                else:
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
            self.store.put_cached_relation(
                a, b, hint, edge, semantic_only=semantic_only
            )
            # A transport failure is not an answer; treat it like the budget.
            if edge.method != "llm_fail":
                confirmed_pairs.add((a.object_id, b.object_id))
            if edge.relation not in ("none", "unknown"):
                edge.stale_rounds = 0
                edges.append(edge)
        return edges, confirmed_pairs


async def scene_graph_loop(
    builder: SceneGraphBuilder,
    stop: asyncio.Event,
) -> None:
    """Rebuild the scene graph every `interval_sec` until `stop` is set."""
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
