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
from typing import Any, Optional

from ..state.object_registry import ObjectRegistry, SceneObject
from .captioner import NodeCaptioner
from .image_relations import ImageRelationInferer
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
        # VLM-primary: when a perception frame bundle is available, one
        # image-grounded VLM call owns the relational + semantic edges. Off
        # falls back to the text-only per-pair inference.
        self.image_relations_enabled = _env_bool("SCENE_GRAPH_IMAGE_RELATIONS", True)
        # Edge hysteresis: keep an edge for up to N rebuild rounds in
        # which it was not re-confirmed by the current candidate set
        # (e.g. one of the endpoints temporarily missing, candidate
        # truncation due to max_candidate_edges, LLM rate limit). A
        # round of 0 edges therefore does not wipe out the UI.
        # An edge re-confirmed as "none"/"unknown" is dropped immediately.
        self.max_stale_rounds = _env_int("SCENE_GRAPH_MAX_STALE_ROUNDS", 2)


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
        object_store: Any = None,
        perception: Any = None,
    ) -> None:
        self.registry = registry
        self.captioner = captioner
        self.relation_inferer = relation_inferer
        self.store = store
        self.cfg = config or SceneGraphConfig()
        # Optional persistence layer (scene_service.persistence.ObjectStore).
        # Only wired in the legacy SCENE_RESTORE_ON_START mode: each rebuild
        # then upserts the current stable objects so the registry can
        # warm-restore at boot. The default Save/Load snapshot path never
        # goes through the builder — service.py passes None.
        self.object_store = object_store
        # Optional perception detector exposing latest_frame_bundle() (the
        # concept-graphs metric-tier detector). When present + enabled, the
        # image-grounded relation pass owns the relational + semantic edges;
        # otherwise the builder uses the text-only fallback.
        self.perception = perception
        self.image_inferer = (
            ImageRelationInferer(relation_inferer.llm_client)
            if self.cfg.image_relations_enabled
            else None
        )

    async def rebuild_once(self) -> SceneGraphSnapshot:
        t0 = time.monotonic()

        # 1. Snapshot registry.
        objs_dict, _ = await self.registry.snapshot()
        # Every object currently in the registry, INCLUDING `missing` ones
        # (soft-evicted this tick, typically re-bound the next). An edge whose
        # endpoints are both still here is kept across a flicker; only an
        # endpoint that has actually left the registry invalidates its edges.
        registry_ids = set(objs_dict.keys())

        # 2. Filter stable objects.
        stable = [
            o for o in objs_dict.values()
            if _is_stable(o, self.cfg.min_observations)
        ]
        # Sort by observation_count desc, truncate.
        stable.sort(key=lambda o: -o.observation_count)
        stable = stable[: self.cfg.max_objects]

        # Transient-blip guard. When (nearly) all objects are momentarily
        # `missing` — soft-evicted by a perception hiccup, re-bound the next
        # tick — fewer than two stay stable. That is NOT an authoritative
        # "scene is empty": rewriting the semantic slice here would wipe every
        # relation (the hysteresis pass drops edges whose endpoints aren't
        # stable nodes), so the graph blinks empty whenever a 30 s rebuild
        # lands on a flicker. Instead preserve the last good edges, dropping
        # only those whose endpoint has actually left the registry. The
        # geometric slice (reachable_by) is owned by the fast loop, untouched.
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

        # 5–6. Relations (VLM-primary). When the perception detector exposes a
        # camera frame bundle, one image-grounded VLM call enumerates all
        # relations among the visible objects; otherwise fall back to the
        # text-only per-pair inference. Both yield (edges, confirmed_pairs) —
        # `confirmed_pairs` are the (source, target) pairs this round had an
        # authoritative answer for; prior edges for unconfirmed pairs survive
        # via the hysteresis pass below.
        edges: list[SceneGraphEdge] = []
        confirmed_pairs: set[tuple[str, str]] = set()

        if self.cfg.relation_enabled and len(nodes) >= 2:
            image_edges = await self._maybe_image_edges(nodes)
            if image_edges is not None:
                edges = image_edges
                confirmed_pairs = {(e.source_id, e.target_id) for e in edges}
            else:
                edges, confirmed_pairs = await self._infer_text_edges(nodes)

        # 6b. Hysteresis: carry forward un-confirmed *semantic* edges from the
        # previous rebuild, capped at cfg.max_stale_rounds. An edge is dropped
        # only when an endpoint has left the registry (`registry_ids`) — NOT
        # merely because the endpoint is `missing`/unstable this round, so an
        # edge to a transiently-flickering object survives (bounded by
        # max_stale_rounds) instead of churning each tick.
        if self.cfg.max_stale_rounds > 0:
            for old in self.store.get_semantic_edges():
                pair = (old.source_id, old.target_id)
                if pair in confirmed_pairs:
                    continue
                if (old.source_id not in registry_ids
                        or old.target_id not in registry_ids):
                    continue
                if old.stale_rounds + 1 > self.cfg.max_stale_rounds:
                    continue
                old.stale_rounds += 1
                edges.append(old)

        # 7. Publish the semantic slice and flush caches. Nodes and the
        # geometric slice are owned by the fast geometric loop; the builder
        # only writes the semantic edges. The returned snapshot is a local
        # convenience (logging / tests), not the composed graph MCP reads.
        self.store.set_semantic_edges(edges)
        # Evict cached edges whose endpoints left the registry before
        # persisting. Keyed on registry membership (incl. `missing` objects),
        # not the stable-node set, so a transiently-missing object's cache is
        # not dropped only to be recomputed when it re-binds next tick. Growth
        # across an object's moves is bounded by put_cached_relation
        # (keep-latest-per-pair); this handles the orthogonal case of an object
        # leaving the scene entirely.
        self.store.prune_relations(registry_ids)
        self.store.flush_caches()
        snapshot = SceneGraphSnapshot(
            nodes={n.object_id: n for n in nodes},
            edges=edges,
            updated_at=time.time(),
        )

        # 8. Persist current stable objects for the legacy boot warm
        # restore (object_store is None outside that mode). `nodes` is
        # built from `stable` in order, so zip pairs each object with the
        # caption just computed for it. Offloaded to a thread because the
        # caption embedding + milvus write are synchronous and must not
        # block the asyncio loop. Persistence errors are swallowed inside
        # ObjectStore.persist — they never break a rebuild.
        if self.object_store is not None and stable:
            pairs = list(zip(stable, (n.caption for n in nodes)))
            loop = asyncio.get_running_loop()
            written = await loop.run_in_executor(
                None, self.object_store.persist, pairs
            )
            log.debug("[scene-graph] persisted %d objects", written)

        dt = time.monotonic() - t0
        log.info(
            "[scene-graph] rebuild: %d nodes, %d edges, %.1fs",
            len(nodes),
            len(edges),
            dt,
        )
        return snapshot

    # ── relation inference paths ─────────────────────────────────────────

    async def _maybe_image_edges(
        self, nodes: list[SceneGraphNode]
    ) -> Optional[list[SceneGraphEdge]]:
        """Run the image-grounded relation pass when the perception detector
        provides a camera frame bundle. Returns the round's edges (possibly
        empty = "ran, no relations"), or None to signal "not run" so the caller
        falls back to the text path. Never raises — failures degrade to None."""
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
            return None

    async def _infer_text_edges(
        self, nodes: list[SceneGraphNode]
    ) -> tuple[list[SceneGraphEdge], set[tuple[str, str]]]:
        """Text-only per-pair relation inference — the fallback used when no
        camera frame bundle is available (e.g. the visual-tier VLM detector,
        which has no camera→map transform). Returns (edges, confirmed_pairs).

        With geometry trimmed to reachable_by, no contact pair is geometry-owned
        among the node set, so candidate pairs get full inference; the
        semantic-only branch is retained for any residual geometric edge (whose
        gripper endpoint is not a node, so it never matches a candidate pair)."""
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
            # Rate-limit LLM calls per cycle. Pairs we couldn't query this
            # round are not added to confirmed_pairs, so any prior edge for
            # them survives via hysteresis.
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
            # An llm_fail (transport / parse error) is not "the LLM said
            # unknown" — treat it like the rate-limit case so a flaky network
            # round does not drop edges.
            if edge.method != "llm_fail":
                confirmed_pairs.add((a.object_id, b.object_id))
            if edge.relation not in ("none", "unknown"):
                edge.stale_rounds = 0
                edges.append(edge)
        return edges, confirmed_pairs


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
