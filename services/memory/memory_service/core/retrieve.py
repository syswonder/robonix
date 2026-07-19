"""Retrieve pipeline — 3-stage search: TagFilter → Rank → Filters.

Stage 2 ranking has three paths, chosen by availability:

  a) LLM credentials set → LLM ranks the full memory graph (default, best quality)
  b) LLM unavailable, embedding model installed → BM25 + Cosine hybrid
  c) Neither → chronological fallback (most recent first)

Pipeline:
  1. Tag filter: tag_index.query(tag_filter) → candidate set O(1)
  2. Rank: llm_rank (LLM default) or vector_store.search (embedding) or chronological
  3. Causal filter (Phase1: skip, Phase2: check parent nodes are reproducible)
  4. Time filter: filter by time_range
  5. Weight sort: weight × score → final ranking
  6. Fetch full MemoryNode from GraphStore → return
"""

from __future__ import annotations

import logging
import time
from typing import List, Set

from ..storage.graph_store import GraphStore
from ..storage.tag_index import TagIndex
from ..storage.vector_store import VectorStore
from . import llm_search
from .types import (
    MemoryNode, SearchRequest, SearchResponse, TagFilter,
)

log = logging.getLogger("scribe_mem")


class RetrievePipeline:
    """Orchestrate the search (read) path across all storage layers."""

    def __init__(self, graph_store: GraphStore, tag_index: TagIndex,
                 vector_store: VectorStore):
        self._graph = graph_store
        self._tags = tag_index
        self._vectors = vector_store  # kept for optional embedding path

    async def execute(self, request: SearchRequest) -> SearchResponse:
        """Execute the search pipeline: TagFilter → LLM rank → filters.

        Returns:
            SearchResponse with ranked MemoryNode list.
        """
        # ── Stage 1: Tag filter (O(1) inverted index) ──
        tag_filter = request.tags or TagFilter()
        candidate_ids = self._tags.query(tag_filter)
        if not candidate_ids:
            log.debug("search: tag filter returned empty set")
            return SearchResponse(nodes=[])

        log.debug("search: tag filter → %d candidates", len(candidate_ids))

        # ── Stage 2: LLM rank (default) → embedding → chronological ──
        top_k = max(1, request.top_k)
        overfetch = max(top_k * 3, 10)

        if llm_search.llm_search_available():
            # Path A: LLM (default — best quality, works without embedding)
            log.info("search: using LLM ranker")
            ranked = await llm_search.llm_rank(
                query=request.query,
                candidate_ids=candidate_ids,
                graph_get=self._graph.get_node,
                top_k=overfetch,
            )
        elif self._vectors.is_semantic:
            # Path B: BM25 + Cosine hybrid (embedding model installed)
            log.info("search: LLM unavailable — using embedding ranker")
            ranked = self._vectors.search(
                query=request.query,
                candidate_ids=candidate_ids,
                top_k=overfetch,
                alpha=request.alpha,
            )
        else:
            # Path C: chronological fallback (best-effort)
            log.info("search: LLM and embedding unavailable — chronological")
            nodes = [
                self._graph.get_node(nid) for nid in candidate_ids
            ]
            nodes = [n for n in nodes if n is not None]
            nodes.sort(key=lambda n: n.timestamp, reverse=True)
            ranked = [
                (n.node_id, max(0.05, 1.0 - i * 0.1))
                for i, n in enumerate(nodes[:overfetch])
            ]

        if not ranked:
            return SearchResponse(nodes=[])

        # Build (node_id → hybrid_score) map
        score_map = {nid: score for nid, score in ranked}

        # ── Stage 3: Causal filter (Phase1: skip) ──
        # Phase2: for each candidate, check if all causal_chain parents
        # are satisfied in the current environment.
        post_causal = set(nid for nid, _ in ranked)

        # ── Stage 4: Time filter ──
        if request.time_range is not None:
            tr = request.time_range
            now = time.time_ns()
            end_ts = tr.end_ts if tr.end_ts > 0 else now
            post_time: Set[int] = set()
            for nid in post_causal:
                node = self._graph.get_node(nid)
                if node is not None and tr.start_ts <= node.timestamp <= end_ts:
                    post_time.add(nid)
            post_causal = post_time
            if not post_causal:
                return SearchResponse(nodes=[])

        # ── Stage 5: Weight sort ──
        final_scores: List[tuple[int, float]] = []
        for nid in post_causal:
            node = self._graph.get_node(nid)
            if node is None:
                continue
            weight = node.weight
            hybrid = score_map.get(nid, 0.0)
            final_scores.append((nid, weight * hybrid))

        final_scores.sort(key=lambda x: x[1], reverse=True)

        # ── Fetch full nodes ──
        result_nodes: List[MemoryNode] = []
        for nid, _ in final_scores[:top_k]:
            node = self._graph.get_node(nid)
            if node is not None:
                # Update access metadata (persist to GraphStore so it
                # survives reboots — used by forget/compact scoring).
                node.last_access = time.time_ns()
                node.access_count += 1
                self._graph.update_node(nid, node)
                result_nodes.append(node)

        log.info("search: \"%s\" → %d results", request.query[:60], len(result_nodes))

        # ── Stage 6: VLM QA (optional) ──
        vlm_answer = ""
        if request.vlm_qa and result_nodes:
            # Collect image paths and build node contexts
            all_image_refs: List[str] = []
            node_ctx: List[str] = []
            for n in result_nodes[:3]:
                # Build a compact context string for each node
                parts = [f"summary: \"{n.summary}\""]
                if n.tags:
                    parts.append(f"scene={n.tags.scene_type or '?'}")
                    parts.append(f"action={n.tags.action_type or '?'}")
                    if n.tags.objects_present:
                        parts.append(f"objects={','.join(n.tags.objects_present)}")
                    parts.append(f"success={n.tags.success}")
                if n.spatial_data and n.spatial_data.objects:
                    coords = "; ".join(
                        f"{o.label}@{o.x:.1f},{o.y:.1f},{o.z:.1f}"
                        for o in n.spatial_data.objects
                    )
                    parts.append(f"spatial={coords}")
                node_ctx.append(" | ".join(parts))
                if n.image_refs:
                    all_image_refs.extend(n.image_refs)
            if all_image_refs or node_ctx:
                log.info("search: vlm_qa → %d images, %d contexts for query %r",
                         len(all_image_refs), len(node_ctx), request.query[:60])
                from .observe import vlm_answer_question
                answer = await vlm_answer_question(
                    query=request.query,
                    image_paths=all_image_refs,
                    node_contexts=node_ctx,
                )
                if answer:
                    vlm_answer = answer
                    log.info("search: vlm_qa answer → %r", answer[:120])
                else:
                    log.info("search: vlm_qa — VLM unavailable, skipping")

        return SearchResponse(nodes=result_nodes, vlm_answer=vlm_answer)
