"""Retrieve pipeline — 3-stage search: TagFilter → BM25+Embedding → Causal/Time/Weight.

Pipeline:
  1. Tag filter: tag_index.query(tag_filter) → candidate set O(1)
  2. Hybrid rank: vector_store.search(query, candidates, top_k, alpha)
  3. Causal filter (Phase1: skip, Phase2: check parent nodes are reproducible)
  4. Time filter: filter by time_range
  5. Weight sort: weight × hybrid_score → final ranking
  6. Fetch full MemoryNode from GraphStore → return
"""

from __future__ import annotations

import logging
import time
from typing import List, Optional, Set

from ..storage.graph_store import GraphStore
from ..storage.tag_index import TagIndex
from ..storage.vector_store import VectorStore
from .types import (
    MemoryNode, SearchRequest, SearchResponse, TagFilter, TimeRange,
)

log = logging.getLogger("scribe_mem")


class RetrievePipeline:
    """Orchestrate the search (read) path across all storage layers."""

    def __init__(self, graph_store: GraphStore, tag_index: TagIndex,
                 vector_store: VectorStore):
        self._graph = graph_store
        self._tags = tag_index
        self._vectors = vector_store

    async def execute(self, request: SearchRequest) -> SearchResponse:
        """Execute the 3-stage search pipeline.

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

        # ── Stage 2: BM25 + Embedding hybrid ranking ──
        top_k = max(1, request.top_k)
        ranked = self._vectors.search(
            query=request.query,
            candidate_ids=candidate_ids,
            top_k=max(top_k * 3, 10),  # over-fetch for downstream filtering
            alpha=request.alpha,
        )
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
                # Update access metadata
                node.last_access = time.time_ns()
                node.access_count += 1
                result_nodes.append(node)

        log.info("search: \"%s\" → %d results", request.query[:60], len(result_nodes))
        return SearchResponse(nodes=result_nodes)
