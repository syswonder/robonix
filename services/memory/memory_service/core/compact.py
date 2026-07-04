"""Compact pipeline — promote ShortTerm nodes to LongTerm, generate summary.

Phase1: promotes short-term nodes (0-999) to long-term range (1000+)
if they exceed a configurable count threshold.
Phase2: LLM-based summarization of compacted nodes.
"""

from __future__ import annotations

import logging
import time
from typing import List

from ..storage.graph_store import GraphStore
from .types import CompactResponse, MemoryNode, NodeType

log = logging.getLogger("scribe_mem")

# Phase1: compact when short-term nodes exceed this count
DEFAULT_SHORT_TERM_THRESHOLD = 50


class CompactPipeline:
    """Promote short-term nodes → long-term, returning a summary."""

    def __init__(self, graph_store: GraphStore,
                 short_term_threshold: int = DEFAULT_SHORT_TERM_THRESHOLD):
        self._graph = graph_store
        self._threshold = short_term_threshold

    async def execute(self, _: None = None) -> CompactResponse:
        """Run compaction: promote overflow short-term nodes to long-term.

        Returns CompactResponse with summary and count.
        """
        short_nodes = self._graph.list_by_type(NodeType.SHORT_TERM, limit=10000)
        total_short = len(short_nodes)

        if total_short <= self._threshold:
            log.debug("compact: %d short-term nodes ≤ threshold %d, skipping",
                     total_short, self._threshold)
            return CompactResponse(
                summary=f"No compaction needed ({total_short} short-term nodes, "
                        f"threshold {self._threshold}).",
                nodes_compacted=0,
            )

        # Promote overflow: oldest first (FIFO)
        overflow = total_short - self._threshold
        to_promote = sorted(short_nodes, key=lambda n: n.created_at)[:overflow]
        promoted_count = 0

        for node in to_promote:
            new_id = self._graph.promote_to_long_term(node.node_id)
            if new_id is not None:
                promoted_count += 1
                log.debug("compact: promoted %d → %d", node.node_id, new_id)

        summary = (
            f"Compacted {promoted_count} short-term nodes → long-term "
            f"({total_short - promoted_count} short-term remaining, "
            f"{self._graph.count()} total)."
        )
        log.info("compact: %s", summary)
        return CompactResponse(summary=summary, nodes_compacted=promoted_count)
