"""Tag Store — in-memory inverted index for 4-dimension tag filtering.

Phase1: Python dict-based inverted index.
Phase2: swap to Redis with same logical interface.

The tag index is the FIRST retrieval stage: it reduces a candidate set
from O(1000s) to O(10s) before vector ranking is applied.
"""

from __future__ import annotations

from collections import defaultdict
from typing import Dict, List, Optional, Set

from ..core.types import TagFilter, TagSet, MemoryNode, difficulty_leq


class TagIndex:
    """Inverted index over TagSet dimensions.

    Each dimension value maps to a set of node_ids.
    Filter queries compute the intersection of dimension-indexed sets.
    """

    def __init__(self):
        # dimension name -> value -> set of node_ids
        self._inverted: Dict[str, Dict[str, Set[int]]] = defaultdict(
            lambda: defaultdict(set)
        )
        # node_id -> TagSet (for rebuild / remove)
        self._node_tags: Dict[int, TagSet] = {}
        # All known node IDs (for "no filter" query)
        self._all_ids: Set[int] = set()

    # ── Public API ────────────────────────────────────────────────────

    def insert(self, node_id: int, tags: TagSet) -> None:
        """Index a node's tags into all dimension inverted lists."""
        self._all_ids.add(node_id)
        self._node_tags[node_id] = tags
        self._index_tags(node_id, tags)

    def remove(self, node_id: int) -> None:
        """Remove a node from all indices."""
        self._all_ids.discard(node_id)
        tags = self._node_tags.pop(node_id, None)
        if tags is None:
            return
        self._deindex_tags(node_id, tags)

    def query(self, tag_filter: TagFilter) -> Set[int]:
        """Return candidate node_ids matching the filter (AND semantics).

        Empty TagFilter returns all indexed node IDs.
        """
        if tag_filter.is_empty():
            return set(self._all_ids)

        result_sets: List[Set[int]] = []

        # Spatial: scene_type
        if tag_filter.scene_type is not None:
            result_sets.append(
                self._inverted["scene_type"].get(tag_filter.scene_type, set())
            )

        # Spatial: objects (intersection: node must contain ALL requested objects)
        if tag_filter.objects is not None:
            obj_matches: List[Set[int]] = []
            for obj in tag_filter.objects:
                obj_matches.append(self._inverted["objects_present"].get(obj, set()))
            if obj_matches:
                # Node must contain all requested objects
                obj_intersection = obj_matches[0]
                for s in obj_matches[1:]:
                    obj_intersection = obj_intersection & s
                result_sets.append(obj_intersection)

        # Behaviour: action_type
        if tag_filter.action_type is not None:
            result_sets.append(
                self._inverted["action_type"].get(tag_filter.action_type, set())
            )

        # Behaviour: success (bitmap-like)
        if tag_filter.success is not None:
            key = "success" if tag_filter.success else "failure"
            result_sets.append(self._inverted["success"].get(key, set()))

        # Cognitive: task_type
        if tag_filter.task_type is not None:
            result_sets.append(
                self._inverted["task_type"].get(tag_filter.task_type, set())
            )

        # Cognitive: difficulty_max (range filter)
        if tag_filter.difficulty_max is not None:
            diff_set: Set[int] = set()
            for level in ("easy", "medium", "hard"):
                if difficulty_leq(level, tag_filter.difficulty_max):
                    diff_set |= self._inverted["difficulty"].get(level, set())
                else:
                    break
            result_sets.append(diff_set)

        if not result_sets:
            return set(self._all_ids)

        # AND all result sets
        final = result_sets[0]
        for s in result_sets[1:]:
            final = final & s
        return final

    def rebuild(self, nodes: List[MemoryNode]) -> None:
        """Full rebuild from a list of nodes (e.g., after graph_store reload)."""
        self._inverted.clear()
        self._node_tags.clear()
        self._all_ids.clear()
        for node in nodes:
            if node.tags is None:
                continue
            self._all_ids.add(node.node_id)
            self._node_tags[node.node_id] = node.tags
            self._index_tags(node.node_id, node.tags)

    def count(self) -> int:
        """Number of indexed nodes."""
        return len(self._all_ids)

    def get_tags(self, node_id: int) -> Optional[TagSet]:
        """Return the TagSet for a given node, or None."""
        return self._node_tags.get(node_id)

    # ── Internal ──────────────────────────────────────────────────────

    def _index_tags(self, node_id: int, tags: TagSet) -> None:
        """Add a node to all inverted index dimensions."""
        sid = {node_id}

        if tags.scene_type:
            self._inverted["scene_type"][tags.scene_type] |= sid
        for obj in tags.objects_present:
            if obj:
                self._inverted["objects_present"][obj] |= sid
        if tags.region:
            self._inverted["region"][tags.region] |= sid

        if tags.action_type:
            self._inverted["action_type"][tags.action_type] |= sid
        success_key = "success" if tags.success else "failure"
        self._inverted["success"][success_key] |= sid
        for tool in tags.tool_used:
            if tool:
                self._inverted["tool_used"][tool] |= sid

        if tags.task_type:
            self._inverted["task_type"][tags.task_type] |= sid
        if tags.difficulty:
            self._inverted["difficulty"][tags.difficulty] |= sid

    def _deindex_tags(self, node_id: int, tags: TagSet) -> None:
        """Remove a node from all inverted index dimensions."""
        sid = {node_id}

        if tags.scene_type:
            self._inverted["scene_type"][tags.scene_type] -= sid
        for obj in tags.objects_present:
            if obj:
                self._inverted["objects_present"][obj] -= sid
        if tags.region:
            self._inverted["region"][tags.region] -= sid

        if tags.action_type:
            self._inverted["action_type"][tags.action_type] -= sid
        success_key = "success" if tags.success else "failure"
        self._inverted["success"][success_key] -= sid
        for tool in tags.tool_used:
            if tool:
                self._inverted["tool_used"][tool] -= sid

        if tags.task_type:
            self._inverted["task_type"][tags.task_type] -= sid
        if tags.difficulty:
            self._inverted["difficulty"][tags.difficulty] -= sid
