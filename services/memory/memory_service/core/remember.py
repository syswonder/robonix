"""Remember pipeline — transform LogRecord → MemoryNode and persist.

Pipeline:
  1. Extract tags from LogRecord + SpatialContext (rule-based for Phase1)
  2. Generate one-line summary (template-based for Phase1)
  3. Build MemoryNode
  4. tag_index.insert(node_id, tags)        — tags first
  5. vector_store.insert(node_id, emb, summary) — vector + BM25
  6. graph_store.add_node(node)            — persist to CKG
  7. If parent_node_id → add causal edge
  8. Return RememberResponse
"""

from __future__ import annotations

import logging
import time
from typing import List, Optional

from ..storage.graph_store import GraphStore
from ..storage.tag_index import TagIndex
from ..storage.vector_store import VectorStore
from .types import (
    LogRecord, MemoryNode, NodeType, SpatialContext, TagSet,
    RememberRequest, RememberResponse,
)

log = logging.getLogger("scribe_mem")

# ── Keyword sets for rule-based tag extraction (Phase1) ─────────────────

_SCENE_KEYWORDS = {
    "kitchen": ["kitchen", "sink", "stove", "fridge", "counter", "oven"],
    "living_room": ["living room", "sofa", "couch", "television", "coffee table"],
    "workshop": ["workshop", "workbench", "crafting table", "table saw", "tool rack"],
    "bedroom": ["bedroom", "bed", "wardrobe", "closet", "pillow"],
    "outdoor": ["outdoor", "garden", "yard", "street", "park", "outside"],
}

_ACTION_KEYWORDS = {
    "grasp": ["grasp", "grab", "pick", "hold", "take", "collect"],
    "place": ["place", "put", "set", "drop", "release", "leave"],
    "navigate": ["navigate", "move", "go", "walk", "drive", "travel", "approach"],
    "craft": ["craft", "make", "build", "create", "assemble", "combine"],
    "observe": ["observe", "see", "look", "watch", "scan", "inspect", "detect"],
}

_TASK_KEYWORDS = {
    "fetch": ["fetch", "get", "bring", "retrieve", "deliver"],
    "build": ["build", "construct", "assemble", "craft", "make", "create"],
    "explore": ["explore", "scan", "survey", "map", "search", "find"],
    "dialogue": ["ask", "tell", "say", "answer", "inform", "report"],
}


def _rule_based_tag_extraction(log_record: LogRecord,
                                spatial: Optional[SpatialContext]) -> TagSet:
    """Extract TagSet using keyword matching on LogRecord.msg.

    Phase1 rule-based extraction. Phase2: upgrade to LLM-based extraction.
    """
    msg_lower = log_record.msg.lower()
    tags = TagSet()

    # ── Scene type ──
    for scene, keywords in _SCENE_KEYWORDS.items():
        for kw in keywords:
            if kw in msg_lower:
                tags.scene_type = scene
                break
        if tags.scene_type:
            break

    # ── Action type ──
    for action, keywords in _ACTION_KEYWORDS.items():
        for kw in keywords:
            if kw in msg_lower:
                tags.action_type = action
                break
        if tags.action_type:
            break

    # ── Task type ──
    for task, keywords in _TASK_KEYWORDS.items():
        for kw in keywords:
            if kw in msg_lower:
                tags.task_type = task
                break
        if tags.task_type:
            break

    # ── Success ──
    tags.success = (log_record.level.lower() not in ("error", "warn"))

    # ── Objects from spatial context ──
    if spatial:
        for obj in spatial.objects:
            if obj.label and obj.label not in tags.objects_present:
                tags.objects_present.append(obj.label)

    # ── Difficulty (heuristic: message length + action complexity) ──
    msg_len = len(log_record.msg)
    if msg_len > 200:
        tags.difficulty = "hard"
    elif msg_len > 80:
        tags.difficulty = "medium"
    else:
        tags.difficulty = "easy"

    # ── Tool / source ──
    if log_record.tag:
        tags.tool_used = [log_record.tag]

    return tags


def _generate_summary(log_record: LogRecord,
                       spatial: Optional[SpatialContext]) -> str:
    """Template-based summary generation. Phase2: upgrade to LLM.

    Format: "[{action}] {success/failure} in {scene}: {key objects}"
    """
    msg_lower = log_record.msg.lower()

    # Determine action
    action = "did something"
    for act, keywords in _ACTION_KEYWORDS.items():
        for kw in keywords:
            if kw in msg_lower:
                action = act
                break
        if action != "did something":
            break

    # Determine outcome
    outcome = "successfully" if log_record.level.lower() not in ("error", "warn") else "failed to"

    # Determine scene
    scene = "unknown area"
    for s, keywords in _SCENE_KEYWORDS.items():
        for kw in keywords:
            if kw in msg_lower:
                scene = s.replace("_", " ")
                break
        if scene != "unknown area":
            break

    # Objects
    obj_names: List[str] = []
    if spatial:
        obj_names = [o.label for o in spatial.objects if o.label]
    obj_str = ", ".join(obj_names) if obj_names else ""

    if obj_str:
        return f"{outcome} {action} {obj_str} in {scene}"
    else:
        return f"{outcome} {action} in {scene}"


# ── Pipeline ────────────────────────────────────────────────────────────

class RememberPipeline:
    """Orchestrate the remember (write) path across all storage layers."""

    def __init__(self, graph_store: GraphStore, tag_index: TagIndex,
                 vector_store: VectorStore):
        self._graph = graph_store
        self._tags = tag_index
        self._vectors = vector_store

    async def execute(self, request: RememberRequest) -> RememberResponse:
        """Execute the remember pipeline.

        Returns:
            RememberResponse with the new node_id.
        """
        log_record = request.log_record
        spatial = request.spatial

        # 1. Extract tags
        tags = _rule_based_tag_extraction(log_record, spatial)

        # 2. Generate summary
        summary = _generate_summary(log_record, spatial)

        # 3. Build MemoryNode (without node_id — GraphStore assigns it)
        now = time.time_ns()
        embedding_text = summary  # Phase1: embed the summary text
        embedding = self._vectors.encode(embedding_text, modality="text")

        node = MemoryNode(
            node_id=0,  # GraphStore will assign
            summary=summary,
            raw_log=log_record,
            timestamp=log_record.ts or now,
            spatial_data=spatial,
            tags=tags,
            weight=0.5,
            embedding=embedding,
            node_type=NodeType.SHORT_TERM,
            created_at=now,
            version=1,
        )

        # 4. Persist to GraphStore first to get node_id
        node_id = self._graph.add_node(node)
        node.node_id = node_id

        # 5. Tags first into inverted index
        self._tags.insert(node_id, tags)

        # 6. Vector + BM25 index
        self._vectors.insert(node_id, embedding, summary)

        # 7. Causal edge
        if request.parent_node_id is not None:
            try:
                self._graph.add_edge(request.parent_node_id, node_id)
            except ValueError:
                log.warning("remember: parent_node_id %d not found, skipping edge",
                           request.parent_node_id)

        log.info("remember: node %d → \"%s\"", node_id, summary)
        return RememberResponse(node_id=node_id, message=f"Memory saved as node {node_id}")
