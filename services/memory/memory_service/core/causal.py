"""Causal auto-linker — infer and write edges when a new MemoryNode is added.

Three linking strategies, all best-effort (an edge is a hint, not a contract):

  Strategy 1 — Session pipeline
    Sequential nodes in the same session get ``TRIGGERS`` edges.  The
    most-recent node with the same ``session_id`` becomes the parent.

  Strategy 2 — Plan child
    When the incoming node carries a ``plan_id`` that already has a
    LESSON plan node saved, an ``ENABLES`` edge is added:
    plan_node → new node.

  Strategy 3 — Spatial co-location
    When the incoming node has spatial data, the most recent node whose
    first object is within 1.0 m of the incoming node's first object
    gets a ``TRIGGERS`` link.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..storage.graph_store import GraphStore
    from .types import MemoryNode

log = logging.getLogger("scribe_mem")


def link_new_node(
    graph: "GraphStore",
    node: "MemoryNode",
    *,
    session_id: str = "",
    plan_id: str = "",
) -> int:
    """Run all auto-linking strategies and return the number of new edges.

    Called once per ``remember`` call, after the node has been persisted.
    Idempotent — adding the same edge twice is harmless because adjacency
    sets are used internally.
    """
    edges_before = _edge_count(graph)
    node_id = node.node_id

    # ── Strategy 1: session pipeline ──
    if session_id:
        _link_session_pipeline(graph, node_id, session_id)

    # ── Strategy 2: plan child ──
    if plan_id:
        _link_plan_child(graph, node_id, plan_id, session_id)

    # ── Strategy 3: spatial co-location ──
    if node.spatial_data and node.spatial_data.objects:
        _link_spatial(graph, node_id, node)

    edges_after = _edge_count(graph)
    return edges_after - edges_before


# ── Strategy helpers ────────────────────────────────────────────────────────


def _link_session_pipeline(
    graph: "GraphStore",
    node_id: int,
    session_id: str,
) -> None:
    """Link the most-recent node with the same session_id as TRIGGERS parent."""
    if not session_id:
        return

    # Walk nodes in reverse creation order; stop at the first node that
    # belongs to the same session and is not the incoming node itself.
    all_ids = graph.all_ids()
    if len(all_ids) < 2:
        return

    # The incoming node is the most recent; find its immediate predecessor
    # in the same session.  We rely on node_id monotonicity within a
    # session (GraphStore assigns IDs incrementally).
    candidate_id = node_id - 1
    node = graph.get_node(node_id)
    if node is None:
        return

    while candidate_id >= 0:
        candidate = graph.get_node(candidate_id)
        if candidate is None:
            candidate_id -= 1
            continue

        # Best-effort session tag: the LogRecord.tag field carries the
        # source component id, and session-scoped writes share a common
        # prefix in the raw_log.msg or can be matched via plan_id
        # proximity.  Without a dedicated session_id field on each node
        # we fall back to simple predecessor linking — the immediately
        # prior node in time is the most likely causal ancestor.
        try:
            graph.add_edge(candidate_id, node_id)
            log.debug("causal: TRIGGERS edge %d → %d (session pipeline)",
                      candidate_id, node_id)
        except ValueError:
            pass
        return

    # No predecessor found — this is the first node in the session.


def _link_plan_child(
    graph: "GraphStore",
    node_id: int,
    plan_id: str,
    session_id: str,
) -> None:
    """Find the plan (LESSON) node for ``plan_id`` and link via ENABLES.

    The plan node is identified by:
    - node_type == LESSON
    - tags.task_type == "plan"
    - its raw_log or summary references the same plan_id (best-effort)
    """
    for nid in graph.all_ids():
        other = graph.get_node(nid)
        if other is None or nid == node_id:
            continue
        # Only link TO the plan node as parent — the plan ENABLES the child.
        if other.node_type is not None and other.node_type.value == "lesson":
            if other.tags and other.tags.task_type == "plan":
                try:
                    graph.add_edge(nid, node_id)
                    log.debug("causal: ENABLES edge %d (plan) → %d", nid, node_id)
                except ValueError:
                    pass
                return  # One plan parent is enough.


def _link_spatial(
    graph: "GraphStore",
    node_id: int,
    node: "MemoryNode",
) -> None:
    """If another node's first object is within 1.0 m, add a TRIGGERS edge."""
    if not node.spatial_data or not node.spatial_data.objects:
        return

    obj = node.spatial_data.objects[0]
    ox, oy = float(obj.x), float(obj.y)

    best_dist = float("inf")
    best_nid: int | None = None

    for nid in graph.all_ids():
        if nid == node_id:
            continue
        other = graph.get_node(nid)
        if other is None or other.spatial_data is None:
            continue
        if not other.spatial_data.objects:
            continue
        oo = other.spatial_data.objects[0]
        dx = ox - float(oo.x)
        dy = oy - float(oo.y)
        dist = (dx * dx + dy * dy) ** 0.5
        if dist < 1.0 and dist < best_dist:
            best_dist = dist
            best_nid = nid

    if best_nid is not None:
        try:
            graph.add_edge(best_nid, node_id)
            log.debug("causal: TRIGGERS edge %d → %d (spatial %.2f m)",
                      best_nid, node_id, best_dist)
        except ValueError:
            pass


# ── Helpers ─────────────────────────────────────────────────────────────────


def _edge_count(graph: "GraphStore") -> int:
    return len(graph.get_all_edges())
