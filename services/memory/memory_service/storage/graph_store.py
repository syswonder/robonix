"""Graph Store — in-memory dict + JSON file persistence for MemoryNode.

Phase1 implementation: Python dict with JSON file backup.
Interface is designed so a NetworkX / Neo4j backend can be swapped in later.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

from ..core.types import CausalRelation, CausalEdge, MemoryNode, NodeType


class GraphStore:
    """Persistent store for MemoryNode and causal edges.

    Phase1: dict-based adjacency list + JSON file persistence.
    """

    # Phase1: local directory under CWD (matches service MEMORY_DIR default).
    # When run via rbnx boot, CWD = services/memory/ → data at ./memory/
    DEFAULT_DATA_DIR = os.path.join(os.getcwd(), "memory")

    def __init__(self, data_dir: str = ""):
        self._data_dir = Path(data_dir or self.DEFAULT_DATA_DIR)
        self._data_dir.mkdir(parents=True, exist_ok=True)

        self._nodes: Dict[int, MemoryNode] = {}
        # adjacency: node_id -> set of child node_ids
        self._children: Dict[int, Set[int]] = {}
        # reverse: node_id -> set of parent node_ids
        self._parents: Dict[int, Set[int]] = {}
        self._next_short_term_id: int = 0
        self._next_long_term_id: int = 1000

        self._load()

    # ── Node CRUD ──────────────────────────────────────────────────────

    def add_node(self, node: MemoryNode) -> int:
        """Write a node, auto-assigning node_id if not set.

        Returns the assigned node_id.
        """
        if node.node_id <= 0:
            node.node_id = self._next_id()
        else:
            # Ensure counter stays ahead of explicit IDs
            if node.node_id < 1000:
                self._next_short_term_id = max(self._next_short_term_id, node.node_id + 1)
            else:
                self._next_long_term_id = max(self._next_long_term_id, node.node_id + 1)

        if node.created_at == 0:
            node.created_at = time.time_ns()

        self._nodes[node.node_id] = node
        if node.node_id not in self._children:
            self._children[node.node_id] = set()
        if node.node_id not in self._parents:
            self._parents[node.node_id] = set()

        self._persist()
        return node.node_id

    def get_node(self, node_id: int) -> Optional[MemoryNode]:
        """Get a single node by ID."""
        return self._nodes.get(node_id)

    def get_nodes(self, node_ids: List[int]) -> List[MemoryNode]:
        """Batch query nodes."""
        return [self._nodes[nid] for nid in node_ids if nid in self._nodes]

    def update_node(self, node_id: int, node: MemoryNode) -> None:
        """Update an existing node with optimistic-lock version check.

        Raises ValueError if the node doesn't exist or version mismatch.
        """
        existing = self._nodes.get(node_id)
        if existing is None:
            raise ValueError(f"Node {node_id} not found")
        if node.version != existing.version:
            raise ValueError(
                f"Version conflict for node {node_id}: "
                f"expected {existing.version}, got {node.version}"
            )
        node.version += 1
        self._nodes[node_id] = node
        self._persist()

    def list_by_type(self, node_type: NodeType, limit: int = 100) -> List[MemoryNode]:
        """List nodes by NodeType, sorted by created_at descending."""
        matching = [n for n in self._nodes.values() if n.node_type == node_type]
        matching.sort(key=lambda n: n.created_at, reverse=True)
        return matching[:limit]

    def list_by_time(self, start_ts: int, end_ts: int, limit: int = 100) -> List[MemoryNode]:
        """List nodes within a timestamp range.

        end_ts=0 means no upper bound (up to current time).
        """
        if end_ts == 0:
            end_ts = time.time_ns()
        matching = [n for n in self._nodes.values()
                    if start_ts <= n.timestamp <= end_ts]
        matching.sort(key=lambda n: n.timestamp, reverse=True)
        return matching[:limit]

    def remove_node(self, node_id: int) -> bool:
        """Remove a node and its edges. Returns True if the node existed."""
        if node_id not in self._nodes:
            return False
        # Remove edges
        for child in list(self._children.get(node_id, set())):
            self._parents.get(child, set()).discard(node_id)
        for parent in list(self._parents.get(node_id, set())):
            self._children.get(parent, set()).discard(node_id)
        del self._nodes[node_id]
        self._children.pop(node_id, None)
        self._parents.pop(node_id, None)
        self._persist()
        return True

    def count(self) -> int:
        """Total number of nodes."""
        return len(self._nodes)

    def all_ids(self) -> List[int]:
        """Return all node IDs."""
        return list(self._nodes.keys())

    # ── Causal edges ───────────────────────────────────────────────────

    def add_edge(self, parent_id: int, child_id: int) -> None:
        """Add a causal edge: parent → child."""
        if parent_id not in self._nodes or child_id not in self._nodes:
            raise ValueError(f"Edge endpoint not found: {parent_id} -> {child_id}")

        self._children.setdefault(parent_id, set()).add(child_id)
        self._parents.setdefault(child_id, set()).add(parent_id)

        # Update causal_chain on child node
        child_node = self._nodes[child_id]
        if parent_id not in child_node.causal_chain:
            child_node.causal_chain = list(child_node.causal_chain) + [parent_id]

        self._persist()

    def get_parents(self, node_id: int) -> List[int]:
        """Return parent node IDs for a given node."""
        return sorted(self._parents.get(node_id, set()))

    def get_children(self, node_id: int) -> List[int]:
        """Return child node IDs for a given node."""
        return sorted(self._children.get(node_id, set()))

    def get_all_edges(self) -> List[CausalEdge]:
        """Return all causal edges in the graph."""
        edges: List[CausalEdge] = []
        for parent_id, children in self._children.items():
            for child_id in children:
                edges.append(CausalEdge(
                    from_node_id=parent_id,
                    to_node_id=child_id,
                    relation=CausalRelation.ENABLES,
                ))
        return edges

    # ── Internal ───────────────────────────────────────────────────────

    def _next_id(self) -> int:
        """Auto-increment ID: 0-999 short-term, 1000+ long-term."""
        if self._next_short_term_id < 999:
            nid = self._next_short_term_id
            self._next_short_term_id += 1
            return nid
        nid = self._next_long_term_id
        self._next_long_term_id += 1
        return nid

    def promote_to_long_term(self, node_id: int) -> Optional[int]:
        """Move a short-term node (0-999) to the long-term range (1000+).

        Returns the new node_id, or None if already long-term.
        """
        node = self._nodes.get(node_id)
        if node is None or node.node_id >= 1000:
            return None

        new_id = self._next_long_term_id
        self._next_long_term_id += 1

        # Remap edges
        old_parents = list(self._parents.get(node_id, set()))
        old_children = list(self._children.get(node_id, set()))

        self.remove_node(node_id)

        node.node_id = new_id
        node.node_type = NodeType.LONG_TERM
        self._nodes[new_id] = node
        self._children[new_id] = set()
        self._parents[new_id] = set()

        for pid in old_parents:
            self._children.setdefault(pid, set()).add(new_id)
            self._parents[new_id].add(pid)
        for cid in old_children:
            self._parents.setdefault(cid, set()).add(new_id)
            self._children[new_id].add(cid)

        self._persist()
        return new_id

    def _persist(self) -> None:
        """Flush nodes and edges to a JSON file."""
        nodes_data = [n.to_dict() for n in self._nodes.values()]
        edges_data = []
        for parent_id, children in self._children.items():
            for child_id in children:
                edges_data.append({"from": parent_id, "to": child_id})

        payload = {
            "next_short_id": self._next_short_term_id,
            "next_long_id": self._next_long_term_id,
            "nodes": nodes_data,
            "edges": edges_data,
        }
        filepath = self._data_dir / "graph_store.json"
        tmp = str(filepath) + ".tmp"
        with open(tmp, "w") as f:
            json.dump(payload, f, indent=2)
        os.replace(tmp, str(filepath))

    def _load(self) -> None:
        """Restore from JSON file if it exists."""
        filepath = self._data_dir / "graph_store.json"
        if not filepath.exists():
            return
        with open(filepath) as f:
            data = json.load(f)

        self._next_short_term_id = data.get("next_short_id", 0)
        self._next_long_term_id = data.get("next_long_id", 1000)

        for nd in data.get("nodes", []):
            node = MemoryNode.from_dict(nd)
            self._nodes[node.node_id] = node
            self._children[node.node_id] = set()
            self._parents[node.node_id] = set()

        for ed in data.get("edges", []):
            pid, cid = ed["from"], ed["to"]
            self._children.setdefault(pid, set()).add(cid)
            self._parents.setdefault(cid, set()).add(pid)
