# SPDX-License-Identifier: MulanPSL-2.0
"""The objects matching a class, a region and one scene-graph relation to
another object.

Each constraint filters; none is scored. Survivors come nearest the robot
first, and an empty answer says which constraint emptied it.
"""
from __future__ import annotations

import math
import re
from typing import Any, Optional

from . import geometry

# Relations the scene graph maintains, so asking for one of these is a lookup.
GRAPH_RELATIONS = frozenset(
    {"near", "on_top_of", "under", "inside", "contains", "reachable_by"})

# Anything that is not a letter, a digit or CJK separates words.
_WORD_SPLIT = re.compile(r"[^0-9a-z\u4e00-\u9fff]+")


def _tokens(text: str) -> set[str]:
    return {t for t in _WORD_SPLIT.split((text or "").strip().lower()) if t}


def _words(obj: Any) -> set[str]:
    """Every word the object answers to: its class and its caption."""
    return _tokens(f"{getattr(obj, 'label', '')} {getattr(obj, 'caption', '')}")


def _edge_holds(edges: list, subject_id: str, relation: str, anchor_id: str) -> bool:
    return any(e.source_id == subject_id and e.relation == relation
               and e.target_id == anchor_id and e.shown_on_map() for e in edges)


def find(objects: dict, *, label: str = "", region: str = "", relation: str = "",
         anchor: str = "", regions: Optional[list] = None, edges: Optional[list] = None,
         robot_xy: Optional[tuple[float, float]] = None) -> tuple[list, str]:
    """`(matches, why_empty)`; empty constraints match everything."""
    pool = [o for o in objects.values()
            if not o.missing and not o.attributes.get("is_robot")]
    if label:
        want = _tokens(label)
        pool = [o for o in pool if want & _words(o)]
        if not pool:
            return [], f"no object matches {label!r}"
    if region:
        pool = [o for o in pool if geometry.region_of(
            float(o.pose.x), float(o.pose.y), regions or []).lower() == region.lower()]
        if not pool:
            return [], f"none of them is in region {region!r}"
    if relation:
        if relation not in GRAPH_RELATIONS:
            return [], (f"unknown relation {relation!r}; one of "
                        f"{', '.join(sorted(GRAPH_RELATIONS))}")
        pool = [o for o in pool if _edge_holds(edges or [], o.object_id, relation, anchor)]
        if not pool:
            return [], f"none of them is {relation} {anchor!r}"
    if robot_xy is not None:
        pool.sort(key=lambda o: math.hypot(o.pose.x - robot_xy[0], o.pose.y - robot_xy[1]))
    return pool, ""
