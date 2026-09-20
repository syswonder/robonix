# SPDX-License-Identifier: MulanPSL-2.0
"""Take the robot to something named, or say which of several was meant.

The operation, rather than a fragment of it. Reaching an object used to mean
list_objects, then a choice, then goal_near, then navigate -- with the caller
carrying a pose between the last two. Measured here: with eight chairs in the
room the choosing step had nothing to choose on and chose anyway, and drove
there.

Two ideas do the work, and both come from the same observation: perception is
not accurate enough for a single attempt to be the whole answer.

The first is that ambiguity is answered, not resolved. When the candidates
cannot be separated the caller gets them and a question, plus a `query_id` to
hand back with the choice -- because resolving a second time can produce a
different set than the one the person was shown, and confirming one thing
while going to another is worse than asking twice.

The second is retry. A first candidate that turns out to be unreachable is
evidence about that candidate, not about the request, so the next one is
tried. That is what lets an imprecise map still complete a task.

The navigation call is injected. It is the one part that needs a robot, and
keeping it out of here means the decisions -- which candidate, when to ask,
when to give up -- can be tested without one.
"""
from __future__ import annotations

import math
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

from . import find as find_impl

DEFAULT_ATTEMPTS = 2

# How long a query_id stays answerable. Long enough for someone to be asked
# and reply; short enough that the scene it describes still resembles the one
# they were shown.
QUERY_TTL_S = 300.0


@dataclass
class Approach:
    """Where to stand to be at something, and whether that is possible."""
    reachable: bool
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    reason: str = ""


@dataclass
class GoToResult:
    status: str                       # arrived | unreachable |
                                      # needs_clarification | nav_offline |
                                      # not_found
    object_id: str = ""
    label: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    candidates: list = field(default_factory=list)
    question: str = ""
    query_id: str = ""
    attempts: list[str] = field(default_factory=list)
    detail: str = ""


class PendingQueries:
    """Questions asked and not yet answered.

    Holding the candidate list is the point. The alternative -- re-running
    the query when the answer comes back -- can return a different set,
    because objects move and perception revises itself, and then the thing
    confirmed is not the thing driven to.
    """

    def __init__(self, ttl_s: float = QUERY_TTL_S) -> None:
        self._ttl = ttl_s
        self._store: dict[str, tuple[float, list]] = {}

    def put(self, candidates: list, *, now: Optional[float] = None) -> str:
        now = time.time() if now is None else now
        self._expire(now)
        key = uuid.uuid4().hex[:12]
        self._store[key] = (now, list(candidates))
        return key

    def take(self, key: str, *, now: Optional[float] = None) -> Optional[list]:
        now = time.time() if now is None else now
        self._expire(now)
        entry = self._store.pop(key, None)
        return None if entry is None else entry[1]

    def _expire(self, now: float) -> None:
        for key, (at, _) in list(self._store.items()):
            if now - at > self._ttl:
                del self._store[key]


def go_to(
    *,
    query: Optional[find_impl.Query] = None,
    object_id: str = "",
    query_id: str = "",
    chosen_object_id: str = "",
    objects: dict,
    regions: Optional[list] = None,
    edges: Optional[list] = None,
    robot_xy: Optional[tuple[float, float]] = None,
    approach_of: Callable[[str], Approach],
    navigate: Optional[Callable[[float, float, float], tuple[bool, str]]],
    pending: PendingQueries,
    max_attempts: int = DEFAULT_ATTEMPTS,
    resolve_id: Optional[Callable[[str], Optional[str]]] = None,
    now: Optional[float] = None,
) -> GoToResult:
    """Resolve, approach, drive. See the module docstring for why each step.

    `navigate` is None when the navigation capability is not reachable, which
    is answered rather than attempted: a contract that cannot do what it says
    should say so, not fail halfway.
    """
    now = time.time() if now is None else now

    if navigate is None:
        return GoToResult(
            "nav_offline",
            detail="navigation is not available; nothing was attempted")

    # ── an answer to a question we asked ──────────────────────────────────
    if query_id:
        held = pending.take(query_id, now=now)
        if held is None:
            return GoToResult(
                "not_found",
                detail=("that question has expired; ask again so the choice "
                        "is made against what is there now"))
        if chosen_object_id:
            order = [c for c in held if c.object_id == chosen_object_id]
            if not order:
                return GoToResult(
                    "not_found",
                    detail=f"{chosen_object_id!r} was not one of the candidates")
        else:
            order = held
    # ── a specific object ─────────────────────────────────────────────────
    elif object_id:
        live = object_id
        if object_id not in objects and resolve_id is not None:
            # An id someone was handed a while ago. Ids drift as perception
            # merges its own objects, and a stale one is still a question
            # about the same thing.
            live = resolve_id(object_id) or ""
        if not live or live not in objects:
            return GoToResult(
                "not_found", detail=f"no object {object_id!r} in the scene")
        obj = objects[live]
        order = [find_impl.Candidate(
            object_id=live, cls=str(getattr(obj, "cls", "")),
            label=find_impl._label_of(obj), score=1.0)]
    # ── a description ─────────────────────────────────────────────────────
    else:
        if query is None:
            return GoToResult("not_found", detail="nothing was asked for")
        found = find_impl.find(
            query, objects, regions=regions, edges=edges,
            robot_xy=robot_xy, now=now)
        if found.verdict == "empty":
            return GoToResult(
                "not_found", detail=found.detail or "nothing matched")
        if found.verdict == "ambiguous":
            key = pending.put(found.candidates, now=now)
            return GoToResult(
                "needs_clarification",
                candidates=found.candidates,
                question=found.question,
                query_id=key,
                detail=found.detail)
        order = found.candidates

    # ── drive, and try the next one when the first does not work out ──────
    attempts: list[str] = []
    for candidate in order[: max(1, max_attempts)]:
        approach = approach_of(candidate.object_id)
        if not approach.reachable:
            attempts.append(
                f"{candidate.object_id}: no approach pose "
                f"({approach.reason or 'unreachable'})")
            continue
        ok, detail = navigate(approach.x, approach.y, approach.yaw)
        if ok:
            return GoToResult(
                "arrived",
                object_id=candidate.object_id,
                label=candidate.label or candidate.cls,
                x=approach.x, y=approach.y, yaw=approach.yaw,
                attempts=attempts,
                detail=detail)
        attempts.append(f"{candidate.object_id}: navigation failed ({detail})")

    return GoToResult(
        "unreachable",
        candidates=list(order[: max(1, max_attempts)]),
        attempts=attempts,
        # Names what was tried. "Could not get there" without saying which
        # things were tried leaves the caller no way to widen the request.
        detail=(f"tried {len(attempts)} candidate(s), none reachable"
                if attempts else "no candidate had an approach pose"))


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) for a rotation about z, which is all a floor robot has."""
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))
