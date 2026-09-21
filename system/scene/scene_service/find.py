# SPDX-License-Identifier: MulanPSL-2.0
"""Turn "that chair" into the right chair, or into a question worth asking.

Measured on this stack: asked for "that chair" with eight of them present,
the planner picked one on no basis and drove there; asked for "that picture"
with six frames, it explored for three minutes and still picked arbitrarily.
Neither is a reasoning failure. No contract could say "I am not sure", so the
only moves available were to guess or to keep looking.

The shape here follows OVSG (CoRL 2023), which grounds free-form text in an
open-vocabulary 3D scene graph. Three things are taken from it:

  * the query is a *star graph* -- one target, and the anchors it is
    described by hanging off it -- not a bag of keywords;
  * constraints of different kinds are never mixed into one number. A region
    is a polygon test, a relation is an edge lookup, a class is a vocabulary
    match. Cross-type comparison is not down-weighted, it is impossible;
  * candidates are proposed, then re-ranked on what surrounds them.

The one place we differ is deliberate. OVSG computes spatial relations at
query time because its input is a static scan with no relation layer. Scene
maintains one continuously -- `near`, `on_top_of`, `under`, `inside`,
`contains` from the LLM pass, `reachable_by` from the geometric loop -- so
asking "is it beside the sofa" is a lookup, not a computation. What the graph
cannot hold is bounded and known: superlatives depend on the candidate set,
`between` is ternary, and allocentric and view-relative terms depend on an
orientation only worth computing for the pair actually named. Those are a
small fixed predicate library, evaluated here.

Ambiguity is a first-class answer, after KnowNo (CoRL 2023): when the top
candidates are too close to separate, the caller is handed the candidates and
a question rather than a choice. The threshold here is a margin, which is an
uncalibrated stand-in for KnowNo's conformal prediction set -- honest to say
so, and the calibration needs a labelled set (ReferIt3D's Hard subset) that
is not wired up yet.
"""
from __future__ import annotations

import math
import re

from . import geometry
import time
from dataclasses import dataclass, field
from typing import Any, Iterable, Optional

# Relations the scene graph maintains, so asking for one of these is a lookup.
GRAPH_RELATIONS = frozenset(
    {"near", "on_top_of", "under", "inside", "contains", "reachable_by"})

# Predicates that cannot be edges and are evaluated against the candidate set.
# Kept explicit and small: the LLM picks from this list rather than emitting
# code, which is both testable and one fewer way to run arbitrary code in a
# robot service.
SET_PREDICATES = frozenset({"closest", "farthest"})

# An LLM relation edge is a judgement and can be wrong; a geometric one is
# arithmetic. Only the second is treated as fact.
_TRUSTED_METHODS = frozenset({"geometric"})
_LLM_EDGE_FLOOR = 0.45

# Two candidates closer than this in score are not separated by the evidence.
# A stand-in for a calibrated prediction set -- see the module docstring.
DEFAULT_MARGIN = 0.12

# Past this, an object is remembered rather than seen, and the caller is told.
STALE_AFTER_S = 120.0


@dataclass(frozen=True)
class RelationConstraint:
    """`relation` holds between the target and something called `target`."""
    relation: str
    target: str            # a class name, a label, or an object id


@dataclass(frozen=True)
class Query:
    """A star graph: the thing asked for, and what it was described by."""
    text: str = ""
    cls: str = ""
    region: str = ""
    relations: tuple[RelationConstraint, ...] = ()
    predicate: str = ""            # one of SET_PREDICATES
    min_confidence: float = 0.0
    k: int = 5


@dataclass
class Candidate:
    object_id: str
    cls: str
    label: str
    score: float
    region: str = ""
    distance_m: Optional[float] = None
    last_seen_s: Optional[float] = None
    stale: bool = False
    matched: tuple[str, ...] = ()
    # Why this one lost, in the caller's terms. What makes a useful question
    # possible: "both are in the living room, only one is by the sofa".
    missed: tuple[str, ...] = ()


@dataclass
class FindResult:
    verdict: str                   # unique | ambiguous | empty
    candidates: list[Candidate] = field(default_factory=list)
    margin: float = 0.0
    question: str = ""
    detail: str = ""
    # Which constraint emptied the set, when one did. "no chair in the
    # kitchen; there are two, both in the bedroom" is actionable where
    # "not found" is not.
    narrowed_by: str = ""


def _norm(text: str) -> str:
    return re.sub(r"[\s_\-]+", " ", (text or "").strip().lower())


# Anything that is not a letter, a digit, or CJK is a separator. Class names
# out of a detector never need this; a label somebody typed does, and that is
# the one place the free text really has to work.
_WORD_SPLIT = re.compile(r"[^0-9a-z\u4e00-\u9fff]+")


def _tokens(text: str) -> set[str]:
    return {t for t in _WORD_SPLIT.split(_norm(text)) if t}


def _describes(obj: Any) -> str:
    """Every word this object answers to: its class and its description.

    Both, not one or the other. "take me to my favourite desk" has to reach
    the desk somebody described that way, and "the desk" has to keep reaching
    it too -- a description does not stop the thing being a desk, which is
    the whole reason a caption is not a class correction.

    (An earlier version read `attributes["label_override"]`, a field no code
    ever wrote: the matching side was built for a person's words years before
    the storing side had anywhere to put them.)
    """
    return " ".join(
        p for p in (str(getattr(obj, "cls", "") or ""),
                    str(getattr(obj, "caption", "") or "")) if p)


def _region_of(obj: Any, regions: list[dict]) -> str:
    """Which region this object stands in. See `geometry.region_of`."""
    pose = getattr(obj, "pose", None)
    if pose is None:
        return ""
    return geometry.region_of(float(pose.x), float(pose.y), regions)


def _edge_holds(
    edges: list, subject_id: str, relation: str, object_ids: set[str],
) -> bool:
    """Whether the maintained graph carries this relation for this object.

    A geometric edge is arithmetic and is believed. An LLM edge is a
    judgement, so it has to clear a confidence floor -- OVSG can assume its
    relations are given; ours are inferred and some are wrong.
    """
    want = _norm(relation).replace(" ", "_")
    for e in edges or []:
        if getattr(e, "source_id", None) != subject_id:
            continue
        if _norm(getattr(e, "relation", "")).replace(" ", "_") != want:
            continue
        if getattr(e, "target_id", None) not in object_ids:
            continue
        method = str(getattr(e, "method", ""))
        if method in _TRUSTED_METHODS:
            return True
        if float(getattr(e, "confidence", 0.0) or 0.0) >= _LLM_EDGE_FLOOR:
            return True
    return False


def _anchor_ids(objects: dict, anchor: str) -> set[str]:
    """Which objects the phrase naming an anchor could mean."""
    if anchor in objects:
        return {anchor}
    want = _tokens(anchor)
    out = set()
    for oid, obj in objects.items():
        if want & _tokens(_describes(obj)):
            out.add(oid)
    return out


def _text_score(query_text: str, obj: Any) -> float:
    """How well the leftover description fits, in [0, 1].

    Token overlap against the label and class. Deliberately simple: this is
    the residue after the structured constraints have done their work, and
    its job is to order survivors, not to decide membership.
    """
    want = _tokens(query_text)
    if not want:
        return 0.5           # nothing said: everything fits equally
    have = _tokens(_describes(obj))
    if not have:
        return 0.0
    return len(want & have) / float(len(want))


def find(
    query: Query,
    objects: dict,
    *,
    regions: Optional[list[dict]] = None,
    edges: Optional[list] = None,
    robot_xy: Optional[tuple[float, float]] = None,
    now: Optional[float] = None,
    margin_threshold: float = DEFAULT_MARGIN,
) -> FindResult:
    """Resolve a query against a scene snapshot.

    Filters first and ranks second, and the order is the point: a margin
    between candidates only means something once they are all things the
    query actually allows. Comparing a chair in the kitchen against a chair
    in the living room on similarity alone blends "wrong room" into "slightly
    different wording", which is how a confident wrong answer happens.
    """
    now = time.time() if now is None else now
    regions = regions or []
    edges = edges or []

    pool = {
        oid: obj for oid, obj in objects.items()
        if not (getattr(obj, "attributes", None) or {}).get("is_robot")
    }
    if not pool:
        return FindResult("empty", detail="scene holds no objects")

    # ── hard constraints, each answered in its own terms ──────────────────
    narrowed_by = ""
    stage = dict(pool)

    if query.cls:
        want = _tokens(query.cls)
        nxt = {o: ob for o, ob in stage.items()
               if want & _tokens(_describes(ob))}
        if not nxt:
            return FindResult(
                "empty", narrowed_by="cls",
                detail=f"nothing of class {query.cls!r} in the scene")
        stage, narrowed_by = nxt, "cls"

    region_of = {oid: _region_of(ob, regions) for oid, ob in stage.items()}
    if query.region:
        want = _norm(query.region)
        nxt = {o: ob for o, ob in stage.items() if _norm(region_of[o]) == want}
        if not nxt:
            elsewhere = sorted({region_of[o] for o in stage if region_of[o]})
            return FindResult(
                "empty", narrowed_by="region",
                detail=(f"none in {query.region!r}; "
                        + (f"there are {len(stage)} elsewhere"
                           + (f" ({', '.join(elsewhere)})" if elsewhere else "")
                           if stage else "there are none at all")))
        stage, narrowed_by = nxt, "region"

    for rc in query.relations:
        if _norm(rc.relation).replace(" ", "_") not in GRAPH_RELATIONS:
            # Not something the graph maintains. Said rather than silently
            # ignored: dropping a constraint changes the answer.
            return FindResult(
                "empty", narrowed_by="relation",
                detail=(f"relation {rc.relation!r} is not one scene maintains "
                        f"({', '.join(sorted(GRAPH_RELATIONS))})"))
        anchors = _anchor_ids(pool, rc.target)
        if not anchors:
            return FindResult(
                "empty", narrowed_by="relation",
                detail=f"nothing in the scene matches anchor {rc.target!r}")
        nxt = {o: ob for o, ob in stage.items()
               if _edge_holds(edges, o, rc.relation, anchors)}
        if not nxt:
            return FindResult(
                "empty", narrowed_by="relation",
                detail=(f"{len(stage)} matched otherwise, but none is "
                        f"{rc.relation} {rc.target}"))
        stage, narrowed_by = nxt, "relation"

    if query.min_confidence > 0:
        nxt = {o: ob for o, ob in stage.items()
               if float(getattr(ob, "confidence", 0.0)) >= query.min_confidence}
        if not nxt:
            return FindResult(
                "empty", narrowed_by="confidence",
                detail=(f"{len(stage)} matched, none above confidence "
                        f"{query.min_confidence:.2f}"))
        stage, narrowed_by = nxt, "confidence"

    # ── rank the survivors ────────────────────────────────────────────────
    def distance(obj: Any) -> Optional[float]:
        if robot_xy is None:
            return None
        pose = getattr(obj, "pose", None)
        if pose is None:
            return None
        return math.hypot(float(pose.x) - robot_xy[0], float(pose.y) - robot_xy[1])

    rows: list[Candidate] = []
    for oid, obj in stage.items():
        last = getattr(obj, "last_seen", None)
        age = (now - float(last)) if last else None
        d = distance(obj)
        rows.append(Candidate(
            object_id=oid,
            cls=str(getattr(obj, "cls", "")),
            # What to show when asking which one. The caption is the thing
            # that discriminates -- "grandma's chair" tells the two chairs
            # apart and `chair_0007` does not -- so it wins when there is one.
            label=str(getattr(obj, "caption", "")
                      or getattr(obj, "display_name", "")
                      or getattr(obj, "cls", "")),
            region=region_of.get(oid, ""),
            distance_m=None if d is None else round(d, 3),
            last_seen_s=None if age is None else round(age, 1),
            stale=bool(age is not None and age > STALE_AFTER_S),
            score=_text_score(query.text, obj),
            matched=tuple(x for x in (
                "cls" if query.cls else "",
                "region" if query.region else "",
                "relation" if query.relations else "") if x),
        ))

    # Set predicates are the part the graph cannot hold, applied to what
    # survived rather than to the whole scene -- "the closest chair in the
    # kitchen" is closest among kitchen chairs.
    predicate = _norm(query.predicate).replace(" ", "_")
    if predicate in SET_PREDICATES and any(r.distance_m is not None for r in rows):
        known = [r for r in rows if r.distance_m is not None]
        pick = min(known, key=lambda r: r.distance_m) if predicate == "closest" \
            else max(known, key=lambda r: r.distance_m)
        for r in rows:
            if r is pick:
                r.score = 1.0
                r.matched = r.matched + (predicate,)
            else:
                r.score = min(r.score, 0.2)
                r.missed = r.missed + (predicate,)

    for r in rows:
        # Tie-breaks, small enough not to overturn a real difference in fit:
        # nearer is likelier meant, and something seen recently is likelier
        # what someone is pointing at.
        if r.distance_m is not None:
            r.score += 0.05 / (1.0 + r.distance_m)
        if r.stale:
            r.score -= 0.05

    rows.sort(key=lambda r: -r.score)
    top = rows[: max(1, query.k)]
    margin = (top[0].score - top[1].score) if len(top) > 1 else 1.0

    if len(top) == 1:
        return FindResult("unique", top, margin=1.0, narrowed_by=narrowed_by)
    if margin >= margin_threshold:
        return FindResult("unique", top, margin=margin, narrowed_by=narrowed_by)

    return FindResult(
        "ambiguous", top, margin=margin,
        question=discriminating_question(top),
        narrowed_by=narrowed_by,
        detail=f"{len(top)} candidates within {margin:.3f}")


def discriminating_question(candidates: list[Candidate]) -> str:
    """Ask about the attribute that actually separates these.

    Computed, not canned. If two chairs are in different rooms the question
    is about rooms; if they share a room it has to be about something else,
    and if nothing distinguishes them in words then the honest question is
    the one that shows the pictures. A generic "which one?" asks the person
    to do the work of finding the difference themselves.
    """
    if len(candidates) < 2:
        return ""
    regions = [c.region for c in candidates]
    if len({r for r in regions if r}) == len([r for r in regions if r]) > 1:
        named = " / ".join(f"{c.region}" for c in candidates if c.region)
        return f"which one — {named}?"

    labels = [c.label or c.cls for c in candidates]
    if len(set(labels)) == len(labels):
        return "which one — " + " / ".join(labels) + "?"

    known = [c for c in candidates if c.distance_m is not None]
    if len(known) >= 2 and abs(known[0].distance_m - known[1].distance_m) > 0.5:
        nearest = min(known, key=lambda c: c.distance_m)
        return (f"which one — the nearest ({nearest.distance_m:.1f} m away) "
                f"or another?")

    # Nothing separates them in words. Say so plainly rather than ask a
    # question the answer to which cannot be given in words either.
    return "which one — they look alike from here; please pick from the pictures"
