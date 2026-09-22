# SPDX-License-Identifier: MulanPSL-2.0
"""Resolving "that chair" — and refusing to, when the evidence cannot.

These are written against the two failures measured on the live stack: asked
for "that chair" among eight, the planner picked one on no basis; asked for
"that picture" among six, it explored for three minutes and still picked
arbitrarily. What is pinned here is that neither is possible through this
path: the constraints filter before anything is scored, and when the
survivors cannot be separated the answer is a question, not a choice.
"""
import pytest

from scene_service.find import (
    Query,
    RelationConstraint,
    discriminating_question,
    find,
)


class _Obj:
    def __init__(self, oid, cls, x, y, *, label=None, conf=0.9,
                 last_seen=1000.0, is_robot=False):
        self.object_id = oid
        self.cls = cls
        self.confidence = conf
        self.last_seen = last_seen
        self.pose = type("P", (), {"x": x, "y": y, "z": 0.0})()
        self.attributes = {}
        if label:
            self.attributes["label_override"] = label
        if is_robot:
            self.attributes["is_robot"] = True


class _Edge:
    def __init__(self, src, rel, dst, *, method="llm", confidence=0.9):
        self.source_id = src
        self.relation = rel
        self.target_id = dst
        self.method = method
        self.confidence = confidence


def _scene(*objs):
    return {o.object_id: o for o in objs}


LIVING = {"name": "living room",
          "points": [[0, 0], [5, 0], [5, 5], [0, 5]]}
KITCHEN = {"name": "kitchen",
           "points": [[5, 0], [10, 0], [10, 5], [5, 5]]}


# Queries a Chinese-speaking operator would type. Written as escapes so the
# source stays English while the fixture stays what it is for: `find` must
# resolve a query whose text is non-ASCII and matches nothing in the object
# labels, by class alone.
_NON_ASCII_PLANT = "\u7eff\u690d"   # "potted plant"
_NON_ASCII_CHAIR = "\u6905\u5b50"   # "chair"


def test_one_match_is_unique():
    objs = _scene(_Obj("plant_1", "potted_plant", 1, 1),
                  _Obj("chair_1", "chair", 2, 2))
    got = find(Query(text=_NON_ASCII_PLANT, cls="potted_plant"), objs, now=1000.0)
    assert got.verdict == "unique"
    assert got.candidates[0].object_id == "plant_1"


def test_the_robot_is_never_a_candidate():
    objs = _scene(_Obj("robot_1", "robot", 0, 0, is_robot=True),
                  _Obj("chair_1", "chair", 2, 2))
    got = find(Query(cls="chair"), objs, now=1000.0)
    assert [c.object_id for c in got.candidates] == ["chair_1"]


def test_eight_indistinguishable_chairs_produce_a_question_not_a_pick():
    """The measured failure, directly: eight chairs, no way to tell them
    apart, and the old path drove to chair_014."""
    objs = _scene(*[_Obj(f"chair_{i}", "chair", float(i), 0.0)
                    for i in range(8)])
    got = find(Query(text=_NON_ASCII_CHAIR, cls="chair"), objs, now=1000.0,
               robot_xy=(3.4, 0.0))
    assert got.verdict == "ambiguous"
    assert got.question
    assert len(got.candidates) <= 5


def test_a_region_narrows_before_anything_is_scored():
    """Wrong room is not "slightly worse wording". The polygon test answers
    yes or no and the loser never reaches the ranking."""
    objs = _scene(_Obj("chair_1", "chair", 1, 1),      # living room
                  _Obj("chair_2", "chair", 7, 1))      # kitchen
    got = find(Query(cls="chair", region="kitchen"), objs,
               regions=[LIVING, KITCHEN], now=1000.0)
    assert got.verdict == "unique"
    assert got.candidates[0].object_id == "chair_2"
    assert got.candidates[0].region == "kitchen"


def test_an_empty_region_says_where_they_actually_are():
    """"No chair in the kitchen; there are two, both in the living room" is
    actionable. "Not found" is not."""
    objs = _scene(_Obj("chair_1", "chair", 1, 1), _Obj("chair_2", "chair", 2, 2))
    got = find(Query(cls="chair", region="kitchen"), objs,
               regions=[LIVING, KITCHEN], now=1000.0)
    assert got.verdict == "empty"
    assert got.narrowed_by == "region"
    assert "living room" in got.detail


def test_a_relation_is_a_graph_lookup_not_a_distance_guess():
    """Scene maintains the relation layer, so "beside the sofa" is an edge
    that exists or does not -- not a proximity score that can be argued."""
    objs = _scene(_Obj("chair_1", "chair", 1, 1),
                  _Obj("chair_2", "chair", 1.2, 1),
                  _Obj("sofa_1", "sofa", 1.1, 1))
    edges = [_Edge("chair_2", "near", "sofa_1")]
    got = find(Query(cls="chair",
                     relations=(RelationConstraint("near", "sofa"),)),
               objs, edges=edges, now=1000.0)
    assert got.verdict == "unique"
    assert got.candidates[0].object_id == "chair_2"


def test_a_low_confidence_llm_edge_is_not_treated_as_fact():
    """Our relations are inferred, unlike the published work this follows.
    A judgement the model was unsure of must not decide the answer."""
    objs = _scene(_Obj("chair_1", "chair", 1, 1), _Obj("sofa_1", "sofa", 1.1, 1))
    weak = [_Edge("chair_1", "near", "sofa_1", method="llm", confidence=0.1)]
    got = find(Query(cls="chair",
                     relations=(RelationConstraint("near", "sofa"),)),
               objs, edges=weak, now=1000.0)
    assert got.verdict == "empty"
    assert got.narrowed_by == "relation"


def test_a_geometric_edge_is_believed_without_a_confidence():
    """Arithmetic, not judgement."""
    objs = _scene(_Obj("chair_1", "chair", 1, 1), _Obj("sofa_1", "sofa", 1.1, 1))
    edges = [_Edge("chair_1", "reachable_by", "sofa_1",
                   method="geometric", confidence=0.0)]
    got = find(Query(cls="chair",
                     relations=(RelationConstraint("reachable_by", "sofa"),)),
               objs, edges=edges, now=1000.0)
    assert got.verdict == "unique"


def test_a_relation_scene_does_not_maintain_is_refused_not_ignored():
    """Silently dropping a constraint changes the answer while looking like
    it did not."""
    objs = _scene(_Obj("chair_1", "chair", 1, 1))
    got = find(Query(cls="chair",
                     relations=(RelationConstraint("to_the_left_of", "sofa"),)),
               objs, now=1000.0)
    assert got.verdict == "empty"
    assert "maintains" in got.detail


def test_closest_is_decided_among_the_survivors_not_the_whole_scene():
    """"The closest chair in the kitchen" is closest among kitchen chairs.
    A superlative cannot be an edge because it depends on the query."""
    objs = _scene(_Obj("chair_near", "chair", 6.0, 1.0),   # kitchen, far
                  _Obj("chair_far", "chair", 9.0, 1.0),    # kitchen, farther
                  _Obj("chair_home", "chair", 0.2, 0.2))   # living room, nearest
    got = find(Query(cls="chair", region="kitchen", predicate="closest"),
               objs, regions=[LIVING, KITCHEN], robot_xy=(0.0, 0.0),
               now=1000.0)
    assert got.verdict == "unique"
    assert got.candidates[0].object_id == "chair_near"


def test_an_object_not_seen_for_a_while_is_marked_rather_than_hidden():
    """Scene keeps what it has seen, so it can answer about things not in
    view. The obligation that comes with that is saying so."""
    objs = _scene(_Obj("chair_1", "chair", 1, 1, last_seen=100.0))
    got = find(Query(cls="chair"), objs, now=1000.0)
    assert got.candidates[0].stale is True
    assert got.candidates[0].last_seen_s == pytest.approx(900.0, abs=1.0)


def test_an_operator_label_is_what_the_object_is_called():
    objs = _scene(_Obj("chair_1", "chair", 1, 1, label="grandma's chair"))
    got = find(Query(text="grandma"), objs, now=1000.0)
    assert got.candidates[0].label == "grandma's chair"
    assert got.candidates[0].score > 0


# ── the question ───────────────────────────────────────────────────────────

def test_the_question_asks_about_the_rooms_when_the_rooms_differ():
    objs = _scene(_Obj("chair_1", "chair", 1, 1), _Obj("chair_2", "chair", 7, 1))
    got = find(Query(cls="chair"), objs, regions=[LIVING, KITCHEN], now=1000.0)
    assert got.verdict == "ambiguous"
    assert "living room" in got.question and "kitchen" in got.question


def test_the_question_falls_back_to_the_pictures_when_words_cannot_separate():
    """Two identical chairs side by side in one room. Asking "which one" in
    words is asking the person to do work that cannot be done in words."""
    from scene_service.find import Candidate

    same = [Candidate("chair_1", "chair", "chair", 0.5, region="living room",
                      distance_m=2.0),
            Candidate("chair_2", "chair", "chair", 0.5, region="living room",
                      distance_m=2.1)]
    assert "pictures" in discriminating_question(same)


def test_one_candidate_needs_no_question():
    from scene_service.find import Candidate
    assert discriminating_question([Candidate("a", "chair", "chair", 1.0)]) == ""
