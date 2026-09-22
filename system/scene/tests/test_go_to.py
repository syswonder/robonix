# SPDX-License-Identifier: MulanPSL-2.0
"""Going to a named thing, and refusing to guess which.

The navigation call is injected, so what these exercise is the decision
making: when to ask, which candidate next, and what to say when none of them
work out. That is the part perception's inaccuracy actually lands on.
"""
import pytest

from scene_service import find as find_impl
from scene_service.go_to import (
    Approach,
    PendingQueries,
    go_to,
    yaw_to_quaternion,
)


class _Obj:
    def __init__(self, oid, cls, x, y, label=None):
        self.object_id = oid
        self.cls = cls
        self.confidence = 0.9
        self.last_seen = 1000.0
        self.pose = type("P", (), {"x": x, "y": y, "z": 0.0})()
        self.attributes = {"label_override": label} if label else {}


def _scene(*objs):
    return {o.object_id: o for o in objs}


def _reachable(_oid):
    return Approach(True, 1.0, 2.0, 0.5, "ok")


def _unreachable(_oid):
    return Approach(False, reason="no free cell within radius")


def _drives(log=None):
    def navigate(x, y, yaw):
        if log is not None:
            log.append((x, y, yaw))
        return True, "succeeded"
    return navigate


def _refuses(x, y, yaw):
    return False, "planner gave up"


def test_one_match_is_driven_to():
    objs = _scene(_Obj("plant_1", "potted_plant", 1, 1))
    log = []
    got = go_to(query=find_impl.Query(cls="potted_plant"), objects=objs,
                approach_of=_reachable, navigate=_drives(log),
                pending=PendingQueries(), now=1000.0)
    assert got.status == "arrived"
    assert got.object_id == "plant_1"
    assert log == [(1.0, 2.0, 0.5)]


def test_eight_alike_chairs_are_asked_about_not_driven_to():
    """The measured failure. Nothing moves until a person answers."""
    objs = _scene(*[_Obj(f"chair_{i}", "chair", float(i), 0.0) for i in range(8)])
    log = []
    got = go_to(query=find_impl.Query(cls="chair"), objects=objs,
                approach_of=_reachable, navigate=_drives(log),
                pending=PendingQueries(), now=1000.0)
    assert got.status == "needs_clarification"
    assert got.question and got.query_id
    assert log == [], "the robot moved before anyone answered"


def test_the_answer_goes_to_the_thing_that_was_shown():
    """Resolving again could return a different set -- objects move and
    perception revises itself -- so the candidates are held against the id."""
    objs = _scene(*[_Obj(f"chair_{i}", "chair", float(i), 0.0) for i in range(8)])
    pending = PendingQueries()
    asked = go_to(query=find_impl.Query(cls="chair"), objects=objs,
                  approach_of=_reachable, navigate=_drives(),
                  pending=pending, now=1000.0)

    # From the list that was actually offered: only the top few are shown,
    # and which ones is not fixed when nothing separates them.
    picked = asked.candidates[1].object_id
    log = []
    answered = go_to(query_id=asked.query_id, chosen_object_id=picked,
                     objects=objs, approach_of=_reachable,
                     navigate=_drives(log), pending=pending, now=1001.0)
    assert answered.status == "arrived"
    assert answered.object_id == picked
    assert len(log) == 1


def test_an_expired_question_is_asked_again_rather_than_acted_on():
    """The scene it described is no longer the scene."""
    objs = _scene(*[_Obj(f"chair_{i}", "chair", float(i), 0.0) for i in range(3)])
    pending = PendingQueries(ttl_s=10.0)
    asked = go_to(query=find_impl.Query(cls="chair"), objects=objs,
                  approach_of=_reachable, navigate=_drives(),
                  pending=pending, now=1000.0)

    late = go_to(query_id=asked.query_id, chosen_object_id="chair_1",
                 objects=objs, approach_of=_reachable, navigate=_drives(),
                 pending=pending, now=2000.0)
    assert late.status == "not_found"
    assert "expired" in late.detail


def test_a_choice_that_was_not_offered_is_refused():
    objs = _scene(*[_Obj(f"chair_{i}", "chair", float(i), 0.0) for i in range(3)])
    pending = PendingQueries()
    asked = go_to(query=find_impl.Query(cls="chair"), objects=objs,
                  approach_of=_reachable, navigate=_drives(),
                  pending=pending, now=1000.0)
    got = go_to(query_id=asked.query_id, chosen_object_id="sofa_9",
                objects=objs, approach_of=_reachable, navigate=_drives(),
                pending=pending, now=1001.0)
    assert got.status == "not_found"


def test_an_unreachable_first_candidate_does_not_end_the_task():
    """Perception is not accurate enough for one attempt to be the answer.
    A candidate that cannot be approached is evidence about that candidate,
    so the runner-up is tried rather than the request abandoned."""
    # A label makes one of them the clear answer; the other stays behind it
    # as the fallback. Two identical chairs would be ambiguous instead, and
    # asking is the right answer there.
    objs = _scene(_Obj("chair_1", "chair", 1, 0, label="reading chair"),
                  _Obj("chair_2", "chair", 9, 0))
    tried = []

    def approach(oid):
        tried.append(oid)
        return _unreachable(oid) if oid == "chair_1" else _reachable(oid)

    got = go_to(query=find_impl.Query(text="reading", cls="chair"),
                objects=objs, approach_of=approach, navigate=_drives(),
                pending=PendingQueries(), robot_xy=(0.0, 0.0),
                max_attempts=2, now=1000.0)
    assert got.status == "arrived"
    assert len(tried) == 2
    assert got.attempts and "no approach pose" in got.attempts[0]


def test_navigation_failing_moves_on_to_the_next_candidate():
    objs = _scene(_Obj("chair_1", "chair", 1, 0, label="reading chair"),
                  _Obj("chair_2", "chair", 9, 0))
    calls = []

    def navigate(x, y, yaw):
        calls.append((x, y))
        return (False, "planner gave up") if len(calls) == 1 else (True, "ok")

    got = go_to(query=find_impl.Query(text="reading", cls="chair"),
                objects=objs, approach_of=_reachable, navigate=navigate,
                pending=PendingQueries(), robot_xy=(0.0, 0.0),
                max_attempts=2, now=1000.0)
    assert got.status == "arrived"
    assert len(calls) == 2


def test_running_out_of_candidates_says_what_was_tried():
    """"Could not get there" without saying which things were tried leaves
    the caller nothing to widen."""
    objs = _scene(_Obj("chair_1", "chair", 1, 0, label="reading chair"),
                  _Obj("chair_2", "chair", 9, 0))
    got = go_to(query=find_impl.Query(text="reading", cls="chair"),
                objects=objs, approach_of=_reachable, navigate=_refuses,
                pending=PendingQueries(), robot_xy=(0.0, 0.0),
                max_attempts=2, now=1000.0)
    assert got.status == "unreachable"
    assert len(got.attempts) == 2
    assert all("navigation failed" in a for a in got.attempts)


def test_navigation_being_offline_is_answered_not_attempted():
    objs = _scene(_Obj("chair_1", "chair", 1, 0))
    got = go_to(query=find_impl.Query(cls="chair"), objects=objs,
                approach_of=_reachable, navigate=None,
                pending=PendingQueries(), now=1000.0)
    assert got.status == "nav_offline"
    assert "nothing was attempted" in got.detail


def test_a_stale_id_is_followed_to_the_object_it_became():
    """Ids drift as perception merges its own objects; someone holding an old
    one is asking about the same thing."""
    objs = _scene(_Obj("chair_2", "chair", 1, 0))
    log = []
    got = go_to(object_id="chair_1", objects=objs, approach_of=_reachable,
                navigate=_drives(log), pending=PendingQueries(),
                resolve_id=lambda _old: "chair_2", now=1000.0)
    assert got.status == "arrived"
    assert got.object_id == "chair_2"


def test_yaw_becomes_a_rotation_about_z_only():
    """A floor robot has one axis of rotation; the other two must stay zero
    or the goal pose tilts the base."""
    x, y, z, w = yaw_to_quaternion(0.0)
    assert (x, y, z, w) == (0.0, 0.0, 0.0, 1.0)
    x, y, z, w = yaw_to_quaternion(3.14159265358979)
    assert x == 0.0 and y == 0.0
    assert z == pytest.approx(1.0, abs=1e-6)
