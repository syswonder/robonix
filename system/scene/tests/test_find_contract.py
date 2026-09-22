# SPDX-License-Identifier: MulanPSL-2.0
"""Call the find contract, not just its logic.

find.py is pure and tested on its own. This exercises the translation layer
around it, which is where a name that only exists at call time hides -- the
first version of the handler called a `_map_epoch()` that had never been
written, and imported fine.
"""
import asyncio

import pytest

pytest.importorskip("robonix_api", reason="contract types need robonix_api")
pytest.importorskip("semantic_map_mcp", reason="run codegen first")

from scene_service import mcp_tools as tools           # noqa: E402
from scene_service.state import BBox3D, Pose3D         # noqa: E402
from scene_service.state.object_registry import ObjectRegistry  # noqa: E402
from semantic_map_mcp import Find_Request, RelationConstraint   # noqa: E402


def _registry(n_chairs=8):
    reg = ObjectRegistry()
    for i in range(n_chairs):
        reg.insert_object(
            cls="chair",
            pose=Pose3D(x=float(i), y=0.0, z=0.0, yaw=0.0, frame_id="map"),
            bbox=BBox3D(size_x=.3, size_y=.3, size_z=.3, yaw=0.0,
                        frame_id="map"),
            confidence=0.9, now=1000.0)
    return reg


def _call(request):
    tools.attach_state(registry=_registry())
    return asyncio.get_event_loop().run_until_complete(tools.find(request))


def test_eight_alike_chairs_come_back_as_a_question():
    """The measured failure: with eight chairs the old path picked
    chair_014 and drove there."""
    got = _call(Find_Request(text="chair", cls="chair", k=5))
    assert got.verdict == "ambiguous"
    assert got.question
    assert len(got.candidates) == 5


def test_a_candidate_carries_what_a_caller_needs_and_no_more():
    got = _call(Find_Request(cls="chair", k=2))
    c = got.candidates[0]
    assert c.object_id and c.cls == "chair"
    # -1, not 0: zero metres away is a real answer and "unknown" is not.
    assert c.distance_m == -1.0
    assert c.last_seen_s >= 0.0


def test_nothing_of_that_class_says_which_constraint_emptied_it():
    got = _call(Find_Request(cls="giraffe"))
    assert got.verdict == "empty"
    assert got.narrowed_by == "cls"


def test_a_relation_scene_does_not_maintain_is_refused_with_the_list():
    got = _call(Find_Request(
        cls="chair",
        relations=[RelationConstraint(relation="left_of", anchor="sofa")]))
    assert got.verdict == "empty"
    assert "maintains" in got.detail
    assert "near" in got.detail
