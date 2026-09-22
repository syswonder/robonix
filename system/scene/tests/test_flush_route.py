# SPDX-License-Identifier: MulanPSL-2.0
"""The web route that empties the object set.

Flush existed only as an MCP contract, so the one place a person is actually
looking at a wrong detection set -- the dock -- had no way to clear it. These
pin the two things that made that worth a route rather than a second
implementation: it reaches the same coordinator the MCP tool reaches, and it
carries the epoch, so a flush aimed at one map cannot land on the next one
after a switch.
"""
import pytest


class _Coordinator:
    """Records the call the way ObjectMutationCoordinator would receive it."""

    def __init__(self, *, fails=None):
        self.calls = []
        self.fails = fails

    async def flush_objects(self, *, expected_map_id, expected_generation,
                            persist_to_snapshot, note=""):
        self.calls.append({
            "expected_map_id": expected_map_id,
            "expected_generation": expected_generation,
            "persist_to_snapshot": persist_to_snapshot,
            "note": note,
        })
        if self.fails is not None:
            raise self.fails
        return 7, persist_to_snapshot, "lab", 3


def _client(mutations):
    from starlette.testclient import TestClient

    from scene_service import web

    return TestClient(web.make_app(
        registry=None, hub=None, object_mutations=mutations,
        map_binding={"map_id": "lab"},
    ))


def test_the_route_reaches_the_same_coordinator_the_mcp_tool_does():
    """One function decides what a flush does, whatever protocol asked."""
    mut = _Coordinator()
    body = _client(mut).post("/api/objects/flush", json={
        "expected_map_id": "lab", "expected_generation": 3,
    }).json()
    assert body["ok"] and body["deleted"] == 7
    assert len(mut.calls) == 1
    assert mut.calls[0]["expected_map_id"] == "lab"
    assert mut.calls[0]["expected_generation"] == 3


def test_the_epoch_travels_with_the_flush():
    """A flush the operator started while looking at one map must not land
    on whichever map is bound by the time it arrives."""
    mut = _Coordinator(fails=LookupError("map changed under the request"))
    r = _client(mut).post("/api/objects/flush", json={
        "expected_map_id": "stale", "expected_generation": 1,
    })
    assert r.status_code == 409
    assert not r.json()["ok"]


def test_a_flush_persists_unless_the_caller_says_otherwise():
    """The set being cleared is a wrong one; leaving it in the snapshot to
    return at the next boot is not what pressing this meant."""
    mut = _Coordinator()
    _client(mut).post("/api/objects/flush", json={})
    assert mut.calls[0]["persist_to_snapshot"] is True

    mut = _Coordinator()
    _client(mut).post("/api/objects/flush",
                      json={"persist_to_snapshot": False})
    assert mut.calls[0]["persist_to_snapshot"] is False


def test_a_deployment_without_a_coordinator_says_so():
    """Rather than a traceback that reads like the objects were the problem."""
    r = _client(None).post("/api/objects/flush", json={})
    assert r.status_code == 503
    assert "coordinator" in r.json()["detail"]
