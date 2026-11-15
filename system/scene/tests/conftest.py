# SPDX-License-Identifier: MulanPSL-2.0
"""Put the Scene package root on sys.path for every test in this directory.

pytest prepends the directory that holds a test file, not the package root, so
`from scene_service import ...` only resolves when pytest is run from
system/scene. The older tests each carry their own sys.path.insert; the ones
that do not import nothing when CI runs pytest from the repository root, which
is how test_goal_planner.py could sit in the tree unexercised. Doing it once
here means a new test does not have to remember.
"""
import copy
import json
import os
import sys

import pytest
from starlette.testclient import TestClient

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scene_service.annotations import AnnotationStore  # noqa: E402
from scene_service.map_meta import MapMetaStore  # noqa: E402
from scene_service.state.object_registry import ObjectRegistry  # noqa: E402
from scene_service.web import make_app  # noqa: E402


@pytest.fixture
def scene(tmp_path, monkeypatch):
    """TestClient over the app with one indoor and one outdoor map provider."""
    from scene_service import map_clients
    snapshot = dict(
        map_id="outside",
        version="v1",
        map_type="outdoor",
        origin_wgs84=dict(latitude=30.0, longitude=120.0),
        network=dict(nodes=[], roads=[dict(id="r", points=[dict(x=0.0, y=0.0), dict(x=5.0, y=0.0)], gait="flat")]),
    )
    backend = dict(map=copy.deepcopy(snapshot), recording=False, available=True, collision=False)
    calls = []

    def call(namespace, op, payload=None, timeout_s=20, provider_id=""):
        calls.append((namespace, op, provider_id, payload))
        if op == "list_maps":
            maps = [dict(map_id="outside" if backend["collision"] else "inside")] if provider_id == "indoor-map" else [dict(map_id="outside")]
            return dict(ok=True, maps_json=json.dumps(maps))
        if op == "status":
            if not backend["available"]:
                return dict(ok=False, detail="disconnected")
            return dict(
                ok=True,
                map_id=backend["map"]["map_id"],
                version=backend["map"]["version"],
                generation=1,
                pose=backend.get("pose"),
                session=dict(state="recording" if backend["recording"] else "finished"),
            )
        if op == "get_map":
            return dict(ok=True, **backend["map"])
        return dict(ok=True, detail="")

    monkeypatch.setattr(map_clients, "call", call)
    annotations = AnnotationStore(str(tmp_path / "annotations"), map_id=".live")
    binding = dict(map_id="inside", map_type="indoor", mode="localization")
    app = make_app(
        registry=ObjectRegistry(),
        anno_store=annotations,
        map_meta=MapMetaStore(str(tmp_path / "meta")),
        map_binding=binding,
        map_providers={"indoor": "indoor-map", "outdoor": "outdoor-map"},
    )
    return TestClient(app), backend, binding, annotations, calls
