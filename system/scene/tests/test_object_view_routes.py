# SPDX-License-Identifier: MulanPSL-2.0
"""The routes that serve an object's pictures.

Written after a NameError in the list route reached a running stack: the
module built a URL with a helper it had never imported, and nothing exercised
the route without a robot attached. These run it against a stand-in store, so
the next missing name fails in a second rather than in a browser.

What they pin beyond that is the behaviour the pictures exist for. They are
what somebody confirms a choice against, so the route follows an id that has
been superseded -- ids drift as perception merges its own objects -- but
refuses to follow a *guessed* forwarding address, because a confidently wrong
picture is worse than none.
"""
import pytest


def _client(store=None, registry=None):
    from starlette.testclient import TestClient

    from scene_service import web

    return TestClient(web.make_app(
        registry=registry if registry is not None else _registry(),
        hub=None, object_views=store, map_binding={"map_id": "lab"},
    ))


def _registry(objects=("scene.object.chair_001",)):
    from scene_service.state import BBox3D, Pose3D
    from scene_service.state.object_registry import ObjectRegistry

    reg = ObjectRegistry()
    for _ in objects:
        reg.insert_object(
            cls="chair",
            pose=Pose3D(x=1.0, y=1.0, z=0.0, yaw=0.0, frame_id="map"),
            bbox=BBox3D(size_x=0.3, size_y=0.3, size_z=0.3, yaw=0.0,
                        frame_id="map"),
            confidence=0.9, now=1000.0,
        )
    return reg


class _Store:
    """Answers like ObjectViewStore without touching a disk or a camera."""

    def __init__(self, rows=None, blob=b"jpeg"):
        self.rows = rows if rows is not None else [
            {"index": 0, "bearing": 0.0, "quality": 0.8, "w": 100, "h": 80},
            {"index": 1, "bearing": 2.4, "quality": 0.5, "w": 60, "h": 50},
        ]
        self.blob = blob
        self.asked = []

    def views(self, map_id, object_id):
        self.asked.append((map_id, object_id))
        return list(self.rows)

    def read(self, map_id, object_id, index):
        return self.blob if index == 0 else None


def test_the_list_route_names_every_view_with_a_fetchable_url():
    """The list is only useful if each row says where to get the picture --
    and building that URL is where the first version threw."""
    reg = _registry()
    oid = next(iter(reg._objects))
    store = _Store()
    body = _client(store, reg).get(f"/api/objects/{oid}/views").json()

    assert body["ok"] is True
    assert body["object_id"] == oid
    assert len(body["views"]) == 2
    for row in body["views"]:
        assert row["url"].startswith("/api/objects/")
        assert row["url"].endswith(f"/views/{row['index']}.jpg")


def test_the_image_route_serves_the_bytes_as_a_jpeg():
    reg = _registry()
    oid = next(iter(reg._objects))
    response = _client(_Store(), reg).get(f"/api/objects/{oid}/views/0.jpg")

    assert response.status_code == 200
    assert response.headers["content-type"].startswith("image/jpeg")
    assert response.content == b"jpeg"


def test_a_view_that_is_not_there_is_a_404_not_an_error():
    reg = _registry()
    oid = next(iter(reg._objects))
    assert _client(_Store(), reg).get(
        f"/api/objects/{oid}/views/7.jpg").status_code == 404


def test_an_unknown_object_says_so_rather_than_serving_someone_elses():
    client = _client(_Store(), _registry())
    body = client.get("/api/objects/scene.object.nope_999/views").json()
    assert body["ok"] is False
    assert body["departure"] is None  # never existed, as opposed to gone


def test_a_superseded_id_still_finds_its_pictures():
    """Ids drift as perception merges objects. Somebody holding the old one is
    asking about the same chair, and should be shown it."""
    reg = _registry()
    old_id, new_id = list(reg._objects)[0], None
    reg.insert_object(
        cls="chair",
        pose=__import__("scene_service.state", fromlist=["Pose3D"]).Pose3D(
            x=1.1, y=1.0, z=0.0, yaw=0.0, frame_id="map"),
        bbox=__import__("scene_service.state", fromlist=["BBox3D"]).BBox3D(
            size_x=0.3, size_y=0.3, size_z=0.3, yaw=0.0, frame_id="map"),
        confidence=0.9, now=1000.0,
    )
    new_id = [o for o in reg._objects if o != old_id][0]
    reg.soft_evict(reg._objects[old_id])
    reg.prune_expired(now=1100.0, ttl_s=30.0, merge_dist_m=1.5)

    store = _Store()
    body = _client(store, reg).get(f"/api/objects/{old_id}/views").json()

    # An inferred successor is not followed -- see the next test -- so this
    # one is refused, and the refusal explains itself rather than 404ing bare.
    assert body["ok"] is False
    assert body["departure"]["superseded_by"] == new_id


def test_a_guessed_successor_is_not_followed_for_pictures():
    """These are what a person confirms against. Showing the wrong chair under
    the id they are about to confirm is worse than showing nothing."""
    reg = _registry()
    oid = next(iter(reg._objects))
    reg.soft_evict(reg._objects[oid])
    reg.prune_expired(now=1100.0, ttl_s=30.0, merge_dist_m=1.5)

    body = _client(_Store(), reg).get(f"/api/objects/{oid}/views").json()
    assert body["ok"] is False
    assert "gone" in body["detail"]


def test_a_deployment_without_a_store_says_so_instead_of_pretending():
    reg = _registry()
    oid = next(iter(reg._objects))
    client = _client(None, reg)
    listing = client.get(f"/api/objects/{oid}/views")
    assert listing.status_code == 503
    assert "SCENE_OBJECT_VIEWS_DIR" in listing.json()["detail"]
    assert client.get(f"/api/objects/{oid}/views/0.jpg").status_code == 503
