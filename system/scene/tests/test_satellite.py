# SPDX-License-Identifier: MulanPSL-2.0
"""Satellite basemap stitching and its HTTP facade, offline (no tile network)."""
import base64
import io

import pytest
from PIL import Image

from scene_service import satellite as satellite_tiles


def _tile_jpeg(color=(90, 110, 100)) -> bytes:
    """A valid 256x256 JPEG standing in for one fetched Esri tile."""
    buffer = io.BytesIO()
    Image.new("RGB", (256, 256), color).save(buffer, format="JPEG")
    return buffer.getvalue()


def _points():
    return [
        dict(x=0.0, y=0.0),
        dict(x=5.0, y=0.0),
        dict(x=5.0, y=3.0),
        dict(x=0.0, y=3.0),
    ]


def test_stitch_places_image_over_metric_bounds(monkeypatch):
    monkeypatch.setattr(satellite_tiles, "_fetch", lambda url: _tile_jpeg())
    origin = dict(latitude=30.0, longitude=120.0)
    result = satellite_tiles.stitch(origin, _points())
    assert round(result["x"], 6) == -10.0
    assert round(result["y_top"], 6) == 13.0
    assert round(result["width"], 6) == 25.0
    assert round(result["height"], 6) == 23.0
    raw = base64.b64decode(result["image"].split(",", 1)[1])
    assert Image.open(io.BytesIO(raw)).size[0] > 0


def test_stitch_requires_origin_and_points():
    with pytest.raises(ValueError):
        satellite_tiles.stitch(None, _points())
    with pytest.raises(ValueError):
        satellite_tiles.stitch(dict(latitude=30.0, longitude=120.0), [])


def test_satellite_endpoint_fetches_once_then_serves_cache(scene, monkeypatch):
    client, backend, binding, annotations, calls = scene
    payload = dict(
        image="data:image/jpeg;base64,AA==",
        x=-10.0,
        y_top=13.0,
        width=25.0,
        height=23.0,
    )
    fetched = []

    def fake_stitch(origin_arg, points):
        fetched.append(origin_arg)
        return dict(payload)

    monkeypatch.setattr(satellite_tiles, "stitch", fake_stitch)
    client.post("/api/maps/load", json=dict(map_id="outside", map_type="outdoor"))
    first = client.get("/api/outdoor/satellite")
    assert first.status_code == 200, first.text
    body = first.json()
    assert body["ok"] is True and body["cached"] is False
    assert body["image"] == payload["image"] and body["x"] == payload["x"]
    assert len(fetched) == 1
    second = client.get("/api/outdoor/satellite")
    assert second.json()["cached"] is True
    assert len(fetched) == 1  # cache hit: no re-stitch


def test_satellite_endpoint_rejects_map_without_points(scene):
    client, backend, binding, annotations, calls = scene
    backend["map"]["network"]["roads"] = []
    client.post("/api/maps/load", json=dict(map_id="outside", map_type="outdoor"))
    result = client.get("/api/outdoor/satellite")
    assert result.status_code == 502
    assert result.json()["ok"] is False
