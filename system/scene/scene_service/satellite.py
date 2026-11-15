# SPDX-License-Identifier: MulanPSL-2.0
"""Esri World Imagery tiles stitched into a north-up JPEG for the outdoor map.

The outdoor route map is metric (X east, Y north) and its snapshot carries the
metric origin's WGS84 position, so stitching needs no manual alignment: the
road points' bounding box becomes a lon/lat box, that box becomes global
Web-Mercator pixel corners at the chosen zoom, and the covered tiles are
cropped into one image whose pixel grid maps back onto the metric grid.
"""
from __future__ import annotations

import base64
import io
import math
import subprocess
import time
import urllib.request

from PIL import Image

_TILE_URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
# Esri has no z20 imagery over the campus; z19 is ~0.26 m/px, enough to read
# buildings and roads during a demo.
_ZOOM = 19
_TILE = 256
_HEADERS = {"User-Agent": "outdoornav-scene/1.0"}
_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3


def _meters_per_degree(lat_deg: float) -> tuple[float, float]:
    """(meters per longitude degree, meters per latitude degree) at a latitude."""
    lat = math.radians(lat_deg)
    sin = math.sin(lat)
    denom = math.sqrt(1 - _WGS84_E2 * sin * sin)
    east = (_WGS84_A / denom) * math.cos(lat)
    north = (_WGS84_A * (1 - _WGS84_E2)) / denom**3
    return east * math.pi / 180, north * math.pi / 180


def _geo_to_global_px(lon: float, lat: float) -> tuple[float, float]:
    """Lon/lat to Web-Mercator global pixel coordinates at the stitch zoom."""
    n = _TILE * 2**_ZOOM
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def _fetch(url: str) -> bytes:
    """Tile bytes; urllib first (stdlib), curl as a fallback because the tile
    host occasionally drops Python TLS handshakes on some networks."""
    for attempt in range(3):
        try:
            with urllib.request.urlopen(
                urllib.request.Request(url, headers=_HEADERS), timeout=20
            ) as response:
                return response.read()
        except Exception as exc:  # noqa: BLE001
            last = exc
            time.sleep(1 + attempt)
    proc = subprocess.run(
        ["curl", "-fsS", "--max-time", "20", "-A", _HEADERS["User-Agent"], url],
        capture_output=True,
    )
    if proc.returncode == 0 and proc.stdout:
        return proc.stdout
    raise OSError(f"tile download failed: {url}: {last}")


def stitch(origin_wgs84: dict, points: list[dict], margin_m: float = 10.0) -> dict:
    """Stitch the satellite image covering the given metric points.

    Returns {"image": data-url JPEG, "x": min_x, "y_top": max_y,
    "width": max_x-min_x, "height": max_y-min_y}; the page places the image at
    x / -y_top because SVG's y axis grows downward. Raises ValueError on a
    missing origin or points, OSError when tiles cannot be fetched.
    """
    if not origin_wgs84 or "latitude" not in origin_wgs84 or "longitude" not in origin_wgs84:
        raise ValueError("Map snapshot has no WGS84 origin")
    xs = [p["x"] for p in points]
    ys = [p["y"] for p in points]
    min_x, max_x = min(xs) - margin_m, max(xs) + margin_m
    min_y, max_y = min(ys) - margin_m, max(ys) + margin_m

    dlon, dlat = _meters_per_degree(origin_wgs84["latitude"])
    lon_min = origin_wgs84["longitude"] + min_x / dlon
    lon_max = origin_wgs84["longitude"] + max_x / dlon
    lat_min = origin_wgs84["latitude"] + min_y / dlat
    lat_max = origin_wgs84["latitude"] + max_y / dlat

    # Top-left and bottom-right of the target image in global pixel space.
    px_a, py_a = _geo_to_global_px(lon_min, lat_max)
    px_b, py_b = _geo_to_global_px(lon_max, lat_min)
    tx0, ty0 = math.floor(px_a / _TILE), math.floor(py_a / _TILE)
    tx1, ty1 = math.floor(px_b / _TILE), math.floor(py_b / _TILE)

    # Keep native tile resolution on the east axis and the same meters per
    # pixel on the north axis: Web-Mercator stretches latitudes, so the north
    # pixel span is divided back out to keep the stitched image north-up.
    out_w = round(px_b - px_a)
    out_h = round((px_b - px_a) * (max_y - min_y) / (max_x - min_x))

    mosaic = Image.new("RGB", ((tx1 - tx0 + 1) * _TILE, (ty1 - ty0 + 1) * _TILE))
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            data = _fetch(_TILE_URL.format(z=_ZOOM, y=ty, x=tx))
            try:
                tile = Image.open(io.BytesIO(data)).convert("RGB")
            except Exception as exc:  # noqa: BLE001
                raise OSError(f"bad tile {tx},{ty}: {exc}") from exc
            mosaic.paste(tile, ((tx - tx0) * _TILE, (ty - ty0) * _TILE))

    # Target pixel (u, v) samples mosaic at the matching global pixel; mosaic's
    # local frame is the global one minus its first tile's origin.
    image = mosaic.transform(
        (out_w, out_h),
        Image.Transform.AFFINE,
        (
            (px_b - px_a) / out_w,
            0,
            px_a - tx0 * _TILE,
            0,
            (py_b - py_a) / out_h,
            py_a - ty0 * _TILE,
        ),
        resample=Image.Resampling.BILINEAR,
    )
    buffer = io.BytesIO()
    image.save(buffer, format="JPEG", quality=85)
    return {
        "image": "data:image/jpeg;base64," + base64.b64encode(buffer.getvalue()).decode(),
        "x": min_x,
        "y_top": max_y,
        "width": max_x - min_x,
        "height": max_y - min_y,
    }
