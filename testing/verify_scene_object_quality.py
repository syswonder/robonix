#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Validate live Scene object spatial sanity against its occupancy map."""

from __future__ import annotations

import json
import math
import os
import sys
import time
import urllib.request
from typing import Any


def _state(url: str) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=3) as response:
        return json.load(response)


def _inside_map(obj: dict[str, Any], occupancy: dict[str, Any], margin_m: float) -> bool:
    pose = obj.get("pose") or {}
    x, y = float(pose["x"]), float(pose["y"])
    dx = x - float(occupancy["origin_x"])
    dy = y - float(occupancy["origin_y"])
    yaw = float(occupancy.get("origin_yaw") or 0.0)
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    width_m = int(occupancy["width"]) * float(occupancy["resolution"])
    height_m = int(occupancy["height"]) * float(occupancy["resolution"])
    return (
        -margin_m <= local_x <= width_m + margin_m
        and -margin_m <= local_y <= height_m + margin_m
    )


def _finite_bbox(obj: dict[str, Any], max_extent_m: float) -> bool:
    bbox = obj.get("bbox") or {}
    values = [
        float(bbox["size_x"]),
        float(bbox["size_y"]),
        float(bbox["size_z"]),
        float(bbox.get("yaw") or 0.0),
    ]
    return (
        all(math.isfinite(value) for value in values)
        and all(0.0 < value <= max_extent_m for value in values[:3])
    )


def main() -> int:
    port = os.environ.get("SCENE_WEB_PORT", "50107")
    url = f"http://127.0.0.1:{port}/api/state"
    deadline = time.monotonic() + 30.0
    state: dict[str, Any] = {}
    while time.monotonic() < deadline:
        try:
            state = _state(url)
            quality = state.get("perception_quality") or {}
            visible = [
                obj
                for obj in state.get("objects") or []
                if not obj.get("missing")
                and str(obj.get("cls") or "").strip().lower() != "robot"
            ]
            if (
                visible
                and state.get("occupancy")
                and int(quality.get("healthy_frames") or 0) > 0
            ):
                break
        except Exception:
            pass
        time.sleep(1.0)

    quality = state.get("perception_quality") or {}
    occupancy = state.get("occupancy") or {}
    visible = [
        obj
        for obj in state.get("objects") or []
        if not obj.get("missing")
        and str(obj.get("cls") or "").strip().lower() != "robot"
    ]
    max_extent_m = float(quality.get("max_bbox_extent_m") or 0.0)
    margin_m = float(quality.get("map_bounds_margin_m") or 0.0)
    off_map = [
        str(obj.get("id") or "")
        for obj in visible
        if not occupancy or not _inside_map(obj, occupancy, margin_m)
    ]
    invalid_bbox = [
        str(obj.get("id") or "")
        for obj in visible
        if max_extent_m <= 0.0 or not _finite_bbox(obj, max_extent_m)
    ]
    background_labels = [
        str(obj.get("id") or "")
        for obj in visible
        if str(obj.get("cls") or "").strip().lower()
        in {"floor", "wall", "ceiling", "carpet"}
    ]
    result = {
        "ok": bool(
            visible
            and occupancy
            and quality.get("require_occupancy_bounds") is True
            and quality.get("frame_dbscan") is True
            and not off_map
            and not invalid_bbox
            and not background_labels
        ),
        "visible_objects": len(visible),
        "off_map_count": len(off_map),
        "invalid_bbox_count": len(invalid_bbox),
        "background_label_count": len(background_labels),
        "quality": quality,
        "off_map_ids": off_map,
        "invalid_bbox_ids": invalid_bbox,
        "background_label_ids": background_labels,
    }
    print(json.dumps(result, separators=(",", ":")))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
