#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Validate live Scene objects against occupancy and Webots ground truth."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.request
from pathlib import Path
from typing import Any

from scene_quality_ground_truth import (
    evaluate_ground_truth,
    evaluate_scene_ground_truth,
    load_ground_truth,
    load_ground_truths,
    nearest_object,
)


def _json(url: str) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=3) as response:
        return json.load(response)


def _inside_map(
    obj: dict[str, Any], occupancy: dict[str, Any], margin_m: float
) -> bool:
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
    return all(math.isfinite(value) for value in values) and all(
        0.0 < value <= max_extent_m for value in values[:3]
    )


def _visible_objects(state: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        obj
        for obj in state.get("objects") or []
        if not obj.get("missing")
        and str(obj.get("cls") or "").strip().lower() != "robot"
    ]


def _round_floats(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, 6)
    if isinstance(value, list):
        return [_round_floats(item) for item in value]
    if isinstance(value, dict):
        return {key: _round_floats(item) for key, item in value.items()}
    return value


def main() -> int:
    repository_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene-url",
        default=f"http://127.0.0.1:{os.environ.get('SCENE_WEB_PORT', '50107')}",
    )
    parser.add_argument(
        "--ground-truth",
        type=Path,
        default=(repository_root / "testing/fixtures/webots_office_scene_quality.json"),
    )
    parser.add_argument(
        "--require-scene-ground-truth",
        action="store_true",
        help=(
            "fail unless the final persistent registry satisfies the WBT "
            "multi-object recall, label, duplicate, center, size, and yaw gates"
        ),
    )
    args = parser.parse_args()
    state_url = f"{args.scene_url.rstrip('/')}/api/state"
    objects3d_url = f"{args.scene_url.rstrip('/')}/api/objects3d"

    try:
        truth, fixture = load_ground_truth(
            args.ground_truth,
            repository_root=repository_root,
        )
        truths, _ = load_ground_truths(
            args.ground_truth,
            repository_root=repository_root,
        )
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"ground truth: {exc}"}))
        return 1

    deadline = time.monotonic() + 45.0
    state: dict[str, Any] = {}
    while time.monotonic() < deadline:
        try:
            state = _json(state_url)
            quality = state.get("perception_quality") or {}
            if (
                state.get("occupancy")
                and int(quality.get("healthy_frames") or 0) > 0
                and nearest_object(_visible_objects(state), truth) is not None
            ):
                break
        except Exception:
            pass
        time.sleep(1.0)

    quality = state.get("perception_quality") or {}
    occupancy = state.get("occupancy") or {}
    visible = _visible_objects(state)
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

    sampling = fixture["sampling"]
    expected_samples = int(sampling["sample_count"])
    interval_s = float(sampling["interval_s"])
    samples: list[dict[str, Any]] = []
    point_samples: list[dict[str, Any]] = []
    for index in range(expected_samples):
        try:
            state = _json(state_url)
            target = nearest_object(_visible_objects(state), truth)
            if target is not None:
                samples.append(target)
            snapshot = _json(objects3d_url)
            point_target = nearest_object(
                snapshot.get("objects") or [],
                truth,
                center_key="center",
            )
            if point_target is not None:
                point_samples.append(point_target)
        except Exception:
            pass
        if index + 1 < expected_samples:
            time.sleep(interval_s)

    ground_truth = evaluate_ground_truth(
        samples,
        point_samples,
        truth,
        expected_samples=expected_samples,
        thresholds=fixture["thresholds"],
    )
    final_visible = _visible_objects(state)
    scene_ground_truth = evaluate_scene_ground_truth(
        final_visible,
        truths,
        thresholds=fixture.get("scene_thresholds") or {},
    )
    spatial_ok = bool(
        visible
        and occupancy
        and quality.get("require_occupancy_bounds") is True
        and quality.get("frame_dbscan") is True
        and not off_map
        and not invalid_bbox
        and not background_labels
    )
    result = {
        "ok": bool(
            spatial_ok
            and ground_truth["ok"]
            and (
                not args.require_scene_ground_truth
                or scene_ground_truth["ok"]
            )
        ),
        "require_scene_ground_truth": args.require_scene_ground_truth,
        "visible_objects": len(visible),
        "off_map_count": len(off_map),
        "invalid_bbox_count": len(invalid_bbox),
        "background_label_count": len(background_labels),
        "quality": quality,
        "off_map_ids": off_map,
        "invalid_bbox_ids": invalid_bbox,
        "background_label_ids": background_labels,
        "ground_truth_metrics": ground_truth,
        "scene_ground_truth_metrics": scene_ground_truth,
    }
    print(json.dumps(_round_floats(result), separators=(",", ":")))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
