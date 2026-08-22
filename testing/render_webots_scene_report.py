#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Render a small self-contained HTML review of Webots Scene results."""

from __future__ import annotations

import argparse
import base64
import html as html_module
import io
import json
import math
import zlib
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from PIL import Image

try:
    import yaml
except ModuleNotFoundError:  # Scene-only reports do not read saved-map YAML.
    yaml = None

from scene_quality_ground_truth import transform_planar_center_yaw


def _read(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _evaluation_path(run_dir: Path) -> Path:
    for name in (
        "evaluation.semantic.json",
        "evaluation-v2.json",
        "evaluation.json",
    ):
        candidate = run_dir / name
        if candidate.exists() and candidate.stat().st_size:
            return candidate
    raise FileNotFoundError(f"no evaluation JSON under {run_dir}")


def _status_maps(evaluation: dict[str, Any]) -> tuple[dict[str, str], dict[str, str]]:
    predicted: dict[str, str] = {}
    truth: dict[str, str] = {}
    for row in evaluation.get("per_target") or ():
        identity = str(row.get("identity") or "")
        object_id = str(row.get("object_id") or "")
        if not row.get("matched"):
            if identity:
                truth[identity] = "miss"
            continue
        status = "match" if row.get("label_correct") else "label"
        if identity:
            truth[identity] = status
        if object_id:
            predicted[object_id] = status
    for row in evaluation.get("duplicates") or ():
        predicted[str(row.get("object_id") or "")] = "duplicate"
    for row in evaluation.get("ghosts") or ():
        predicted[str(row.get("object_id") or "")] = "ghost"
    return predicted, truth


def _box_from_object(obj: dict[str, Any]) -> dict[str, Any] | None:
    pose = obj.get("pose") or {}
    bbox = obj.get("bbox") or {}
    try:
        return {
            "id": str(obj["id"]),
            "label": str(obj.get("cls") or "object"),
            "center": [float(pose[k]) for k in ("x", "y", "z")],
            "size": [float(bbox[k]) for k in ("size_x", "size_y", "size_z")],
            "yaw": float(bbox.get("yaw", pose.get("yaw", 0.0))),
        }
    except (KeyError, TypeError, ValueError):
        return None


def _box_from_debug_candidate(
    obj: dict[str, Any],
) -> dict[str, Any] | None:
    """Build a review-only box for a candidate withheld from the Registry."""
    corners = obj.get("bbox_corners") or ()
    if len(corners) != 8:
        return None
    try:
        points = [
            [float(value) for value in corner[:3]]
            for corner in corners
        ]
        if any(len(point) != 3 for point in points):
            return None
        center = [
            sum(point[axis] for point in points) / len(points)
            for axis in range(3)
        ]

        def edge(left: int, right: int) -> float:
            return math.dist(points[left], points[right])

        x_edge = [
            points[1][axis] - points[0][axis]
            for axis in range(3)
        ]
        return {
            "id": f"candidate:{obj.get('id') or 'unknown'}",
            "label": str(obj.get("cls") or "object"),
            "center": center,
            # export_3d_snapshot emits corners 0→1, 0→2 and 0→3 along
            # the robust box's local x, y and z axes respectively.
            "size": [edge(0, 1), edge(0, 2), edge(0, 3)],
            "yaw": math.atan2(x_edge[1], x_edge[0]),
            "status": "candidate",
            "confirmation_unique_frames": int(
                obj.get("confirmation_unique_frames") or 0
            ),
            "confirmation_min_unique_frames": int(
                obj.get("confirmation_min_unique_frames") or 0
            ),
            "confirmation_mean_confidence": float(
                obj.get("confirmation_mean_confidence")
                if obj.get("confirmation_mean_confidence") is not None
                else obj.get("conf_mean") or 0.0
            ),
        }
    except (TypeError, ValueError):
        return None


def _nearest_status(
    center: list[float],
    label: str,
    predicted_boxes: list[dict[str, Any]],
) -> str:
    candidates = [
        (
            math.dist(center, box["center"]),
            box["status"],
        )
        for box in predicted_boxes
        if box["label"] == label
    ]
    if not candidates:
        candidates = [
            (math.dist(center, box["center"]), box["status"])
            for box in predicted_boxes
        ]
    if not candidates:
        return "other"
    distance, status = min(candidates)
    return status if distance <= 0.6 else "other"


def _costmap_layer_payload(layer: dict[str, Any]) -> dict[str, Any]:
    width = int(layer["width"])
    height = int(layer["height"])
    if layer.get("encoding") != "ros-occupancy-int8-zlib-base64":
        raise ValueError(f"unsupported costmap encoding {layer.get('encoding')!r}")
    raw = zlib.decompress(base64.b64decode(layer["data"]))
    if len(raw) != width * height:
        raise ValueError(
            f"costmap payload has {len(raw)} cells, expected {width * height}"
        )
    rgba = bytearray(width * height * 4)
    # OccupancyGrid rows begin at the grid's lower-left corner.  PNG rows begin
    # at the upper-left, so reverse rows before browser-side world projection.
    for source_row in range(height):
        target_row = height - source_row - 1
        for column in range(width):
            value = raw[source_row * width + column]
            offset = (target_row * width + column) * 4
            if value == 255 or value == 0:
                continue
            if value >= 100:
                color = (222, 48, 58, 220)
            elif value >= 99:
                color = (242, 126, 32, 205)
            else:
                alpha = min(155, 24 + value)
                color = (244, 178, 40, alpha)
            rgba[offset : offset + 4] = bytes(color)
    image = Image.frombytes("RGBA", (width, height), bytes(rgba))
    encoded = io.BytesIO()
    image.save(encoded, format="PNG", optimize=True)
    return {
        key: layer[key]
        for key in (
            "frame_id",
            "source_frame_id",
            "width",
            "height",
            "resolution",
            "origin_x",
            "origin_y",
            "origin_yaw",
        )
        if key in layer
    } | {"png_b64": base64.b64encode(encoded.getvalue()).decode("ascii")}


def _load_costmap_file(path: Path) -> dict[str, dict[str, Any]]:
    if not path.exists():
        return {}
    payload = _read(path)
    if payload.get("schema") != "robonix.webots.costmaps.v1":
        raise ValueError(f"{path} has unsupported schema {payload.get('schema')!r}")
    return {
        str(name): _costmap_layer_payload(layer)
        for name, layer in (payload.get("layers") or {}).items()
    }


def _load_costmaps(run_dir: Path) -> dict[str, dict[str, Any]]:
    return _load_costmap_file(run_dir / "costmaps.json")


def _saved_map_payload(yaml_path: Path) -> dict[str, Any]:
    if yaml is None:
        raise RuntimeError(
            "PyYAML is required only when rendering --navigation-grid data"
        )
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image_path = Path(str(metadata["image"]))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image = Image.open(image_path).convert("L")
    encoded = io.BytesIO()
    image.save(encoded, format="PNG", optimize=True)
    origin_x, origin_y, origin_yaw = (
        float(value) for value in metadata["origin"]
    )
    return {
        "width": image.width,
        "height": image.height,
        "resolution": float(metadata["resolution"]),
        "origin_x": origin_x,
        "origin_y": origin_y,
        "origin_yaw": origin_yaw,
        "png_b64": base64.b64encode(encoded.getvalue()).decode("ascii"),
    }


def _navigation_payload(
    name: str,
    map_yaml: Path,
    costmaps_json: Path,
    world_path: Path | None,
) -> dict[str, Any]:
    from evaluate_webots_occupancy import load_structural_segments

    segments = (
        [
            {
                "name": segment.name,
                "start": list(segment.start),
                "end": list(segment.end),
            }
            for segment in load_structural_segments(
                world_path,
                robot_type="TiagoLite",
                robot_name="my_robot",
            )
        ]
        if world_path is not None
        else []
    )
    return {
        "world": name,
        "occupancy": _saved_map_payload(map_yaml),
        "costmaps": _load_costmap_file(costmaps_json),
        "truth": [],
        "predicted": [],
        "clouds": [],
        "robot": {},
        "structural_segments": segments,
    }


def _occupancy_plane_payload(occupancy: dict[str, Any]) -> dict[str, Any]:
    """Convert a Scene occupancy PNG into bounded metric floor-plane cells."""
    encoded = occupancy.get("png_b64")
    if not encoded:
        return {}
    width = int(occupancy.get("width") or 0)
    height = int(occupancy.get("height") or 0)
    resolution = float(occupancy.get("resolution") or 0.0)
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return {}
    image = Image.open(io.BytesIO(base64.b64decode(encoded))).convert("L")
    if image.size != (width, height):
        raise ValueError(
            f"occupancy PNG is {image.width}x{image.height}, "
            f"metadata says {width}x{height}"
        )
    pixels = image.load()
    origin_x = float(occupancy.get("origin_x") or 0.0)
    origin_y = float(occupancy.get("origin_y") or 0.0)
    yaw = float(occupancy.get("origin_yaw") or 0.0)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)

    def world(column: float, row_from_top: float) -> list[float]:
        local_x = column * resolution
        local_y = (height - row_from_top) * resolution
        return [
            round(origin_x + cosine * local_x - sine * local_y, 4),
            round(origin_y + sine * local_x + cosine * local_y, 4),
        ]

    occupied = [
        world(column + 0.5, row + 0.5)
        for row in range(height)
        for column in range(width)
        if int(pixels[column, row]) < 65
    ]
    max_occupied = 12_000
    if len(occupied) > max_occupied:
        stride = max(1, math.ceil(len(occupied) / max_occupied))
        occupied = occupied[::stride]

    # Free-space fill is deliberately coarser than walls. A block is rendered
    # only when every source cell is free, so unknown space never becomes a
    # fictitious navigable floor in the 3D review.
    block = max(2, math.ceil(max(width, height) / 100))
    free = []
    for row in range(0, height, block):
        for column in range(0, width, block):
            x1 = min(width, column + block)
            y1 = min(height, row + block)
            if all(
                # Scene's web snapshot palette uses 240 for free, 128 for
                # unknown, and 20 for occupied.  Saved-map PGM files commonly
                # use 254/205/0 instead.  A threshold above 220 accepts both
                # free encodings while keeping both unknown encodings
                # transparent.
                int(pixels[x, y]) > 220
                for y in range(row, y1)
                for x in range(column, x1)
            ):
                center = world(
                    column + (x1 - column) * 0.5,
                    row + (y1 - row) * 0.5,
                )
                free.append(
                    [
                        center[0],
                        center[1],
                        round((x1 - column) * resolution, 4),
                        round((y1 - row) * resolution, 4),
                    ]
                )
    extent = [
        world(0.0, float(height)),
        world(float(width), float(height)),
        world(float(width), 0.0),
        world(0.0, 0.0),
    ]
    return {
        "yaw": yaw,
        "resolution": resolution,
        "occupied": occupied,
        "free": free,
        "extent": extent,
    }


def _world_payload(world: str, run_dir: Path) -> dict[str, Any]:
    state = _read(run_dir / "state.json")
    cloud = _read(run_dir / "objects3d.json")
    debug_path = run_dir / "objects3d-debug.json"
    debug_cloud = _read(debug_path) if debug_path.exists() else {"objects": []}
    ground_truth = _read(run_dir / "truth.json")
    visibility = _read(run_dir / "visibility.json")
    evaluation = _read(_evaluation_path(run_dir))
    predicted_status, truth_status = _status_maps(evaluation)

    predicted_boxes = []
    for obj in state.get("objects") or ():
        if obj.get("missing") or str(obj.get("cls")) == "robot":
            continue
        box = _box_from_object(obj)
        if box is None:
            continue
        box["status"] = predicted_status.get(box["id"], "other")
        predicted_boxes.append(box)

    withheld_candidates = []
    for obj in debug_cloud.get("objects") or ():
        if obj.get("published_to_registry") is not False:
            continue
        box = _box_from_debug_candidate(obj)
        if box is None:
            continue
        withheld_candidates.append(box)
        predicted_boxes.append(box)

    truth_boxes = []
    visible = set(visibility.get("visible_truth_ids") or ())
    alignment = visibility.get("truth_alignment")
    if not isinstance(alignment, dict) or alignment.get("target_frame") != "map":
        raise ValueError(
            f"{run_dir}/visibility.json requires truth_alignment in map frame"
        )
    for item in ground_truth.get("truths") or ():
        identity = str(item.get("identity") or "")
        if visible and identity not in visible:
            continue
        if "yaw_rad" not in item:
            raise ValueError(
                f"{run_dir}/truth.json omits yaw_rad; regenerate WBT truth"
            )
        center, yaw = transform_planar_center_yaw(
            item["center_m"],
            item["yaw_rad"],
            translation_m=alignment.get("translation_m") or (),
            alignment_yaw_rad=alignment.get("yaw_rad"),
        )
        truth_boxes.append(
            {
                "id": identity,
                "label": str(item.get("label") or "object"),
                "center": list(center),
                "size": [float(value) for value in item["size_m"]],
                "yaw": yaw,
                "status": truth_status.get(identity, "miss"),
            }
        )

    point_clouds = []
    for item in cloud.get("objects") or ():
        points = item.get("points") or ()
        center = [float(value) for value in item.get("center") or (0, 0, 0)]
        label = str(item.get("cls") or "object")
        point_clouds.append(
            {
                "label": label,
                "status": _nearest_status(center, label, predicted_boxes),
                "points": [
                    [round(float(value), 4) for value in point[:3]]
                    for point in points[::2]
                ],
                "colors": [
                    [round(float(value), 3) for value in color[:3]]
                    for color in (item.get("point_colors") or ())[::2]
                ],
            }
        )
    for item in debug_cloud.get("objects") or ():
        if item.get("published_to_registry") is not False:
            continue
        point_clouds.append(
            {
                "label": str(item.get("cls") or "object"),
                "status": "candidate",
                "points": [
                    [round(float(value), 4) for value in point[:3]]
                    for point in (item.get("points") or ())[::2]
                ],
                "colors": [
                    [round(float(value), 3) for value in color[:3]]
                    for color in (item.get("point_colors") or ())[::2]
                ],
            }
        )

    occupancy = state.get("occupancy") or {}
    occupancy_evaluation_path = run_dir / "occupancy-evaluation.json"
    occupancy_evaluation = (
        _read(occupancy_evaluation_path)
        if occupancy_evaluation_path.exists()
        else {}
    )
    return {
        "world": world,
        "benchmark_status": "evaluated",
        "status_message": "",
        "coverage": (
            len(visibility.get("visible_truth_ids") or ())
            / max(1, int(visibility.get("truth_count") or 0))
        ),
        "path_length_m": visibility.get("path_length_m"),
        "odometry_path_agreement": visibility.get(
            "odometry_path_agreement"
        ) or {},
        "localized_pose_agreement": visibility.get(
            "localized_pose_agreement"
        ) or {},
        "perception_quality": state.get("perception_quality") or {},
        "withheld_candidate_count": len(withheld_candidates),
        "renderable": True,
        "metrics": {
            key: evaluation.get(key)
            for key in (
                "tp",
                "fp",
                "fn",
                "precision",
                "recall",
                "recall_at_covered",
                "f1",
                "label_accuracy",
                "duplicate_rate",
                "ghost_rate",
                "median_center_error_m",
                "ok",
                "gate_failures",
                "per_size_bucket",
            )
        }
        | {
            "raw_label_accuracy": (
                (evaluation.get("label_evaluation") or {}).get(
                    "raw_label_accuracy",
                    evaluation.get("label_accuracy"),
                )
            ),
            "label_evaluation_method": (
                (evaluation.get("label_evaluation") or {}).get("method")
            ),
            "label_judge": (
                (evaluation.get("label_evaluation") or {}).get("judge")
            ),
        },
        "point_cloud_inlier_fraction": (
            (evaluation.get("point_cloud") or {}).get(
                "median_point_inlier_fraction"
            )
        ),
        "mapping_metrics": {
            key: occupancy_evaluation.get(key)
            for key in (
                "observed_wall_segments",
                "unobserved_wall_segments",
                "observed_wall_coverage_0_10",
                "observed_wall_p95_error_m",
                "wall_angle_p95_error_deg",
            )
        },
        "predicted": predicted_boxes,
        "truth": truth_boxes,
        "clouds": point_clouds,
        "occupancy": occupancy,
        "occupancy_plane": _occupancy_plane_payload(occupancy),
        "costmaps": _load_costmaps(run_dir),
        "robot": state.get("robot") or {},
        "structural_segments": [],
    }


def _invalid_world_payload(world: str, run_dir: Path) -> dict[str, Any]:
    """Describe a rejected acquisition without manufacturing Scene metrics."""
    visibility = _read(run_dir / "visibility.json")
    visible_count = len(visibility.get("visible_truth_ids") or ())
    truth_count = int(visibility.get("truth_count") or 0)
    coverage = visible_count / max(1, truth_count)
    reasons = []
    if coverage < 0.25:
        reasons.append(
            f"visible WBT coverage {coverage:.1%} is below the 25.0% validity gate"
        )
    if int(visibility.get("odom_discontinuity_count") or 0):
        reasons.append(
            f"{int(visibility['odom_discontinuity_count'])} odometry discontinuities"
        )
    path_agreement = visibility.get("odometry_path_agreement") or {}
    if path_agreement and not path_agreement.get("valid", False):
        reasons.extend(
            str(reason)
            for reason in path_agreement.get("failures") or ()
        )
    localized_pose_agreement = visibility.get("localized_pose_agreement") or {}
    localized_pose_gate = localized_pose_agreement.get("gate") or {}
    if localized_pose_gate and not localized_pose_gate.get("valid", False):
        reasons.extend(
            str(reason)
            for reason in localized_pose_gate.get("failures") or ()
        )
    if visibility.get("aborted_reason"):
        reasons.append(str(visibility["aborted_reason"]))
    return {
        "world": world,
        "benchmark_status": "invalid",
        "status_message": "; ".join(reasons) or "acquisition rejected before scoring",
        "coverage": coverage,
        "path_length_m": visibility.get("path_length_m"),
        "odometry_path_agreement": path_agreement,
        "localized_pose_agreement": localized_pose_agreement,
        "perception_quality": {},
        "renderable": False,
        "metrics": {},
        "point_cloud_inlier_fraction": None,
        "mapping_metrics": {},
        "predicted": [],
        "truth": [],
        "clouds": [],
        "occupancy": {},
        "occupancy_plane": {},
        "costmaps": {},
        "robot": {},
        "structural_segments": [],
    }


_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Scene Webots review</title>
<style>
:root{--ink:#20252b;--muted:#66717d;--line:#ccd1d8;--paper:#fff;--ground:#f4f5f7}
*{box-sizing:border-box}
body{margin:20px;background:var(--ground);color:var(--ink);font:14px system-ui,sans-serif}
h1{font-size:22px;margin:0 0 8px}h2{font-size:17px;margin:30px 0 6px}
.legend,.metrics{display:flex;gap:14px;flex-wrap:wrap;margin:6px 0 10px}
.swatch{display:inline-block;width:10px;height:10px;margin-right:5px;border-radius:2px}
canvas{display:block;width:100%;height:680px;background:var(--paper)}
.orbit{height:min(92vh,1400px);min-height:820px}
.viewer{position:relative;width:100%;margin:6px 0 18px;background:var(--paper);border:1px solid var(--line)}
.viewer-toolbar{min-height:42px;display:flex;align-items:center;gap:8px;padding:6px 8px;border-bottom:1px solid var(--line);background:#f8f9fa}
.viewer-toolbar strong{font-size:12px;margin-right:auto}.viewer-toolbar label{font-size:12px;color:var(--muted)}
.zoom-readout{min-width:52px;text-align:center;color:var(--muted);font:11px ui-monospace,SFMono-Regular,Menlo,monospace}
.viewer-toolbar button,.viewer-toolbar select{height:28px;border:1px solid #b9c0c8;border-radius:4px;background:#fff;color:var(--ink);font:12px system-ui,sans-serif}
.viewer-toolbar button{min-width:30px;padding:0 9px;cursor:pointer}.viewer-toolbar button:hover{background:#edf1f5}
.viewer-toolbar button:focus-visible,.viewer-toolbar select:focus-visible{outline:2px solid #3578b8;outline-offset:1px}
.orbit,.spatial{cursor:grab;touch-action:none}.orbit:active,.spatial:active{cursor:grabbing}
.spatial{height:720px}
.object-readout{min-height:34px;padding:8px 10px;border-top:1px solid var(--line);color:var(--muted);font:12px ui-monospace,SFMono-Regular,Menlo,monospace}
.viewer:fullscreen{width:100%;height:100%;border:0;display:flex;flex-direction:column;background:#fff}
.viewer:fullscreen canvas{height:auto;min-height:0;flex:1}.viewer:fullscreen .object-readout{flex:0 0 auto}
.coverage-warning{width:min(1400px,100%);padding:8px 10px;margin:6px 0 10px;border-left:4px solid #d48718;background:#fff7e8;color:#62420d;font-size:12px;line-height:1.45}
.hint{color:#66717d;font-size:12px;margin:4px 0 8px}
.summary{width:min(1180px,100%);margin:22px 0 12px}
.summary-head{display:flex;align-items:baseline;justify-content:space-between;gap:16px;margin-bottom:8px}
.summary-head h2{margin:0}.result{font-weight:650;color:#a22f3d}
.table-wrap{overflow-x:auto;background:var(--paper);border:1px solid var(--line)}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
th,td{padding:9px 10px;text-align:right;border-bottom:1px solid #e2e5e9;white-space:nowrap}
th{font-size:11px;letter-spacing:.04em;text-transform:uppercase;color:#59636e;background:#f8f9fa}
th:first-child,td:first-child{text-align:left}
tfoot td{font-weight:700;border-top:2px solid #aeb5bd;border-bottom:0}
.pass{color:#238b57;font-weight:700}.fail{color:#a22f3d;font-weight:700}
.definitions{display:grid;grid-template-columns:repeat(auto-fit,minmax(235px,1fr));gap:0;border:1px solid var(--line);background:var(--paper);margin-top:10px}
.definition{padding:10px 12px;border-right:1px solid #e2e5e9;border-bottom:1px solid #e2e5e9}
.definition b{display:block;font-size:12px;margin-bottom:3px}.definition span{color:var(--muted);font-size:12px;line-height:1.45}
.notice{width:min(1180px,100%);padding:10px 12px;margin:12px 0 2px;border-left:4px solid #d48718;background:#fff7e8;color:#62420d;line-height:1.45}
@media(max-width:700px){body{margin:12px}.summary-head{display:block}.result{display:block;margin-top:5px}canvas{height:520px}.spatial{height:560px}.viewer-toolbar{flex-wrap:wrap}.viewer-toolbar strong{width:100%}}
</style>
</head>
<body>
<h1>Scene Webots review</h1>
<div class="hint">Generated __GENERATED__ · fixed review URL</div>
__DATASET_NOTICE__
<div class="legend">
 <span><i class="swatch" style="background:#238b57"></i>matched</span>
 <span><i class="swatch" style="background:#d48718"></i>label wrong</span>
 <span><i class="swatch" style="background:#c43d4b"></i>ghost / missed GT</span>
 <span><i class="swatch" style="background:#7d4bb3"></i>duplicate</span>
 <span><i class="swatch" style="background:#376f9f"></i>withheld candidate</span>
 <span><i class="swatch" style="background:#87919c"></i>unscored</span>
</div>
<section class="summary">
 <div class="summary-head">
  <h2>Experiment summary</h2>
  <span class="result" id="overall-result"></span>
 </div>
 <div class="table-wrap"><table id="summary-table"></table></div>
 <div class="definitions">
  <div class="definition"><b>TP / FP / FN</b><span>Matched visible WBT objects / unmatched Scene objects / visible WBT objects not detected.</span></div>
  <div class="definition"><b>Precision</b><span>TP ÷ (TP + FP). Higher means fewer duplicate and ghost objects.</span></div>
  <div class="definition"><b>Recall</b><span>TP ÷ (TP + FN). Higher means fewer visible objects were missed.</span></div>
  <div class="definition"><b>F1</b><span>Harmonic mean of precision and recall; both must improve to raise it.</span></div>
  <div class="definition"><b>Label GPT / raw</b><span>Codex GPT semantic-equivalence accuracy / exact plus configured-equivalence accuracy among the same matched objects. The judge model and reasons are stored in each evaluation artifact.</span></div>
  <div class="definition"><b>Wall ≤10 cm</b><span>Share of samples on sufficiently observed WBT wall surfaces supported by the SLAM occupancy grid within 10 cm.</span></div>
  <div class="definition"><b>Wall P95 / angle P95</b><span>95th-percentile distance to the physical WBT wall envelope / robust observed-wall heading error. Coverage and accuracy are reported separately.</span></div>
  <div class="definition"><b>Duplicate / ghost</b><span>Unmatched predictions near a real WBT object / predictions with no admissible nearby truth.</span></div>
  <div class="definition"><b>Candidate</b><span>Raw ConceptGraphs hypothesis retained in the debug snapshot but not admitted to persistent Scene memory because temporal/confidence confirmation did not pass. Candidates are drawn but excluded from TP/FP/FN.</span></div>
  <div class="definition"><b>Center error</b><span>Median 3D distance between matched Scene and WBT box centers, in metres.</span></div>
  <div class="definition"><b>Cloud inlier</b><span>Median fraction of sampled object points inside the matched WBT box (with evaluation margin).</span></div>
 </div>
 <div class="hint">All recall and label metrics are visibility-scoped. WBT geometry sets admissibility; semantic preference only resolves legal one-to-one candidates.</div>
</section>
<main id="root"></main>
<script>
const DATA=__DATA__;
const COLORS={match:"#238b57",label:"#d48718",ghost:"#c43d4b",
  miss:"#c43d4b",duplicate:"#7d4bb3",candidate:"#376f9f",
  other:"#87919c"};
const EDGES=[[0,1],[1,3],[3,2],[2,0],[4,5],[5,7],[7,6],[6,4],
  [0,4],[1,5],[2,6],[3,7]];
function corners(b){
  const [cx,cy,cz]=b.center,[sx,sy,sz]=b.size,c=Math.cos(b.yaw),s=Math.sin(b.yaw);
  return [[-1,-1,-1],[1,-1,-1],[-1,1,-1],[1,1,-1],[-1,-1,1],[1,-1,1],[-1,1,1],[1,1,1]].map(v=>{
    const x=v[0]*sx/2,y=v[1]*sy/2;
    return [cx+c*x-s*y,cy+s*x+c*y,cz+v[2]*sz/2];
  });
}
function fitExtent(world){
  const pts=[];
  world.truth.concat(world.predicted).forEach(b=>corners(b).forEach(p=>pts.push(p)));
  world.clouds.forEach(c=>c.points.forEach(p=>pts.push(p)));
  (world.occupancy_plane?.extent||[]).forEach(p=>pts.push([p[0],p[1],0]));
  if(!pts.length)return {center:[0,0,0],radius:1,points:[]};
  const lo=[0,1,2].map(k=>Math.min(...pts.map(p=>p[k])));
  const hi=[0,1,2].map(k=>Math.max(...pts.map(p=>p[k])));
  return {center:lo.map((v,k)=>(v+hi[k])/2),radius:Math.max(...hi.map((v,k)=>v-lo[k]))/2||1,points:pts};
}
function roomExtent(world){
  const pts=[];
  world.truth.forEach(b=>corners(b).forEach(p=>pts.push(p)));
  (world.occupancy_plane?.extent||[]).forEach(p=>pts.push([p[0],p[1],0]));
  // A review capture can contain exactly the off-map ghost that we are trying
  // to diagnose.  Do not let that one prediction push the whole room away
  // from the camera.  Fall back to all evidence only when no room reference
  // exists.
  if(!pts.length)return fitExtent(world);
  const lo=[0,1,2].map(k=>Math.min(...pts.map(p=>p[k])));
  const hi=[0,1,2].map(k=>Math.max(...pts.map(p=>p[k])));
  return {center:lo.map((v,k)=>(v+hi[k])/2),radius:Math.max(...hi.map((v,k)=>v-lo[k]))/2||1,points:pts};
}
function objectExtent(world){
  const pts=[];
  // The default inspection view should frame the Scene result, not every WBT
  // truth box in an only partially explored world.  Exclude evaluator-classed
  // ghosts so a single off-map prediction cannot make all useful boxes tiny.
  world.predicted.filter(b=>b.status!=="ghost").forEach(b=>corners(b).forEach(p=>pts.push(p)));
  if(!pts.length)world.predicted.forEach(b=>corners(b).forEach(p=>pts.push(p)));
  if(!pts.length)return roomExtent(world);
  const lo=[0,1,2].map(k=>Math.min(...pts.map(p=>p[k])));
  const hi=[0,1,2].map(k=>Math.max(...pts.map(p=>p[k])));
  return {center:lo.map((v,k)=>(v+hi[k])/2),radius:Math.max(...hi.map((v,k)=>v-lo[k]))/2||1,points:pts};
}
function sameItem(item,box,dashed){
  return Boolean(item&&item.box.id===box.id&&item.source===(dashed?"truth":"scene"));
}
function shouldLabel(box,dashed,mode,selected=false,hovered=false){
  if(selected||hovered)return true;
  if(mode==="none")return false;
  if(mode==="all")return true;
  if(mode==="hover")return false;
  return (dashed&&box.status==="miss")||(!dashed&&box.status!=="match");
}
function placeLabel(ctx,text,anchor,color,occupied,width,height,force=false){
  ctx.font="11px ui-monospace,SFMono-Regular,Menlo,monospace";
  const tw=Math.ceil(ctx.measureText(text).width),bw=tw+12,bh=20;
  const candidates=[[8,-26],[8,7],[-bw-8,-26],[-bw-8,7],[8,-48],[-bw-8,-48]];
  let rect=null;
  for(const [dx,dy] of candidates){
    const candidate={x:anchor[0]+dx,y:anchor[1]+dy,w:bw,h:bh};
    const inside=candidate.x>=2&&candidate.y>=2&&candidate.x+bw<=width-2&&candidate.y+bh<=height-2;
    const clear=!occupied.some(r=>candidate.x<r.x+r.w+3&&candidate.x+bw+3>r.x&&candidate.y<r.y+r.h+3&&candidate.y+bh+3>r.y);
    if(inside&&clear){rect=candidate;break}
  }
  if(!rect&&!force)return;
  rect=rect||{x:Math.max(2,Math.min(width-bw-2,anchor[0]+8)),y:Math.max(2,Math.min(height-bh-2,anchor[1]-26)),w:bw,h:bh};
  occupied.push(rect);ctx.fillStyle="rgba(255,255,255,.92)";ctx.fillRect(rect.x,rect.y,rect.w,rect.h);
  ctx.fillStyle=color;ctx.fillRect(rect.x,rect.y,3,rect.h);ctx.fillStyle="#20252b";ctx.fillText(text,rect.x+7,rect.y+14);
}
function selectionText(item){
  if(!item)return "Hover to identify a box; click to pin it. Use Focus selected for a close inspection.";
  const b=item.box,c=b.center.map(v=>v.toFixed(2)).join(", "),s=b.size.map(v=>v.toFixed(2)).join(" × ");
  const confirmation=b.status==="candidate"?` · evidence ${b.confirmation_unique_frames}/${b.confirmation_min_unique_frames} frame(s), mean confidence ${(b.confirmation_mean_confidence||0).toFixed(3)}`:"";
  return `${item.source.toUpperCase()} · ${b.label} · ${b.status}${confirmation} · center [${c}] m · size [${s}] m`;
}
function draw3d(canvas,world,view){
  const dpr=devicePixelRatio||1,w=canvas.clientWidth,h=canvas.clientHeight;
  canvas.width=w*dpr;canvas.height=h*dpr;const x=canvas.getContext("2d");x.scale(dpr,dpr);
  x.clearRect(0,0,w,h);const ext=fitExtent(world),room=roomExtent(world),objects=objectExtent(world),focal=Math.min(w,h)*.92;
  const frame=view.focusRadius?{center:view.target,radius:view.focusRadius,points:[]}:(view.frameMode==="room"?room:objects);
  const target=frame.center,frameRadius=frame.radius;
  // Keep the perspective camera outside the framed room/object and zoom by
  // changing the lens.  The previous dolly-style zoom eventually moved the
  // camera through the geometry, after which the near-depth clamp made further
  // wheel input appear to do nothing.
  const cameraDistance=frameRadius*3.0;
  const cy=Math.cos(view.yaw),sy=Math.sin(view.yaw),cp=Math.cos(view.pitch),sp=Math.sin(view.pitch);
  function cameraCoordinates(p){const a=p[0]-target[0],b=p[1]-target[1],z=p[2]-target[2];
    const rx=-cy*a+sy*b,ry=sy*a+cy*b,vertical=cp*z-sp*ry,forward=cp*ry+sp*z;
    return [rx,vertical,Math.max(frameRadius*.001,cameraDistance-forward)];}
  const framePoints=frame.points.length?frame.points:[
    [-1,-1,-1],[1,-1,-1],[-1,1,-1],[1,1,-1],[-1,-1,1],[1,-1,1],[-1,1,1],[1,1,1]
  ].map(v=>target.map((value,k)=>value+v[k]*frameRadius));
  const normalized=framePoints.map(p=>{const [rx,vertical,depth]=cameraCoordinates(p);return [rx/depth,vertical/depth]});
  const spanX=Math.max(...normalized.map(p=>p[0]))-Math.min(...normalized.map(p=>p[0]));
  const spanY=Math.max(...normalized.map(p=>p[1]))-Math.min(...normalized.map(p=>p[1]));
  // zoom=1 means the selected frame fills the viewport.  This makes the
  // initial scale independent of world size and still leaves deep optical
  // zoom available for individual bbox inspection.
  const fitScale=Math.min(
    spanX>0?w*.84/(focal*spanX):1,
    spanY>0?h*.82/(focal*spanY):1
  );
  function project(p){let a=p[0]-target[0],b=p[1]-target[1],z=p[2]-target[2];
    // Forward in XY is (sin(yaw), cos(yaw)).  Screen-right must therefore be
    // (-cos(yaw), sin(yaw)) so right × up = forward.  The opposite sign
    // mirrors the map horizontally even though all metric coordinates agree.
    const [rx,vertical,depth]=cameraCoordinates(p),perspective=focal*fitScale*view.zoom/depth;
    return [w/2+view.panX+rx*perspective,h/2+view.panY-vertical*perspective,depth];}
  function groundGrid(){
    const step=ext.radius>8?2:ext.radius>4?1:.5,pad=ext.radius*1.15;
    const x0=Math.floor((ext.center[0]-pad)/step)*step,x1=Math.ceil((ext.center[0]+pad)/step)*step;
    const y0=Math.floor((ext.center[1]-pad)/step)*step,y1=Math.ceil((ext.center[1]+pad)/step)*step;
    x.lineWidth=1;x.strokeStyle="#dfe3e8";x.setLineDash([]);
    for(let gx=x0;gx<=x1+step*.1;gx+=step){const a=project([gx,y0,0]),b=project([gx,y1,0]);x.beginPath();x.moveTo(a[0],a[1]);x.lineTo(b[0],b[1]);x.stroke();}
    for(let gy=y0;gy<=y1+step*.1;gy+=step){const a=project([x0,gy,0]),b=project([x1,gy,0]);x.beginPath();x.moveTo(a[0],a[1]);x.lineTo(b[0],b[1]);x.stroke();}
    const origin=project([0,0,0]),xAxis=project([Math.min(x1,Math.max(step,x0+2*step)),0,0]);
    const yAxis=project([0,Math.min(y1,Math.max(step,y0+2*step)),0]),zAxis=project([0,0,2*step]);
    [[xAxis,"#c94b50","x"],[yAxis,"#3578b8","y"],[zAxis,"#2c8b57","z"]].forEach(([end,color,label])=>{
      x.strokeStyle=color;x.lineWidth=2;x.beginPath();x.moveTo(origin[0],origin[1]);x.lineTo(end[0],end[1]);x.stroke();
      x.fillStyle=color;x.font="11px system-ui";x.fillText(label,end[0]+4,end[1]-4);
    });
    x.fillStyle="#78828c";x.font="10px system-ui";x.fillText(`${step} m grid`,12,h-12);
  }
  groundGrid();
  function occupancyPlane(){
    const plane=world.occupancy_plane||{},yaw=plane.yaw||0,c=Math.cos(yaw),s=Math.sin(yaw);
    function cell(cx,cy,sx,sy,color,z){
      const points=[[-sx/2,-sy/2],[sx/2,-sy/2],[sx/2,sy/2],[-sx/2,sy/2]].map(([a,b])=>project([cx+c*a-s*b,cy+s*a+c*b,z]));
      x.fillStyle=color;x.beginPath();x.moveTo(points[0][0],points[0][1]);for(let i=1;i<points.length;i++)x.lineTo(points[i][0],points[i][1]);x.closePath();x.fill();
    }
    (plane.free||[]).forEach(v=>cell(v[0],v[1],v[2],v[3],"#b8c2cc",.001));
    // A one-pixel SLAM wall is only 5 cm wide in the captured maps.  Give it
    // a 7.5 cm visual minimum so it remains legible after perspective
    // projection; this affects the review rendering only, never geometry or
    // benchmark measurements.
    const size=Math.max(plane.resolution||.05,.075);
    (plane.occupied||[]).forEach(v=>cell(v[0],v[1],size,size,"#1f242a",.003));
    const boundary=(plane.extent||[]).map(v=>project([v[0],v[1],.004]));
    if(boundary.length===4){
      x.strokeStyle="#7f8a96";x.lineWidth=1.25;x.setLineDash([5,4]);
      x.beginPath();x.moveTo(boundary[0][0],boundary[0][1]);
      for(let i=1;i<boundary.length;i++)x.lineTo(boundary[i][0],boundary[i][1]);
      x.closePath();x.stroke();x.setLineDash([]);
    }
  }
  occupancyPlane();
  const dots=[];world.clouds.forEach(cl=>cl.points.forEach((p,i)=>{const q=project(p);dots.push([q[2],q,cl.colors[i]||null]);}));
  dots.sort((a,b)=>b[0]-a[0]).forEach(d=>{const c=d[2];x.fillStyle=c?`rgb(${c.map(v=>Math.round(v*255)).join(",")})`:"#8a929b";x.fillRect(d[1][0],d[1][1],1.5,1.5);});
  const hits=[],labels=[];
  function box(b,dashed){const cs=corners(b).map(project);x.strokeStyle=COLORS[b.status];x.lineWidth=dashed?1.5:2.5;x.setLineDash(dashed?[5,4]:[]);
    EDGES.forEach(e=>{x.beginPath();x.moveTo(cs[e[0]][0],cs[e[0]][1]);x.lineTo(cs[e[1]][0],cs[e[1]][1]);x.stroke();});
    const c=project(b.center);x.setLineDash([]);
    hits.push({x:c[0],y:c[1],box:b,source:dashed?"truth":"scene"});
    const selected=sameItem(view.selected,b,dashed),hovered=sameItem(view.hovered,b,dashed);
    if(shouldLabel(b,dashed,view.labelMode,selected,hovered))labels.push({box:b,anchor:c,dashed,selected,hovered});}
  world.truth.forEach(b=>box(b,true));world.predicted.forEach(b=>box(b,false));
  const occupied=[];labels.sort((a,b)=>Number(b.selected)-Number(a.selected)||Number(b.hovered)-Number(a.hovered)||(a.box.status==="match")-(b.box.status==="match")).forEach(item=>{
    placeLabel(x,item.box.label,item.anchor,COLORS[item.box.status],occupied,w,h,item.selected||item.hovered);
  });
  view.hits=hits;
 }
function nearestHit(hits,x,y,maxDistance=24){
  let best=null,distance=maxDistance;
  (hits||[]).forEach(hit=>{const d=Math.hypot(hit.x-x,hit.y-y);if(d<distance){best=hit;distance=d}});
  return best;
}
function wireToolbar(viewer,state,render,reset,focusSelected){
  const readout=viewer.querySelector(".object-readout"),select=viewer.querySelector("[data-labels]"),zoomReadout=viewer.querySelector("[data-zoom-readout]");
  const minZoom=state.minZoom??.05,maxZoom=state.maxZoom??4096,zoomStep=state.zoomStep??2;
  const syncZoom=()=>{if(zoomReadout)zoomReadout.textContent=`${state.zoom.toFixed(state.zoom<10?1:0)}×`};
  state.syncZoom=syncZoom;syncZoom();
  select.value=state.labelMode;select.onchange=()=>{state.labelMode=select.value;render()};
  viewer.querySelector("[data-fit]").onclick=()=>{reset("room");readout.textContent=selectionText(null);syncZoom();render()};
  const fitObjects=viewer.querySelector("[data-fit-objects]");
  if(fitObjects)fitObjects.onclick=()=>{reset("objects");readout.textContent=selectionText(null);syncZoom();render()};
  const closeView=viewer.querySelector("[data-close-view]");
  if(closeView)closeView.onclick=()=>{reset("close");readout.textContent=selectionText(null);syncZoom();render()};
  viewer.querySelector("[data-zoom-in]").onclick=()=>{state.zoom=Math.min(maxZoom,state.zoom*zoomStep);syncZoom();render()};
  viewer.querySelector("[data-zoom-out]").onclick=()=>{state.zoom=Math.max(minZoom,state.zoom/zoomStep);syncZoom();render()};
  const focus=viewer.querySelector("[data-focus]");
  if(focus)focus.onclick=()=>{if(state.selected)focusSelected(state.selected)};
  viewer.querySelector("[data-fullscreen]").onclick=()=>viewer.requestFullscreen?.();
  return item=>{state.selected=item;readout.textContent=selectionText(item);render()};
}
function enableOrbit(viewer,world){
  // Open on the useful Scene object extent rather than the whole occupancy
  // map. Large partially explored maps otherwise make every bbox tiny even at
  // a seemingly large numeric zoom. Start close enough for bbox inspection;
  // Fit room remains available when the reviewer needs global context.
  // zoom=1 already means that the active frame fills the viewport.  Keep the
  // default only modestly closer than "Fit all objects"; very large defaults
  // crop away every box around the frame centre and look like an empty,
  // unzoomable view.  A selected object gets its own much smaller frame, so it
  // also needs only a small multiplier to fill the canvas.
  const closeZoom=2.25,focusZoom=1.8,fitObjectZoom=1.0,roomZoom=.96;
  const canvas=viewer.querySelector("canvas"),v={yaw:-.75,pitch:.65,zoom:closeZoom,minZoom:.03,maxZoom:1e18,zoomStep:2.0,panX:0,panY:0,target:null,focusRadius:null,frameMode:"objects",labelMode:"hover",selected:null,hovered:null,hits:[]};
  let drag=false,panning=false,moved=false,last=[0,0],selectItem;
  const reset=(mode="room")=>{v.yaw=-.75;v.pitch=.65;v.zoom=mode==="close"?closeZoom:mode==="objects"?fitObjectZoom:roomZoom;v.panX=0;v.panY=0;v.target=null;v.focusRadius=null;v.frameMode=mode==="room"?"room":"objects";v.selected=null;v.hovered=null};
  const focusItem=item=>{v.target=item.box.center.slice();v.focusRadius=Math.max(.08,...item.box.size.map(value=>value/2));v.frameMode="focus";v.zoom=focusZoom;v.panX=0;v.panY=0;v.syncZoom?.();render()};
  const render=()=>draw3d(canvas,world,v);selectItem=wireToolbar(viewer,v,render,reset,focusItem);
  canvas.oncontextmenu=e=>e.preventDefault();
  canvas.onpointerdown=e=>{drag=true;panning=e.button===2||e.shiftKey;moved=false;last=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};
  canvas.onpointermove=e=>{if(!drag){const hovered=nearestHit(v.hits,e.offsetX,e.offsetY,18);
      if((hovered?.box.id||null)!==(v.hovered?.box.id||null)||(hovered?.source||null)!==(v.hovered?.source||null)){v.hovered=hovered;render()}return}
    const dx=e.clientX-last[0],dy=e.clientY-last[1];if(Math.abs(dx)+Math.abs(dy)>2)moved=true;
    if(panning){v.panX+=dx;v.panY+=dy}else{v.yaw+=dx*.008;v.pitch=Math.max(.08,Math.min(1.48,v.pitch+dy*.008))}
    last=[e.clientX,e.clientY];render()};
  canvas.onpointerup=e=>{drag=false;if(!moved)selectItem(nearestHit(v.hits,e.offsetX,e.offsetY))};
  canvas.onpointerleave=()=>{if(!drag&&v.hovered){v.hovered=null;render()}};
  canvas.ondblclick=e=>{const item=nearestHit(v.hits,e.offsetX,e.offsetY,32);if(item){selectItem(item);focusItem(item)}};
  canvas.onwheel=e=>{e.preventDefault();const old=v.zoom,deviceDelta=e.deltaY*(e.deltaMode===1?24:e.deltaMode===2?canvas.clientHeight:1),wheelDelta=Math.max(-240,Math.min(240,deviceDelta)),next=Math.max(v.minZoom,Math.min(v.maxZoom,old*Math.exp(-wheelDelta*.008))),ratio=next/old;
    v.panX=e.offsetX-canvas.clientWidth/2-ratio*(e.offsetX-canvas.clientWidth/2-v.panX);
    v.panY=e.offsetY-canvas.clientHeight/2-ratio*(e.offsetY-canvas.clientHeight/2-v.panY);
    v.zoom=next;v.syncZoom?.();render()};
  render();return render;
}
function gridCorners(grid){
  const ox=grid.origin_x||0,oy=grid.origin_y||0,yaw=grid.origin_yaw||0;
  const width=(grid.width||0)*(grid.resolution||0),height=(grid.height||0)*(grid.resolution||0);
  const c=Math.cos(yaw),s=Math.sin(yaw);
  return [[0,0],[width,0],[width,height],[0,height]].map(([x,y])=>[ox+c*x-s*y,oy+s*x+c*y]);
}
function drawGridImage(x,grid,im,px,py,scale,opacity){
  const yaw=grid.origin_yaw||0,c=Math.cos(yaw),s=Math.sin(yaw),r=grid.resolution||.05;
  const heightM=(grid.height||0)*r,topX=(grid.origin_x||0)-s*heightM,topY=(grid.origin_y||0)+c*heightM;
  x.save();x.globalAlpha=opacity;x.imageSmoothingEnabled=false;
  x.transform(c*r*scale,-s*r*scale,s*r*scale,c*r*scale,px(topX),py(topY));
  x.drawImage(im,0,0);x.restore();
}
function draw2d(canvas,world,visibility,view,imageCache){
  const dpr=devicePixelRatio||1,w=canvas.clientWidth,h=canvas.clientHeight;canvas.width=w*dpr;canvas.height=h*dpr;
  const x=canvas.getContext("2d");x.scale(dpr,dpr);x.clearRect(0,0,w,h);
  const o=world.occupancy,pad=35,points=[];
  if(o.width&&o.height)points.push(...gridCorners(o));
  Object.values(world.costmaps||{}).forEach(grid=>points.push(...gridCorners(grid)));
  world.truth.concat(world.predicted).forEach(b=>corners(b).forEach(p=>points.push(p)));
  const x0=points.length?Math.min(...points.map(p=>p[0])):-5,y0=points.length?Math.min(...points.map(p=>p[1])):-5;
  const x1=points.length?Math.max(...points.map(p=>p[0])):5,y1=points.length?Math.max(...points.map(p=>p[1])):5;
  const baseScale=Math.min((w-2*pad)/(x1-x0),(h-2*pad)/(y1-y0));
  const centerX=view.target?.[0]??(x0+x1)/2,centerY=view.target?.[1]??(y0+y1)/2;
  const scale=baseScale*view.zoom,px=X=>w/2+(X-centerX)*scale+view.panX,py=Y=>h/2-(Y-centerY)*scale+view.panY;
  function overlays(){const hits=[],labels=[];function box(b,dashed){const cs=corners(b).slice(0,4).map(p=>[px(p[0]),py(p[1])]);x.strokeStyle=COLORS[b.status];x.lineWidth=dashed?1.5:2.5;x.setLineDash(dashed?[5,4]:[]);
      x.beginPath();x.moveTo(...cs[0]);x.lineTo(...cs[1]);x.lineTo(...cs[3]);x.lineTo(...cs[2]);x.closePath();x.stroke();x.setLineDash([]);
      const c=[px(b.center[0]),py(b.center[1])];
      hits.push({x:c[0],y:c[1],box:b,source:dashed?"truth":"scene"});
      const selected=sameItem(view.selected,b,dashed),hovered=sameItem(view.hovered,b,dashed);
      if(shouldLabel(b,dashed,view.labelMode,selected,hovered))labels.push({box:b,anchor:c,dashed,selected,hovered});}
    world.truth.forEach(b=>box(b,true));world.predicted.forEach(b=>box(b,false));
    const occupied=[];labels.sort((a,b)=>Number(b.selected)-Number(a.selected)||Number(b.hovered)-Number(a.hovered)||(a.box.status==="match")-(b.box.status==="match")).forEach(item=>{
      placeLabel(x,item.box.label,item.anchor,COLORS[item.box.status],occupied,w,h,item.selected||item.hovered);
    });
    view.hits=hits;
    x.setLineDash([]);x.strokeStyle="#00b846";x.lineWidth=2;
    (world.structural_segments||[]).forEach(segment=>{x.beginPath();x.moveTo(px(segment.start[0]),py(segment.start[1]));x.lineTo(px(segment.end[0]),py(segment.end[1]));x.stroke()});
    if(Number.isFinite(world.robot.x)){const a=world.robot.yaw||0,cx=px(world.robot.x),cy=py(world.robot.y);x.fillStyle="#1e5da8";x.beginPath();x.moveTo(cx+Math.cos(a)*12,cy-Math.sin(a)*12);x.lineTo(cx+Math.cos(a+2.5)*8,cy-Math.sin(a+2.5)*8);x.lineTo(cx+Math.cos(a-2.5)*8,cy-Math.sin(a-2.5)*8);x.closePath();x.fill();}
  }
  const images=imageCache||{},needed=[];
  if(o.png_b64&&!images.occupancy)needed.push(["occupancy",o]);
  Object.entries(world.costmaps||{}).forEach(([name,grid])=>{if(grid.png_b64&&!images[name])needed.push([name,grid])});
  let remaining=needed.length;
  function render(){x.clearRect(0,0,w,h);
    if(images.occupancy&&visibility.occupancy!==false)drawGridImage(x,o,images.occupancy,px,py,scale,.8);
    ["global","local"].forEach(name=>{const grid=(world.costmaps||{})[name];if(grid&&images[name]&&visibility[name])drawGridImage(x,grid,images[name],px,py,scale,name==="global"?.72:.9)});
    overlays();
  }
  if(!remaining){render();return render}
  needed.forEach(([name,grid])=>{const im=new Image();im.onload=()=>{images[name]=im;if(!--remaining)render()};im.src="data:image/png;base64,"+grid.png_b64});
  return render;
}
function enableSpatial(viewer,world,visibility){
  const canvas=viewer.querySelector("canvas"),v={zoom:1.25,minZoom:.05,maxZoom:1e9,zoomStep:2.2,panX:0,panY:0,target:null,labelMode:"hover",selected:null,hovered:null,hits:[]};
  const images={};let paint=()=>{},drag=false,moved=false,last=[0,0],selectItem;
  const reset=()=>{v.zoom=1;v.panX=0;v.panY=0;v.target=null;v.selected=null;v.hovered=null};
  const focusItem=item=>{const room=roomExtent(world),boxRadius=Math.max(.08,item.box.size[0]/2,item.box.size[1]/2);
    v.target=item.box.center.slice(0,2);v.panX=0;v.panY=0;v.zoom=Math.max(3,Math.min(v.maxZoom,room.radius*.42/boxRadius));v.syncZoom?.();render()};
  const render=()=>{paint=draw2d(canvas,world,visibility,v,images)};selectItem=wireToolbar(viewer,v,render,reset,focusItem);
  canvas.onpointerdown=e=>{drag=true;moved=false;last=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};
  canvas.onpointermove=e=>{if(!drag){const hovered=nearestHit(v.hits,e.offsetX,e.offsetY,18);
      if((hovered?.box.id||null)!==(v.hovered?.box.id||null)||(hovered?.source||null)!==(v.hovered?.source||null)){v.hovered=hovered;render()}return}
    const dx=e.clientX-last[0],dy=e.clientY-last[1];if(Math.abs(dx)+Math.abs(dy)>2)moved=true;v.panX+=dx;v.panY+=dy;last=[e.clientX,e.clientY];render()};
  canvas.onpointerup=e=>{drag=false;if(!moved)selectItem(nearestHit(v.hits,e.offsetX,e.offsetY))};
  canvas.onpointerleave=()=>{if(!drag&&v.hovered){v.hovered=null;render()}};
  canvas.ondblclick=e=>{const item=nearestHit(v.hits,e.offsetX,e.offsetY,32);if(item){selectItem(item);focusItem(item)}};
  canvas.onwheel=e=>{e.preventDefault();const old=v.zoom,next=Math.max(v.minZoom,Math.min(v.maxZoom,old*Math.exp(-e.deltaY*.02))),ratio=next/old;
    v.panX=e.offsetX-canvas.clientWidth/2-(e.offsetX-canvas.clientWidth/2-v.panX)*ratio;
    v.panY=e.offsetY-canvas.clientHeight/2-(e.offsetY-canvas.clientHeight/2-v.panY)*ratio;
    v.zoom=next;render()};
  render();return render;
}
function viewerMarkup(kind,title){
  const fitObjects=kind==="orbit"?'<button data-close-view title="Restore the closer default inspection view">Close view</button><button data-fit-objects>Fit all objects</button>':"";
  return `<div class="viewer ${kind}-viewer"><div class="viewer-toolbar"><strong>${title}</strong><label>Labels <select data-labels><option value="hover">Hover + selected</option><option value="issues">Issues only</option><option value="all">All</option><option value="none">None</option></select></label><button data-focus title="Focus the selected box">Focus selected</button><button data-zoom-out title="Zoom out">−</button><span class="zoom-readout" data-zoom-readout aria-live="polite"></span><button data-zoom-in title="Zoom in">+</button>${fitObjects}<button data-fit>${kind==="orbit"?"Fit room":"Fit map"}</button><button data-fullscreen>Fullscreen</button></div><canvas class="${kind}"></canvas><div class="object-readout">${selectionText(null)}</div></div>`;
}
const root=document.getElementById("root"),redraw=[];
function pct(value){return Number.isFinite(value)?`${(100*value).toFixed(1)}%`:"—"}
function metres(value){return Number.isFinite(value)?value.toFixed(3):"—"}
function renderSummary(){
  const table=document.getElementById("summary-table");
  const headers=["World","Coverage","Path m","Wall ≤10 cm","Wall P95 m","Angle P95°","TP","FP","FN","Candidates","Precision","Recall@covered","F1","Label GPT / raw","Duplicate","Ghost","Center m","Cloud inlier","Gate"];
  let tp=0,fp=0,fn=0,candidates=0,labelCorrect=0,rawLabelCorrect=0,matched=0,judgedMatched=0,dup=0,ghost=0,evaluated=0,allPass=true,hasInvalid=false;
  const rows=DATA.map(world=>{const m=world.metrics||{},invalid=world.benchmark_status==="invalid",ok=!invalid&&Boolean(m.ok);
    const map=world.mapping_metrics||{},judged=Boolean(m.label_judge);
    if(invalid)hasInvalid=true;
    else{evaluated++;tp+=m.tp||0;fp+=m.fp||0;fn+=m.fn||0;candidates+=world.withheld_candidate_count||0;matched+=m.tp||0;if(judged){judgedMatched+=m.tp||0;labelCorrect+=(m.tp||0)*(m.label_accuracy||0)}rawLabelCorrect+=(m.tp||0)*(m.raw_label_accuracy??m.label_accuracy??0);
      dup+=m.duplicate_rate||0;ghost+=m.ghost_rate||0;allPass=allPass&&ok;}
    const gate=invalid?"INVALID":ok?"PASS":"FAIL";
    const title=world.status_message?` title="${world.status_message.replaceAll('"',"&quot;")}"`:"";
    const label=judged?`${pct(m.label_accuracy)} / ${pct(m.raw_label_accuracy??m.label_accuracy)}`:`pending / ${pct(m.raw_label_accuracy??m.label_accuracy)}`;
    return `<tr><td>${world.world}</td><td>${pct(world.coverage)}</td><td>${metres(world.path_length_m)}</td><td>${pct(map.observed_wall_coverage_0_10)}</td><td>${metres(map.observed_wall_p95_error_m)}</td><td>${Number.isFinite(map.wall_angle_p95_error_deg)?map.wall_angle_p95_error_deg.toFixed(2):"—"}</td><td>${m.tp??"—"}</td><td>${m.fp??"—"}</td><td>${m.fn??"—"}</td><td>${world.withheld_candidate_count??0}</td><td>${pct(m.precision)}</td><td>${pct(m.recall_at_covered??m.recall)}</td><td>${pct(m.f1)}</td><td>${label}</td><td>${pct(m.duplicate_rate)}</td><td>${pct(m.ghost_rate)}</td><td>${metres(m.median_center_error_m)}</td><td>${pct(world.point_cloud_inlier_fraction)}</td><td class="${ok?"pass":"fail"}"${title}>${gate}</td></tr>`;
  }).join("");
  const precision=tp+fp?tp/(tp+fp):0,recall=tp+fn?tp/(tp+fn):0,f1=precision+recall?2*precision*recall/(precision+recall):0;
  const count=evaluated||1,overallGate=hasInvalid?"INCOMPLETE":allPass?"PASS":"FAIL";
  table.innerHTML=`<thead><tr>${headers.map(h=>`<th>${h}</th>`).join("")}</tr></thead><tbody>${rows}</tbody><tfoot><tr><td>Evaluated subtotal</td><td>—</td><td>—</td><td>—</td><td>—</td><td>—</td><td>${tp}</td><td>${fp}</td><td>${fn}</td><td>${candidates}</td><td>${pct(precision)}</td><td>${pct(recall)}</td><td>${pct(f1)}</td><td>${judgedMatched?pct(labelCorrect/judgedMatched):"pending"} / ${pct(matched?rawLabelCorrect/matched:0)}</td><td>${pct(dup/count)}</td><td>${pct(ghost/count)}</td><td>—</td><td>—</td><td class="${allPass&&!hasInvalid?"pass":"fail"}">${overallGate}</td></tr></tfoot>`;
  document.getElementById("overall-result").textContent=hasInvalid?"Run incomplete: invalid acquisition":allPass?"Acceptance gates passed":"Acceptance gates not met";
  document.getElementById("overall-result").className=`result ${allPass?"pass":"fail"}`;
}
renderSummary();
DATA.forEach(world=>{const section=document.createElement("section"),m=world.metrics;
  if(world.renderable===false){
    section.innerHTML=`<h2>${world.world}</h2><div class="notice"><b>Invalid acquisition.</b> ${world.status_message}. No Scene accuracy metrics were manufactured for this world.</div>`;
    root.appendChild(section);return;
  }
  const incomplete=(world.coverage??0)<.9;
  const coverageNote=incomplete?`<div class="coverage-warning"><b>Incomplete acquisition:</b> ${(100*(world.coverage||0)).toFixed(1)}% of WBT objects became RGB-D visible over ${(world.path_length_m||0).toFixed(2)} m of travel. The SLAM image below is only the explored area, not the full world map.</div>`:"";
  const map=world.mapping_metrics||{},labelText=m.label_judge?`label GPT ${(m.label_accuracy||0).toFixed(3)} / raw ${(m.raw_label_accuracy??m.label_accuracy??0).toFixed(3)}`:`label GPT pending · raw ${(m.raw_label_accuracy??m.label_accuracy??0).toFixed(3)}`;
  const size=m.per_size_bucket||{},small=size.small_cube_edge_lt_0_30m||{},medium=size.medium_cube_edge_0_30_to_0_75m||{},large=size.large_cube_edge_ge_0_75m||{};
  const sizeText=`equivalent-cube edge small <30 cm: coverage ${pct(small.coverage)}, recall@covered ${pct(small.recall_at_covered)}, label ${pct(small.label_accuracy_among_matched)} · medium 30–75 cm: coverage ${pct(medium.coverage)}, recall@covered ${pct(medium.recall_at_covered)}, label ${pct(medium.label_accuracy_among_matched)} · large ≥75 cm: coverage ${pct(large.coverage)}, recall@covered ${pct(large.recall_at_covered)}, label ${pct(large.label_accuracy_among_matched)}`;
  const quality=world.perception_quality||{},snap=`surface snap ${quality.surface_snap_applied??0}/${quality.surface_snap_attempts??0}`;
  section.innerHTML=`<h2>${world.world}</h2><div class="metrics">coverage ${(100*(world.coverage||0)).toFixed(1)}% · recall@covered ${pct(m.recall_at_covered??m.recall)} · path ${(world.path_length_m||0).toFixed(2)} m · walls ≤10 cm ${pct(map.observed_wall_coverage_0_10)} · wall P95 ${metres(map.observed_wall_p95_error_m)} m · angle P95 ${Number.isFinite(map.wall_angle_p95_error_deg)?map.wall_angle_p95_error_deg.toFixed(2):"—"}° · TP ${m.tp} · FP ${m.fp} · FN ${m.fn} · candidates ${world.withheld_candidate_count??0} · F1 ${(m.f1||0).toFixed(3)} · ${labelText} · ${snap}</div><div class="metrics">${sizeText}</div>${coverageNote}<div class="hint">Solid boxes = confirmed Scene objects; blue boxes = withheld raw candidates; dashed boxes = WBT truth. Candidates remain inspectable but are excluded from TP/FP/FN. Drag to rotate; Shift-drag or right-drag to pan; wheel zooms toward the pointer; double-click or use Focus selected for a close view.</div>${viewerMarkup("orbit","Perspective 3D")}<div class="hint">The same boxes in the live SLAM map frame. Drag to pan; wheel / + / − to zoom; double-click a box to focus.</div>${viewerMarkup("spatial","Spatial map")}`;
  root.appendChild(section);const viewers=section.querySelectorAll(".viewer"),visibility={occupancy:true,global:false,local:false};
  redraw.push(enableOrbit(viewers[0],world));redraw.push(enableSpatial(viewers[1],world,visibility));
  });
addEventListener("resize",()=>redraw.forEach(fn=>fn()));
</script>
</body></html>
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--run",
        action="append",
        required=True,
        metavar="WORLD=DIR",
        help="one evaluated world directory; repeat for each WBT",
    )
    parser.add_argument(
        "--invalid-run",
        action="append",
        default=[],
        metavar="WORLD=DIR",
        help="one rejected acquisition to show in the summary without scoring",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--dataset-note",
        default="",
        help="optional reader-facing note describing the displayed dataset",
    )
    parser.add_argument(
        "--navigation-grid",
        action="append",
        default=[],
        metavar="NAME=MAP_YAML,COSTMAP_JSON[,WBT]",
        help="add one measured SLAM/costmap overlay before Scene object results",
    )
    args = parser.parse_args()
    worlds = []
    for value in args.run:
        world, separator, raw_path = value.partition("=")
        if not separator or not world or not raw_path:
            parser.error(f"invalid --run {value!r}; expected WORLD=DIR")
        worlds.append(_world_payload(world, Path(raw_path)))
    for value in args.invalid_run:
        world, separator, raw_path = value.partition("=")
        if not separator or not world or not raw_path:
            parser.error(f"invalid --invalid-run {value!r}; expected WORLD=DIR")
        worlds.append(_invalid_world_payload(world, Path(raw_path)))
    navigation = []
    for value in args.navigation_grid:
        name, separator, raw_paths = value.partition("=")
        paths = raw_paths.split(",") if separator else []
        if not name or len(paths) not in (2, 3) or not all(paths):
            parser.error(
                f"invalid --navigation-grid {value!r}; expected "
                "NAME=MAP_YAML,COSTMAP_JSON[,WBT]"
            )
        navigation.append(
            _navigation_payload(
                name,
                Path(paths[0]),
                Path(paths[1]),
                Path(paths[2]) if len(paths) == 3 else None,
            )
        )
    encoded = json.dumps(worlds, separators=(",", ":")).replace("</", "<\\/")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    generated = datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")
    dataset_notice = (
        f'<div class="notice">{html_module.escape(args.dataset_note)}</div>'
        if args.dataset_note
        else ""
    )
    html = (
        _HTML.replace("__DATA__", encoded)
        .replace("__GENERATED__", generated)
        .replace("__DATASET_NOTICE__", dataset_notice)
    )
    args.output.write_text(html, encoding="utf-8")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
