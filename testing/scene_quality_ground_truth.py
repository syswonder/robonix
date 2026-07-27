#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Ground-truth helpers for the Webots Scene object-quality acceptance gate.

Runtime Scene code must remain deployment-neutral.  The simulator, however,
has an authoritative world file.  This module resolves a named robot and a
named fixture from that file, converts the fixture into the initial map frame,
and computes label/center/box/point-cloud metrics without copying world
coordinates into the verifier.
"""

from __future__ import annotations

import json
import math
import re
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

_FLOAT = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


@dataclass(frozen=True)
class ObjectGroundTruth:
    """One simulator fixture expressed in Scene's initial map frame."""

    label: str
    center_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    yaw_rad: float
    association_radius_m: float


def _node_blocks(world_text: str, node_type: str) -> Iterable[str]:
    """Yield balanced ``NodeType { ... }`` blocks from a Webots world."""

    pattern = re.compile(rf"(?m)^\s*{re.escape(node_type)}\s*\{{")
    for match in pattern.finditer(world_text):
        brace = world_text.find("{", match.start())
        depth = 0
        quoted = False
        escaped = False
        for index in range(brace, len(world_text)):
            char = world_text[index]
            if quoted:
                if escaped:
                    escaped = False
                elif char == "\\":
                    escaped = True
                elif char == '"':
                    quoted = False
                continue
            if char == '"':
                quoted = True
            elif char == "{":
                depth += 1
            elif char == "}":
                depth -= 1
                if depth == 0:
                    yield world_text[match.start() : index + 1]
                    break


def _named_node(world_text: str, node_type: str, name: str) -> str:
    name_pattern = re.compile(rf'(?m)^\s*name\s+"{re.escape(name)}"\s*$')
    matches = [
        block
        for block in _node_blocks(world_text, node_type)
        if name_pattern.search(block)
    ]
    if len(matches) != 1:
        raise ValueError(
            f"expected one {node_type} named {name!r}, found {len(matches)}"
        )
    return matches[0]


def _vector_field(block: str, field: str, length: int) -> tuple[float, ...]:
    values = r"\s+".join(f"({_FLOAT})" for _ in range(length))
    match = re.search(rf"(?m)^\s*{re.escape(field)}\s+{values}\s*$", block)
    if match is None:
        raise ValueError(f"node is missing {field}")
    parsed = tuple(float(value) for value in match.groups())
    if not all(math.isfinite(value) for value in parsed):
        raise ValueError(f"{field} contains a non-finite value")
    return parsed


def _planar_yaw(rotation: Sequence[float]) -> float:
    axis_x, axis_y, axis_z, angle = rotation
    norm = math.sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z)
    if norm <= 1e-12:
        return 0.0
    axis_x /= norm
    axis_y /= norm
    axis_z /= norm
    if math.hypot(axis_x, axis_y) > 0.05 or abs(axis_z) < 0.95:
        raise ValueError("quality fixture rotation is not planar")
    return axis_z * angle


def _normalise_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def load_ground_truth(
    config_path: Path,
    *,
    repository_root: Path,
) -> tuple[ObjectGroundTruth, dict[str, Any]]:
    """Load a fixture description and resolve its live coordinates from WBT."""

    config = json.loads(config_path.read_text(encoding="utf-8"))
    world_path = repository_root / str(config["world"])
    world_text = world_path.read_text(encoding="utf-8")
    robot_spec = config["robot"]
    target_spec = config["target"]
    robot = _named_node(
        world_text,
        str(robot_spec["node_type"]),
        str(robot_spec["name"]),
    )
    target = _named_node(
        world_text,
        str(target_spec["node_type"]),
        str(target_spec["name"]),
    )

    robot_xyz = _vector_field(robot, "translation", 3)
    target_xyz = _vector_field(target, "translation", 3)
    robot_yaw = _planar_yaw(_vector_field(robot, "rotation", 4))
    target_yaw = _planar_yaw(_vector_field(target, "rotation", 4))
    size_m = tuple(float(value) for value in target_spec["bbox_size_m"])
    if len(size_m) != 3 or any(
        not math.isfinite(value) or value <= 0.0 for value in size_m
    ):
        raise ValueError("target.bbox_size_m must contain three positive SI values")

    # Webots x/y are the ground plane and z is vertical.  The mapping stack
    # anchors map XY to the robot's initial pose, so rotate the world-space
    # displacement by the inverse initial robot yaw.
    dx = target_xyz[0] - robot_xyz[0]
    dy = target_xyz[1] - robot_xyz[1]
    cosine = math.cos(-robot_yaw)
    sine = math.sin(-robot_yaw)
    center_x = cosine * dx - sine * dy
    center_y = sine * dx + cosine * dy
    ground_z = float(config.get("map_ground_z_world_m", 0.0))
    center_z = target_xyz[2] - ground_z + size_m[2] * 0.5
    truth = ObjectGroundTruth(
        label=str(target_spec["label"]).strip().lower(),
        center_m=(center_x, center_y, center_z),
        size_m=(size_m[0], size_m[1], size_m[2]),
        yaw_rad=_normalise_angle(target_yaw - robot_yaw),
        association_radius_m=float(target_spec["association_radius_m"]),
    )
    return truth, config


def nearest_object(
    objects: Sequence[dict[str, Any]],
    truth: ObjectGroundTruth,
    *,
    center_key: str = "pose",
) -> dict[str, Any] | None:
    """Associate by metric position so an incorrect label remains measurable."""

    candidates: list[tuple[float, dict[str, Any]]] = []
    for obj in objects:
        if bool(obj.get("missing")):
            continue
        if str(obj.get("cls") or "").strip().lower() == "robot":
            continue
        center = obj.get(center_key) or {}
        if isinstance(center, (list, tuple)):
            if len(center) < 2:
                continue
            x, y = float(center[0]), float(center[1])
        else:
            try:
                x, y = float(center["x"]), float(center["y"])
            except (KeyError, TypeError, ValueError):
                continue
        distance = math.hypot(x - truth.center_m[0], y - truth.center_m[1])
        candidates.append((distance, obj))
    if not candidates:
        return None
    distance, obj = min(candidates, key=lambda item: item[0])
    return obj if distance <= truth.association_radius_m else None


def _rectangle(
    center_x: float,
    center_y: float,
    size_x: float,
    size_y: float,
    yaw: float,
) -> list[tuple[float, float]]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    half_x, half_y = size_x * 0.5, size_y * 0.5
    return [
        (
            center_x + cosine * x - sine * y,
            center_y + sine * x + cosine * y,
        )
        for x, y in (
            (-half_x, -half_y),
            (half_x, -half_y),
            (half_x, half_y),
            (-half_x, half_y),
        )
    ]


def _polygon_area(polygon: Sequence[tuple[float, float]]) -> float:
    if len(polygon) < 3:
        return 0.0
    return (
        abs(
            sum(
                x0 * y1 - x1 * y0
                for (x0, y0), (x1, y1) in zip(
                    polygon,
                    [*polygon[1:], polygon[0]],
                )
            )
        )
        * 0.5
    )


def _clip_polygon(
    subject: Sequence[tuple[float, float]],
    clip: Sequence[tuple[float, float]],
) -> list[tuple[float, float]]:
    """Sutherland-Hodgman clipping for two counter-clockwise rectangles."""

    output = list(subject)
    for edge_start, edge_end in zip(clip, [*clip[1:], clip[0]]):
        input_points = output
        output = []
        if not input_points:
            break

        def inside(point: tuple[float, float]) -> bool:
            return (edge_end[0] - edge_start[0]) * (point[1] - edge_start[1]) - (
                edge_end[1] - edge_start[1]
            ) * (point[0] - edge_start[0]) >= -1e-12

        def intersection(
            first: tuple[float, float],
            second: tuple[float, float],
        ) -> tuple[float, float]:
            dx1, dy1 = second[0] - first[0], second[1] - first[1]
            dx2, dy2 = edge_end[0] - edge_start[0], edge_end[1] - edge_start[1]
            denominator = dx1 * dy2 - dy1 * dx2
            if abs(denominator) <= 1e-12:
                return second
            t = (
                (edge_start[0] - first[0]) * dy2 - (edge_start[1] - first[1]) * dx2
            ) / denominator
            return first[0] + t * dx1, first[1] + t * dy1

        previous = input_points[-1]
        previous_inside = inside(previous)
        for current in input_points:
            current_inside = inside(current)
            if current_inside:
                if not previous_inside:
                    output.append(intersection(previous, current))
                output.append(current)
            elif previous_inside:
                output.append(intersection(previous, current))
            previous = current
            previous_inside = current_inside
    return output


def bbox_iou_3d(obj: dict[str, Any], truth: ObjectGroundTruth) -> float:
    """Return yaw-oriented 3D IoU between a Scene box and simulator truth."""

    pose = obj.get("pose") or {}
    bbox = obj.get("bbox") or {}
    try:
        center = (float(pose["x"]), float(pose["y"]), float(pose["z"]))
        size = (
            float(bbox["size_x"]),
            float(bbox["size_y"]),
            float(bbox["size_z"]),
        )
        yaw = float(bbox.get("yaw") or pose.get("yaw") or 0.0)
    except (KeyError, TypeError, ValueError):
        return 0.0
    if any(not math.isfinite(value) or value <= 0.0 for value in size):
        return 0.0
    predicted_xy = _rectangle(center[0], center[1], size[0], size[1], yaw)
    truth_xy = _rectangle(
        truth.center_m[0],
        truth.center_m[1],
        truth.size_m[0],
        truth.size_m[1],
        truth.yaw_rad,
    )
    area = _polygon_area(_clip_polygon(predicted_xy, truth_xy))
    predicted_low = center[2] - size[2] * 0.5
    predicted_high = center[2] + size[2] * 0.5
    truth_low = truth.center_m[2] - truth.size_m[2] * 0.5
    truth_high = truth.center_m[2] + truth.size_m[2] * 0.5
    overlap_z = max(
        0.0, min(predicted_high, truth_high) - max(predicted_low, truth_low)
    )
    intersection = area * overlap_z
    predicted_volume = size[0] * size[1] * size[2]
    truth_volume = truth.size_m[0] * truth.size_m[1] * truth.size_m[2]
    union = predicted_volume + truth_volume - intersection
    return intersection / union if union > 0.0 else 0.0


def point_inlier_fraction(
    points: Sequence[Sequence[float]],
    truth: ObjectGroundTruth,
    *,
    margin_m: float,
) -> float | None:
    """Fraction of exported points inside the fixture box plus SI margin."""

    valid = [
        (float(point[0]), float(point[1]), float(point[2]))
        for point in points
        if len(point) >= 3 and all(math.isfinite(float(value)) for value in point[:3])
    ]
    if not valid:
        return None
    cosine = math.cos(-truth.yaw_rad)
    sine = math.sin(-truth.yaw_rad)
    half = tuple(value * 0.5 + margin_m for value in truth.size_m)
    inliers = 0
    for x, y, z in valid:
        dx, dy, dz = (
            x - truth.center_m[0],
            y - truth.center_m[1],
            z - truth.center_m[2],
        )
        local_x = cosine * dx - sine * dy
        local_y = sine * dx + cosine * dy
        if abs(local_x) <= half[0] and abs(local_y) <= half[1] and abs(dz) <= half[2]:
            inliers += 1
    return inliers / len(valid)


def _median(values: Sequence[float]) -> float | None:
    return float(statistics.median(values)) if values else None


def _maximum_pairwise_xy(samples: Sequence[dict[str, Any]]) -> float | None:
    centers = [
        (float(sample["pose"]["x"]), float(sample["pose"]["y"])) for sample in samples
    ]
    if not centers:
        return None
    return max(
        math.hypot(first[0] - second[0], first[1] - second[1])
        for first in centers
        for second in centers
    )


def evaluate_ground_truth(
    samples: Sequence[dict[str, Any]],
    point_samples: Sequence[dict[str, Any]],
    truth: ObjectGroundTruth,
    *,
    expected_samples: int,
    thresholds: dict[str, Any],
) -> dict[str, Any]:
    """Summarise and gate observations without hiding missing measurements."""

    xy_errors = [
        math.hypot(
            float(sample["pose"]["x"]) - truth.center_m[0],
            float(sample["pose"]["y"]) - truth.center_m[1],
        )
        for sample in samples
    ]
    z_errors = [
        abs(float(sample["pose"]["z"]) - truth.center_m[2]) for sample in samples
    ]
    ious = [bbox_iou_3d(sample, truth) for sample in samples]
    labels = [str(sample.get("cls") or "").strip().lower() for sample in samples]
    ids = {str(sample.get("id") or "") for sample in samples if sample.get("id")}
    navigation_grade_fraction = (
        sum(bool(sample.get("navigation_grade")) for sample in samples) / len(samples)
        if samples
        else 0.0
    )
    inlier_values = [
        value
        for sample in point_samples
        if (
            value := point_inlier_fraction(
                sample.get("points") or (),
                truth,
                margin_m=float(thresholds["point_margin_m"]),
            )
        )
        is not None
    ]
    point_counts = [
        int(sample.get("n_points") or len(sample.get("points") or ()))
        for sample in point_samples
    ]
    target_recall = len(samples) / max(1, expected_samples)
    label_accuracy = (
        sum(label == truth.label for label in labels) / len(labels) if labels else 0.0
    )
    metrics = {
        "sample_count": len(samples),
        "expected_samples": expected_samples,
        "target_recall": target_recall,
        "label_accuracy": label_accuracy,
        "stable_id_count": len(ids),
        "median_center_xy_error_m": _median(xy_errors),
        "max_center_xy_drift_m": _maximum_pairwise_xy(samples),
        "median_center_z_error_m": _median(z_errors),
        "median_bbox_iou_3d": _median(ious),
        "median_point_inlier_fraction": _median(inlier_values),
        "median_point_count": _median([float(value) for value in point_counts]),
        "navigation_grade_fraction": navigation_grade_fraction,
    }
    failures: list[str] = []

    def require_at_least(metric: str, threshold: str) -> None:
        value = metrics[metric]
        if value is None or float(value) < float(thresholds[threshold]):
            failures.append(f"{metric} below {thresholds[threshold]}")

    def require_at_most(metric: str, threshold: str) -> None:
        value = metrics[metric]
        if value is None or float(value) > float(thresholds[threshold]):
            failures.append(f"{metric} above {thresholds[threshold]}")

    require_at_least("target_recall", "min_target_recall")
    require_at_least("label_accuracy", "min_label_accuracy")
    if not ids or len(ids) > int(thresholds["max_stable_ids"]):
        failures.append(f"stable_id_count above {thresholds['max_stable_ids']}")
    require_at_most(
        "median_center_xy_error_m",
        "max_median_center_xy_error_m",
    )
    require_at_most("max_center_xy_drift_m", "max_center_xy_drift_m")
    require_at_most(
        "median_center_z_error_m",
        "max_median_center_z_error_m",
    )
    require_at_least("median_bbox_iou_3d", "min_median_bbox_iou_3d")
    require_at_least(
        "median_point_inlier_fraction",
        "min_median_point_inlier_fraction",
    )
    require_at_least("median_point_count", "min_median_point_count")
    require_at_least(
        "navigation_grade_fraction",
        "min_navigation_grade_fraction",
    )
    return {
        "ok": not failures,
        "failures": failures,
        "ground_truth": {
            "label": truth.label,
            "center_m": list(truth.center_m),
            "bbox_size_m": list(truth.size_m),
            "yaw_rad": truth.yaw_rad,
        },
        **metrics,
    }
