#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Compare a saved Webots occupancy grid with structural WBT geometry.

This is an evaluation tool, not runtime mapping code.  It derives wall/window
poses from the checked-in world, expresses them in the robot's initial local
map frame, and reports metric wall support and line residuals.
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy.spatial import cKDTree

from scene_quality_ground_truth import (
    _named_node,
    _node_blocks,
    _optional_vector_field,
    _planar_yaw,
    _vector_field,
)


@dataclass(frozen=True)
class Segment:
    name: str
    start: tuple[float, float]
    end: tuple[float, float]
    thickness_m: float


def _world_to_initial_map(
    point: tuple[float, float],
    *,
    robot_xy: tuple[float, float],
    robot_yaw: float,
) -> tuple[float, float]:
    dx = point[0] - robot_xy[0]
    dy = point[1] - robot_xy[1]
    cosine = math.cos(-robot_yaw)
    sine = math.sin(-robot_yaw)
    return (
        cosine * dx - sine * dy,
        sine * dx + cosine * dy,
    )


def load_structural_segments(
    world_path: Path,
    *,
    robot_type: str,
    robot_name: str,
) -> list[Segment]:
    text = world_path.read_text(encoding="utf-8")
    robot = _named_node(text, robot_type, robot_name)
    robot_xyz = _vector_field(robot, "translation", 3)
    rotation = _optional_vector_field(robot, "rotation", 4)
    robot_yaw = _planar_yaw(rotation or (0.0, 0.0, 1.0, 0.0))
    segments: list[Segment] = []

    for node_type in ("Wall", "Window"):
        for occurrence, block in enumerate(_node_blocks(text, node_type), 1):
            translation = _optional_vector_field(block, "translation", 3)
            size = _optional_vector_field(block, "size", 3)
            if translation is None or size is None:
                continue
            yaw = _planar_yaw(
                _optional_vector_field(block, "rotation", 4)
                or (0.0, 0.0, 1.0, 0.0)
            )
            length_axis = 0 if size[0] >= size[1] else 1
            length = size[length_axis]
            thickness = size[1 - length_axis]
            axis_yaw = yaw + (math.pi * 0.5 if length_axis == 1 else 0.0)
            half_dx = math.cos(axis_yaw) * length * 0.5
            half_dy = math.sin(axis_yaw) * length * 0.5
            start_world = (
                translation[0] - half_dx,
                translation[1] - half_dy,
            )
            end_world = (
                translation[0] + half_dx,
                translation[1] + half_dy,
            )
            name_match = _optional_name(block)
            name = name_match or f"{node_type}#{occurrence}"
            segments.append(
                Segment(
                    name=name,
                    start=_world_to_initial_map(
                        start_world,
                        robot_xy=(robot_xyz[0], robot_xyz[1]),
                        robot_yaw=robot_yaw,
                    ),
                    end=_world_to_initial_map(
                        end_world,
                        robot_xy=(robot_xyz[0], robot_xyz[1]),
                        robot_yaw=robot_yaw,
                    ),
                    thickness_m=float(thickness),
                )
            )
    return segments


def _optional_name(block: str) -> str:
    import re

    match = re.search(r'(?m)^\s*name\s+"([^"]+)"\s*$', block)
    return match.group(1) if match else ""


def _sample_segment(segment: Segment, spacing_m: float) -> np.ndarray:
    start = np.asarray(segment.start, dtype=float)
    end = np.asarray(segment.end, dtype=float)
    length = float(np.linalg.norm(end - start))
    count = max(2, int(math.ceil(length / spacing_m)) + 1)
    return start[None, :] + np.linspace(0.0, 1.0, count)[:, None] * (
        end - start
    )[None, :]


def _load_grid(
    image_path: Path,
    yaml_path: Path,
) -> tuple[np.ndarray, np.ndarray, dict]:
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image = np.asarray(Image.open(image_path).convert("L"))
    return image, _occupied_world_points(image, metadata), metadata


def _occupied_world_points(image: np.ndarray, metadata: dict) -> np.ndarray:
    """Project occupied image cells into the grid's declared world frame."""
    occupied_rc = np.argwhere(image < 65)
    resolution = float(metadata["resolution"])
    origin_x, origin_y, origin_yaw = (
        float(value) for value in metadata["origin"]
    )
    height = image.shape[0]
    local_x = (occupied_rc[:, 1] + 0.5) * resolution
    local_y = (height - occupied_rc[:, 0] - 0.5) * resolution
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    return np.column_stack(
        (
            origin_x + cosine * local_x - sine * local_y,
            origin_y + sine * local_x + cosine * local_y,
        )
    )


def _load_scene_state_grid(
    state_path: Path,
) -> tuple[np.ndarray, np.ndarray, dict]:
    """Decode the bounded occupancy snapshot exported by Scene."""
    payload = json.loads(state_path.read_text(encoding="utf-8"))
    occupancy = payload.get("occupancy") or {}
    encoded = occupancy.get("png_b64")
    if not encoded:
        raise ValueError(f"{state_path} has no occupancy.png_b64")
    image = np.asarray(
        Image.open(io.BytesIO(base64.b64decode(encoded))).convert("L")
    )
    expected_size = (
        int(occupancy.get("width") or 0),
        int(occupancy.get("height") or 0),
    )
    if image.shape[::-1] != expected_size:
        raise ValueError(
            f"occupancy PNG is {image.shape[1]}x{image.shape[0]}, "
            f"metadata says {expected_size[0]}x{expected_size[1]}"
        )
    metadata = {
        "resolution": float(occupancy["resolution"]),
        "origin": [
            float(occupancy.get("origin_x") or 0.0),
            float(occupancy.get("origin_y") or 0.0),
            float(occupancy.get("origin_yaw") or 0.0),
        ],
    }
    return image, _occupied_world_points(image, metadata), metadata


def evaluate(
    image_path: Path,
    yaml_path: Path,
    segments: Iterable[Segment],
) -> dict:
    image, occupied_xy, metadata = _load_grid(image_path, yaml_path)
    return _evaluate_loaded_grid(image, occupied_xy, metadata, segments)


def evaluate_scene_state(
    state_path: Path,
    segments: Iterable[Segment],
) -> dict:
    image, occupied_xy, metadata = _load_scene_state_grid(state_path)
    return _evaluate_loaded_grid(image, occupied_xy, metadata, segments)


def _evaluate_loaded_grid(
    image: np.ndarray,
    occupied_xy: np.ndarray,
    metadata: dict,
    segments: Iterable[Segment],
) -> dict:
    segments = list(segments)
    if not len(occupied_xy):
        raise ValueError("occupancy grid has no occupied cells")
    segment_samples = [
        _sample_segment(segment, 0.025) for segment in segments
    ]
    samples = np.concatenate(segment_samples, axis=0)
    sample_half_thickness = np.concatenate(
        [
            np.full(len(points), segment.thickness_m * 0.5, dtype=float)
            for segment, points in zip(segments, segment_samples)
        ]
    )
    sample_segment_index = np.concatenate(
        [
            np.full(len(points), index, dtype=int)
            for index, points in enumerate(segment_samples)
        ]
    )
    occupied_tree = cKDTree(occupied_xy)
    sample_centerline_distance, _ = occupied_tree.query(samples, k=1)
    # LiDAR returns lie on a wall face, while WBT Wall/Window translation
    # describes its centre plane. Score distance to the physical envelope,
    # otherwise a perfectly mapped 20 cm wall receives a systematic 10 cm
    # error.
    sample_distance = np.maximum(
        0.0,
        sample_centerline_distance - sample_half_thickness,
    )

    sample_tree = cKDTree(samples)
    occupied_centerline_distance, occupied_sample_index = sample_tree.query(
        occupied_xy,
        k=1,
    )
    occupied_distance = np.maximum(
        0.0,
        occupied_centerline_distance
        - sample_half_thickness[occupied_sample_index],
    )
    occupied_segment_index = sample_segment_index[occupied_sample_index]
    # Geometry accuracy and exploration coverage are different questions.
    # Restrict the residual-only metric to wall samples that have some nearby
    # occupancy support; otherwise a long, unexplored tail dominates P95 even
    # when every actually observed wall cell is sharp and correctly placed.
    supported_sample_distance = sample_distance[sample_distance <= 0.25]

    segment_metrics = []
    segment_distance_arrays: list[np.ndarray] = []
    for segment_index, (segment, points) in enumerate(
        zip(segments, segment_samples)
    ):
        centerline_distances, _ = occupied_tree.query(points, k=1)
        distances = np.maximum(
            0.0,
            centerline_distances - segment.thickness_m * 0.5,
        )
        segment_distance_arrays.append(distances)
        # Fit orientation only to occupied cells whose nearest structural
        # segment is this wall and whose physical-envelope residual is within
        # 15 cm. Reusing nearest cells from every truth sample pulls adjacent
        # corners or a distant unobserved wall into the PCA and can report a
        # large angle error for an actually straight observed wall.
        assigned = occupied_xy[
            (occupied_segment_index == segment_index)
            & (occupied_distance <= 0.15)
        ]
        expected = np.asarray(segment.end) - np.asarray(segment.start)
        expected_angle = math.atan2(expected[1], expected[0])
        angle_error_deg = None
        if len(assigned) >= 3:
            centered = assigned - assigned.mean(axis=0)
            _, _, right = np.linalg.svd(centered, full_matrices=False)
            observed_angle = math.atan2(right[0, 1], right[0, 0])
            delta = abs(
                math.atan2(
                    math.sin(observed_angle - expected_angle),
                    math.cos(observed_angle - expected_angle),
                )
            )
            angle_error_deg = math.degrees(min(delta, abs(math.pi - delta)))
        segment_metrics.append(
            {
                "name": segment.name,
                "length_m": float(np.linalg.norm(expected)),
                "median_error_m": float(np.median(distances)),
                "p95_error_m": float(np.percentile(distances, 95)),
                "coverage_0_10": float(np.mean(distances <= 0.10)),
                "coverage_0_15": float(np.mean(distances <= 0.15)),
                "angle_error_deg": angle_error_deg,
                "angle_support_cells": int(len(assigned)),
            }
        )

    observed_indices = [
        index
        for index, metric in enumerate(segment_metrics)
        if metric["coverage_0_15"] >= 0.5
    ]
    observed_distances = (
        np.concatenate(
            [segment_distance_arrays[index] for index in observed_indices],
            axis=0,
        )
        if observed_indices
        else np.asarray([], dtype=float)
    )
    finite_angles = [
        segment_metrics[index]["angle_error_deg"]
        for index in observed_indices
        if segment_metrics[index]["angle_error_deg"] is not None
    ]
    return {
        "world_wall_segments": len(segments),
        "occupied_cells": int(len(occupied_xy)),
        "wall_sample_count": int(len(samples)),
        "wall_median_error_m": float(np.median(sample_distance)),
        "wall_p95_error_m": float(np.percentile(sample_distance, 95)),
        "wall_coverage_0_10": float(np.mean(sample_distance <= 0.10)),
        "wall_coverage_0_15": float(np.mean(sample_distance <= 0.15)),
        "wall_coverage_0_25": float(np.mean(sample_distance <= 0.25)),
        "supported_wall_sample_count": int(len(supported_sample_distance)),
        "supported_wall_sample_share": float(
            len(supported_sample_distance) / len(sample_distance)
        ),
        "supported_wall_median_error_m": (
            float(np.median(supported_sample_distance))
            if len(supported_sample_distance)
            else None
        ),
        "supported_wall_p95_error_m": (
            float(np.percentile(supported_sample_distance, 95))
            if len(supported_sample_distance)
            else None
        ),
        "observed_wall_segments": len(observed_indices),
        "unobserved_wall_segments": len(segments) - len(observed_indices),
        "observed_wall_median_error_m": (
            float(np.median(observed_distances))
            if len(observed_distances)
            else None
        ),
        "observed_wall_p95_error_m": (
            float(np.percentile(observed_distances, 95))
            if len(observed_distances)
            else None
        ),
        "observed_wall_coverage_0_10": (
            float(np.mean(observed_distances <= 0.10))
            if len(observed_distances)
            else None
        ),
        "observed_wall_coverage_0_15": (
            float(np.mean(observed_distances <= 0.15))
            if len(observed_distances)
            else None
        ),
        "occupied_share_near_wall_0_15": float(
            np.mean(occupied_distance <= 0.15)
        ),
        "occupied_share_near_wall_0_25": float(
            np.mean(occupied_distance <= 0.25)
        ),
        "wall_angle_median_error_deg": (
            float(np.median(finite_angles)) if finite_angles else None
        ),
        "wall_angle_p95_error_deg": (
            float(np.percentile(finite_angles, 95)) if finite_angles else None
        ),
        "grid": {
            "width": int(image.shape[1]),
            "height": int(image.shape[0]),
            "resolution": float(metadata["resolution"]),
            "origin": [float(value) for value in metadata["origin"]],
        },
        "segments": segment_metrics,
    }


def render_overlay(
    image_path: Path,
    yaml_path: Path,
    segments: Iterable[Segment],
    output_path: Path,
) -> None:
    image, _, metadata = _load_grid(image_path, yaml_path)
    _render_overlay_loaded(image, metadata, segments, output_path)


def render_scene_state_overlay(
    state_path: Path,
    segments: Iterable[Segment],
    output_path: Path,
) -> None:
    image, _, metadata = _load_scene_state_grid(state_path)
    _render_overlay_loaded(image, metadata, segments, output_path)


def _render_overlay_loaded(
    image: np.ndarray,
    metadata: dict,
    segments: Iterable[Segment],
    output_path: Path,
) -> None:
    canvas = Image.fromarray(image).convert("RGB").resize(
        (image.shape[1] * 3, image.shape[0] * 3),
        Image.Resampling.NEAREST,
    )
    draw = ImageDraw.Draw(canvas)
    resolution = float(metadata["resolution"])
    origin_x, origin_y, _ = (float(value) for value in metadata["origin"])
    height = image.shape[0]

    def pixel(point: tuple[float, float]) -> tuple[float, float]:
        column = (point[0] - origin_x) / resolution
        row = height - (point[1] - origin_y) / resolution
        return (column * 3.0, row * 3.0)

    for segment in segments:
        draw.line(
            [pixel(segment.start), pixel(segment.end)],
            fill=(0, 220, 70),
            width=2,
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", type=Path, required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--map-image", type=Path)
    source.add_argument("--scene-state", type=Path)
    parser.add_argument("--map-yaml", type=Path)
    parser.add_argument("--robot-type", default="TiagoLite")
    parser.add_argument("--robot-name", default="my_robot")
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--overlay-out", type=Path)
    args = parser.parse_args()
    segments = load_structural_segments(
        args.world,
        robot_type=args.robot_type,
        robot_name=args.robot_name,
    )
    if args.scene_state:
        result = evaluate_scene_state(args.scene_state, segments)
    else:
        if args.map_yaml is None:
            parser.error("--map-yaml is required with --map-image")
        result = evaluate(args.map_image, args.map_yaml, segments)
    if args.overlay_out:
        if args.scene_state:
            render_scene_state_overlay(
                args.scene_state,
                segments,
                args.overlay_out,
            )
        else:
            render_overlay(
                args.map_image,
                args.map_yaml,
                segments,
                args.overlay_out,
            )
    payload = json.dumps(result, indent=2, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
