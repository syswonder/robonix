# SPDX-License-Identifier: MulanPSL-2.0
"""Pure geometry helpers for RGB-D-backed Webots truth visibility."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class VisibilityEvidence:
    visible: bool
    reason: str
    projected_area_px: int
    clipped_fraction: float
    consistent_depth_pixels: int
    consistent_depth_fraction: float
    required_depth_pixels: int
    depth_interval_m: tuple[float, float] | None


def _yaw_bbox_corners(
    center_m: Sequence[float],
    size_m: Sequence[float],
    yaw_rad: float,
) -> np.ndarray:
    center = np.asarray(center_m, dtype=np.float64)
    size = np.asarray(size_m, dtype=np.float64)
    if center.shape != (3,) or size.shape != (3,):
        raise ValueError("center_m and size_m must each contain three values")
    if not np.all(np.isfinite(center)) or not np.all(np.isfinite(size)):
        raise ValueError("center_m and size_m must be finite")
    if np.any(size <= 0.0) or not math.isfinite(float(yaw_rad)):
        raise ValueError("size_m must be positive and yaw_rad must be finite")
    half = size * 0.5
    local = np.asarray(
        [
            [sx * half[0], sy * half[1], sz * half[2]]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ],
        dtype=np.float64,
    )
    cosine = math.cos(float(yaw_rad))
    sine = math.sin(float(yaw_rad))
    rotation = np.asarray(
        [
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return local @ rotation.T + center


def rgbd_visibility_evidence(
    *,
    depth_m: np.ndarray,
    center_m: Sequence[float],
    size_m: Sequence[float],
    yaw_rad: float,
    map_to_camera: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    min_range_m: float = 0.15,
    max_range_m: float = 6.0,
    depth_margin_m: float = 0.15,
    min_projected_area_px: int = 64,
    min_clipped_fraction: float = 0.25,
    min_consistent_pixels: int = 8,
    min_consistent_fraction: float = 0.05,
) -> VisibilityEvidence:
    """Test whether an oriented WBT bbox has enough matching RGB-D support.

    The old benchmark checked only the target center and accepted three
    depth-consistent pixels from a heuristic square. In clutter, pixels from a
    foreground object or nearby wall therefore made many fully occluded targets
    "visible". This helper projects all eight WBT bbox corners, clips the true
    image footprint, and requires a measurable fraction of that footprint to
    lie inside the bbox's camera-depth interval.
    """

    depth = np.asarray(depth_m, dtype=np.float32)
    transform = np.asarray(map_to_camera, dtype=np.float64)
    if depth.ndim != 2:
        raise ValueError("depth_m must be a two-dimensional metric depth image")
    if transform.shape != (4, 4):
        raise ValueError("map_to_camera must be a 4x4 homogeneous transform")
    if not all(
        math.isfinite(value)
        for value in (fx, fy, cx, cy, min_range_m, max_range_m)
    ):
        raise ValueError("camera intrinsics and range limits must be finite")
    if fx <= 0.0 or fy <= 0.0 or max_range_m <= min_range_m:
        raise ValueError("camera focal lengths/range limits are invalid")

    corners = _yaw_bbox_corners(center_m, size_m, yaw_rad)
    camera = corners @ transform[:3, :3].T + transform[:3, 3]
    corner_depths = camera[:, 2]
    center = np.asarray(center_m, dtype=np.float64)
    camera_center = transform[:3, :3] @ center + transform[:3, 3]
    center_depth = float(camera_center[2])
    if center_depth <= min_range_m or center_depth > max_range_m:
        return VisibilityEvidence(
            False,
            "center_out_of_range",
            0,
            0.0,
            0,
            0.0,
            0,
            None,
        )
    front = corner_depths > min_range_m
    if np.count_nonzero(front) < 4:
        return VisibilityEvidence(
            False,
            "bbox_behind_camera",
            0,
            0.0,
            0,
            0.0,
            0,
            None,
        )
    projected = camera[front]
    u = fx * projected[:, 0] / projected[:, 2] + cx
    v = fy * projected[:, 1] / projected[:, 2] + cy
    raw_x0 = float(np.min(u))
    raw_x1 = float(np.max(u))
    raw_y0 = float(np.min(v))
    raw_y1 = float(np.max(v))
    raw_width = max(0.0, raw_x1 - raw_x0)
    raw_height = max(0.0, raw_y1 - raw_y0)
    raw_area = raw_width * raw_height
    if raw_area <= 1e-9:
        return VisibilityEvidence(
            False,
            "degenerate_projection",
            0,
            0.0,
            0,
            0.0,
            0,
            None,
        )

    height, width = depth.shape
    x0 = max(0, min(width, int(math.floor(raw_x0))))
    x1 = max(0, min(width, int(math.ceil(raw_x1))))
    y0 = max(0, min(height, int(math.floor(raw_y0))))
    y1 = max(0, min(height, int(math.ceil(raw_y1))))
    projected_area = max(0, x1 - x0) * max(0, y1 - y0)
    clipped_fraction = min(1.0, projected_area / raw_area)
    if projected_area < int(min_projected_area_px):
        return VisibilityEvidence(
            False,
            "projected_area_too_small",
            projected_area,
            clipped_fraction,
            0,
            0.0,
            0,
            None,
        )
    if clipped_fraction < float(min_clipped_fraction):
        return VisibilityEvidence(
            False,
            "bbox_mostly_outside_image",
            projected_area,
            clipped_fraction,
            0,
            0.0,
            0,
            None,
        )

    window = depth[y0:y1, x0:x1]
    valid = np.isfinite(window) & (window >= min_range_m)
    depth_low = max(
        min_range_m,
        float(np.min(corner_depths[front])) - float(depth_margin_m),
    )
    depth_high = min(
        max_range_m,
        float(np.max(corner_depths[front])) + float(depth_margin_m),
    )
    consistent = int(
        np.count_nonzero(
            valid & (window >= depth_low) & (window <= depth_high)
        )
    )
    required = max(
        int(min_consistent_pixels),
        int(math.ceil(projected_area * float(min_consistent_fraction))),
    )
    fraction = consistent / projected_area
    visible = consistent >= required
    return VisibilityEvidence(
        visible,
        "visible" if visible else "insufficient_depth_support",
        projected_area,
        clipped_fraction,
        consistent,
        fraction,
        required,
        (depth_low, depth_high),
    )
