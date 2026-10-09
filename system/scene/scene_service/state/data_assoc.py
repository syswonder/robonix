# SPDX-License-Identifier: MulanPSL-2.0
"""Associate per-frame detections with registry objects: same-class, gated
Hungarian matching; matches EMA-update the object, the rest become new objects.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.optimize import linear_sum_assignment

from .object_registry import (
    BBox3D,
    ObjectRegistry,
    Pose3D,
    SceneObject,
    now_unix,
)

log = logging.getLogger(__name__)


# Per-class gate across the floor, metres.
_GATE_RADIUS_M: dict[str, float] = {
    "cup": 0.30,
    "bottle": 0.30,
    "tool": 0.30,
    "tray": 0.50,
    "table": 1.00,
    "chair": 0.80,
    "door": 1.50,
    "person": 1.50,
    "robot": 1.00,
}
_DEFAULT_GATE_RADIUS_M = 0.50

# Height is gated separately and loosely: depth error mostly moves z, but a
# tabletop object must still not match one on the floor beneath it.
_GATE_Z_M: dict[str, float] = {
    "person": 1.50,
    "door": 2.00,
}
_DEFAULT_GATE_Z_M = 1.20

# cost = distance + alpha * (1 - confidence): unsure detections prefer a new object.
_COST_ALPHA = 0.5


@dataclass
class Detection:
    """One per-frame perception output; `pose` and `bbox` share one frame."""
    label: str
    pose: Pose3D
    bbox: BBox3D
    confidence: float
    source: str = "perception"


def _gate_radius(label: str) -> float:
    return _GATE_RADIUS_M.get(label, _DEFAULT_GATE_RADIUS_M)


def _gate_z(label: str) -> float:
    return _GATE_Z_M.get(label, _DEFAULT_GATE_Z_M)


def _euclid(p: Pose3D, q: Pose3D) -> float:
    return math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2 + (p.z - q.z) ** 2)


def _floor_dist(p: Pose3D, q: Pose3D) -> float:
    return math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2)


def associate(
    registry: ObjectRegistry,
    detections: list[Detection],
    *,
    now: Optional[float] = None,
) -> tuple[list[str], list[str]]:
    """Match detections to registry objects in place; returns (matched_ids, new_ids).
    Caller holds `registry.lock()`. Unmatched objects are left to mark_stale."""
    if now is None:
        now = now_unix()
    if not detections:
        return [], []
    valid_detections: list[Detection] = []
    for detection in detections:
        pose_frame = str(detection.pose.frame_id or "").strip()
        bbox_frame = str(detection.bbox.frame_id or "").strip()
        if not pose_frame or pose_frame != bbox_frame:
            log.warning(
                "dropping detection %r with unknown or mixed frames "
                "(pose=%s bbox=%s)",
                detection.label,
                pose_frame or "unknown",
                bbox_frame or "unknown",
            )
            continue
        valid_detections.append(detection)
    if not valid_detections:
        return [], []

    # Bucket by class and frame: coordinates from different frames never compare.
    by_key: dict[tuple[str, str], list[SceneObject]] = {}
    for obj in registry.all_objects():
        if obj.attributes.get("is_robot"):
            continue
        pose_frame = str(obj.pose.frame_id or "").strip()
        bbox_frame = str(obj.bbox.frame_id or "").strip()
        if pose_frame and pose_frame == bbox_frame:
            by_key.setdefault((obj.label, pose_frame), []).append(obj)

    matched_ids: list[str] = []
    new_ids: list[str] = []

    def insert(d: Detection) -> None:
        obj = registry.insert_object(label=d.label, pose=d.pose, bbox=d.bbox,
                                     confidence=d.confidence, now=now, source=d.source)
        new_ids.append(obj.object_id)

    by_key_dets: dict[tuple[str, str], list[Detection]] = {}
    for detection in valid_detections:
        key = (detection.label, str(detection.pose.frame_id).strip())
        by_key_dets.setdefault(key, []).append(detection)

    for (label, frame_id), dets in by_key_dets.items():
        objs = by_key.get((label, frame_id), [])
        gate = _gate_radius(label)
        gate_z = _gate_z(label)
        if not objs:
            for d in dets:
                insert(d)
            continue

        # Rows are detections, columns objects; `big` marks pairs outside the gate.
        big = 1e6
        M = len(dets)
        N = len(objs)
        cost = np.full((M, N), big, dtype=np.float64)
        for i, d in enumerate(dets):
            for j, o in enumerate(objs):
                if _floor_dist(d.pose, o.pose) > gate:
                    continue
                if abs(d.pose.z - o.pose.z) > gate_z:
                    continue
                # Within the gate, rank by full 3D distance.
                dist = _euclid(d.pose, o.pose)
                cost[i, j] = dist + _COST_ALPHA * (1.0 - max(0.0, min(1.0, d.confidence)))

        # Pad to square with `big`.
        K = max(M, N)
        if K > M or K > N:
            padded = np.full((K, K), big, dtype=np.float64)
            padded[:M, :N] = cost
            cost = padded

        row_ind, col_ind = linear_sum_assignment(cost)

        matched_rows: set[int] = set()
        for r, c in zip(row_ind, col_ind):
            if r >= M or c >= N or cost[r, c] >= big:
                continue
            matched_rows.add(r)
            d = dets[r]
            o = objs[c]
            # Low pose EMA: per-frame depth jitters by several centimetres.
            registry.update_object_pose(o, d.pose, d.confidence, now, ema_pose=0.10)
            o.bbox = BBox3D(
                size_x=0.7 * o.bbox.size_x + 0.3 * d.bbox.size_x,
                size_y=0.7 * o.bbox.size_y + 0.3 * d.bbox.size_y,
                size_z=0.7 * o.bbox.size_z + 0.3 * d.bbox.size_z,
                yaw=d.bbox.yaw,
                frame_id=d.bbox.frame_id,
            )
            matched_ids.append(o.object_id)

        for i, d in enumerate(dets):
            if i not in matched_rows:
                insert(d)

    return matched_ids, new_ids
