# SPDX-License-Identifier: MulanPSL-2.0
"""Associate per-frame detections with persistent scene objects.

The version 1 algorithm applies these steps:

1. Gate candidates spatially using a class-specific radius.
2. Keep candidates with the same class.
3. Run Hungarian minimum-cost matching.
4. Allocate objects for unmatched detections.
5. Leave unmatched-object expiry to ``mark_stale``.
6. Update matched object poses with an exponential moving average.

Not implemented in v1 (deferred): Kalman filtering, IoU-based gating,
appearance embeddings, learned association.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np
from scipy.optimize import linear_sum_assignment

from .object_registry import (
    BBox3D,
    ObjectRegistry,
    Pose3D,
    SceneObject,
    now_unix,
)


# Per-class gating radius in metres. Below this distance, an existing
# object is considered as a candidate for the incoming detection. Tune
# per-class because a "table" reasonably moves 1m frame-to-frame
# (camera shifted) while a "cup" should not.
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

# Cost = ||D.pose - O.pose|| + alpha * (1 - D.confidence)
# Higher alpha penalises low-confidence matches harder, biasing toward
# "let it become a new object" when the detector is unsure.
_COST_ALPHA = 0.5


@dataclass
class Detection:
    """One per-frame perception output. Stable id is NOT supplied — it's
    this layer's job to assign / find one. `pose` is in `map` frame
    (ingest does the TF transform before producing Detection)."""
    cls: str
    pose: Pose3D
    bbox: BBox3D
    confidence: float
    source: str = "perception"


def _gate_radius(cls: str) -> float:
    return _GATE_RADIUS_M.get(cls, _DEFAULT_GATE_RADIUS_M)


def _euclid(p: Pose3D, q: Pose3D) -> float:
    return math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2 + (p.z - q.z) ** 2)


def associate(
    registry: ObjectRegistry,
    detections: list[Detection],
    *,
    now: Optional[float] = None,
) -> tuple[list[str], list[str]]:
    """Resolve detections against the registry. Caller must hold
    `registry.lock()`.

    Returns `(matched_ids, new_ids)` for logging / metrics. The registry
    is mutated in place: matched detections EMA-update existing records,
    unmatched detections allocate new ones, unmatched objects are NOT
    touched (mark_stale runs separately on a periodic tick).
    """
    if now is None:
        now = now_unix()
    if not detections:
        return [], []

    # Bucket existing objects by class — class-match gate is a hard
    # filter; no point including, say, all "table" objects in the
    # cost matrix when we're matching cups.
    by_cls: dict[str, list[SceneObject]] = {}
    for obj in registry.all_objects():
        # Robot self-object never participates in detection matching.
        if obj.attributes.get("is_robot"):
            continue
        by_cls.setdefault(obj.cls, []).append(obj)

    matched_ids: list[str] = []
    new_ids: list[str] = []

    # Process per-class so the cost matrix stays small.
    by_cls_dets: dict[str, list[Detection]] = {}
    for d in detections:
        by_cls_dets.setdefault(d.cls, []).append(d)

    for cls, dets in by_cls_dets.items():
        objs = by_cls.get(cls, [])
        gate = _gate_radius(cls)
        if not objs:
            for d in dets:
                obj = registry.insert_object(
                    cls=d.cls,
                    pose=d.pose,
                    bbox=d.bbox,
                    confidence=d.confidence,
                    now=now,
                    source=d.source,
                )
                new_ids.append(obj.object_id)
            continue

        # Build cost matrix (rows = detections, cols = candidate objects).
        # Use a large sentinel for pairs outside the gate so the
        # Hungarian solver naturally avoids them; they'll only get
        # picked when there's no alternative, and we'll filter those
        # post-hoc below.
        big = 1e6
        M = len(dets)
        N = len(objs)
        cost = np.full((M, N), big, dtype=np.float64)
        for i, d in enumerate(dets):
            for j, o in enumerate(objs):
                dist = _euclid(d.pose, o.pose)
                if dist > gate:
                    continue
                cost[i, j] = dist + _COST_ALPHA * (1.0 - max(0.0, min(1.0, d.confidence)))

        # Pad to square so linear_sum_assignment can solve. Pad with
        # `big` so padded slots never get chosen unless forced.
        K = max(M, N)
        if K > M or K > N:
            padded = np.full((K, K), big, dtype=np.float64)
            padded[:M, :N] = cost
            cost = padded

        row_ind, col_ind = linear_sum_assignment(cost)

        for r, c in zip(row_ind, col_ind):
            if r >= M or c >= N:
                continue  # padding pair, ignore
            if cost[r, c] >= big:
                # Pair outside the gate — treat as unmatched (the detection
                # becomes a new object below).
                continue
            d = dets[r]
            o = objs[c]
            # ema_pose=0.10 (was 0.30): VLM-derived poses jitter ~5–10 cm
            # per tick because the depth source isn't precise. With 0.10
            # the existing pose dominates and we still pick up genuine
            # motion within ~10 ticks. Confidence keeps the higher EMA
            # because per-frame confidence is more directly meaningful.
            registry.update_object_pose(o, d.pose, d.confidence, now, ema_pose=0.10)
            # Confidence-weighted bbox blend: new bbox shrinks toward the
            # detection's by 30% per frame. Cheap, stays stable.
            o.bbox = BBox3D(
                size_x=0.7 * o.bbox.size_x + 0.3 * d.bbox.size_x,
                size_y=0.7 * o.bbox.size_y + 0.3 * d.bbox.size_y,
                size_z=0.7 * o.bbox.size_z + 0.3 * d.bbox.size_z,
                yaw=d.bbox.yaw,
                frame_id=d.bbox.frame_id,
            )
            matched_ids.append(o.object_id)

        # Detections that didn't get a match (or were gated out) → new objects.
        matched_rows = {
            r for r, c in zip(row_ind, col_ind)
            if r < M and c < N and cost[r, c] < big
        }
        for i, d in enumerate(dets):
            if i in matched_rows:
                continue
            obj = registry.insert_object(
                cls=d.cls,
                pose=d.pose,
                bbox=d.bbox,
                confidence=d.confidence,
                now=now,
                source=d.source,
            )
            new_ids.append(obj.object_id)

    return matched_ids, new_ids
