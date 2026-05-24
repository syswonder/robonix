# SPDX-License-Identifier: MulanPSL-2.0
"""Cross-frame data association: turn a batch of per-frame `Detection`s
into either updates of existing SceneObject records or allocations of
new ones. v1 algorithm:

  1. spatial gating per class (`_GATE_RADIUS_M`)
  2. class-match (only same-class candidates considered)
  3. Hungarian (scipy.optimize.linear_sum_assignment) min-cost matching
  4. unmatched detections → allocate new SceneObject
  5. unmatched objects → mark_stale handles them; we don't touch them here
  6. matched pairs → ObjectRegistry.update_object_pose (EMA blend)

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
# object is considered as a candidate for the incoming detection.
#
# Widened from the v1 numbers (cup 0.3 / bottle 0.3 / person 1.5) after
# operator feedback: when someone walks a handheld object (phone, cup,
# bottle) through frame, per-frame depth jitter can shift the reported
# pose by 40–80 cm tick-to-tick, which fell outside the tight gates and
# produced a fresh object_id every frame. The web UI then showed the
# list churning + suffixes climbing into the hundreds. Looser gates mean
# slightly more aggressive association — acceptable for the operator
# view; downstream relation queries already ignore matched-by-distance
# false positives via class match.
_GATE_RADIUS_M: dict[str, float] = {
    # Small / hand-held — keep tight; user moves them, two next to each
    # other on a tray should NOT collapse.
    "cup": 0.60,
    "bottle": 0.60,
    "phone": 0.60,
    "cell phone": 0.60,
    "tool": 0.60,
    "tray": 0.80,
    # Furniture-scale, static — loose gate tolerates depth noise on
    # the larger footprint without users physically moving the object.
    "table": 1.20,
    "chair": 1.00,
    "door": 1.50,
    # Static decor that depth+mask gets badly wrong because the support
    # surface is far behind the visible silhouette (potted plants:
    # leaves vs pot, depth picks one or the other and the pose jumps
    # 1-2 m frame-to-frame; wall-mounted picture frames / clocks: depth
    # may land on the wall vs the protruding frame edge). These never
    # move in any real demo, so the gate can be very loose.
    "potted plant": 1.80,
    "potted_plant": 1.80,         # YOLO-World sometimes underscores
    "plant":        1.80,
    "picture frame": 2.00,
    "picture_frame": 2.00,
    "frame":         2.00,
    "clock":         1.50,
    "tv":            1.80,
    "monitor":       1.50,
    "sofa":          1.50,
    "couch":         1.50,
    "bed":           1.80,
    "refrigerator":  1.50,
    "fridge":        1.50,
    "person": 3.00,
    "robot": 1.00,
}
_DEFAULT_GATE_RADIUS_M = 1.00

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
