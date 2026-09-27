# SPDX-License-Identifier: MulanPSL-2.0
"""Image-grounded relation inference.

Tracked objects are projected into the current RGB keyframe as numbered boxes,
and one VLM call names the relations between the numbers. Contact and
containment are read far better from pixels than from boxes; the metric
relations (`near`, `reachable_by`) stay with the geometric loop.
"""
from __future__ import annotations

import base64
import copy
import logging
import math
import os
import time
from dataclasses import dataclass
from typing import Any, Callable, Literal, Optional

import numpy as np

from ..vision_cache import (
    FrameFingerprint,
    InferenceCounters,
    fingerprint_bgr,
    frames_equivalent,
)
from .relations import _LLM_TIMEOUT_SEC, _normalize_relation
from .types import SceneGraphEdge, SceneGraphNode

log = logging.getLogger(__name__)


IMAGE_RELATION_VOCAB = (
    "on_top_of", "under", "inside", "contains",
    "attached_to", "part_of", "same_object",
)


# ── projection (map frame → image pixels) ──────────────────────────────────

def project_point(
    T_cam_map: np.ndarray, K: Any, p_world: tuple[float, float, float]
) -> Optional[tuple[float, float, float]]:
    """Map-frame point → `(u, v, depth)` pixels through the camera-optical→map
    transform and pinhole `K`; None behind the camera."""
    try:
        T_map_cam = np.linalg.inv(np.asarray(T_cam_map, dtype=np.float64))
    except np.linalg.LinAlgError:
        return None
    p = T_map_cam @ np.array([p_world[0], p_world[1], p_world[2], 1.0])
    z = float(p[2])
    if z <= 1e-6:
        return None
    u = K.fx * (p[0] / z) + K.cx
    v = K.fy * (p[1] / z) + K.cy
    return float(u), float(v), z


def _corners(
    center: tuple[float, float, float],
    extent: tuple[float, float, float],
    yaw: float,
) -> list[tuple[float, float, float]]:
    """8 world-frame corners of a yaw-rotated, axis-z-aligned box."""
    hx, hy, hz = extent[0] / 2.0, extent[1] / 2.0, extent[2] / 2.0
    c, s = math.cos(yaw), math.sin(yaw)
    return [
        (center[0] + c * sx * hx - s * sy * hy,
         center[1] + s * sx * hx + c * sy * hy,
         center[2] + sz * hz)
        for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)
    ]


def project_box(
    T_cam_map: np.ndarray,
    K: Any,
    center: tuple[float, float, float],
    extent: tuple[float, float, float],
    yaw: float,
    img_w: int,
    img_h: int,
) -> Optional[tuple[int, int, int, int]]:
    """Clipped pixel rect `(u0, v0, u1, v1)` of a 3D box, or None when fewer
    than two corners are in front of the camera or it is out of frame."""
    us, vs = [], []
    for w in _corners(center, extent, yaw):
        pt = project_point(T_cam_map, K, w)
        if pt is None:
            continue
        us.append(pt[0])
        vs.append(pt[1])
    if len(us) < 2:
        return None
    u0 = max(0, int(math.floor(min(us))))
    v0 = max(0, int(math.floor(min(vs))))
    u1 = min(img_w, int(math.ceil(max(us))))
    v1 = min(img_h, int(math.ceil(max(vs))))
    if u1 <= u0 or v1 <= v0:
        return None
    return u0, v0, u1, v1


# ── frame annotation ───────────────────────────────────────────────────────

def annotate_frame(
    rgb_bgr: np.ndarray,
    boxes: list[tuple[int, tuple[int, int, int, int]]],
    *,
    max_dim: int = 960,
) -> Optional[str]:
    """Numbered rectangles drawn on a copy of the frame, downscaled to
    `max_dim` after drawing, as base64 JPEG; None if encoding fails."""
    try:
        import cv2
    except Exception as e:  # noqa: BLE001
        log.warning("[image-rel] cv2 unavailable, cannot annotate frame: %s", e)
        return None
    img = rgb_bgr.copy()
    for box_id, (u0, v0, u1, v1) in boxes:
        cv2.rectangle(img, (u0, v0), (u1, v1), (0, 255, 0), 2)
        label = str(box_id)
        ty = max(0, v0 - 6)
        cv2.putText(img, label, (u0, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, label, (u0, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
    h, w = img.shape[:2]
    longest = max(h, w)
    if longest > max_dim:
        scale = max_dim / float(longest)
        img = cv2.resize(img, (int(w * scale), int(h * scale)),
                         interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return None
    return base64.b64encode(buf.tobytes()).decode("ascii")


# ── prompt ─────────────────────────────────────────────────────────────────

IMAGE_RELATION_SYSTEM_PROMPT = """\
You are a robot's scene-graph reasoner. You see one RGB camera image in which
objects are outlined by GREEN numbered boxes. Report the physical relations that
hold BETWEEN the numbered objects, judging from what you see.

Return ONLY valid JSON.

Allowed relation values (use these exactly; source/target are box numbers):
- "on_top_of": source rests on top of target.
- "under": source is underneath target.
- "inside": source is inside target.
- "contains": source contains target.
- "attached_to": source is attached to / mounted on target.
- "part_of": source is a part/component of target.
- "same_object": both boxes are the same physical object.

Rules:
- Only relate objects that are actually drawn as numbered boxes. Never invent
  objects or numbers not present.
- Omit a pair entirely when no listed relation holds (do NOT output "none"/"near").
- Prefer the strongest single relation per ordered pair.

Output schema:
{"edges": [{"source": <int>, "target": <int>, "relation": "...",
            "confidence": 0.0, "reason": "brief"}]}
"""


def build_image_relation_user_text(legend: list[tuple[int, str]]) -> str:
    """The box-number → label legend and the instruction."""
    lines = ["Numbered objects in the image:"]
    for box_id, label in legend:
        lines.append(f"  {box_id}: {label}")
    lines.append(
        "Enumerate the relations among these numbered objects as JSON "
        '{"edges": [...]} per the schema. Use only the allowed relations.'
    )
    return "\n".join(lines)


# ── parsing ────────────────────────────────────────────────────────────────

def parse_image_relations(
    raw: dict, box_to_oid: dict[int, str]
) -> list[SceneGraphEdge]:
    """Validated edges from the VLM's `{"edges": [...]}`; unknown boxes,
    self-edges, duplicates and off-vocabulary relations are skipped."""
    edges: list[SceneGraphEdge] = []
    seen: set[tuple[str, str, str]] = set()
    for item in raw.get("edges", []) or []:
        if not isinstance(item, dict):
            continue
        try:
            src = int(item.get("source"))
            tgt = int(item.get("target"))
        except (TypeError, ValueError):
            continue
        if src == tgt or src not in box_to_oid or tgt not in box_to_oid:
            continue
        relation = _normalize_relation(item.get("relation", ""))
        if relation not in IMAGE_RELATION_VOCAB:
            continue
        src_oid, tgt_oid = box_to_oid[src], box_to_oid[tgt]
        key = (src_oid, tgt_oid, relation)
        if key in seen:
            continue
        seen.add(key)
        try:
            confidence = float(item.get("confidence", 0.0))
        except (TypeError, ValueError):
            confidence = 0.0
        edges.append(SceneGraphEdge(
            source_id=src_oid,
            target_id=tgt_oid,
            relation=relation,
            confidence=max(0.0, min(1.0, confidence)),
            method="llm",
            reason=str(item.get("reason", "")),
        ))
    return edges


# ── orchestration ──────────────────────────────────────────────────────────

ImageRelationOutcome = Literal["processed", "cached", "failed", "backoff"]


def _env_float(key: str, default: float) -> float:
    try:
        return float(os.environ.get(key, str(default)))
    except ValueError:
        return default


def _finite(value: Optional[float], key: str, default: float) -> float:
    """`value`, else the env var; `default` when that is not finite."""
    chosen = value if value is not None else _env_float(key, default)
    return chosen if math.isfinite(chosen) else default


def _all_close(left: tuple, right: tuple, tolerance: float) -> bool:
    return len(left) == len(right) and all(
        abs(a - b) <= tolerance for a, b in zip(left, right)
    )


@dataclass(frozen=True)
class _VisibleObjectSignature:

    object_id: str
    label: str
    rect: tuple[int, int, int, int]
    center: tuple[float, float, float]
    extent: tuple[float, float, float]
    yaw: float


@dataclass(frozen=True)
class _RelationInputSignature:

    visible: tuple[_VisibleObjectSignature, ...]
    intrinsics: tuple[int, int, float, float, float, float]
    camera_transform: tuple[float, ...]
    user_message: str
    model: str
    max_dim: int


@dataclass(frozen=True)
class ImageRelationResult:
    """Edges plus the decision that produced them."""

    edges: tuple[SceneGraphEdge, ...]
    outcome: ImageRelationOutcome

    def __post_init__(self) -> None:
        if self.outcome in ("failed", "backoff") and self.edges:
            raise ValueError(f"{self.outcome} relation result cannot contain edges")


class ImageRelationInferer:
    """project → annotate → one VLM call → parse, reusing the last answer
    while the visible objects stay the same."""

    def __init__(
        self,
        llm_client,
        *,
        max_dim: Optional[int] = None,
        frame_change_threshold: Optional[float] = None,
        failure_backoff_base_s: Optional[float] = None,
        failure_backoff_max_s: Optional[float] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.llm_client = llm_client
        if max_dim is None:
            try:
                max_dim = int(os.environ.get("SCENE_GRAPH_IMAGE_MAX_DIM", "960"))
            except ValueError:
                max_dim = 960
        self.max_dim = max_dim
        self.frame_change_threshold = min(1.0, max(0.0, _finite(
            frame_change_threshold, "SCENE_GRAPH_IMAGE_CHANGE_THRESHOLD", 0.01)))
        self.failure_backoff_base_s = max(0.0, _finite(
            failure_backoff_base_s, "SCENE_GRAPH_IMAGE_FAILURE_BACKOFF_BASE_SEC", 30.0))
        self.failure_backoff_max_s = max(self.failure_backoff_base_s, _finite(
            failure_backoff_max_s, "SCENE_GRAPH_IMAGE_FAILURE_BACKOFF_MAX_SEC", 300.0))
        # Relations among the same objects do not change because the camera
        # moved, so a new viewpoint of an unchanged set is not asked about
        # again until this long after the last answer.
        self.refresh_s = max(0.0, _env_float(
            "SCENE_GRAPH_IMAGE_REFRESH_SEC", 600.0))
        self._last_success_at = 0.0
        self._clock = clock
        self._last_success_signature: Optional[_RelationInputSignature] = None
        self._last_success_frame: Optional[FrameFingerprint] = None
        self._last_success_edges: Optional[list[SceneGraphEdge]] = None
        self._failure_streak = 0
        self._retry_at = 0.0
        self._stats = InferenceCounters()

    @property
    def inference_counts(self) -> dict[str, int]:
        return self._stats.as_dict()

    async def infer(
        self,
        nodes: list[SceneGraphNode],
        bundle: tuple[np.ndarray, Any, np.ndarray],
    ) -> Optional[ImageRelationResult]:
        """This round's result, or None when it cannot run. Failure and
        backoff stay distinct: only a real failed call ages the edges."""
        if not self.llm_client.available:
            return None
        rgb_bgr, K, T_cam_map = bundle
        img_h, img_w = rgb_bgr.shape[:2]

        boxes: list[tuple[int, tuple[int, int, int, int]]] = []
        legend: list[tuple[int, str]] = []
        box_to_oid: dict[int, str] = {}
        visible_signature: list[_VisibleObjectSignature] = []
        next_id = 1
        for node in sorted(nodes, key=lambda item: item.object_id):
            rect = project_box(
                T_cam_map, K, node.bbox_center, node.bbox_extent, node.yaw,
                img_w, img_h,
            )
            if rect is None:
                continue
            boxes.append((next_id, rect))
            label = node.caption or node.label
            legend.append((next_id, label))
            box_to_oid[next_id] = node.object_id
            visible_signature.append(_VisibleObjectSignature(
                object_id=node.object_id,
                label=label,
                rect=rect,
                center=tuple(float(v) for v in node.bbox_center),
                extent=tuple(float(v) for v in node.bbox_extent),
                yaw=float(node.yaw),
            ))
            next_id += 1

        if len(box_to_oid) < 2:
            return None

        intrinsics_signature = (
            int(K.width), int(K.height),
            float(K.fx), float(K.fy), float(K.cx), float(K.cy),
        )
        user_message = build_image_relation_user_text(legend)
        signature = _RelationInputSignature(
            visible=tuple(visible_signature),
            intrinsics=intrinsics_signature,
            camera_transform=tuple(
                float(value)
                for value in np.asarray(T_cam_map, dtype=np.float64).reshape(-1)
            ),
            user_message=user_message,
            model=str(getattr(self.llm_client, "model", "")),
            max_dim=self.max_dim,
        )
        frame = fingerprint_bgr(rgb_bgr)
        unchanged = self._same_input(
            signature,
            frame,
            self._last_success_signature,
            self._last_success_frame,
        ) or (
            self._same_objects(signature, self._last_success_signature)
            and self._clock() - self._last_success_at < self.refresh_s
        )
        if unchanged and self._last_success_edges is not None:
            self._record_skip("unchanged-input")
            edges = copy.deepcopy(self._last_success_edges)
            now_unix = time.time()
            for edge in edges:
                edge.method = "cached"
                edge.updated_at = now_unix
                edge.stale_rounds = 0
            return ImageRelationResult(tuple(edges), "cached")

        now = self._clock()
        retrying = self._failure_streak > 0
        if retrying and now < self._retry_at:
            self._record_skip("failure-backoff")
            return ImageRelationResult((), "backoff")

        image_b64 = annotate_frame(rgb_bgr, boxes, max_dim=self.max_dim)
        if image_b64 is None:
            return None
        if retrying:
            self._stats.retried += 1

        try:
            raw = await self.llm_client.chat_json(
                system_prompt=IMAGE_RELATION_SYSTEM_PROMPT,
                user_message=user_message,
                timeout=_LLM_TIMEOUT_SEC,
                images=[image_b64],
            )
        except Exception as e:  # noqa: BLE001
            log.warning("[image-rel] VLM call failed: %s: %s", type(e).__name__, e)
            raw = {}
        raw_edges = raw.get("edges") if isinstance(raw, dict) else None
        if not isinstance(raw_edges, list):
            self._record_failure(self._clock())
            return ImageRelationResult((), "failed")
        edges = parse_image_relations(raw, box_to_oid)
        self._last_success_signature = signature
        self._last_success_frame = frame
        self._last_success_at = self._clock()
        self._last_success_edges = copy.deepcopy(edges)
        self._clear_failure()
        self._stats.processed += 1
        self._log_stats(logging.INFO, "processed")
        return ImageRelationResult(tuple(edges), "processed")

    def _same_input(
        self,
        signature: _RelationInputSignature,
        frame: FrameFingerprint,
        previous_signature: Optional[_RelationInputSignature],
        previous_frame: Optional[FrameFingerprint],
    ) -> bool:
        return self._signatures_equivalent(signature, previous_signature) and frames_equivalent(
            frame,
            previous_frame,
            threshold=self.frame_change_threshold,
        )

    @staticmethod
    def _same_objects(
        current: _RelationInputSignature,
        previous: Optional[_RelationInputSignature],
    ) -> bool:
        """Same objects, same names, none moved; the camera may have."""
        if previous is None or current.model != previous.model:
            return False
        if len(current.visible) != len(previous.visible):
            return False
        for left, right in zip(current.visible, previous.visible):
            if left.object_id != right.object_id or left.label != right.label:
                return False
            if not _all_close(left.center, right.center, 0.05):
                return False
        return True

    @staticmethod
    def _signatures_equivalent(
        current: _RelationInputSignature,
        previous: Optional[_RelationInputSignature],
    ) -> bool:
        """Equal up to sensor jitter."""
        if previous is None:
            return False
        if (
            current.user_message != previous.user_message
            or current.model != previous.model
            or current.max_dim != previous.max_dim
            or current.intrinsics[:2] != previous.intrinsics[:2]
            or len(current.visible) != len(previous.visible)
        ):
            return False
        if not _all_close(current.intrinsics[2:], previous.intrinsics[2:], 0.01):
            return False
        if not _all_close(current.camera_transform, previous.camera_transform, 0.01):
            return False
        for left, right in zip(current.visible, previous.visible):
            if left.object_id != right.object_id or left.label != right.label:
                return False
            if any(abs(a - b) > 4 for a, b in zip(left.rect, right.rect)):
                return False
            if not _all_close(left.center, right.center, 0.02):
                return False
            if not _all_close(left.extent, right.extent, 0.02):
                return False
            if abs(left.yaw - right.yaw) > 0.02:
                return False
        return True

    def _record_skip(self, reason: str) -> None:
        self._stats.skipped += 1
        level = logging.INFO if self._stats.skipped % 10 == 0 else logging.DEBUG
        self._log_stats(level, reason)

    def _record_failure(self, now: float) -> None:
        self._failure_streak += 1
        exponent = min(self._failure_streak - 1, 10)
        delay = min(
            self.failure_backoff_max_s,
            self.failure_backoff_base_s * (2 ** exponent),
        )
        self._retry_at = now + delay
        self._stats.failed += 1
        self._log_stats(logging.WARNING, f"failed; retry_in={delay:.1f}s")

    def _clear_failure(self) -> None:
        self._failure_streak = 0
        self._retry_at = 0.0

    def _log_stats(self, level: int, reason: str) -> None:
        log.log(
            level,
            "[image-rel] inference stats: processed=%d skipped=%d "
            "retried=%d failed=%d reason=%s",
            self._stats.processed,
            self._stats.skipped,
            self._stats.retried,
            self._stats.failed,
            reason,
        )
