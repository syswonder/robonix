# SPDX-License-Identifier: MulanPSL-2.0
"""Image-grounded VLM relation inference (VLM-primary scene graph).

Replaces the text-only, coordinates-as-JSON relation pass with one that lets the
model *see* the scene: tracked objects are projected from the map frame into the
current RGB keyframe, numbered boxes are drawn, and a single multimodal VLM call
enumerates the relations among the numbered objects. Box numbers map back to
``object_id`` to build edges.

Why this layer is image-grounded and geometry is not: contact/containment
("monitor on the desk") is exactly where coordinate-only reasoning and AABB
tests fail (full-volume desk box, perspective), and a VLM reads it holistically;
metric robot-actionable facts (``reachable_by``, ``near``) stay deterministic in
the geometric layer and are intentionally excluded from this vocabulary.
"""
from __future__ import annotations

import base64
import logging
import math
import os
from typing import Any, Optional

import numpy as np

from .relations import _LLM_TIMEOUT_SEC, _normalize_relation
from .types import SceneGraphEdge, SceneGraphNode

log = logging.getLogger(__name__)


# Relations the image pass may emit. `near` and `reachable_by` are deliberately
# absent — those are metric facts owned by the geometric layer (near is served
# as a proximity query, reachable_by as a gripper-distance edge), not visual
# judgements. Keeping them out stops the graph filling with "everything is near".
IMAGE_RELATION_VOCAB = (
    "on_top_of", "under", "inside", "contains",
    "attached_to", "part_of", "same_object",
)


# ── projection (map frame → image pixels) ──────────────────────────────────

def project_point(
    T_cam_map: np.ndarray, K: Any, p_world: tuple[float, float, float]
) -> Optional[tuple[float, float, float]]:
    """Project a map-frame point to ``(u, v, depth)`` pixels, or None if it is
    behind the camera.

    ``T_cam_map`` is the 4×4 camera-optical→map transform (as built by the
    perception detector); its inverse takes a map point into the camera-optical
    frame, then the pinhole model with intrinsics ``K`` (fx/fy/cx/cy) gives the
    pixel. ``depth`` (camera-frame z) is returned so callers can z-sort or gate
    on it. Points at/behind the image plane (z ≤ 0) return None."""
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
    out = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                lx, ly, lz = sx * hx, sy * hy, sz * hz
                # Rotate the local offset about Z (yaw), then translate.
                wx = center[0] + (c * lx - s * ly)
                wy = center[1] + (s * lx + c * ly)
                wz = center[2] + lz
                out.append((wx, wy, wz))
    return out


def project_box(
    T_cam_map: np.ndarray,
    K: Any,
    center: tuple[float, float, float],
    extent: tuple[float, float, float],
    yaw: float,
    img_w: int,
    img_h: int,
) -> Optional[tuple[int, int, int, int]]:
    """Project a 3D box to a clipped 2D pixel rect ``(u0, v0, u1, v1)``, or None
    when the object is not usefully visible.

    Projects the 8 corners, keeps those in front of the camera, takes their
    pixel min/max and clips to the image. Returns None if fewer than two corners
    are in front (object mostly behind the camera) or the clipped rect has no
    area (entirely out of frame)."""
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
    """Draw numbered rectangles on a copy of ``rgb_bgr`` and return it as a
    base64 JPEG string (no data-url prefix), or None if encoding fails.

    ``boxes`` is ``[(box_id, (u0, v0, u1, v1)), ...]``. The frame is downscaled
    so its longest side is ≤ ``max_dim`` (after drawing, so coordinates need no
    rescale) to bound VLM token cost. cv2 is imported lazily — the projection
    helpers above stay importable without it."""
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
    """User-turn text accompanying the annotated image: the box-number → label
    legend plus the instruction. The image itself is attached as a separate
    multimodal part by the caller (``chat_json(images=...)``)."""
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
    """Turn the VLM's ``{"edges": [...]}`` into validated graph edges.

    Drops edges whose box numbers are unknown, self-edges, duplicates of an
    already-seen ``(source, target, relation)``, and relations outside
    ``IMAGE_RELATION_VOCAB`` (after normalization). Confidence is bounded to
    [0, 1]. Never raises on a malformed entry — it is skipped."""
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
        confidence = 0.0
        try:
            confidence = float(item.get("confidence", 0.0))
        except (TypeError, ValueError):
            pass
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

class ImageRelationInferer:
    """Whole-scene image-grounded relation pass: project → annotate → one VLM
    call → parse. One instance per builder; stateless across calls."""

    def __init__(self, llm_client, *, max_dim: Optional[int] = None) -> None:
        self.llm_client = llm_client
        if max_dim is not None:
            self.max_dim = max_dim
        else:
            try:
                self.max_dim = int(os.environ.get("SCENE_GRAPH_IMAGE_MAX_DIM", "960"))
            except ValueError:
                self.max_dim = 960

    async def infer(
        self,
        nodes: list[SceneGraphNode],
        bundle: tuple[np.ndarray, Any, np.ndarray],
    ) -> Optional[list[SceneGraphEdge]]:
        """Return the round's relation edges, or None when the pass could not be
        run (no VLM creds, <2 visible objects, or VLM failure/empty) so the
        builder keeps prior edges via hysteresis instead of wiping the graph.
        An empty list means "ran, found no relations" (authoritative)."""
        if not self.llm_client.available:
            return None
        rgb_bgr, K, T_cam_map = bundle
        img_h, img_w = rgb_bgr.shape[:2]

        boxes: list[tuple[int, tuple[int, int, int, int]]] = []
        legend: list[tuple[int, str]] = []
        box_to_oid: dict[int, str] = {}
        next_id = 1
        for node in nodes:
            rect = project_box(
                T_cam_map, K, node.bbox_center, node.bbox_extent, node.yaw,
                img_w, img_h,
            )
            if rect is None:
                continue
            boxes.append((next_id, rect))
            legend.append((next_id, node.caption or node.label))
            box_to_oid[next_id] = node.object_id
            next_id += 1

        if len(box_to_oid) < 2:
            return None

        image_b64 = annotate_frame(rgb_bgr, boxes, max_dim=self.max_dim)
        if image_b64 is None:
            return None

        raw = await self.llm_client.chat_json(
            system_prompt=IMAGE_RELATION_SYSTEM_PROMPT,
            user_message=build_image_relation_user_text(legend),
            timeout=_LLM_TIMEOUT_SEC,
            images=[image_b64],
        )
        if not raw:
            return None
        return parse_image_relations(raw, box_to_oid)
