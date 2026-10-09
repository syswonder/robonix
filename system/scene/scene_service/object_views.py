# SPDX-License-Identifier: MulanPSL-2.0
"""A few photographs of each object, taken from different sides.

Views are kept for angular spread, not recency: a robot approaching a chair
sees the same face of it for a dozen frames. Files live under the map (or the
unsaved session) they were taken on, so they go when the map goes.
"""
from __future__ import annotations

import json
import logging
import math
import os
import shutil
import time
from pathlib import Path
from typing import Any, Callable, Optional

from .map_binding import sanitize_map_id as _segment

log = logging.getLogger(__name__)

_DEFAULT_MAX_VIEWS = 5
_MIN_CROP_PX = 24
# Closer than ~25 degrees in bearing, two views show the same side.
_MIN_BEARING_GAP = 0.44
# A new side may displace a kept view only if it is not much worse.
_QUALITY_FLOOR = 0.6
# Context around the box, as a fraction of its larger side.
_CROP_MARGIN_FRAC = 0.12
# Past this aspect ratio a crop is a strip rather than a picture of a thing.
_MAX_CROP_ASPECT = 2.2
# Nearest depth may sit this far in front of the object (pose error, near face).
_OCCLUSION_TOLERANCE_M = 0.5
_MIN_DEPTH_COVERAGE = 0.15
# Share of the crop's depth readings that must lie at the object's distance.
_MIN_AT_OBJECT = 0.2


def bearing_of(
    camera_xy: tuple[float, float], object_xy: tuple[float, float],
) -> float:
    """Direction from the object to the camera, in map-frame radians."""
    return math.atan2(camera_xy[1] - object_xy[1], camera_xy[0] - object_xy[0])


def _angular_gap(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def pad_rect(
    rect: tuple[int, int, int, int], img_w: int, img_h: int,
) -> tuple[int, int, int, int]:
    """Add a context margin, widen the short side to `_MAX_CROP_ASPECT`, then
    clamp to the image. Clamping wins: an object near the edge stays off-centre
    rather than being cropped out."""
    u0, v0, u1, v1 = rect
    margin = int(round(max(max(1, u1 - u0), max(1, v1 - v0)) * _CROP_MARGIN_FRAC))
    u0, v0, u1, v1 = u0 - margin, v0 - margin, u1 + margin, v1 + margin
    w, h = u1 - u0, v1 - v0
    # Rounded up, or the result lands a pixel outside the ratio.
    if w > h * _MAX_CROP_ASPECT:
        grow = math.ceil((math.ceil(w / _MAX_CROP_ASPECT) - h) / 2)
        v0, v1 = v0 - grow, v1 + grow
    elif h > w * _MAX_CROP_ASPECT:
        grow = math.ceil((math.ceil(h / _MAX_CROP_ASPECT) - w) / 2)
        u0, u1 = u0 - grow, u1 + grow
    return (max(0, u0), max(0, v0), min(img_w, u1), min(img_h, v1))


def looks_like(depth_m, rect, expected_m: float) -> bool:
    """Whether the crop shows the object: nothing in front of it, and enough
    of the crop at the object's depth (not the wall behind a missed pose).

    No depth stream, or too few depth readings, is not evidence against it.
    """
    if expected_m <= 0.0:
        return False
    if depth_m is None:
        return True
    u0, v0, u1, v1 = rect
    if u1 <= u0 or v1 <= v0:
        return False
    try:
        import numpy as np

        patch = np.asarray(depth_m[v0:v1, u0:u1], dtype="float32")
        if patch.size == 0:
            return False
        valid = patch[np.isfinite(patch) & (patch > 0.05)]
        if valid.size < max(4, int(patch.size * _MIN_DEPTH_COVERAGE)):
            return True
        # 10th percentile: stray pixels at a depth edge should not count.
        nearest = float(np.percentile(valid, 10))
        tolerance = max(_OCCLUSION_TOLERANCE_M, 0.25 * expected_m)
        at_object = float(np.mean(np.abs(valid - expected_m) <= tolerance))
    except Exception:  # noqa: BLE001
        return False
    return (nearest >= expected_m - _OCCLUSION_TOLERANCE_M
            and at_object >= _MIN_AT_OBJECT)


def crop_quality(
    rect: tuple[int, int, int, int], img_w: int, img_h: int,
) -> float:
    """How good a portrait this crop is, in [0, 1]: linear size, penalised
    when cut off by the image edge or still strip-shaped."""
    u0, v0, u1, v1 = rect
    w, h = max(0, u1 - u0), max(0, v1 - v0)
    if w < _MIN_CROP_PX or h < _MIN_CROP_PX:
        return 0.0
    size = math.sqrt((w * h) / float(max(1, img_w * img_h)))
    touches_edge = u0 <= 0 or v0 <= 0 or u1 >= img_w or v1 >= img_h
    aspect = max(w, h) / float(max(1, min(w, h)))
    shape = 1.0 if aspect <= _MAX_CROP_ASPECT else _MAX_CROP_ASPECT / aspect
    return min(1.0, size) * (0.45 if touches_edge else 1.0) * shape


class ObjectViewStore:
    """Up to `max_views` crops per object, kept for angular spread.

    `encode` turns a BGR crop into JPEG bytes; injected so tests need no
    OpenCV, and so a deployment without it stores nothing instead of failing.
    """

    def __init__(
        self,
        root: str | Path,
        *,
        max_views: int = _DEFAULT_MAX_VIEWS,
        encode: Optional[Callable[[Any], Optional[bytes]]] = None,
    ) -> None:
        self.root = Path(root).expanduser()
        self.max_views = max(1, int(max_views))
        self._encode = encode or _default_encoder()
        # Where an unsaved session files its pictures; set once per run.
        self.session_id = "session"

    def partition(self, map_binding: Optional[dict]) -> str:
        """The directory a binding's pictures live under.

        Object ids restart every boot, so a session bound to no saved map uses
        this run's session id rather than a shared "default".
        """
        binding = map_binding or {}
        map_id = str(binding.get("map_id") or "")
        if not map_id or binding.get("source") == "default":
            return self.session_id
        return map_id

    def _dir(self, map_id: str, object_id: str) -> Path:
        return self.root / _segment(map_id) / _segment(object_id)

    def _read_index(self, map_id: str, object_id: str) -> list[dict]:
        try:
            blob = json.loads(
                (self._dir(map_id, object_id) / "index.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return []
        return blob.get("views", []) if isinstance(blob, dict) else []

    def _write_index(self, map_id: str, object_id: str, views: list[dict]) -> None:
        path = self._dir(map_id, object_id) / "index.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps({"views": views}, indent=1), encoding="utf-8")
        tmp.replace(path)

    def _slot_for(self, views: list[dict], bearing: float,
                  quality: float) -> Optional[int]:
        """The slot a candidate takes, or None to keep what is stored."""
        def bearing_at(i: int) -> float:
            return float(views[i].get("bearing", 0.0))

        def quality_at(i: int) -> float:
            return float(views[i].get("quality", 0.0))

        # Same side as a kept view: only a better look at it replaces it. Asked
        # before free space, or a robot standing still fills every slot with
        # one angle.
        if views:
            nearest = min(range(len(views)),
                          key=lambda i: _angular_gap(bearing, bearing_at(i)))
            if _angular_gap(bearing, bearing_at(nearest)) < _MIN_BEARING_GAP:
                return nearest if quality > quality_at(nearest) else None
        if len(views) < self.max_views:
            return len(views)
        # A new side displaces the most redundant view, so the set spreads out.
        victim = min(
            range(len(views)),
            key=lambda i: min((_angular_gap(bearing_at(i), bearing_at(j))
                               for j in range(len(views)) if j != i),
                              default=math.pi))
        if quality < quality_at(victim) * _QUALITY_FLOOR:
            return None
        return victim

    def offer(
        self,
        *,
        map_id: str,
        object_id: str,
        image_bgr: Any,
        rect: tuple[int, int, int, int],
        bearing: float,
        img_w: int,
        img_h: int,
    ) -> bool:
        """Store this crop if it shows a side the kept ones do not.

        Never raises: a missing encoder or an unwritable directory costs
        pictures, not tracking.
        """
        rect = pad_rect(rect, img_w, img_h)
        quality = crop_quality(rect, img_w, img_h)
        if quality <= 0.0:
            return False
        views = self._read_index(map_id, object_id)
        slot = self._slot_for(views, bearing, quality)
        if slot is None:
            return False
        u0, v0, u1, v1 = rect
        try:
            data = self._encode(image_bgr[v0:v1, u0:u1])
        except Exception as error:  # noqa: BLE001
            log.debug("[object-views] crop encode failed: %r", error)
            return False
        if not data:
            return False
        target = self._dir(map_id, object_id) / f"view_{slot}.jpg"
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        except OSError as error:
            log.debug("[object-views] write failed: %r", error)
            return False
        record = {
            "index": slot,
            "bearing": round(float(bearing), 4),
            "quality": round(float(quality), 4),
            "at": time.time(),
            "w": int(u1 - u0),
            "h": int(v1 - v0),
        }
        if slot < len(views):
            views[slot] = record
        else:
            views.append(record)
        self._write_index(map_id, object_id, views)
        return True

    def views(self, map_id: str, object_id: str) -> list[dict]:
        """Stored views, best first."""
        return sorted(self._read_index(map_id, object_id),
                      key=lambda v: -float(v.get("quality", 0.0)))

    def read(self, map_id: str, object_id: str, index: int) -> Optional[bytes]:
        try:
            return (self._dir(map_id, object_id) / f"view_{int(index)}.jpg").read_bytes()
        except OSError:
            return None

    def forget(self, map_id: str, object_id: str) -> None:
        shutil.rmtree(self._dir(map_id, object_id), ignore_errors=True)

    def forget_stale_sessions(self, keep: str) -> int:
        """Drop earlier runs' unsaved sessions: nothing can re-anchor them."""
        dropped = 0
        try:
            entries = list(self.root.iterdir())
        except OSError:
            return 0
        for entry in entries:
            if (not entry.is_dir() or not entry.name.startswith("session-")
                    or entry.name == _segment(keep)):
                continue
            try:
                shutil.rmtree(entry)
                dropped += 1
            except OSError as error:
                log.debug("[object-views] could not drop %s: %r", entry, error)
        return dropped

    def adopt(self, old: str, new: str) -> None:
        """Move one partition's pictures to another, replacing what it held."""
        src, dst = self.root / _segment(old), self.root / _segment(new)
        if src == dst or not src.is_dir():
            return
        shutil.rmtree(dst, ignore_errors=True)
        src.rename(dst)

    def forget_map(self, map_id: str) -> None:
        shutil.rmtree(self.root / _segment(map_id), ignore_errors=True)


def _default_encoder() -> Callable[[Any], Optional[bytes]]:
    """JPEG through OpenCV, imported on first use; None without it."""
    def encode(crop: Any) -> Optional[bytes]:
        try:
            import cv2
        except Exception:  # noqa: BLE001
            return None
        if crop is None or getattr(crop, "size", 0) == 0:
            return None
        ok, buf = cv2.imencode(".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        return bytes(buf) if ok else None
    return encode


def store_from_env() -> Optional[ObjectViewStore]:
    """The configured store, or None when SCENE_OBJECT_VIEWS_DIR is unset."""
    root = (os.environ.get("SCENE_OBJECT_VIEWS_DIR") or "").strip()
    if not root:
        return None
    try:
        limit = int(os.environ.get("SCENE_OBJECT_VIEWS_MAX", "").strip()
                    or _DEFAULT_MAX_VIEWS)
    except ValueError:
        limit = _DEFAULT_MAX_VIEWS
    return ObjectViewStore(root, max_views=limit)
