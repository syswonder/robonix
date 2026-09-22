# SPDX-License-Identifier: MulanPSL-2.0
"""A few pictures of each object, taken from genuinely different sides.

Asked which chair you meant, a list of ids is no help. A picture of each one
is, and scene already has the frame and the geometry to cut it: the RGB image
the perception tick just used, the object's 3D box, and the camera transform
that relates them.

Keeping the most *recent* views would be the obvious thing and the wrong one.
A robot approaching a chair produces a dozen frames of the same face of it, so
"the last five" is one angle five times. What makes a second picture worth
storing is that it shows a side the first one did not, so views are kept for
angular spread: a new one displaces whichever kept view is most redundant --
the one nearest in bearing to another -- and only when it is not markedly
worse.

Quality here means "is this a usable portrait", which is mostly a question of
size and of whether the object runs off the edge of the frame. A box clipped
by the image border is a picture of part of a thing.

The files live under the map they were seen on, so deleting a map's data
deletes its pictures. They are photographs of someone's home; they should not
outlive the map that explains why they exist.
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

log = logging.getLogger(__name__)

# Enough to show a thing from several sides; past this the marginal picture
# mostly costs disk and attention.
_DEFAULT_MAX_VIEWS = 5

# Below this a crop is a smudge, not a portrait.
_MIN_CROP_PX = 24

# A new view has to be at least this far from every kept one, in radians of
# bearing, to count as a different side rather than the same one again.
# ~25 degrees: closer than that and two photographs of a chair are the same
# photograph of a chair.
_MIN_BEARING_GAP = 0.44

# Replacing a kept view with a much worse one loses information even when the
# angle is new, so a newcomer must be within this fraction of the view it
# displaces.
_QUALITY_FLOOR = 0.6


def _sanitize_segment(raw: str) -> str:
    """One path segment that cannot escape its directory.

    Deliberately narrower than the map-id rule: this is only ever a directory
    name derived from an id scene itself minted, so there is nothing to
    preserve and every reason to be strict."""
    keep = [c if (c.isalnum() or c in "._-") else "_" for c in (raw or "")]
    out = "".join(keep).strip(". ") or "unknown"
    return out[:120]


def bearing_of(
    camera_xy: tuple[float, float], object_xy: tuple[float, float],
) -> float:
    """Which side the camera is looking from, in map-frame radians.

    The angle from the object to the camera -- not the camera's heading. Two
    cameras on opposite sides of a chair have bearings pi apart however each
    happens to be pointed, which is the property the spread is built on."""
    return math.atan2(camera_xy[1] - object_xy[1], camera_xy[0] - object_xy[0])


def _angular_gap(a: float, b: float) -> float:
    """Smallest angle between two bearings, in [0, pi]."""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


# Context around the box, as a fraction of its larger side. A crop cut
# exactly at the object's outline is harder to place than one that shows a
# hand's width of what it is sitting on.
_CROP_MARGIN_FRAC = 0.12
# The longest a crop may be relative to its short side before it stops
# reading as a picture of a thing and starts reading as a strip. Projected
# boxes of flat objects -- a tabletop from across it, a frame edge-on --
# land far past this.
_MAX_CROP_ASPECT = 2.2


def pad_rect(
    rect: tuple[int, int, int, int], img_w: int, img_h: int,
    *, margin_frac: float = _CROP_MARGIN_FRAC,
    max_aspect: float = _MAX_CROP_ASPECT,
) -> tuple[int, int, int, int]:
    """Grow `rect` into a crop worth looking at, without leaving the image.

    Two steps, both centred on the original box so the object stays in the
    middle of the picture: a margin for context, then the short side grown
    until the aspect ratio is within `max_aspect`.

    Clamping wins over both. A box against the image edge keeps whatever
    room exists on the other side rather than being shifted off the object
    to satisfy a ratio -- an off-centre object is still recognisable, a
    differently-framed one is not.
    """
    u0, v0, u1, v1 = rect
    w, h = max(1, u1 - u0), max(1, v1 - v0)

    margin = int(round(max(w, h) * max(0.0, margin_frac)))
    u0, v0 = u0 - margin, v0 - margin
    u1, v1 = u1 + margin, v1 + margin
    w, h = u1 - u0, v1 - v0

    if max_aspect >= 1.0:
        # Rounded up on both the target and the split. Flooring either loses
        # up to a pixel a side, which leaves the result a hair outside the
        # ratio it was grown to satisfy -- close enough to look fine and
        # wrong enough to fail the rule it exists for.
        if w > h * max_aspect:
            want = math.ceil(w / max_aspect)
            grow = math.ceil((want - h) / 2)
            v0, v1 = v0 - grow, v1 + grow
        elif h > w * max_aspect:
            want = math.ceil(h / max_aspect)
            grow = math.ceil((want - w) / 2)
            u0, u1 = u0 - grow, u1 + grow

    u0, v0 = max(0, u0), max(0, v0)
    u1, v1 = min(img_w, u1), min(img_h, v1)
    return (u0, v0, u1, v1)


# How far the measured depth may sit in front of the object before the crop is
# treated as showing something else. Half a metre covers pose error and the
# near face of a big object; a wall between camera and object is metres out.
_OCCLUSION_TOLERANCE_M = 0.5
# Fraction of the crop that must carry a usable depth reading for the check to
# mean anything. Below this the depth image tells us nothing and the crop is
# refused rather than guessed at.
_MIN_DEPTH_COVERAGE = 0.15


def looks_like(depth_m, rect, expected_m: float) -> bool:
    """Does what the camera sees at ``rect`` sit where the object does?

    A box projects into the image whenever the geometry says it would be in
    frame -- whether or not anything is in the way. Without this the store
    happily kept a picture of the wall in front of a shelf, and an object
    ended up with five photographs of five different things, every one of them
    geometrically correct.

    The test is deliberately loose: the nearest surface in the crop has to be
    within `_OCCLUSION_TOLERANCE_M` of the object's own distance. A wall in
    between fails it by metres. Missing or sparse depth returns False -- a
    picture we cannot vouch for is not worth keeping, and there will be
    another frame along shortly.
    """
    if expected_m <= 0.0:
        return False
    # No depth at all is not evidence against the crop -- a deployment without
    # a depth stream would otherwise lose every picture. Only a depth image
    # that contradicts the object's distance refuses one.
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
            # Too few readings to disagree with. Not evidence either way.
            return True
        # The 10th percentile rather than the minimum: a handful of stray near
        # pixels at a depth edge should not condemn an otherwise clear view.
        nearest = float(np.percentile(valid, 10))
    except Exception:  # noqa: BLE001
        return False
    return nearest >= expected_m - _OCCLUSION_TOLERANCE_M


def crop_quality(
    rect: tuple[int, int, int, int], img_w: int, img_h: int,
) -> float:
    """How good a portrait this crop is, in [0, 1].

    Area carries most of it -- a bigger crop is a closer, sharper look. A box
    touching the image border is penalised because the object is cut off, and
    which part is missing is exactly what a person is trying to judge."""
    u0, v0, u1, v1 = rect
    w, h = max(0, u1 - u0), max(0, v1 - v0)
    if w < _MIN_CROP_PX or h < _MIN_CROP_PX:
        return 0.0
    # Square-rooted so the score tracks linear size rather than area: a crop
    # twice as wide should read as twice as good, not four times.
    area = math.sqrt((w * h) / float(max(1, img_w * img_h)))
    touches_edge = u0 <= 0 or v0 <= 0 or u1 >= img_w or v1 >= img_h
    # A crop still shaped like a strip after padding is one against an image
    # edge with nowhere to grow. It should not take a slot from a crop that
    # shows the whole object, however much area the strip covers.
    aspect = max(w, h) / float(max(1, min(w, h)))
    shape = 1.0 if aspect <= _MAX_CROP_ASPECT else _MAX_CROP_ASPECT / aspect
    return min(1.0, area) * (0.45 if touches_edge else 1.0) * shape


class ObjectViewStore:
    """Up to `max_views` crops per object, kept for angular spread.

    `encode` turns a cropped BGR array into JPEG bytes. It is injected so the
    selection logic can be exercised without OpenCV, and so a deployment
    without it degrades to storing nothing rather than to crashing the
    perception tick.
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

    # ── layout ────────────────────────────────────────────────────────────
    def _dir(self, map_id: str, object_id: str) -> Path:
        return self.root / _sanitize_segment(map_id) / _sanitize_segment(object_id)

    def _index_path(self, map_id: str, object_id: str) -> Path:
        return self._dir(map_id, object_id) / "index.json"

    def _read_index(self, map_id: str, object_id: str) -> list[dict]:
        path = self._index_path(map_id, object_id)
        try:
            blob = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return []
        return blob.get("views", []) if isinstance(blob, dict) else []

    def _write_index(self, map_id: str, object_id: str, views: list[dict]) -> None:
        path = self._index_path(map_id, object_id)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".json.tmp")
        tmp.write_text(
            json.dumps({"views": views}, ensure_ascii=False, indent=1),
            encoding="utf-8",
        )
        tmp.replace(path)

    # ── selection ─────────────────────────────────────────────────────────
    def _slot_for(self, views: list[dict], bearing: float, quality: float):
        """Which slot a candidate should take, or None to keep what we have.

        Three cases, in order: room left; a side nobody has covered, taken by
        displacing the most redundant view; or the same side as a view we
        already hold, taken only if it is a better look at it."""
        nearest_i, nearest_gap = 0, math.pi
        for i, v in enumerate(views):
            gap = _angular_gap(bearing, float(v.get("bearing", 0.0)))
            if gap < nearest_gap:
                nearest_gap, nearest_i = gap, i

        # Asked first, and regardless of free space: the slots are for sides,
        # not for frames. A robot that has barely moved offers the same side
        # over and over, and spending free slots on those would fill the
        # budget with one angle before the spread rule got a say -- which is
        # exactly what was observed, two slots apart by 0.0002 rad.
        if views and nearest_gap < _MIN_BEARING_GAP:
            if quality > float(views[nearest_i].get("quality", 0.0)):
                return nearest_i, "better_view_of_same_side"
            return None, None

        if len(views) < self.max_views:
            return len(views), None

        # A genuinely new side. Drop whichever kept view is most redundant --
        # closest in bearing to another -- rather than the oldest, so the set
        # keeps spreading out instead of rotating.
        victim, tightest = 0, math.pi
        for i, v in enumerate(views):
            for j, w in enumerate(views):
                if i == j:
                    continue
                gap = _angular_gap(
                    float(v.get("bearing", 0.0)), float(w.get("bearing", 0.0)))
                if gap < tightest:
                    tightest, victim = gap, i
        if quality < float(views[victim].get("quality", 0.0)) * _QUALITY_FLOOR:
            return None, None
        return victim, "new_side"

    # ── the one entry point ───────────────────────────────────────────────
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
        now: Optional[float] = None,
    ) -> bool:
        """Store this crop if it shows something the kept ones do not.

        Returns whether it was stored. Never raises into the perception tick:
        a missing encoder or an unwritable directory costs pictures, not
        tracking."""
        # Padded here rather than at the call site: every picture this store
        # keeps is for the same purpose, so the shape rule belongs with the
        # store and not with each thing that projects a box.
        rect = pad_rect(rect, img_w, img_h)
        quality = crop_quality(rect, img_w, img_h)
        if quality <= 0.0:
            return False
        views = self._read_index(map_id, object_id)
        slot, _why = self._slot_for(views, bearing, quality)
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
            "at": float(now if now is not None else time.time()),
            "w": int(u1 - u0),
            "h": int(v1 - v0),
            "bytes": len(data),
        }
        if slot < len(views):
            views[slot] = record
        else:
            views.append(record)
        self._write_index(map_id, object_id, views)
        return True

    # ── reading ───────────────────────────────────────────────────────────
    def views(self, map_id: str, object_id: str) -> list[dict]:
        """What is stored for this object, best-looking first."""
        return sorted(
            self._read_index(map_id, object_id),
            key=lambda v: -float(v.get("quality", 0.0)),
        )

    def read(self, map_id: str, object_id: str, index: int) -> Optional[bytes]:
        path = self._dir(map_id, object_id) / f"view_{int(index)}.jpg"
        try:
            return path.read_bytes()
        except OSError:
            return None

    def forget(self, map_id: str, object_id: str) -> int:
        """Drop every view of one object; returns how many files went.

        Called when a person deletes an object: they said it is not a thing,
        and its photographs should not outlive that."""
        directory = self._dir(map_id, object_id)
        try:
            count = len(list(directory.glob("view_*.jpg")))
            shutil.rmtree(directory)
            return count
        except OSError:
            return 0

    def forget_stale_sessions(self, keep: str) -> int:
        """Drop every unsaved session's pictures except ``keep``.

        A saved map keeps its own name and its pictures stay: they describe a
        place that still exists. An unsaved session is named per run, so one
        left behind belongs to a map nothing can re-anchor to -- its pictures
        can only mislead. Returns how many were removed.
        """
        import shutil

        dropped = 0
        try:
            entries = list(self.root.iterdir())
        except OSError:
            return 0
        for entry in entries:
            if not entry.is_dir() or not entry.name.startswith("session-"):
                continue
            if entry.name == _sanitize_segment(keep):
                continue
            try:
                shutil.rmtree(entry)
                dropped += 1
            except OSError as error:  # noqa: BLE001
                log.debug("[object-views] could not drop %s: %r", entry, error)
        return dropped

    def forget_map(self, map_id: str) -> None:
        """Drop every view on one map, for when the map itself is deleted."""
        try:
            shutil.rmtree(self.root / _sanitize_segment(map_id))
        except OSError:
            pass


def _default_encoder() -> Callable[[Any], Optional[bytes]]:
    """JPEG through OpenCV when it is there, and nothing when it is not.

    Resolved once per store rather than per crop, and lazily, so importing
    this module costs nothing in a process that has no camera."""
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


def store_from_env(
    *, max_views: Optional[int] = None,
) -> Optional["ObjectViewStore"]:
    """The store this deployment is configured for, or None when it is not.

    Absent configuration means no pictures, which is a supported state: a
    headless or storage-constrained install still tracks objects, it just
    cannot show you one."""
    root = (os.environ.get("SCENE_OBJECT_VIEWS_DIR") or "").strip()
    if not root:
        return None
    try:
        limit = int(
            os.environ.get("SCENE_OBJECT_VIEWS_MAX", "").strip()
            or (max_views or _DEFAULT_MAX_VIEWS)
        )
    except ValueError:
        limit = max_views or _DEFAULT_MAX_VIEWS
    return ObjectViewStore(root, max_views=limit)
