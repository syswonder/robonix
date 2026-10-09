# SPDX-License-Identifier: MulanPSL-2.0
"""User annotations — user-authored semantics anchored to the SLAM map."""
from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Optional

from .map_binding import sanitize_map_id

log = logging.getLogger(__name__)

# The kind whitelist lives here (single source); every consumer checks it
# through validate_annotation_fields below.
VALID_KINDS = ("region", "poi")
# Deprecated: "room" is what a region was called before. Files saved then, and
# callers still sending it, are read as "region" (Annotation.from_json and
# normalize_kind); nothing writes it any more.
LEGACY_KINDS = {"room": "region"}


def normalize_kind(kind):
    """The current name for an annotation kind, mapping deprecated ones."""
    return LEGACY_KINDS.get(kind, kind)
MAX_NAME_LEN = 128


def _annotation_identity(kind: str, name: str) -> tuple[str, str]:
    """Stable user-facing identity for idempotent region creation."""
    normalized = " ".join(str(name).strip().split()).casefold()
    return str(kind).strip().casefold(), normalized


def validate_annotation_fields(
    kind: str,
    name: Any,
    points: Any,
    theta: Any,
) -> Optional[str]:
    """Validate user-supplied annotation fields; return an error string on the
    first violation, None when everything is acceptable.
    """
    if kind not in VALID_KINDS:
        return f"kind must be one of {list(VALID_KINDS)}, got {kind!r}"
    if not isinstance(name, str):
        return "name must be a string"
    if len(name) > MAX_NAME_LEN:
        return f"name too long (max {MAX_NAME_LEN} chars)"
    if not isinstance(points, list):
        return "points must be a list of [x, y] pairs"
    for p in points:
        if (
            not isinstance(p, (list, tuple))
            or len(p) != 2
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in p)
            or not all(math.isfinite(float(v)) for v in p)
        ):
            return "each point must be a finite [x, y] number pair"
    if kind == "poi" and len(points) != 1:
        return "a poi takes exactly one point"
    if kind == "region" and len(points) < 3:
        return "a region polygon needs at least 3 points"
    if theta is not None:
        if kind != "poi":
            return "theta (heading) only applies to a poi"
        if not isinstance(theta, (int, float)) or isinstance(theta, bool) or not math.isfinite(float(theta)):
            return "theta must be a finite number or null"
    return None


@dataclass
class Annotation:
    """One user annotation."""
    annotation_id: str
    kind: str                       # "region" | "poi" (see VALID_KINDS)
    name: str
    points: list[list[float]] = field(default_factory=list)   # [[x, y], ...]
    # Optional poi heading, radians (regions never carry one — validated).
    theta: Optional[float] = None
    created_at: float = 0.0
    updated_at: float = 0.0
    stale: bool = False
    stale_reason: str = ""

    def to_json(self) -> dict:
        return asdict(self)

    @classmethod
    def from_json(cls, d: dict) -> "Annotation":
        """Rebuild from a stored dict, dropping unknown keys so a file written by
        a newer scene still loads (forward-tolerant).
        """
        known = set(cls.__dataclass_fields__)
        fields = {k: v for k, v in d.items() if k in known}
        # A map saved before the rename calls its polygons "room".
        fields["kind"] = normalize_kind(fields.get("kind"))
        return cls(**fields)


def _parse_file(path: Path) -> tuple[Optional[int], list[Annotation]]:
    """A store file's generation and validated annotations; raises on bad data."""
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"top-level JSON is {type(data).__name__}, not an object")
    stored_gen = data.get("generation")
    if stored_gen is not None:
        stored_gen = int(stored_gen)
    anns = []
    for d in data.get("annotations", []):
        if not isinstance(d, dict):
            raise ValueError(f"annotation entry is {type(d).__name__}, not an object")
        a = Annotation.from_json(d)
        err = validate_annotation_fields(a.kind, a.name, a.points, a.theta)
        if err:
            raise ValueError(f"annotation {a.annotation_id!r}: {err}")
        if not isinstance(a.annotation_id, str) or not a.annotation_id:
            raise ValueError("annotation with missing/empty id")
        anns.append(a)
    return stored_gen, anns


class AnnotationStore:
    """Per-map JSON persistence for user annotations."""

    def __init__(self, base_dir: str, *, map_id: str, generation: Optional[int] = None) -> None:
        """Open (creating dirs as needed) the store for `map_id` and load its
        file; a changed map generation marks its regions stale (`_load`).
        """
        self._lock = threading.Lock()
        self._map_id = sanitize_map_id(map_id)
        self._base = Path(base_dir).expanduser()
        self._base.mkdir(parents=True, exist_ok=True)
        self._path = self._base / f"{self._map_id}.json"
        self._generation = generation
        self._annotations: dict[str, Annotation] = {}
        self._load(current_generation=generation)

    @property
    def map_id(self) -> str:
        return self._map_id

    @property
    def path(self) -> Path:
        return self._path

    def read_saved(self, map_id: str) -> list[dict]:
        """Another map's regions, read without rebinding; [] if unreadable."""
        try:
            return [a.to_json() for a in
                    _parse_file(self._base / f"{sanitize_map_id(map_id)}.json")[1]]
        except (OSError, ValueError, TypeError, KeyError, AttributeError):
            return []

    def has_saved(self, map_id: str) -> bool:
        """True when a persisted annotation file already exists for `map_id`.
        """
        return (self._base / f"{sanitize_map_id(map_id)}.json").exists()

    def rebind(self, map_id: str, *, generation: Optional[int] = None,
               carry_current: bool = False) -> None:
        """Switch this store to another map partition."""
        next_id = sanitize_map_id(map_id)
        with self._lock:
            if next_id == self._map_id:
                self._generation = generation
                if carry_current:
                    self._save_locked()
                return
            self._map_id = next_id
            self._path = self._base / f"{self._map_id}.json"
            self._generation = generation
            if carry_current:
                self._save_locked()
            else:
                self._annotations = {}
                self._load(current_generation=generation)

    def delete_map(self, map_id: str) -> bool:
        """Delete exactly one persisted annotation partition."""
        target = sanitize_map_id(map_id)
        target_path = self._base / f"{target}.json"
        with self._lock:
            existed = target_path.exists()
            if existed:
                target_path.unlink()
            if target == self._map_id:
                self._annotations = {}
            return existed

    def _load(self, current_generation: Optional[int]) -> None:
        """Read the per-map file into memory."""
        if not self._path.exists():
            return
        try:
            stored_gen, anns = _parse_file(self._path)
        except (ValueError, TypeError, KeyError, AttributeError) as e:
            aside = self._path.with_suffix(f".json.corrupt-{int(time.time())}")
            # Rename failures re-raise: continuing with an empty store would
            # let the next accepted edit os.replace() the corrupt file —
            # destroying exactly the bytes this path promises to keep. Failing
            # __init__ disables the API loudly instead.
            self._path.rename(aside)
            log.error(
                "[scene-anno] %s failed to load (%s) — moved to %s, "
                "starting empty", self._path, e, aside,
            )
            return
        self._annotations = {a.annotation_id: a for a in anns}
        if current_generation is None:
            # Cannot judge staleness this session; keep the recorded epoch
            # so a later broadcast-bound boot still can.
            self._generation = stored_gen
            return
        if stored_gen is not None and stored_gen != int(current_generation):
            reason = f"map generation {stored_gen}→{current_generation}"
            n = self._flag_all_stale_locked(reason, new_generation=None)
            if n:
                log.warning(
                    "[scene-anno] map frame epoch changed (%s) — %d "
                    "annotation(s) marked stale for user confirmation", reason, n,
                )

    def _save_locked(self) -> None:
        """Atomically write the current state (caller holds the lock or is
        __init__).
        """
        payload = {
            "map_id": self._map_id,
            "generation": self._generation,
            "annotations": [a.to_json() for a in self._annotations.values()],
        }
        tmp = self._path.with_suffix(".json.tmp")
        tmp.write_text(
            json.dumps(payload, ensure_ascii=False, indent=1), encoding="utf-8"
        )
        os.replace(tmp, self._path)

    # list()/get() hand out the LIVE Annotation objects — treat them as read-
    # only.
    def list(self) -> list[Annotation]:
        with self._lock:
            return list(self._annotations.values())

    def get(self, annotation_id: str) -> Optional[Annotation]:
        with self._lock:
            return self._annotations.get(annotation_id)

    def list_json(self) -> list[dict]:
        """JSON-ready dump of every annotation (the `/api/state` field and `GET
        /api/annotations` share this shape).
        """
        with self._lock:
            return [a.to_json() for a in self._annotations.values()]

    def create(self, *, kind: str, name: str, points: list,
               theta: Optional[float] = None) -> Annotation:
        """Create + persist an annotation and return it."""
        now = time.time()
        norm_points = [[float(x), float(y)] for x, y in points]
        norm_theta = None if theta is None else float(theta)
        with self._lock:
            if kind == "region":
                target = _annotation_identity(kind, name)
                for existing in self._annotations.values():
                    if _annotation_identity(existing.kind, existing.name) == target:
                        existing.name = name
                        existing.points = norm_points
                        existing.theta = norm_theta
                        existing.stale = False
                        existing.stale_reason = ""
                        existing.updated_at = now
                        self._save_locked()
                        return existing
            ann = Annotation(
                annotation_id=f"anno.{uuid.uuid4().hex[:8]}",
                kind=kind,
                name=name,
                points=norm_points,
                theta=norm_theta,
                created_at=now,
                updated_at=now,
            )
            self._annotations[ann.annotation_id] = ann
            self._save_locked()
        return ann

    def update(self, annotation_id: str, *, name: Optional[str] = None,
               points: Optional[list] = None, theta: Optional[float] = None,
               clear_stale: bool = False) -> Optional[Annotation]:
        """Apply the provided fields to an existing annotation (None = keep), bump
        `updated_at`, persist, and return it; None when the id is unknown.
        """
        with self._lock:
            ann = self._annotations.get(annotation_id)
            if ann is None:
                return None
            if name is not None:
                ann.name = name
            if points is not None:
                ann.points = [[float(x), float(y)] for x, y in points]
                ann.stale, ann.stale_reason = False, ""
            if theta is not None:
                ann.theta = float(theta)
            if clear_stale:
                ann.stale, ann.stale_reason = False, ""
            ann.updated_at = time.time()
            self._save_locked()
            return ann

    def delete(self, annotation_id: str) -> bool:
        """Remove an annotation; True when it existed. Persists on change."""
        with self._lock:
            if annotation_id not in self._annotations:
                return False
            del self._annotations[annotation_id]
            self._save_locked()
            return True

    def _flag_all_stale_locked(self, reason: str,
                               new_generation: Optional[int]) -> int:
        """Shared body of the two stale-sweep entry points (caller holds the lock,
        or is __init__/_load): flag every non-stale annotation with `reason`,
        optionally adopt `new_generation`, persist when anything changed.
        Returns how many annotations flipped.
        """
        n = 0
        for a in self._annotations.values():
            if not a.stale:
                a.stale, a.stale_reason = True, reason
                n += 1
        gen_changed = new_generation is not None and new_generation != self._generation
        if gen_changed:
            self._generation = new_generation
        if n or gen_changed:
            self._save_locked()
        return n

    def reconcile_generation(self, live_generation: int) -> int:
        """Called when the map's true generation becomes known at runtime (the
        lifecycle watcher's confirmation) — a store built under a static
        binding (generation None) cannot judge staleness at load time, so the
        judgement happens here instead.
        """
        with self._lock:
            if self._generation is None or int(self._generation) == int(live_generation):
                if self._generation != live_generation:
                    self._generation = int(live_generation)
                    self._save_locked()
                return 0
            reason = f"map generation {self._generation}→{live_generation}"
            n = self._flag_all_stale_locked(reason, int(live_generation))
        if n:
            log.warning(
                "[scene-anno] recorded map epoch differs from mapping's (%s) "
                "— %d annotation(s) marked stale for user confirmation",
                reason, n,
            )
        return n

    def mark_all_stale(self, reason: str, *, new_generation: Optional[int] = None) -> int:
        """Flag every non-stale annotation stale with `reason` (map frame epoch
        changed at runtime — the _lifecycle_watch hook).
        """
        with self._lock:
            return self._flag_all_stale_locked(reason, new_generation)
