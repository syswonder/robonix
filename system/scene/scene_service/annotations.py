# SPDX-License-Identifier: MulanPSL-2.0
"""User annotations — user-authored semantics anchored to the SLAM map.

An annotation is a named geometric mark the user draws on the map canvas:
a `room` (polygon + name) or a `poi` (single point, optional heading).
Annotations share the same foundation as perceived objects: coordinates
are map-frame meters, storage is partitioned by `map_id`, and validity is
judged by mapping's `generation` epoch (see map_binding.py). Unlike
perceived objects they are USER ASSETS: a map rebuild never deletes them —
they are kept and flagged `stale` for the user to confirm or redraw.

Storage is one JSON file per map (`<base_dir>/<map_id>.json`, atomic
tmp+rename writes). Annotations carry no vectors, so they stay out of the
milvus object store on purpose; the per-map JSON mirrors SceneGraphStore's
partitioning pattern.
"""
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
# through validate_annotation_fields below. `poi` is model-reserved: the
# store handles it, the first UI iteration only draws rooms.
VALID_KINDS = ("room", "poi")
MAX_NAME_LEN = 128


def validate_annotation_fields(
    kind: str,
    name: Any,
    points: Any,
    theta: Any,
) -> Optional[str]:
    """Validate user-supplied annotation fields; return an error string on
    the first violation, None when everything is acceptable.

    Rules: `kind` must be whitelisted; `name` a string ≤ MAX_NAME_LEN;
    `points` a list of finite [x, y] number pairs — exactly 1 for a poi,
    ≥3 for a room (a polygon needs three vertices); `theta` (a heading)
    only applies to a poi and must be None or a finite number — a region
    has no heading, so a room carrying theta is rejected rather than
    silently stored. NaN/inf anywhere is rejected so bad JSON can never
    poison the stored file."""
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
    if kind == "room" and len(points) < 3:
        return "a room polygon needs at least 3 points"
    if theta is not None:
        if kind != "poi":
            return "theta (heading) only applies to a poi"
        if not isinstance(theta, (int, float)) or isinstance(theta, bool) or not math.isfinite(float(theta)):
            return "theta must be a finite number or null"
    return None


@dataclass
class Annotation:
    """One user annotation. `points` are map-frame meters; `stale` means
    the map's generation changed after this was drawn — geometry may no
    longer line up, and the user must confirm or redraw (never auto-deleted)."""
    annotation_id: str
    kind: str                       # "room" | "poi" (see VALID_KINDS)
    name: str
    points: list[list[float]] = field(default_factory=list)   # [[x, y], ...]
    # Optional poi heading, radians (rooms never carry one — validated).
    # API semantics: theta=None on update means "keep"; a set heading
    # cannot be cleared, only changed. Revisit if/when the poi UI lands.
    theta: Optional[float] = None
    created_at: float = 0.0
    updated_at: float = 0.0
    stale: bool = False
    stale_reason: str = ""

    def to_json(self) -> dict:
        return asdict(self)

    @classmethod
    def from_json(cls, d: dict) -> "Annotation":
        """Rebuild from a stored dict, dropping unknown keys so a file
        written by a newer scene still loads (forward-tolerant)."""
        known = set(cls.__dataclass_fields__)
        return cls(**{k: v for k, v in d.items() if k in known})


class AnnotationStore:
    """Per-map JSON persistence for user annotations.

    One file per map: `<base_dir>/<map_id>.json` (map_id sanitized with the
    same rule as the object store so the two partitions always agree). The
    file records `{map_id, generation, annotations}`; on load, a stored
    generation that differs from the current binding's means the map frame
    was rebuilt since the annotations were drawn — every annotation is
    marked stale (kept, never dropped: user assets, unlike perceived
    objects, are never cleaned up automatically). When either side
    has no generation (static binding, pre-P2 file) staleness cannot be
    judged and the stored value is preserved for the next boot that can.

    All methods are synchronous and guarded by an internal lock; writes are
    atomic (tmp + os.replace) and happen on every mutation — annotation
    traffic is human-speed, so write-through is cheap and never loses an
    accepted edit."""

    def __init__(self, base_dir: str, *, map_id: str, generation: Optional[int] = None) -> None:
        """Open (creating dirs as needed) the store for `map_id` and load
        its file, applying the generation staleness check described on the
        class. A corrupt file is set aside (renamed `*.corrupt-<ts>`) and
        the store starts empty — loudly, and without destroying the bytes."""
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

    # ── load / save ──────────────────────────────────────────────────────
    def _load(self, current_generation: Optional[int]) -> None:
        """Read the per-map file into memory. Missing file → empty store.
        A file that fails to parse, has an unexpected shape, or contains a
        record that would not pass `validate_annotation_fields` (the file
        is a user-visible JSON asset, so hand edits happen) is treated as
        corrupt as a whole: renamed aside + empty store — the user's bytes
        are never dropped or overwritten by a failed load. A stored
        generation differing from `current_generation` marks everything
        stale and persists the flag."""
        if not self._path.exists():
            return
        try:
            data = json.loads(self._path.read_text(encoding="utf-8"))
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
        except (ValueError, TypeError, KeyError, AttributeError) as e:
            aside = self._path.with_suffix(f".json.corrupt-{int(time.time())}")
            # Rename failures re-raise: continuing with an empty store
            # would let the next accepted edit os.replace() the corrupt
            # file — destroying exactly the bytes this path promises to
            # keep. Failing __init__ disables the API loudly instead.
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
        __init__). tmp file lands in the same directory so os.replace stays
        on one filesystem. Raises on I/O failure — callers accepted a user
        edit, losing it silently is worse than a 500. Synchronous file I/O
        on the event loop is deliberate: annotation traffic is human-speed
        and the files are a few KB."""
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

    # ── read ─────────────────────────────────────────────────────────────
    # list()/get() hand out the LIVE Annotation objects — treat them as
    # read-only. Mutating them directly bypasses both the lock and
    # persistence; go through update() instead.
    def list(self) -> list[Annotation]:
        with self._lock:
            return list(self._annotations.values())

    def get(self, annotation_id: str) -> Optional[Annotation]:
        with self._lock:
            return self._annotations.get(annotation_id)

    def list_json(self) -> list[dict]:
        """JSON-ready dump of every annotation (the `/api/state` field and
        `GET /api/annotations` share this shape)."""
        with self._lock:
            return [a.to_json() for a in self._annotations.values()]

    # ── mutate (each call persists before returning) ─────────────────────
    def create(self, *, kind: str, name: str, points: list,
               theta: Optional[float] = None) -> Annotation:
        """Create + persist a new annotation and return it. Fields must
        already be validated (validate_annotation_fields) — the store
        trusts its callers and only owns id/timestamp assignment."""
        now = time.time()
        ann = Annotation(
            annotation_id=f"anno.{uuid.uuid4().hex[:8]}",
            kind=kind,
            name=name,
            points=[[float(x), float(y)] for x, y in points],
            theta=None if theta is None else float(theta),
            created_at=now,
            updated_at=now,
        )
        with self._lock:
            self._annotations[ann.annotation_id] = ann
            self._save_locked()
        return ann

    def update(self, annotation_id: str, *, name: Optional[str] = None,
               points: Optional[list] = None, theta: Optional[float] = None,
               clear_stale: bool = False) -> Optional[Annotation]:
        """Apply the provided fields to an existing annotation (None = keep),
        bump `updated_at`, persist, and return it; None when the id is
        unknown. `clear_stale` is the user's "confirm still valid" action —
        it resets both the flag and the recorded reason. Redrawing the
        points also clears staleness: new geometry is authored against the
        CURRENT map frame, so the old epoch's doubt no longer applies."""
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
        """Shared body of the two stale-sweep entry points (caller holds
        the lock, or is __init__/_load): flag every non-stale annotation
        with `reason`, optionally adopt `new_generation`, persist when
        anything changed. Returns how many annotations flipped."""
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
        """Called when the map's true generation becomes known at runtime
        (the lifecycle watcher's confirmation) — a store built under a
        static binding (generation None) cannot judge staleness at load
        time, so the judgement happens here instead. Adopts the epoch when
        the store had none; when the recorded epoch differs, flags
        everything stale (user assets are never auto-deleted, only
        flagged). Runs decision + sweep under one lock acquisition, so a
        concurrent create() can never be flagged by a sweep it postdates.
        Returns how many flipped."""
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
        """Flag every non-stale annotation stale with `reason` (map frame
        epoch changed at runtime — the _lifecycle_watch hook). Optionally
        records the new generation so the file reflects the epoch the
        staleness was judged against. Returns how many flipped; persists
        when anything changed (flag or generation)."""
        with self._lock:
            return self._flag_all_stale_locked(reason, new_generation)
