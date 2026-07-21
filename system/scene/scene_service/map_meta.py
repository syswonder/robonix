# SPDX-License-Identifier: MulanPSL-2.0
"""Scene-owned per-map semantic snapshot metadata (the epoch sidecar).

mapping owns a saved map's spatial artifact; scene owns the matching semantic
snapshot (objects + room annotations). The sidecar records, per `map_id`,
WHICH object partition holds the snapshot written by the Save that produced
the artifact on disk. Every Save allocates a fresh partition token
(`"<map_id>__s<seq>"`) and repoints the sidecar at it, so:

- Load restores exactly the rows written together with the loaded artifact —
  never rows from an earlier build of a same-named map, whose map frame no
  longer exists (the mis-anchored / off-map restore bug).
- A map with NO sidecar (saved before this mechanism, or a foreign DB)
  restores nothing rather than coordinates in a dead frame.

One JSON file per map under `base_dir`, written atomically (tmp + replace).
`mapping_generation` / `mapping_mode` are recorded for diagnostics when the
lifecycle broadcast is available at save time; the mechanism itself never
depends on them.
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Optional

from .map_binding import sanitize_map_id

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class MapSemanticMeta:
    """Sidecar record for one saved map's semantic snapshot."""
    map_id: str
    object_partition: str        # milvus partition holding the snapshot rows
    save_seq: int                # monotonic per map; allocates the partition
    saved_at_unix: float
    mapping_generation: Optional[int] = None  # broadcast gen at save (diagnostic)
    mapping_mode: str = ""                    # broadcast mode at save (diagnostic)

    def to_json(self) -> dict:
        return asdict(self)

    @classmethod
    def from_json(cls, d: dict) -> "MapSemanticMeta":
        known = set(cls.__dataclass_fields__)
        return cls(**{k: v for k, v in d.items() if k in known})


class MapMetaStore:
    """Per-map sidecar files: `<base_dir>/<map_id>.json`.

    All methods are synchronous; the internal lock guards individual file
    ops only — the `next_partition` → persist rows → `write` sequence is NOT
    atomic here, and callers serialize whole Save/Load/Delete operations
    (the web facade's `ops_lock`). A file that cannot be parsed is treated
    as absent with a WARNING: restoring nothing is the safe outcome, and the
    bytes are left in place for inspection."""

    def __init__(self, base_dir: str) -> None:
        self._lock = threading.Lock()
        self._base = Path(base_dir).expanduser()
        self._base.mkdir(parents=True, exist_ok=True)

    def _path(self, map_id: str) -> Path:
        return self._base / f"{sanitize_map_id(map_id)}.json"

    def read(self, map_id: str) -> Optional[MapSemanticMeta]:
        """The sidecar for `map_id`, or None when absent/unreadable. A
        malformed record must never crash a Load — it degrades to
        "no snapshot", which restores nothing. "Malformed" includes an
        identity mismatch: a parseable record naming ANOTHER map (or a
        partition token outside this map's `<id>__s<seq>` namespace) would
        let Load restore a different map's objects into this one, so it is
        treated as corrupt rather than trusted."""
        clean = sanitize_map_id(map_id)
        path = self._path(map_id)
        with self._lock:
            if not path.exists():
                return None
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                meta = MapSemanticMeta.from_json(data)
                if not meta.object_partition:
                    raise ValueError("empty object_partition")
                if sanitize_map_id(meta.map_id) != clean:
                    raise ValueError(
                        f"sidecar identity is {meta.map_id!r}, not {map_id!r}"
                    )
                seq = int(meta.save_seq)
                if seq < 1:
                    raise ValueError(f"invalid save_seq {meta.save_seq!r}")
                expected_partition = f"{clean}__s{seq}"
                if meta.object_partition != expected_partition:
                    raise ValueError(
                        f"partition {meta.object_partition!r} is not this "
                        f"map's save token {expected_partition!r}"
                    )
                # Return NORMALIZED numeric fields: a foreign sidecar with
                # save_seq "2"/2.0 passes the checks above, but the raw
                # value would then poison downstream arithmetic —
                # `next_partition`'s `save_seq + 1` (TypeError escaping as
                # a 500 mid-Save) or a minted "…__s3.0" token no later
                # read would accept. Same for mapping_generation, which
                # feeds integer comparisons in annotation staleness.
                return replace(
                    meta,
                    save_seq=seq,
                    mapping_generation=(
                        None if meta.mapping_generation is None
                        else int(meta.mapping_generation)
                    ),
                )
            except Exception as e:  # noqa: BLE001
                log.warning(
                    "[scene-mapmeta] unreadable sidecar %s (%s) — treating as "
                    "absent; the map's objects will not be restored", path, e,
                )
                return None

    def next_partition(self, map_id: str) -> tuple[str, int]:
        """The partition token for the NEXT snapshot of `map_id`, without
        touching the sidecar (the caller commits via `write` only after the
        snapshot rows are safely persisted). The sequence advances only on
        commit, so a FAILED attempt's token is handed out again — callers
        clear that partition before writing into it."""
        clean = sanitize_map_id(map_id)
        prev = self.read(clean)
        seq = (prev.save_seq + 1) if prev is not None else 1
        return f"{clean}__s{seq}", seq

    def write(self, meta: MapSemanticMeta) -> None:
        """Atomically persist `meta` as its map's sidecar. Raises on I/O
        failure — a Save must not report success while the sidecar still
        points at the previous snapshot."""
        path = self._path(meta.map_id)
        tmp = path.with_suffix(".json.tmp")
        with self._lock:
            tmp.write_text(
                json.dumps(meta.to_json(), ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            os.replace(tmp, path)

    def delete(self, map_id: str) -> bool:
        """Remove `map_id`'s sidecar (map deletion). True when one existed."""
        path = self._path(map_id)
        with self._lock:
            existed = path.exists()
            if existed:
                path.unlink()
            return existed


def make_meta(map_id: str, partition: str, seq: int, *,
              generation: Optional[int] = None, mode: str = "") -> MapSemanticMeta:
    """Convenience constructor stamping `saved_at_unix` now."""
    return MapSemanticMeta(
        map_id=sanitize_map_id(map_id),
        object_partition=partition,
        save_seq=seq,
        saved_at_unix=time.time(),
        mapping_generation=generation,
        mapping_mode=mode,
    )
