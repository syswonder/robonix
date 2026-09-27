# SPDX-License-Identifier: MulanPSL-2.0
"""Per-map sidecar naming the object partition of the map's last Save.

Every Save writes its objects under a fresh token (`"<map_id>__s<seq>"`) and
then repoints the sidecar at it, so Load restores exactly the rows saved with
the loaded spatial map. A map without a sidecar restores nothing.
"""
from __future__ import annotations

import base64
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
    """`<base_dir>/<map_id>.json`. The lock covers single file ops; callers
    serialize whole Save/Load/Delete sequences."""

    def __init__(self, base_dir: str) -> None:
        self._lock = threading.Lock()
        self._base = Path(base_dir).expanduser()
        self._base.mkdir(parents=True, exist_ok=True)

    def _path(self, map_id: str) -> Path:
        return self._base / f"{sanitize_map_id(map_id)}.json"

    def read(self, map_id: str) -> Optional[MapSemanticMeta]:
        """The sidecar, or None when absent or malformed. A record naming
        another map or a token outside `<id>__s<seq>` counts as malformed:
        trusting it would restore another map's objects."""
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
                # Normalized: "2" or 2.0 would break `save_seq + 1` later.
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
        """The next Save's token. The sequence only advances on `write`, so a
        failed Save's token is handed out again; callers clear it first."""
        clean = sanitize_map_id(map_id)
        prev = self.read(clean)
        seq = (prev.save_seq + 1) if prev is not None else 1
        return f"{clean}__s{seq}", seq

    def write(self, meta: MapSemanticMeta) -> None:
        """Atomic replace; raises on I/O failure so a Save cannot succeed
        while the sidecar points at the previous snapshot."""
        path = self._path(meta.map_id)
        tmp = path.with_suffix(".json.tmp")
        with self._lock:
            tmp.write_text(
                json.dumps(meta.to_json(), ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            os.replace(tmp, path)

    def delete(self, map_id: str) -> bool:
        """Remove `map_id`'s sidecar and preview (map deletion). True when a
        sidecar existed."""
        path = self._path(map_id)
        with self._lock:
            existed = path.exists()
            for p in (path, *self._preview_paths(map_id)):
                p.unlink(missing_ok=True)
            return existed

    def _preview_paths(self, map_id: str) -> tuple[Path, Path]:
        clean = sanitize_map_id(map_id)
        return self._base / f"{clean}.png", self._base / f"{clean}.grid.json"

    def write_preview(self, map_id: str, occupancy: dict) -> None:
        """Keep the grid as saved, so the map library can show it without
        reaching into mapping's files."""
        png, grid = self._preview_paths(map_id)
        geometry = {k: occupancy[k] for k in
                    ("width", "height", "resolution", "origin_x", "origin_y")}
        with self._lock:
            png.write_bytes(base64.b64decode(occupancy["png_b64"]))
            grid.write_text(json.dumps(geometry), encoding="utf-8")

    def read_preview(self, map_id: str) -> tuple[Optional[bytes], Optional[dict]]:
        """The saved grid image and its geometry, each None when absent."""
        png, grid = self._preview_paths(map_id)
        try:
            image = png.read_bytes()
        except OSError:
            image = None
        try:
            geometry = json.loads(grid.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            geometry = None
        return image, geometry


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
