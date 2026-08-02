"""PTDL Store — standalone JSON file for successful RTDL plans.

Separate from the main graph_store.json so that:
  - Plan history survives graph_store clean_start wipes
  - Plans are easily inspectable (human-readable JSON array)
  - No dependency on TagIndex / VectorStore / MCP round-trip

Format (JSON array, one entry per completed user task):
  [
    {
      "query": "向后移动2m",
      "description": "complete task (2 step(s) across 2 planning round(s))",
      "steps": [
        "1. [mapping.map_get_pose] 读取当前位姿",
        "2. [nav2.navigation_navigate] 导航到起点正后方2米处"
      ],
      "timestamp_ns": 1785600000000000000,
      "plan_count": 2,
      "canceled_count": 0
    }
  ]
"""

from __future__ import annotations

import json
import logging
import os
import time
from pathlib import Path
from typing import Any, Dict, List

log = logging.getLogger("scribe_mem")

_DEFAULT_PTDL_PATH = str(
    Path(__file__).resolve().parent.parent.parent / "memory" / "ptdl_store.json"
)


class PtdlStore:
    """Append-only JSON store for successful RTDL plan records."""

    def __init__(self, path: str = ""):
        self._path = Path(path) if path else Path(_DEFAULT_PTDL_PATH)
        self._entries: List[Dict[str, Any]] = []
        self._load()

    # ── Public API ────────────────────────────────────────────────────

    @property
    def path(self) -> str:
        return str(self._path)

    def add(
        self,
        query: str,
        description: str,
        steps: List[str],
        plan_count: int = 1,
        canceled_count: int = 0,
    ) -> None:
        """Append a completed plan record and persist immediately."""
        entry: Dict[str, Any] = {
            "query": query,
            "description": description,
            "steps": steps,
            "timestamp_ns": time.time_ns(),
            "plan_count": plan_count,
            "canceled_count": canceled_count,
        }
        self._entries.append(entry)
        self._save()
        log.info("ptdl_store: saved plan \"%s\" (%d steps)", query, len(steps))

    def list_all(self) -> List[Dict[str, Any]]:
        """Return all saved plan records (most recent last)."""
        return list(self._entries)

    def count(self) -> int:
        return len(self._entries)

    # ── Internal ──────────────────────────────────────────────────────

    def _load(self) -> None:
        if self._path.exists():
            try:
                raw = self._path.read_text(encoding="utf-8")
                self._entries = json.loads(raw) if raw.strip() else []
            except (json.JSONDecodeError, OSError) as e:
                log.warning("ptdl_store: failed to load %s: %s", self._path, e)
                self._entries = []

    def _save(self) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        tmp = str(self._path) + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._entries, f, ensure_ascii=False, indent=2)
            os.replace(tmp, str(self._path))
        except OSError as e:
            log.warning("ptdl_store: failed to save %s: %s", self._path, e)


# Module-level singleton, created on first import.
_ptdl_store: PtdlStore | None = None


def get_ptdl_store(path: str = "") -> PtdlStore:
    global _ptdl_store
    if _ptdl_store is None:
        _ptdl_store = PtdlStore(path)
    return _ptdl_store
