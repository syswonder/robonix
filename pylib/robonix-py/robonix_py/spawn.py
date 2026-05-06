# SPDX-License-Identifier: MulanPSL-2.0
"""Subprocess helper used by Capability.spawn(). Tracks every spawned PGID so
Capability's signal handler can SIGTERM all of them on shutdown."""
from __future__ import annotations

import logging
import os
import signal
import subprocess
from pathlib import Path
from typing import Sequence

log = logging.getLogger("robonix_py.spawn")


class SpawnRegistry:
    """Process-wide registry. One per Capability instance is fine."""

    def __init__(self) -> None:
        self._procs: list[subprocess.Popen] = []

    def spawn(
        self,
        argv: Sequence[str],
        *,
        env: dict[str, str] | None = None,
        log_path: Path | None = None,
        cwd: Path | None = None,
    ) -> subprocess.Popen:
        """Popen + start_new_session=True (own process group, easier cleanup) +
        optional stdout/stderr redirected to a file."""
        merged_env = dict(os.environ)
        if env:
            merged_env.update({k: str(v) for k, v in env.items()})

        out = subprocess.DEVNULL
        err = subprocess.DEVNULL
        if log_path is not None:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_fh = open(log_path, "ab", buffering=0)
            out = log_fh
            err = log_fh

        log.info("spawn %s%s", " ".join(argv), f" → {log_path}" if log_path else "")
        proc = subprocess.Popen(
            list(argv),
            env=merged_env,
            cwd=str(cwd) if cwd else None,
            stdout=out, stderr=err,
            start_new_session=True,
        )
        self._procs.append(proc)
        return proc

    def shutdown_all(self, term_grace_s: float = 5.0) -> None:
        """SIGTERM every tracked PGID; SIGKILL anything that survives the grace period."""
        for p in list(self._procs):
            if p.poll() is not None:
                continue
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except ProcessLookupError:
                continue
        # Wait for graceful exits.
        for p in list(self._procs):
            if p.poll() is not None:
                continue
            try:
                p.wait(timeout=term_grace_s)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._procs.clear()
