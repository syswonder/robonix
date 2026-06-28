# SPDX-License-Identifier: MulanPSL-2.0
"""scribe_logger — Python-side unified logging facade.

Mirrors `system/scribe/src/lib.rs` semantics: single ``log()`` entry
point, lazy init, dual sink (stderr + per-tag JSON-lines file).

Usage::

    from robonix_api import scribe_logger

    scribe_logger.info("atlas", "control plane ready")
    scribe_logger.log(scribe_logger.Level.WARN, "my_driver", "sensor timeout")
"""

from __future__ import annotations

import json
import logging
import os
import sys
import threading
import time
from enum import Enum
from pathlib import Path
from typing import Dict, Optional, TextIO  # noqa: UP035


class Level(Enum):
    """Severity, ordered DEBUG < INFO < WARN < ERROR."""

    DEBUG = "D"
    INFO = "I"
    WARN = "W"
    ERROR = "E"


# ── layout constants (match Rust format.rs) ─────────────────────────

_TAG_WIDTH = 24

# ── global lazy state ────────────────────────────────────────────────

_lock: threading.Lock = threading.Lock()
_log_dir: Optional[Path] = None
_writers: Dict[str, TextIO] = {}


def _ensure_init() -> None:
    """Lazily initialise log directory and internal state on first call."""
    global _log_dir  # noqa: PLW0603
    if _log_dir is not None:
        return
    with _lock:
        if _log_dir is not None:
            return
        _log_dir = Path(os.environ.get("SCRIBE_LOG_DIR", "./logs"))
        _log_dir.mkdir(parents=True, exist_ok=True)


def _decompose_ts(ts_ns: int) -> tuple[int, int, int, int, int, int]:
    """Nanosecond UNIX timestamp → (month, day, hour, min, sec, ms)."""
    secs = ts_ns // 1_000_000_000
    nanos_rem = ts_ns % 1_000_000_000
    ms = nanos_rem // 1_000_000

    # localtime
    t = time.localtime(secs)
    return (t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec, ms)


def _format_console(ts_ns: int, level: Level, tag: str, msg: str) -> str:
    """Render a logcat-style console line (matches Rust format.rs)."""
    month, day, hour, minute, sec, ms = _decompose_ts(ts_ns)
    tag_display = tag[: _TAG_WIDTH - 1] + "…" if len(tag) > _TAG_WIDTH else tag.ljust(_TAG_WIDTH)
    return (
        f"{month:02}-{day:02} {hour:02}:{minute:02}:{sec:02}.{ms:03}"
        f"  {level.value} {tag_display} {msg}\n"
    )


def _ts_fmt(ts_ns: int) -> str:
    """Nanosecond UNIX timestamp → 'YYYY-MM-DD HH:MM:SS.nnnnnnnnn' (local time)."""
    secs, nsec = divmod(ts_ns, 1_000_000_000)
    t = time.localtime(secs)
    return (
        f"{t.tm_year:04}-{t.tm_mon:02}-{t.tm_mday:02} "
        f"{t.tm_hour:02}:{t.tm_min:02}:{t.tm_sec:02}.{nsec:09}"
    )


def _format_json(ts_ns: int, level: Level, tag: str, msg: str) -> str:
    """Render a JSON line (matches Rust LogRecord serialisation)."""
    rec = {
        "ts": _ts_fmt(ts_ns),
        "level": level.name.lower(),
        "tag": tag,
        "msg": msg,
    }
    return json.dumps(rec, ensure_ascii=False)


def _sanitize_tag(tag: str) -> str:
    """Replace non-safe characters to prevent path traversal."""
    return "".join(c if c.isascii() and (c.isalnum() or c in "._-") else "_" for c in tag)


def _get_writer(tag: str) -> Optional[TextIO]:
    """Return (creating if needed) the append-mode file handle for *tag*.

    Returns ``None`` if the log directory is unwritable or the file
    can't be opened — the caller treats file writes as best-effort.
    """
    global _writers  # noqa: PLW0603
    if tag not in _writers:
        safe = _sanitize_tag(tag)
        path = _log_dir / f"{safe}.log"
        try:
            _writers[tag] = open(str(path), "a", encoding="utf-8")  # noqa: SIM115
        except OSError:
            _writers[tag] = None  # type: ignore[assignment]
    return _writers[tag]


# ── public API ───────────────────────────────────────────────────────


def log(level: Level, tag: str, msg: str) -> None:
    """Log one record — the **only** entry point.

    The first call transparently creates ``$SCRIBE_LOG_DIR`` (default
    ``./logs``).  Errors are best-effort (stderr / disk full are
    silently ignored).
    """
    _ensure_init()

    ts_ns = time.time_ns()

    # console → stderr
    try:
        console_line = _format_console(ts_ns, level, tag, msg)
        sys.stderr.write(console_line)
        sys.stderr.flush()
    except Exception:  # noqa: BLE001
        pass

    # file → $SCRIBE_LOG_DIR/{tag}.log
    try:
        json_line = _format_json(ts_ns, level, tag, msg)
        with _lock:
            writer = _get_writer(tag)
            if writer is not None:
                writer.write(json_line + "\n")
                writer.flush()
    except Exception:  # noqa: BLE001
        pass


def debug(tag: str, msg: str) -> None:
    """Convenience: :attr:`Level.DEBUG`."""
    log(Level.DEBUG, tag, msg)


def info(tag: str, msg: str) -> None:
    """Convenience: :attr:`Level.INFO`."""
    log(Level.INFO, tag, msg)


def warn(tag: str, msg: str) -> None:
    """Convenience: :attr:`Level.WARN`."""
    log(Level.WARN, tag, msg)


def error(tag: str, msg: str) -> None:
    """Convenience: :attr:`Level.ERROR`."""
    log(Level.ERROR, tag, msg)


# ── stdlib `logging` → Scribe bridge ─────────────────────────────────

_STDLIB_LEVEL_MAP: Dict[int, Level] = {
    logging.DEBUG: Level.DEBUG,
    logging.INFO: Level.INFO,
    logging.WARNING: Level.WARN,
    logging.ERROR: Level.ERROR,
    logging.CRITICAL: Level.ERROR,
}


class _StdlibBridgeHandler(logging.Handler):
    """A stdlib ``logging.Handler`` that re-emits every record into Scribe.

    Carries a fixed component *tag* so any library's logging (the package's
    own, plus transitive deps that use stdlib ``logging``) lands in that
    component's ``rbnx logs -t <tag>`` stream. Best-effort: a failure to
    forward never propagates back into the caller's logging path.
    """

    def __init__(self, tag: str) -> None:
        super().__init__()
        self._tag = tag

    def emit(self, record: logging.LogRecord) -> None:
        try:
            level = _STDLIB_LEVEL_MAP.get(record.levelno, Level.INFO)
            log(level, self._tag, record.getMessage())
        except Exception:  # noqa: BLE001
            pass


def install_stdlib_bridge(
    tag: str,
    *,
    level: int = logging.INFO,
    logger: Optional[logging.Logger] = None,
    replace_existing_handlers: bool = True,
) -> logging.Logger:
    """Route stdlib ``logging`` through Scribe so packages need no own sink.

    Attaches a :class:`_StdlibBridgeHandler` (tagged *tag*) to *logger* (the
    root logger by default, so transitive-dependency logs are captured too) and
    raises its level to *level*. When *replace_existing_handlers* is true,
    every other handler already on that logger — ``StreamHandler`` (stdout/
    stderr), ``FileHandler`` (a per-package log file), a ``rich`` handler from
    fastmcp/uvicorn, anything — is removed first, so the bridge is the *only*
    sink. This enforces the repo rule "no package owns a log file or duplicate
    stdout sink; everything goes through Scribe" (Scribe already mirrors to
    stderr itself, so console output is not lost).

    Idempotent and re-taggable: exactly one bridge handler ever exists on the
    logger, and the most recent call's *tag* wins (so the framework can re-tag
    with the provider id at bootstrap after a package set a provisional tag at
    import). Returns the configured logger.
    """
    target = logger if logger is not None else logging.getLogger()
    target.setLevel(level)

    for handler in list(target.handlers):
        if isinstance(handler, _StdlibBridgeHandler):
            # Drop any prior bridge so the new tag replaces the old one.
            target.removeHandler(handler)
        elif replace_existing_handlers:
            target.removeHandler(handler)

    target.addHandler(_StdlibBridgeHandler(tag))
    return target
