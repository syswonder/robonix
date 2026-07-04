"""Scribe-format logging for the memory service.

Produces disk-format JSON-lines logs matching the Scribe Log spec (§6):
  {"ts":1765432100123456789,"lvl":"I","tag":"service_memory","msg":"...","sid":"sess-42","pid":"plan-7","kv":{...}}

Also renders a logcat-style console line:
  06-11 10:23:45.123  I service_memory   message text here

Usage:
    from memory_service.scribe_log import setup_scribe_logging
    log = setup_scribe_logging(log_dir="logs", tag="service_memory")
    log.info("something happened")
    log.warning("something concerning")
    log.error("something broke")
"""

from __future__ import annotations

import datetime
import json
import logging
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

# Single-character level mapping (Scribe format)
_LVL_MAP: Dict[int, str] = {
    logging.DEBUG: "D",
    logging.INFO: "I",
    logging.WARN: "W",
    logging.ERROR: "E",
    logging.CRITICAL: "F",
}


class ScribeLogHandler(logging.Handler):
    """Scribe disk-format JSON-lines log file handler.

    One log file per process instance. Format:
      {"ts":<nanosecond>,"lvl":"I","tag":"<tag>","msg":"<message>","sid":...,"pid":...,"kv":{...}}
    """

    def __init__(self, log_dir: str, tag: str = "service_memory"):
        super().__init__()
        self._tag = tag
        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._file = None
        self._open_file()

    def _open_file(self) -> None:
        """Open (or reopen) the log file for appending."""
        fname = f"{self._tag}.log"
        path = self._log_dir / fname
        self._file = open(str(path), "a", encoding="utf-8")

    def emit(self, record: logging.LogRecord) -> None:
        try:
            ts = record.created  # seconds since epoch (float)
            ts_ns = int(ts * 1_000_000_000)

            entry: Dict[str, Any] = {
                "ts": ts_ns,
                "lvl": _LVL_MAP.get(record.levelno, "I"),
                "tag": self._tag,
                "msg": self.format(record),
            }

            # Optional context from LogRecord extra fields (if passed via `extra=`)
            sid = getattr(record, "sid", None)
            pid = getattr(record, "pid", None)
            cid = getattr(record, "cid", None)
            kv = getattr(record, "kv", None)

            if sid:
                entry["sid"] = sid
            if pid:
                entry["pid"] = pid
            if cid:
                entry["cid"] = cid
            if kv:
                entry["kv"] = kv

            self._file.write(json.dumps(entry, ensure_ascii=False) + "\n")
            self._file.flush()
        except Exception:
            self.handleError(record)

    def close(self) -> None:
        if self._file:
            self._file.close()
            self._file = None
        super().close()


class ScribeConsoleFormatter(logging.Formatter):
    """Logcat-style console format: `MM-DD HH:MM:SS.uuu  L tag   msg`"""

    def format(self, record: logging.LogRecord) -> str:
        dt = datetime.datetime.fromtimestamp(record.created)
        ts_str = dt.strftime("%m-%d %H:%M:%S") + f".{int(dt.microsecond / 1000):03d}"
        lvl = _LVL_MAP.get(record.levelno, "I")
        tag = getattr(record, "scribe_tag", "scribe_mem")
        msg = record.getMessage()
        # logcat style: left-align tag in 15-char field
        return f"{ts_str}  {lvl} {tag:<15s}  {msg}"


def setup_scribe_logging(
    log_dir: str = "logs",
    tag: str = "service_memory",
    level: str = "INFO",
) -> logging.Logger:
    """Configure Scribe-format logging for the memory service.

    Sets up:
      - Console: logcat-style formatted output (human-readable)
      - File:   Scribe disk JSON-lines under `<log_dir>/<tag>.log`

    Returns the root logger for the memory service.
    """
    log_level = getattr(logging, level.upper(), logging.INFO)

    # Root logger for the package
    root_logger = logging.getLogger("scribe_mem")
    root_logger.setLevel(log_level)
    root_logger.handlers.clear()  # Remove any pre-existing handlers

    # Console handler — logcat style
    console = logging.StreamHandler(sys.stdout)
    console.setLevel(log_level)
    console.setFormatter(ScribeConsoleFormatter())
    root_logger.addHandler(console)

    # File handler — Scribe disk JSON
    file_handler = ScribeLogHandler(log_dir=log_dir, tag=tag)
    file_handler.setLevel(log_level)
    file_handler.setFormatter(logging.Formatter("%(message)s"))
    root_logger.addHandler(file_handler)

    return root_logger
