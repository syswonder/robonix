# SPDX-License-Identifier: MulanPSL-2.0
"""One JSON line per model request, for usage metrics and later billing.

Records go to ``$ROBONIX_LLM_METRICS_DIR/<consumer>-<pid>.jsonl``; nothing is
written when the variable is unset. Prompts, replies and keys are never
recorded, and the endpoint is reduced to its host. The Rust crate
``robonix-llm-metrics`` writes the same record; both parse usage with the
cases in ``contract/usage_cases.json``.
"""
from __future__ import annotations

import json
import logging
import os
import re
import threading
import time
from pathlib import Path
from typing import Any, Optional

log = logging.getLogger(__name__)

SCHEMA_VERSION = 1
DIRECTORY_ENV = "ROBONIX_LLM_METRICS_DIR"
_SLUG = re.compile(r"[a-z0-9_-]+")
_lock = threading.Lock()
_warned = False
_announced = False


def usage_from_response(body: Any) -> dict[str, Any]:
    """Token counts from a Chat Completions body or stream chunk; absent ones are None."""
    usage = body.get("usage") if isinstance(body, dict) else None
    if not isinstance(usage, dict):
        usage = {}
    prompt = usage.get("prompt_tokens_details")
    completion = usage.get("completion_tokens_details")
    cached = prompt.get("cached_tokens") if isinstance(prompt, dict) else None
    if cached is None:
        # DeepSeek reports cache hits outside prompt_tokens_details.
        cached = usage.get("prompt_cache_hit_tokens")
    model = body.get("model") if isinstance(body, dict) else None
    return {
        "input_tokens": _count(usage.get("prompt_tokens")),
        "output_tokens": _count(usage.get("completion_tokens")),
        "cached_input_tokens": _count(cached),
        "reasoning_tokens": _count(
            completion.get("reasoning_tokens") if isinstance(completion, dict) else None
        ),
        "provider_model": model if isinstance(model, str) and model else None,
    }


def _count(value: Any) -> Optional[int]:
    return value if type(value) is int and value >= 0 else None


def _host(base_url: str) -> str:
    """Authority without scheme, credentials or path; the same string the Rust writer keeps."""
    rest = base_url.split("://", 1)[1] if "://" in base_url else (base_url or "")
    authority = re.split(r"[/?#]", rest, maxsplit=1)[0]
    return authority.rsplit("@", 1)[-1].lower()


class Call:
    """One model request; use as a context manager around the HTTP exchange.

    ``finish`` stores the outcome. Leaving the block without it records the
    exception kind (``timeout``, ``cancelled`` or ``transport_error``), so a
    failed request is still counted: providers can bill those too.
    """

    def __init__(
        self,
        consumer: str,
        operation: str,
        base_url: str,
        model: str,
        *,
        stream: bool = False,
    ) -> None:
        if not _SLUG.fullmatch(consumer):
            raise ValueError(f"llm metrics consumer {consumer!r} must match [a-z0-9_-]+")
        self.record: dict[str, Any] = {
            "v": SCHEMA_VERSION,
            "consumer": consumer,
            "operation": operation,
            "host": _host(base_url),
            "model": model,
            "stream": stream,
            "attempts": 1,
        }
        self._started = 0.0
        self._done = False

    def __enter__(self) -> "Call":
        self._started = time.monotonic()
        return self

    def retry(self) -> None:
        """Count one more request sent for this call, such as a retry."""
        self.record["attempts"] += 1

    def finish(self, response: Any = None) -> None:
        """Record an HTTP response (``status_code`` and ``json()``, as httpx has).

        Without a response the call is ``ok`` with unknown usage. A body that is
        not JSON leaves usage unknown; the request was still answered.
        """
        status = getattr(response, "status_code", None)
        try:
            body = response.json() if response is not None else None
        except ValueError:
            body = None
        self.record["status"] = (
            "ok" if status is None or 200 <= status < 300 else "http_error"
        )
        self.record["http_status"] = status
        self.record.update(usage_from_response(body))
        self._done = True

    def __exit__(self, exc_type, _exc, _tb) -> bool:
        if not self._done:
            name = exc_type.__name__ if exc_type else ""
            if "Timeout" in name:
                status = "timeout"
            elif name == "CancelledError":
                status = "cancelled"
            else:
                status = "transport_error"
            self.record.update(status=status, http_status=None, **usage_from_response(None))
        self.record["latency_ms"] = round((time.monotonic() - self._started) * 1000)
        write(self.record)
        return False


def write(record: dict[str, Any]) -> None:
    """Append ``record`` with timestamp, boot and process ids; never raises."""
    global _warned, _announced  # noqa: PLW0603
    directory = os.environ.get(DIRECTORY_ENV, "").strip()
    if not _announced:
        # Without this line an unset variable loses every record silently.
        _announced = True
        log.info("llm metrics: %s", f"records -> {directory}" if directory
                 else f"off ({DIRECTORY_ENV} unset)")
    if not directory:
        return
    line = dict(record, ts_ms=int(time.time() * 1000),
                boot_id=os.environ.get("RBNX_BOOT_ID") or None, pid=os.getpid())
    path = Path(directory) / f"{record['consumer']}-{os.getpid()}.jsonl"
    # "replace": a provider may name a model with a lone surrogate, which
    # UTF-8 cannot encode; dropping the record would lose the usage it reports.
    data = (json.dumps(line, ensure_ascii=False, separators=(",", ":")) + "\n").encode(
        "utf-8", "replace")
    try:
        with _lock:
            path.parent.mkdir(parents=True, exist_ok=True)
            # One write of the whole line, so a reader or a crash never sees
            # half a record.
            fd = os.open(path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
            try:
                os.write(fd, data)
            finally:
                os.close(fd)
    except (OSError, ValueError) as error:
        # A metrics failure must not fail the model call; say so once.
        with _lock:
            first = not _warned
            _warned = True
        if first:
            log.warning("llm metrics: cannot write %s: %s", path, error)
