# SPDX-License-Identifier: MulanPSL-2.0

from collections.abc import Callable
from contextvars import ContextVar

_tts_request_active: ContextVar[Callable[[], bool] | None] = ContextVar(
    "tts_request_active", default=None
)


def bind_tts_request_active(callback: Callable[[], bool] | None):
    """Bind an optional transport-liveness callback to the current synthesis."""
    return _tts_request_active.set(callback)


def reset_tts_request_active(token) -> None:
    """Restore the liveness callback that preceded this synthesis."""
    _tts_request_active.reset(token)


def tts_request_is_active() -> bool:
    """Return true outside an RPC or while its transport remains active."""
    callback = _tts_request_active.get()
    return callback is None or callback()
