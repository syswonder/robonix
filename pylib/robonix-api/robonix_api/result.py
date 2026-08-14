# SPDX-License-Identifier: MulanPSL-2.0
"""Lifecycle handler return type.

Every `@<provider>.on_init / on_activate / on_deactivate / on_shutdown`
handler MUST return one of `Ok` / `Err` / `Deferred`. Only `on_init`
receives `cfg: dict` (the CMD_INIT config_json, JSON-decoded); the
other three take no args. Don't `raise` — the framework will catch
and convert raises into `Err(repr(exc))` defensively but logs a
warning telling you to use `Err(...)` explicitly instead.

  Ok()                 — handler succeeded; framework advances state.
  Err("...")           — handler failed; provider goes to ERROR.
  Deferred("...")      — handler can't proceed yet; provider stays in
                         current state. v0.1 reports the reason to the
                         operator and moves on (no deferred queue).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Ok:
    """Handler succeeded. Carries no payload — the lifecycle framework
    advances state based on which CMD_* the handler ran on, not on the
    return value."""
    pass


@dataclass(frozen=True, slots=True)
class Err:
    """Handler failed. `message` shows up in `rbnx caps` state_detail
    and atlas's logs. Use this for both expected failures (config
    invalid, dependency missing) and caught exceptions."""
    message: str


@dataclass(frozen=True, slots=True)
class Deferred:
    """Handler can't proceed yet — typically waiting for an upstream
    provider to come online, a sensor to publish first message, etc.
    The provider stays in its current state; framework expects the operator
    or eviction policy to retry later. `reason` is shown in
    `rbnx caps`."""
    reason: str


Result = Ok | Err | Deferred
