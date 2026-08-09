# SPDX-License-Identifier: MulanPSL-2.0
"""Return types for lifecycle handlers.

Every ``on_init``, ``on_activate``, ``on_deactivate``, and ``on_shutdown``
handler returns one of the following values:

* :class:`Ok`: the handler succeeded and the framework advances state.
* :class:`Err`: the handler failed and the provider enters ``ERROR``.
* :class:`Deferred`: the handler cannot proceed and the provider remains in
  its current state. Version 0.1 reports the reason but has no deferred queue.

Only ``on_init`` receives the decoded ``cfg`` dictionary. Raising an exception
is converted defensively to :class:`Err`, but returning :class:`Err` explicitly
produces clearer diagnostics.
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
