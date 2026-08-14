# SPDX-License-Identifier: MulanPSL-2.0
"""Framework-internal lifecycle state push to atlas.

`SetLifecycleState` is a privileged operation -- only the Robonix
framework (Primitive / Service / Skill class plumbing) should call it,
not user code. Putting it in a private module with a leading-underscore
name keeps it off `from robonix_api import ATLAS`.
"""
from __future__ import annotations

import logging
from typing import Any

from .atlas import ATLAS
from .atlas_types import LifecycleState

log = logging.getLogger("robonix_api._lifecycle_internal")


def _resolve_state(s: LifecycleState | str | int) -> LifecycleState:
    if isinstance(s, LifecycleState):
        return s
    if isinstance(s, int):
        return LifecycleState(s)
    name = str(s).strip().lower()
    return {
        "registered": LifecycleState.REGISTERED,
        "inactive":   LifecycleState.INACTIVE,
        "active":     LifecycleState.ACTIVE,
        "error":      LifecycleState.ERROR,
        "terminated": LifecycleState.TERMINATED,
    }.get(name, LifecycleState.UNSPECIFIED)


def _set_lifecycle_state(
    id: str,
    state: LifecycleState | str | int,
    detail: str = "",
) -> None:
    """Push a lifecycle state transition. Framework-only.

    Atlas-side validation is SOFT: illegal transitions are logged but
    still applied. Unknown state (UNSPECIFIED) is hard-rejected with a
    gRPC InvalidArgument; we drop those locally without raising so a
    misuse doesn't crash the provider process.
    """
    cs = _resolve_state(state)
    if cs == LifecycleState.UNSPECIFIED:
        log.warning("_set_lifecycle_state: unknown state %r, skipping", state)
        return
    ATLAS._ensure_stub()
    pb = ATLAS._wire_pb
    try:
        ATLAS._wire_stub.SetLifecycleState(pb.SetLifecycleStateRequest(
            id=id,
            state=int(cs),
            detail=detail,
        ))
    except Exception as e:  # noqa: BLE001
        # State pushes are a correctness signal — when atlas drops them
        # the local provider thinks ACTIVE but `rbnx caps` still shows
        # the old state. Warn loudly so the silent-divergence is visible
        # in default-level logs (was log.debug; silenced real outages).
        log.warning("SetLifecycleState(%s, %s) failed: %s", id, cs.name, e)
