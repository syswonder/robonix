# SPDX-License-Identifier: MulanPSL-2.0
"""Tests for bounded provider-to-Atlas lifecycle state reporting."""

from types import SimpleNamespace

from robonix_api import _lifecycle_internal
from robonix_api.atlas_types import LifecycleState


class _Request:
    """Minimal SetLifecycleState request used by the fake wire module."""

    def __init__(self, **fields) -> None:
        self.fields = fields


class _Stub:
    """Record the deadline supplied to the lifecycle state RPC."""

    def __init__(self, *, fail: bool = False) -> None:
        self.fail = fail
        self.timeouts: list[float] = []

    def SetLifecycleState(self, request, *, timeout):  # noqa: N802
        """Capture the deadline and optionally emulate a transport timeout."""
        self.timeouts.append(timeout)
        if self.fail:
            raise TimeoutError("lost response")
        return request


def _atlas(stub: _Stub) -> SimpleNamespace:
    """Build the private Atlas surface used by lifecycle state reporting."""
    return SimpleNamespace(
        _ensure_stub=lambda: None,
        _wire_pb=SimpleNamespace(SetLifecycleStateRequest=_Request),
        _wire_stub=stub,
    )


def test_state_push_has_a_deadline(monkeypatch) -> None:
    """A lifecycle handler must never make an unbounded Atlas RPC."""
    stub = _Stub()
    monkeypatch.setattr(_lifecycle_internal, "ATLAS", _atlas(stub))

    _lifecycle_internal._set_lifecycle_state(
        "explore", LifecycleState.INACTIVE
    )

    assert stub.timeouts == [_lifecycle_internal._STATE_PUSH_TIMEOUT_S]


def test_state_push_timeout_does_not_block_driver_response(monkeypatch, caplog) -> None:
    """A lost Atlas response is logged and contained by lifecycle plumbing."""
    stub = _Stub(fail=True)
    monkeypatch.setattr(_lifecycle_internal, "ATLAS", _atlas(stub))

    _lifecycle_internal._set_lifecycle_state(
        "explore", LifecycleState.INACTIVE
    )

    assert stub.timeouts == [_lifecycle_internal._STATE_PUSH_TIMEOUT_S]
    assert "lost response" in caplog.text
