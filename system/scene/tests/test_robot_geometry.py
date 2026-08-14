# SPDX-License-Identifier: MulanPSL-2.0
import asyncio
from types import SimpleNamespace

import pytest

from scene_service import robot_geometry
from scene_service.robot_geometry import (
    RobotGeometryState,
    reconcile_robot_geometry,
    validate_footprint_response,
)


def _response(points, base_frame="fixture_base", inscribed=0.2, circumscribed=0.3):
    """Build the generated-response subset used by the validation boundary."""
    return SimpleNamespace(
        points=[SimpleNamespace(x=x, y=y) for x, y in points],
        base_frame=base_frame,
        inscribed_radius_m=inscribed,
        circumscribed_radius_m=circumscribed,
    )


def test_validate_footprint_response_preserves_soma_geometry():
    footprint = validate_footprint_response(
        _response([(-0.2, -0.1), (0.2, -0.1), (0.2, 0.1), (-0.2, 0.1)]),
        provider_id="fixture_soma",
    )
    assert footprint.base_frame == "fixture_base"
    assert footprint.provider_id == "fixture_soma"
    assert footprint.size_x_m == pytest.approx(0.4)
    assert footprint.size_y_m == pytest.approx(0.2)


def test_geometry_state_discards_stale_soma_footprint():
    footprint = validate_footprint_response(
        _response([(-0.2, -0.1), (0.2, -0.1), (0.2, 0.1), (-0.2, 0.1)])
    )
    state = RobotGeometryState()
    assert state.update(footprint)
    assert state.current() == footprint
    assert state.clear()
    assert state.current() is None
    assert not state.clear()


def test_reconciliation_clears_footprint_when_soma_is_unavailable(monkeypatch):
    footprint = validate_footprint_response(
        _response([(-0.2, -0.1), (0.2, -0.1), (0.2, 0.1), (-0.2, 0.1)])
    )
    state = RobotGeometryState()
    state.update(footprint)

    async def stop_after_reconciliation(_delay):
        raise asyncio.CancelledError

    monkeypatch.setattr(robot_geometry.asyncio, "sleep", stop_after_reconciliation)
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(reconcile_robot_geometry(state, fetcher=lambda: None))
    assert state.current() is None


@pytest.mark.parametrize(
    "response",
    [
        _response([(0.0, 0.0), (1.0, 0.0)]),
        _response([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]),
        _response([(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)], base_frame=""),
        _response(
            [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)],
            inscribed=0.4,
            circumscribed=0.3,
        ),
    ],
)
def test_validate_footprint_response_rejects_ambiguous_geometry(response):
    with pytest.raises(ValueError):
        validate_footprint_response(response)
