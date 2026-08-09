# SPDX-License-Identifier: MulanPSL-2.0
"""Authoritative robot geometry resolved from Soma through Atlas."""
from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import dataclass
from typing import Callable, Optional

log = logging.getLogger(__name__)

SOMA_FOOTPRINT_CONTRACT = "robonix/system/soma/footprint"


@dataclass(frozen=True)
class RobotFootprint:
    """Validated 2D collision footprint in the Soma-declared base frame."""

    points: tuple[tuple[float, float], ...]
    base_frame: str
    inscribed_radius_m: float
    circumscribed_radius_m: float
    provider_id: str = ""
    received_at_unix: float = 0.0

    @property
    def size_x_m(self) -> float:
        xs = [point[0] for point in self.points]
        return max(xs) - min(xs)

    @property
    def size_y_m(self) -> float:
        ys = [point[1] for point in self.points]
        return max(ys) - min(ys)

    def to_json(self) -> dict:
        return {
            "points": [[x, y] for x, y in self.points],
            "base_frame": self.base_frame,
            "inscribed_radius_m": self.inscribed_radius_m,
            "circumscribed_radius_m": self.circumscribed_radius_m,
            "provider_id": self.provider_id,
            "received_at_unix": self.received_at_unix,
        }


class RobotGeometryState:
    """Small atomic snapshot shared by Scene tools, self-state, and web UI."""

    def __init__(self) -> None:
        self._footprint: Optional[RobotFootprint] = None

    def current(self) -> Optional[RobotFootprint]:
        return self._footprint

    def update(self, footprint: RobotFootprint) -> bool:
        previous = self._footprint
        changed = previous is None or (
            previous.points != footprint.points
            or previous.base_frame != footprint.base_frame
            or previous.inscribed_radius_m != footprint.inscribed_radius_m
            or previous.circumscribed_radius_m != footprint.circumscribed_radius_m
            or previous.provider_id != footprint.provider_id
        )
        self._footprint = footprint
        return changed

    def clear(self) -> bool:
        """Discard geometry that can no longer be confirmed by Soma."""
        changed = self._footprint is not None
        self._footprint = None
        return changed


def validate_footprint_response(response, *, provider_id: str = "") -> RobotFootprint:
    """Validate the wire response again at the Scene trust boundary."""
    points = tuple((float(point.x), float(point.y)) for point in response.points)
    base_frame = str(response.base_frame or "").strip()
    inscribed = float(response.inscribed_radius_m)
    circumscribed = float(response.circumscribed_radius_m)
    if len(points) < 3:
        raise ValueError("Soma footprint must contain at least three points")
    if not base_frame:
        raise ValueError("Soma footprint base_frame is empty")
    if any(not math.isfinite(value) for point in points for value in point):
        raise ValueError("Soma footprint contains a non-finite coordinate")
    signed_area_twice = sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1])
    )
    if abs(signed_area_twice) <= 1e-9:
        raise ValueError("Soma footprint polygon has zero area")
    if (
        not math.isfinite(inscribed)
        or not math.isfinite(circumscribed)
        or inscribed <= 0.0
        or circumscribed <= 0.0
        or inscribed > circumscribed
    ):
        raise ValueError("Soma footprint radii are invalid")
    return RobotFootprint(
        points=points,
        base_frame=base_frame,
        inscribed_radius_m=inscribed,
        circumscribed_radius_m=circumscribed,
        provider_id=provider_id,
        received_at_unix=time.time(),
    )


def fetch_soma_footprint() -> Optional[RobotFootprint]:
    """Resolve and call the standard Soma footprint capability once."""
    import grpc
    import robonix_contracts_pb2_grpc as contracts_grpc  # type: ignore
    import soma_pb2  # type: ignore
    from robonix_api import ATLAS

    capability = ATLAS.find_unique_capability(
        contract_id=SOMA_FOOTPRINT_CONTRACT,
        transport="grpc",
    )
    channel_ref = ATLAS.connect_capability(
        consumer_id="scene",
        provider_id=capability.provider_id,
        contract_id=SOMA_FOOTPRINT_CONTRACT,
        transport="grpc",
    )
    try:
        endpoint = str(channel_ref.endpoint or "").strip()
        if not endpoint:
            raise RuntimeError("Atlas returned an empty Soma footprint endpoint")
        request_cls = getattr(soma_pb2, "GetFootprint_Request", None)
        if request_cls is None:
            request_cls = getattr(soma_pb2, "GetFootprintRequest")
        with grpc.insecure_channel(endpoint) as grpc_channel:
            stub = contracts_grpc.RobonixSystemSomaFootprintStub(grpc_channel)
            response = stub.GetFootprint(request_cls(), timeout=5.0)
        return validate_footprint_response(
            response,
            provider_id=capability.provider_id,
        )
    finally:
        channel_ref.close()


async def reconcile_robot_geometry(
    state: RobotGeometryState,
    *,
    fetcher: Callable[[], Optional[RobotFootprint]] = fetch_soma_footprint,
) -> None:
    """Continuously reconcile Soma geometry without blocking Scene startup."""
    retry_s = 1.0
    refresh_s = 30.0
    last_error = ""
    while True:
        try:
            footprint = await asyncio.to_thread(fetcher)
            if footprint is None:
                state.clear()
                if last_error != "missing":
                    log.warning(
                        "[scene-geometry] waiting for %s; spatial goals are unavailable",
                        SOMA_FOOTPRINT_CONTRACT,
                    )
                    last_error = "missing"
                await asyncio.sleep(retry_s)
                continue
            if state.update(footprint):
                log.info(
                    "[scene-geometry] Soma footprint: provider=%s frame=%s "
                    "points=%d inscribed=%.3fm circumscribed=%.3fm",
                    footprint.provider_id,
                    footprint.base_frame,
                    len(footprint.points),
                    footprint.inscribed_radius_m,
                    footprint.circumscribed_radius_m,
                )
            last_error = ""
            await asyncio.sleep(refresh_s)
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001
            state.clear()
            message = str(exc)
            if message != last_error:
                log.warning(
                    "[scene-geometry] Soma footprint unavailable: %s; "
                    "spatial goals remain fail-closed",
                    exc,
                )
                last_error = message
            await asyncio.sleep(retry_s)
