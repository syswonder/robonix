"""Robonix Service forwarding robonix/service/body/place_on to the embodiment service."""
import sys

from robonix_api import Ok, Service
from body_mcp import PlaceOn_Request, PlaceOn_Response
from navigation_mcp import (GetNavigationStatus_Request, GetNavigationStatus_Response,
                            CancelNavigation_Request, CancelNavigation_Response)

import body_bridge

svc = Service(id="body_place_on", namespace="robonix/service/body")


@svc.mcp("robonix/service/body/place_on")
def place_on(req: PlaceOn_Request) -> PlaceOn_Response:
    """Put the held object down on a named surface. Finishes when the object rests on it; fails if nothing is held or the surface is out of reach. Argument place: the surface's name as listed in the scene; the robot must be within reach of it and must be holding an object."""
    ok, run_id, detail = body_bridge.start("place", {"place": req.place}, "robonix")
    return PlaceOn_Response(accepted=ok, run_id=run_id, detail=detail)


@svc.mcp("robonix/service/body/place_on/status")
def status(req: GetNavigationStatus_Request) -> GetNavigationStatus_Response:
    """Status of a place_on run. Empty run_id means the most recent one."""
    known, state, detail = body_bridge.status(req.run_id)
    return GetNavigationStatus_Response(known=known, state=state, detail=detail)


@svc.mcp("robonix/service/body/place_on/cancel")
def cancel(req: CancelNavigation_Request) -> CancelNavigation_Response:
    """Stop a place_on run. Empty run_id means the most recent one."""
    ok, detail = body_bridge.cancel(req.run_id)
    return CancelNavigation_Response(accepted=ok, detail=detail)


@svc.on_init
def init(cfg):
    """Nothing to prepare; the embodiment service is contacted on first use."""
    return Ok()


def main() -> int:
    svc.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
