"""Robonix Service forwarding robonix/service/body/navigate_to to the embodiment service."""
import sys

from robonix_api import Ok, Service
from body_mcp import NavigateTo_Request, NavigateTo_Response
from navigation_mcp import (GetNavigationStatus_Request, GetNavigationStatus_Response,
                            CancelNavigation_Request, CancelNavigation_Response)

import body_bridge

svc = Service(id="body_navigate_to", namespace="robonix/service/body")


@svc.mcp("robonix/service/body/navigate_to")
def navigate_to(req: NavigateTo_Request) -> NavigateTo_Response:
    """Walk the robot base to a named object or place in the scene. Finishes when the base has arrived and stopped; fails if the target is unknown or unreachable. Give exactly one of object (an object listed in the scene) or place (a piece of furniture listed in the scene)."""
    ok, run_id, detail = body_bridge.start("navigate", {k: v for k, v in (("object", req.object), ("place", req.place)) if v}, "robonix")
    return NavigateTo_Response(accepted=ok, run_id=run_id, detail=detail)


@svc.mcp("robonix/service/body/navigate_to/status")
def status(req: GetNavigationStatus_Request) -> GetNavigationStatus_Response:
    """Status of a navigate_to run. Empty run_id means the most recent one."""
    known, state, detail = body_bridge.status(req.run_id)
    return GetNavigationStatus_Response(known=known, state=state, detail=detail)


@svc.mcp("robonix/service/body/navigate_to/cancel")
def cancel(req: CancelNavigation_Request) -> CancelNavigation_Response:
    """Stop a navigate_to run. Empty run_id means the most recent one."""
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
