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
    """Drive the robot base next to a scene object. Argument: the object's instance name, exactly as given in the task (for example bowl_1). Returns a run_id at once; the motion continues in the background."""
    ok, run_id, detail = body_bridge.start("navigate", {"object": req.object}, "robonix")
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
