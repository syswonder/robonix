"""Robonix Service forwarding robonix/service/body/grasp to the embodiment service."""
import sys

from robonix_api import Ok, Service
from body_mcp import Grasp_Request, Grasp_Response
from navigation_mcp import (GetNavigationStatus_Request, GetNavigationStatus_Response,
                            CancelNavigation_Request, CancelNavigation_Response)

import body_bridge

svc = Service(id="body_grasp", namespace="robonix/service/body")


@svc.mcp("robonix/service/body/grasp")
def grasp(req: Grasp_Request) -> Grasp_Response:
    """Grasp a scene object with the gripper. Argument: the object's instance name. Drive next to the object first. Returns a run_id at once; the grasp continues in the background."""
    ok, run_id, detail = body_bridge.start("grasp", {"object": req.object}, "robonix")
    return Grasp_Response(accepted=ok, run_id=run_id, detail=detail)


@svc.mcp("robonix/service/body/grasp/status")
def status(req: GetNavigationStatus_Request) -> GetNavigationStatus_Response:
    """Status of a grasp run. Empty run_id means the most recent one."""
    known, state, detail = body_bridge.status(req.run_id)
    return GetNavigationStatus_Response(known=known, state=state, detail=detail)


@svc.mcp("robonix/service/body/grasp/cancel")
def cancel(req: CancelNavigation_Request) -> CancelNavigation_Response:
    """Stop a grasp run. Empty run_id means the most recent one."""
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
