#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""simple_nav atlas bridge — Capability + contract-typed MCP tools.

Resolves chassis odom + lidar scan + chassis cmd_vel + map occupancy_grid
(and optionally chassis pose + camera depth) through atlas, brings up
NavNode, and exposes navigate / status / cancel as MCP tools — typed
against the codegen-generated Request/Response dataclasses for the
service/navigation/srv/* contracts (Navigate, GetNavigationStatus,
CancelNavigation). No hand-written JSON schemas: service.mcp introspects
each request class's `.json_schema()` automatically.
"""
from __future__ import annotations

import logging
import math
import time
import uuid

from robonix_api import ATLAS, Service, Ok, Err, Deferred

from .nav_node import Goal, NavNode

log = logging.getLogger("simple_nav")

simple_nav = Service(id="simple_nav", namespace="robonix/service/navigation")

nav: NavNode | None = None


def resolve_inputs(deadline_s: float = 30.0) -> dict[str, str]:
    """Resolve every ROS2 topic this service consumes from atlas. No hardcoded
    topic names anywhere — the deploy is one primitive/service swap away from
    running on a different robot."""
    wanted = {
        "odom_topic":  "robonix/primitive/chassis/odom",
        "scan_topic":  "robonix/primitive/lidar/lidar",
        "cmd_topic":   "robonix/primitive/chassis/twist_in",
        "map_topic":   "robonix/service/map/occupancy_grid",
        # SLAM-corrected map-frame pose for A* start point. Optional:
        # without it nav runs in odom-only degraded mode (drifts across
        # episodes but still lands short goals). World-frame localisation
        # lives in the mapping service — chassis primitives only emit
        # odom-frame data.
        "pose_topic":  "robonix/service/map/pose",
        # Optional: depth image for the second-line forward e-stop.
        # Lidar at chassis height passes through tall thin obstacles
        # (potted plants, table legs); depth catches them.
        "depth_topic": "robonix/primitive/camera/depth",
    }
    required = ("odom_topic", "scan_topic", "cmd_topic", "map_topic")
    resolved: dict[str, str] = {}
    deadline = time.time() + deadline_s
    while time.time() < deadline:
        for key, contract in wanted.items():
            if key in resolved:
                continue
            caps = ATLAS.find_capability(contract_id=contract, transport="ros2")
            if not caps:
                continue
            try:
                ch = simple_nav.connect_capability(caps[0], contract, "ros2")
            except Exception:  # noqa: BLE001
                continue
            ep = ch.endpoint
            ch.close()
            if ep:
                resolved[key] = ep
                log.info("resolved %s → %s", contract, ep)
        if all(k in resolved for k in required):
            break
        time.sleep(2.0)
    return resolved


# ── MCP tools (typed against codegen Request/Response from srv) ─────────────
from navigation_mcp import (  # noqa: E402
    Navigate_Request, Navigate_Response,
    GetNavigationStatus_Request, GetNavigationStatus_Response,
    CancelNavigation_Request, CancelNavigation_Response,
)


def quat_to_yaw(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


@simple_nav.mcp("robonix/service/navigation/navigate")
def navigate(req: Navigate_Request) -> Navigate_Response:
    """Drive the robot to the goal pose in the map frame.

    The goal's position.{x,y} is the target xy. The goal's
    orientation is interpreted as a yaw — when non-trivial, the goal
    includes an in-place rotation phase after xy arrival. (Don't fill
    orientation if you don't care about final heading; the robot
    succeeds on xy alone.)

    Per Navigate.srv the response carries optional `run_id` directly; track
    via the `navigate/status` and `navigate/cancel` async sub-contracts.
    Empty run_id on those contracts means the most recent navigation call."""
    if nav is None:
        raise RuntimeError("nav not initialized")
    goal = req.goal
    target_yaw = quat_to_yaw(goal.pose.orientation.z, goal.pose.orientation.w)
    # Heuristic: if orientation is the identity quaternion (z=0,w=1), the
    # caller didn't bother specifying a yaw. Don't impose one.
    use_yaw = not (abs(goal.pose.orientation.z) < 1e-6
                   and abs(goal.pose.orientation.w - 1.0) < 1e-6)
    run_id = f"nav-{uuid.uuid4().hex[:8]}"
    nav.set_goal(Goal(
        goal_id=run_id,
        target_x=float(goal.pose.position.x),
        target_y=float(goal.pose.position.y),
        target_yaw=target_yaw if use_yaw else None,
        # Tolerance is a service-side default — not a contract knob.
        tolerance_m=0.5,
    ))
    msg = (f"goto ({goal.pose.position.x:.2f},{goal.pose.position.y:.2f})"
           + (f" yaw={target_yaw:.2f}" if use_yaw else ""))
    return Navigate_Response(accepted=True, run_id=run_id, detail=msg)


@simple_nav.mcp("robonix/service/navigation/navigate/status")
def status(req: GetNavigationStatus_Request) -> GetNavigationStatus_Response:
    """Get current status of a navigation goal. Empty `run_id` = most recent."""
    if nav is None:
        raise RuntimeError("nav not initialized")
    s = nav.goal_status(req.run_id or None)
    if s is None:
        raise RuntimeError("no active goal")
    state = _executor_state(str(s.get("state", "unknown")))
    detail = str(s.get("detail", ""))
    return GetNavigationStatus_Response(
        known=True,
        state=state,
        detail=detail,
    )


@simple_nav.mcp("robonix/service/navigation/navigate/cancel")
def cancel(req: CancelNavigation_Request) -> CancelNavigation_Response:
    """Cancel an active navigation goal. Empty `run_id` cancels the
    currently active goal. Idempotent."""
    if nav is None:
        raise RuntimeError("nav not initialized")
    ok = nav.cancel_goal(req.run_id or None)
    if not ok:
        raise RuntimeError("no active goal")
    return CancelNavigation_Response(
        accepted=True, detail="cancel requested",
    )


def _executor_state(state: str) -> str:
    """Map simple_nav's internal goal state to executor status state names."""
    return {
        "active": "RUNNING",
        "succeeded": "SUCCEEDED",
        "aborted": "FAILED",
        "cancelled": "CANCELED",
        "canceled": "CANCELED",
    }.get(state.lower(), "RUNNING")


# ── lifecycle ────────────────────────────────────────────────────────────────
@simple_nav.on_init
def init(cfg):
    global nav
    inputs = resolve_inputs()
    missing = [k for k in ("odom_topic", "scan_topic", "cmd_topic", "map_topic")
               if k not in inputs]
    if missing:
        return Err(
            f"missing required atlas resolutions: {missing} (chassis + lidar + mapping "
            f"all online before simple_nav?)"
        )

    nav = NavNode(
        scan_topic=inputs["scan_topic"],
        odom_topic=inputs["odom_topic"],
        cmd_topic=inputs["cmd_topic"],
        map_topic=inputs["map_topic"],
        pose_topic=inputs.get("pose_topic"),
        depth_topic=inputs.get("depth_topic"),
    )
    nav.start()
    log.info(
        "nav node up: scan=%s odom=%s cmd=%s map=%s pose=%s depth=%s",
        inputs["scan_topic"], inputs["odom_topic"], inputs["cmd_topic"],
        inputs["map_topic"], inputs.get("pose_topic"),
        inputs.get("depth_topic", "(none)"))
    return Ok()


def main() -> int:
    simple_nav.run()
    if nav is not None:
        nav.stop()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
