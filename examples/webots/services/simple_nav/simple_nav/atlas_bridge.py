#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""simple_nav atlas bridge — Capability + contract-typed MCP tools.

Resolves chassis odom + lidar scan + chassis cmd_vel + map occupancy_grid
(and optionally chassis pose + camera depth) through atlas, brings up
NavNode, and exposes navigate / status / cancel as MCP tools — typed
against the codegen-generated Request/Response dataclasses for the
service/navigation/srv/* contracts (Navigate, GetNavigationStatus,
CancelNavigation). No hand-written JSON schemas: cap.mcp introspects
each request class's `.json_schema()` automatically.
"""
from __future__ import annotations

import json
import logging
import math
import time
import uuid
from typing import Optional

from robonix_api import Capability, Ok, Err, Deferred, atlas

from .nav_node import Goal, NavNode

log = logging.getLogger("simple_nav")

cap = Capability(id="simple_nav", namespace="robonix/service/navigation")

nav: NavNode | None = None
# We pack our internal goal_id + tolerance_m through the contract by
# stamping them into Navigate_Response.status_message as JSON, since
# the contract's response has only `accepted: bool` + `status_message: str`.
goal_id_by_ts: dict[str, str] = {}


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
            recs = atlas.find(contract, transport="ros2")
            if not recs:
                continue
            try:
                ch = cap.connect(provider=recs[0], contract_id=contract, transport="ros2")
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


@cap.mcp("robonix/service/navigation/navigate")
def navigate(req: Navigate_Request) -> Navigate_Response:
    """Drive the robot to the goal pose in the map frame.

    The goal's position.{x,y} is the target xy. The goal's
    orientation is interpreted as a yaw — when non-trivial, the goal
    includes an in-place rotation phase after xy arrival. (Don't fill
    orientation if you don't care about final heading; the robot
    succeeds on xy alone.)

    Per Navigate.srv the response carries `goal_id` directly; track
    via the sibling `status` / `cancel` contracts. `status_message`
    is free-form text only — never a JSON envelope."""
    if nav is None:
        return Navigate_Response(
            accepted=False, goal_id="", status_message="nav not initialized",
        )
    goal = req.goal
    target_yaw = quat_to_yaw(goal.pose.orientation.z, goal.pose.orientation.w)
    # Heuristic: if orientation is the identity quaternion (z=0,w=1), the
    # caller didn't bother specifying a yaw. Don't impose one.
    use_yaw = not (abs(goal.pose.orientation.z) < 1e-6
                   and abs(goal.pose.orientation.w - 1.0) < 1e-6)
    goal_id = f"nav-{uuid.uuid4().hex[:8]}"
    nav.set_goal(Goal(
        goal_id=goal_id,
        target_x=float(goal.pose.position.x),
        target_y=float(goal.pose.position.y),
        target_yaw=target_yaw if use_yaw else None,
        # Tolerance is a service-side default — not a contract knob.
        tolerance_m=0.5,
    ))
    msg = (f"goto ({goal.pose.position.x:.2f},{goal.pose.position.y:.2f})"
           + (f" yaw={target_yaw:.2f}" if use_yaw else ""))
    return Navigate_Response(accepted=True, goal_id=goal_id, status_message=msg)


@cap.mcp("robonix/service/navigation/status")
def status(req: GetNavigationStatus_Request) -> GetNavigationStatus_Response:
    """Get current status of a navigation goal. Empty `goal_id` =
    most recent. Returns known/state/terminal — `state` is a
    free-form string ('running', 'reached', 'failed', ...)."""
    if nav is None:
        return GetNavigationStatus_Response(
            known=False, status="nav not initialized", terminal=True,
        )
    s = nav.goal_status(req.goal_id or None)
    if s is None:
        return GetNavigationStatus_Response(known=False, status="no active goal", terminal=True)
    state = str(s.get("state", "unknown"))
    detail = str(s.get("detail", ""))
    return GetNavigationStatus_Response(
        known=True,
        status=f"{state}: {detail}" if detail else state,
        terminal=state in ("succeeded", "aborted", "cancelled"),
    )


@cap.mcp("robonix/service/navigation/cancel")
def cancel(req: CancelNavigation_Request) -> CancelNavigation_Response:
    """Cancel an active navigation goal. Empty `goal_id` cancels the
    currently active goal. Idempotent."""
    if nav is None:
        return CancelNavigation_Response(accepted=False, status_message="nav not initialized")
    ok = nav.cancel_goal(req.goal_id or None)
    return CancelNavigation_Response(
        accepted=ok, status_message="cancelled" if ok else "no active goal",
    )


# ── lifecycle ────────────────────────────────────────────────────────────────
@cap.on_init
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
    cap.run()
    if nav is not None:
        nav.stop()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
