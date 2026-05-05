# SPDX-License-Identifier: MulanPSL-2.0
"""simple_nav atlas bridge.

  1. RegisterCapability (com.robonix.example.simple_nav).
  2. QueryCapabilities + ConnectCapability for the inputs we need
     (chassis/odom, lidar/lidar). twist_in is a CONSUMER side — we
     publish to /cmd_vel directly via rclpy.
  3. Spin up NavNode (rclpy thread) + a FastMCP server exposing
     navigate / status / cancel.
  4. DeclareInterface(MCP) for those three tools so the pilot can
     find them via atlas.
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import sys
import time
import uuid
from typing import Optional

logging.basicConfig(level=logging.INFO,
                    format="[simple_nav] %(levelname)s %(message)s")
log = logging.getLogger("simple_nav")


# Generated proto stubs are at <pkg>/proto_gen/ (rbnx codegen output).
import grpc  # noqa: E402

# Sys.path is set by start.sh (PYTHONPATH covers <pkg>:<pkg>/proto_gen).
import atlas_pb2 as pb  # type: ignore
import atlas_pb2_grpc as pb_grpc  # type: ignore

from .nav_node import Goal, NavNode


ATLAS_ENDPOINT = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
CAP_ID = os.environ.get("ROBONIX_CAPABILITY_ID", "com.robonix.example.simple_nav")
NAMESPACE = "robonix/service/navigation"
MCP_PORT = int(os.environ.get("SIMPLE_NAV_MCP_PORT", "50122"))
HEARTBEAT_PERIOD_S = 10.0


def _atlas() -> pb_grpc.AtlasStub:
    return pb_grpc.AtlasStub(grpc.insecure_channel(ATLAS_ENDPOINT))


def _connect_endpoint(stub, contract_id: str) -> Optional[str]:
    """QueryCapabilities + ConnectCapability for one ROS2 input contract."""
    try:
        resp = stub.QueryCapabilities(pb.QueryCapabilitiesRequest(
            contract_id=contract_id, transport=pb.TRANSPORT_ROS2))
    except grpc.RpcError as e:
        log.warning("query %s failed: %s", contract_id, e)
        return None
    for rec in resp.records:
        for iface in rec.interfaces:
            if iface.contract_id != contract_id or iface.transport != pb.TRANSPORT_ROS2:
                continue
            try:
                conn = stub.ConnectCapability(pb.ConnectCapabilityRequest(
                    consumer_id=CAP_ID,
                    capability_id=rec.capability_id,
                    contract_id=contract_id,
                    transport=pb.TRANSPORT_ROS2,
                ))
                if conn.endpoint:
                    return conn.endpoint
            except grpc.RpcError as e:
                log.warning("connect %s failed: %s", contract_id, e)
    return None


def _resolve_inputs(stub, deadline_s: float = 30.0) -> dict:
    """Resolve every ROS2 topic this service consumes from atlas.

    The wanted set is (key → contract):
        odom_topic  ← robonix/primitive/chassis/odom        REQUIRED
        scan_topic  ← robonix/primitive/lidar/lidar         REQUIRED
        cmd_topic   ← robonix/primitive/chassis/twist_in    REQUIRED
                      (where the chassis is *listening* for Twist; we
                       publish here)
        map_topic   ← robonix/service/map/occupancy_grid    REQUIRED
        pose_topic  ← robonix/primitive/chassis/pose        OPTIONAL
                      (when present, used as alt source for map-frame
                       robot pose; otherwise tf2 lookup)

    No hardcoded topic names anywhere — the deploy is one
    primitive/service swap away from running on a different robot.
    """
    wanted = {
        "odom_topic":  "robonix/primitive/chassis/odom",
        "scan_topic":  "robonix/primitive/lidar/lidar",
        "cmd_topic":   "robonix/primitive/chassis/twist_in",
        "map_topic":   "robonix/service/map/occupancy_grid",
        "pose_topic":  "robonix/primitive/chassis/pose",
        # Optional: depth image for the second-line forward e-stop.
        # Lidar at chassis height passes through tall thin obstacles
        # (potted plants, table legs); depth catches them. Robots
        # without an RGBD camera fall back to lidar-only e-stop.
        "depth_topic": "robonix/primitive/camera/depth",
    }
    required = ("odom_topic", "scan_topic", "cmd_topic", "map_topic")
    resolved: dict[str, str] = {}
    deadline = time.time() + deadline_s
    while time.time() < deadline:
        for key, contract in wanted.items():
            if key in resolved:
                continue
            ep = _connect_endpoint(stub, contract)
            if ep:
                resolved[key] = ep
                log.info("resolved %s → %s", contract, ep)
        if all(k in resolved for k in required):
            break
        time.sleep(2.0)
    return resolved


def _register(stub) -> None:
    try:
        stub.RegisterCapability(pb.RegisterCapabilityRequest(
            capability_id=CAP_ID, namespace=NAMESPACE, capability_md_path=""))
        log.info("registered cap %s", CAP_ID)
    except grpc.RpcError as e:
        if e.code() == grpc.StatusCode.ALREADY_EXISTS:
            log.info("cap %s already registered", CAP_ID)
        else:
            raise


def _declare_mcp_tools(stub) -> None:
    """Declare the three navigation tools as MCP capabilities. Endpoint
    is the FastMCP HTTP URL; description + schema are the LLM hints."""
    base = f"http://127.0.0.1:{MCP_PORT}/mcp/"
    tools = [
        ("robonix/service/navigation/navigate",
         "Drive the robot to a 2D pose (target_x, target_y in map frame).",
         {"type": "object",
          "properties": {
              "target_x": {"type": "number"},
              "target_y": {"type": "number"},
              "tolerance_m": {"type": "number", "default": 0.5},
          },
          "required": ["target_x", "target_y"]}),
        ("robonix/service/navigation/status",
         "Get the current navigation goal status.",
         {"type": "object",
          "properties": {"goal_id": {"type": "string"}}}),
        ("robonix/service/navigation/cancel",
         "Cancel the active navigation goal.",
         {"type": "object",
          "properties": {"goal_id": {"type": "string"}}}),
    ]
    for contract_id, desc, schema in tools:
        try:
            stub.DeclareInterface(pb.DeclareInterfaceRequest(
                capability_id=CAP_ID,
                contract_id=contract_id,
                transport=pb.TRANSPORT_MCP,
                endpoint=base,
                params=pb.TransportParams(
                    mcp=pb.McpParams(description=desc,
                                      input_schema_json=json.dumps(schema)),
                ),
            ))
            log.info("declared %s → MCP %s", contract_id, base)
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.ALREADY_EXISTS:
                pass
            else:
                log.warning("declare %s: %s", contract_id, e)


def _heartbeat_loop(stub) -> None:
    while True:
        try:
            stub.Heartbeat(pb.HeartbeatRequest(capability_id=CAP_ID))
        except Exception:
            pass
        time.sleep(HEARTBEAT_PERIOD_S)


# ── FastMCP server ─────────────────────────────────────────────────
def _make_mcp_server(nav: NavNode):
    from fastmcp import FastMCP

    mcp = FastMCP("simple_nav")

    @mcp.tool()
    def navigate(target_x: float, target_y: float,
                 tolerance_m: float = 0.5,
                 target_yaw: Optional[float] = None) -> dict:
        """Drive the robot to (target_x, target_y) in the map frame.

        When `target_yaw` is provided (radians), the goal includes an
        in-place rotation phase after xy arrival until heading matches.
        Critical for explore-skill sweep legs: a goal of (current_x,
        current_y, target_yaw=π/2) with no yaw arg succeeds immediately
        because xy already match — the robot never rotates.
        """
        goal_id = f"nav-{uuid.uuid4().hex[:8]}"
        nav.set_goal(Goal(
            goal_id=goal_id,
            target_x=float(target_x), target_y=float(target_y),
            target_yaw=float(target_yaw) if target_yaw is not None else None,
            tolerance_m=float(tolerance_m),
        ))
        return {"ok": True, "goal_id": goal_id,
                "target_x": float(target_x), "target_y": float(target_y),
                "target_yaw": target_yaw}

    @mcp.tool()
    def status(goal_id: str = "") -> dict:
        s = nav.goal_status(goal_id or None)
        if s is None:
            return {"ok": False, "detail": "no active goal"}
        return {"ok": True, **s}

    @mcp.tool()
    def cancel(goal_id: str = "") -> dict:
        ok = nav.cancel_goal(goal_id or None)
        return {"ok": ok, "goal_id": goal_id or "(active)"}

    return mcp


def main() -> int:
    log.info("starting; atlas=%s mcp=:%d", ATLAS_ENDPOINT, MCP_PORT)

    stub = _atlas()
    # Atlas may have just started — retry register a few times.
    for _ in range(10):
        try:
            _register(stub)
            break
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.ALREADY_EXISTS:
                break
            time.sleep(1.0)

    inputs = _resolve_inputs(stub)
    missing = [k for k in ("odom_topic", "scan_topic", "cmd_topic", "map_topic")
               if k not in inputs]
    if missing:
        log.error(
            "atlas resolution incomplete (missing: %s). simple_nav refuses "
            "to fall back to hardcoded topic names — robot/sim swap then "
            "becomes silent breakage. Make sure chassis (odom + twist_in), "
            "lidar (lidar), and a mapping service (occupancy_grid) are all "
            "running before starting simple_nav.", missing)
        return 2

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

    # FastMCP server (Streamable HTTP).
    mcp = _make_mcp_server(nav)

    import threading
    threading.Thread(target=_heartbeat_loop, args=(stub,), daemon=True).start()

    _declare_mcp_tools(stub)

    log.info("FastMCP listening on 0.0.0.0:%d", MCP_PORT)
    try:
        mcp.run(transport="streamable-http", host="0.0.0.0", port=MCP_PORT)
    except KeyboardInterrupt:
        pass
    finally:
        nav.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
