#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportArgumentType=false
"""Tiago chassis primitive — Capability-based driver.

Owns `robonix/primitive/chassis/*`. Publishes /cmd_vel from the `move` rpc.

Capability surface:

  primitive/chassis/driver    rpc        gRPC lifecycle (Capability built-in)
  primitive/chassis/move      rpc        gRPC ExecuteMoveCommand — burst-style
                                          velocity command. Deliberately NOT
                                          MCP — direct LLM control of an
                                          unfiltered velocity primitive bumps
                                          into things. Path-planned motion
                                          goes through service/navigation/*
                                          which composes safe_goal + nav.
  primitive/chassis/odom      topic_out  ROS 2 /odom
  primitive/chassis/twist_in  topic_in   ROS 2 /cmd_vel
"""
from __future__ import annotations

import json
import math
import os
import time

from robonix_api import Primitive, Ok, Err, Deferred

tiago_chassis = Primitive(id="tiago_chassis", namespace="robonix/primitive/chassis")

cmd_vel_pub = None  # rclpy publisher to /cmd_vel; created in init()


# ── gRPC RPC: `move` (no MCP — keep velocity primitive off the LLM tool list) ─
import chassis_pb2  # noqa: E402  (proto codegen, on PYTHONPATH via rbnx-build/codegen/proto_gen)
import std_msgs_pb2  # noqa: E402


@tiago_chassis.grpc("robonix/primitive/chassis/move")
def move(req: "chassis_pb2.ExecuteMoveCommand_Request") -> "chassis_pb2.ExecuteMoveCommand_Response":
    """Velocity-mode chassis command. Service callers (simple_nav, nav2_wrapper,
    teleop) reach this via gRPC. NOT exposed as MCP — the LLM should invoke
    `service/navigation/navigate` instead, which handles obstacle avoidance.

    Three modes by priority:
      1. command.forward_m != 0   drive straight by signed distance (m)
      2. command.rotate_deg != 0  in-place yaw rotation by signed degrees
      3. velocity mode (linear_x/angular_z used directly for duration_sec)
    """
    if cmd_vel_pub is None:
        return chassis_pb2.ExecuteMoveCommand_Response(
            status=std_msgs_pb2.String(data=json.dumps({"error": "ROS2 not initialized"})),
        )

    msg = req.command
    speed_mps = float(os.environ.get("TIAGO_CHASSIS_SPEED_MPS", "0.3"))
    ang_speed_rps = float(os.environ.get("TIAGO_CHASSIS_ANG_SPEED_RPS", "0.6"))
    default_dur = float(os.environ.get("TIAGO_CHASSIS_CMD_DURATION_SEC", "1.0"))

    forward_m = float(getattr(msg, "forward_m", 0.0))
    rotate_deg = float(getattr(msg, "rotate_deg", 0.0))
    duration_sec = float(getattr(msg, "duration_sec", 0.0))

    from geometry_msgs.msg import Twist  # type: ignore
    tw = Twist()
    if forward_m != 0.0:
        sign = 1.0 if forward_m > 0 else -1.0
        tw.linear.x = sign * speed_mps
        duration = abs(forward_m) / speed_mps
        mode = "forward_m"
    elif rotate_deg != 0.0:
        rad = math.radians(rotate_deg)
        sign = 1.0 if rad > 0 else -1.0
        tw.angular.z = sign * ang_speed_rps
        duration = abs(rad) / ang_speed_rps
        mode = "rotate_deg"
    else:
        tw.linear.x = float(msg.linear_x)
        tw.linear.y = float(msg.linear_y)
        tw.linear.z = float(msg.linear_z)
        tw.angular.x = float(msg.angular_x)
        tw.angular.y = float(msg.angular_y)
        tw.angular.z = float(msg.angular_z)
        duration = duration_sec if duration_sec > 0 else default_dur
        mode = "velocity"

    stop = Twist()
    steps = max(1, int(duration / 0.1))
    for _ in range(steps):
        cmd_vel_pub.publish(tw)
        time.sleep(0.1)
    cmd_vel_pub.publish(stop)
    return chassis_pb2.ExecuteMoveCommand_Response(
        status=std_msgs_pb2.String(data=json.dumps({
            "status": "done", "mode": mode,
            "forward_m": forward_m, "rotate_deg": rotate_deg,
            "duration_sec": duration,
            "linear_x": tw.linear.x, "angular_z": tw.angular.z,
        })),
    )


# ── lifecycle ────────────────────────────────────────────────────────────────
@tiago_chassis.on_init
def init(cfg):
    global cmd_vel_pub
    odom_topic = cfg.get("odom_topic") or os.environ.get("TIAGO_ODOM_TOPIC", "/odom")
    twist_in_topic = cfg.get("twist_in_topic") or os.environ.get("TIAGO_CMD_VEL_TOPIC", "/cmd_vel")

    from geometry_msgs.msg import Twist  # type: ignore
    cmd_vel_pub = tiago_chassis.create_publisher(
        "robonix/primitive/chassis/twist_in",
        topic=twist_in_topic, msg_type=Twist, qos="reliable",
        declare=False,
    )
    tiago_chassis.declare_ros2_topic("robonix/primitive/chassis/twist_in", twist_in_topic, qos="reliable")
    tiago_chassis.declare_ros2_topic("robonix/primitive/chassis/odom",     odom_topic,     qos="reliable")
    return Ok()


@tiago_chassis.on_shutdown
def shutdown():
    global cmd_vel_pub
    if cmd_vel_pub is not None:
        try:
            from geometry_msgs.msg import Twist  # type: ignore
            cmd_vel_pub.publish(Twist())
        except Exception as exc:
            print(f"[tiago_chassis] shutdown zero Twist failed: {exc}", flush=True)
        finally:
            cmd_vel_pub = None
    return Ok()


if __name__ == "__main__":
    tiago_chassis.run()
