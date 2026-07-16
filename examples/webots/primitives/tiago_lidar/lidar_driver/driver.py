#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Tiago lidar primitive — Capability-based driver.

Owns `primitive/lidar/*`. Subscribes to a normalised LaserScan topic that
scripts/start.sh's relay produces from webots' raw /scanner, then exposes:

  primitive/lidar/lidar     topic_out  ROS 2 LaserScan stream
  primitive/lidar/snapshot  rpc        MCP one-shot LaserScan capture
  primitive/lidar/driver    rpc        gRPC lifecycle (Init waits for first scan)
"""
from __future__ import annotations

import os
import threading

from robonix_api import Primitive, Ok, Err, Deferred

tiago_lidar = Primitive(id="tiago_lidar", namespace="robonix/primitive/lidar")

# ── shared state — the latest rclpy LaserScan we've seen ────────────────────
state_lock = threading.Lock()
latest_scan = None


def on_scan(msg):
    global latest_scan
    with state_lock:
        latest_scan = msg


# ── MCP snapshot tool (typed against codegen MCP dataclasses) ──────────────
import builtin_interfaces_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from sensor_msgs_mcp import LaserScan  # noqa: E402
from std_msgs_mcp import Empty  # noqa: E402


def ros_to_mcp(msg) -> LaserScan:
    h = msg.header
    stamp = builtin_interfaces_mcp.Time(sec=int(h.stamp.sec), nanosec=int(h.stamp.nanosec))
    header = std_msgs_mcp.Header(stamp=stamp, frame_id=str(h.frame_id))
    intensities = [float(x) for x in msg.intensities] if len(msg.intensities) else []
    return LaserScan(
        header=header,
        angle_min=float(msg.angle_min),
        angle_max=float(msg.angle_max),
        angle_increment=float(msg.angle_increment),
        time_increment=float(msg.time_increment),
        scan_time=float(msg.scan_time),
        range_min=float(msg.range_min),
        range_max=float(msg.range_max),
        ranges=[float(r) for r in msg.ranges],
        intensities=intensities,
    )


@tiago_lidar.mcp("robonix/primitive/lidar/snapshot")
def snapshot(msg: Empty) -> LaserScan:
    """Get the latest planar lidar scan. Returns sensor_msgs/LaserScan;
    `ranges[i]` is the distance (m) at angle `angle_min + i*angle_increment`.
    Useful for "obstacle in front?" / "where's the nearest open space?"
    Contract: robonix/primitive/lidar/snapshot."""
    _ = msg
    with state_lock:
        ros_scan = latest_scan
    if ros_scan is None:
        raise RuntimeError("no LaserScan received yet")
    return ros_to_mcp(ros_scan)


# ── lifecycle ────────────────────────────────────────────────────────────────
@tiago_lidar.on_init
def init(cfg):
    topic = cfg.get("scan_topic") or os.environ.get("TIAGO_SCAN_TOPIC", "/scanner_normalized")
    tiago_lidar.create_subscription(
        "robonix/primitive/lidar/lidar",
        topic=topic, msg_type="LaserScan",
        callback=on_scan, qos="best_effort",
        declare=False,  # we declare topic_out below (we own this contract)
    )
    if not tiago_lidar.wait_for_topic(topic, "LaserScan", float(cfg.get("sentinel_timeout_s", 15.0))):
        return Err(f"no LaserScan received on {topic} within timeout")
    tiago_lidar.declare_ros2_topic("robonix/primitive/lidar/lidar", topic, qos="best_effort")
    return Ok()


@tiago_lidar.on_shutdown
def shutdown():
    return Ok()


if __name__ == "__main__":
    tiago_lidar.run()
