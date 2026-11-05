#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Capture a bounded, read-only ROS interval before stopping a failed deployment.

Run under an external timeout inside the owned Nav2 container: middleware
initialization and shutdown can block before the collection deadline applies.
"""

from __future__ import annotations

import argparse
import array
import json
import math
from pathlib import Path
import sys
import time

try:
    import numpy as np
except ImportError:
    np = None

MAX_CELLS = 1_000_000
MAX_TRANSFORMS = 1024
MAX_RECORD_BYTES = 8 * 1024 * 1024
MAX_OUTPUT_BYTES = 64 * 1024 * 1024


def message_values(value, budget=None, depth=0):
    if budget is None:
        budget = [MAX_CELLS + 100_000]
    budget[0] -= 1
    if budget[0] < 0 or depth > 16:
        raise ValueError("message exceeds field budget")
    if isinstance(value, str):
        if len(value) > 1024:
            raise ValueError("message string exceeds 1024 characters")
        return value
    if isinstance(value, (bool, int)) or value is None:
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("message contains a non-finite number")
        return value
    if isinstance(value, (list, tuple, array.array)):
        if len(value) > budget[0]:
            raise ValueError("message sequence exceeds field budget")
        return [message_values(item, budget, depth + 1) for item in value]
    if np is not None and isinstance(value, np.ndarray):
        # Generated ROS fixed-size arrays (including pose covariance) use NumPy.
        if value.size > budget[0] or value.nbytes > MAX_RECORD_BYTES:
            raise ValueError("message array exceeds field or byte budget")
        return message_values(value.tolist(), budget, depth + 1)
    return {
        field: message_values(getattr(value, field), budget, depth + 1)
        for field in value.get_fields_and_field_types()
    }


def sample_record(msg, wall_ns, monotonic_ns):
    if hasattr(msg, "info") and hasattr(msg, "data"):
        if len(msg.data) > MAX_CELLS:
            raise ValueError("grid exceeds cell limit")
        if msg.info.width * msg.info.height != len(msg.data):
            raise ValueError("grid dimensions differ from cell count")
    if hasattr(msg, "transforms") and len(msg.transforms) > MAX_TRANSFORMS:
        raise ValueError("TF message exceeds transform limit")
    result = {
        "receipt_wall_ns": wall_ns,
        "receipt_monotonic_ns": monotonic_ns,
        "message": message_values(msg),
    }
    encoded = (json.dumps(result, separators=(",", ":"), allow_nan=False) + "\n").encode()
    if len(encoded) > MAX_RECORD_BYTES:
        raise ValueError("sample exceeds byte limit")
    return encoded


def write_sample(out, stream, msg, wall_ns, monotonic_ns):
    stream["received"] += 1
    # First and most recent samples preserve an interval without an unbounded log.
    is_tf = hasattr(msg, "transforms")
    if not is_tf and monotonic_ns - stream.get("last_attempt_ns", -10**18) < 500_000_000:
        return
    stream["last_attempt_ns"] = monotonic_ns
    try:
        if is_tf:
            if len(msg.transforms) > MAX_TRANSFORMS:
                raise ValueError("TF message exceeds transform limit")
            frames = stream.setdefault("_tf_frames", {})
            for transform in msg.transforms:
                value = message_values(transform)
                key = value["header"]["frame_id"] + "\0" + value["child_frame_id"]
                if key not in frames and len(frames) >= MAX_TRANSFORMS:
                    raise ValueError("TF frame-pair capacity exhausted")
                receipt = {"receipt_wall_ns": wall_ns, "receipt_monotonic_ns": monotonic_ns,
                           "transform": value}
                frame = frames.setdefault(key, {"first": receipt})
                frame["latest"] = receipt
            encoded = (json.dumps({"frame_pairs": list(frames.values())},
                                  separators=(",", ":"), allow_nan=False) + "\n").encode()
            if len(encoded) > MAX_RECORD_BYTES:
                raise ValueError("TF samples exceed byte limit")
            slot = "frames"
        else:
            encoded = sample_record(msg, wall_ns, monotonic_ns)
            slot = "first" if not stream["samples"] else "latest"
        filename = f"{stream['name']}-{slot}.json"
        path = out / filename
        current_size = sum(p.stat().st_size for p in out.glob("*.json"))
        if current_size + len(encoded) > MAX_OUTPUT_BYTES:
            raise ValueError("capture exceeds output byte limit")
        temporary = path.with_suffix(".tmp")
        temporary.write_bytes(encoded)
        temporary.replace(path)
        stream["samples"][slot] = filename
    except (ValueError, TypeError, AttributeError, OSError) as exc:
        stream["rejected"] += 1
        stream["last_error"] = str(exc)[:512]


def qos_record(endpoint):
    qos = endpoint.qos_profile
    return {
        "node_name": endpoint.node_name[:256],
        "node_namespace": endpoint.node_namespace[:256],
        "topic_type": endpoint.topic_type[:256],
        "reliability": str(qos.reliability),
        "durability": str(qos.durability),
        "history": str(qos.history),
        "depth": qos.depth,
    }


def save_manifest(out, manifest):
    path = out / "manifest.json"
    temporary = path.with_suffix(".tmp")
    snapshot = dict(manifest)
    snapshot["streams"] = [{key: value for key, value in stream.items() if not key.startswith("_")}
                           for stream in manifest["streams"]]
    temporary.write_text(json.dumps(snapshot, indent=2) + "\n")
    temporary.replace(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for stream in ("map", "global-costmap", "tf", "tf-static", "clock"):
        parser.add_argument(f"--{stream}-topic", required=True)
    parser.add_argument("--pose-topic")
    parser.add_argument("--local-costmap-topic")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=15)
    args = parser.parse_args(argv)
    if not 0 < args.duration <= 30:
        parser.error("duration must be in (0, 30] seconds")
    topics = {name.removesuffix("_topic"): value for name, value in vars(args).items()
              if name.endswith("_topic")}
    if any(topic is not None and (not topic.startswith("/") or len(topic) > 1024)
           for topic in topics.values()):
        parser.error("topics must be absolute ROS names of at most 1024 characters")
    args.out.mkdir(parents=True, exist_ok=False)
    manifest = {
        "status": "initializing",
        "start_wall_ns": time.time_ns(),
        "start_monotonic_ns": time.monotonic_ns(),
        "duration_seconds": args.duration,
        "requested_topics": topics,
        "limits": {"cells": MAX_CELLS, "transforms": MAX_TRANSFORMS,
                   "record_bytes": MAX_RECORD_BYTES, "output_bytes": MAX_OUTPUT_BYTES},
        "interval_note": "Postfailure receipt interval; not a simultaneous failed planner tick.",
        "local_costmap_configured": bool(args.local_costmap_topic),
        "unconfigured_streams": [name for name, topic in
                                 (("pose", args.pose_topic),
                                  ("local_costmap", args.local_costmap_topic)) if not topic],
        "streams": [],
    }
    save_manifest(args.out, manifest)
    print("initializing ROS middleware; external timeout required", file=sys.stderr, flush=True)
    node = None
    rclpy = None
    try:
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage

        rclpy.init()
        node = rclpy.create_node("robonix_ci_capture_nav_diagnostics")
        specs = [
            ("map", args.map_topic, OccupancyGrid),
            ("global_costmap", args.global_costmap_topic, OccupancyGrid),
            ("tf", args.tf_topic, TFMessage),
            ("tf_static", args.tf_static_topic, TFMessage),
            ("clock", args.clock_topic, Clock),
        ]
        if args.pose_topic:
            specs.append(("pose", args.pose_topic, PoseWithCovarianceStamped))
        if args.local_costmap_topic:
            specs.append(("local_costmap", args.local_costmap_topic, OccupancyGrid))
        for name, topic, msg_type in specs:
            offered = node.get_publishers_info_by_topic(topic)
            latched = name in ("map", "global_costmap", "tf_static")
            if name == "local_costmap":
                latched = bool(offered) and all(
                    ep.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                    for ep in offered
                )
            qos = QoSProfile(
                depth=1,
                reliability=(ReliabilityPolicy.RELIABLE if name in ("map", "global_costmap")
                             else ReliabilityPolicy.BEST_EFFORT),
                durability=(DurabilityPolicy.TRANSIENT_LOCAL if latched
                            else DurabilityPolicy.VOLATILE),
            )
            stream = {"name": name, "topic": topic, "type": msg_type.__name__,
                      "received": 0, "rejected": 0, "samples": {},
                      "offered_qos_at_subscribe": [qos_record(ep) for ep in offered[:32]],
                      "offered_publishers": len(offered),
                      "requested_qos": {"reliability": str(qos.reliability),
                                        "durability": str(qos.durability), "depth": 1}}
            manifest["streams"].append(stream)
            node.create_subscription(
                msg_type, topic,
                lambda msg, stream=stream: write_sample(
                    args.out, stream, msg, time.time_ns(), time.monotonic_ns()), qos,
            )
        manifest["status"] = "collecting"
        manifest["collection_start_monotonic_ns"] = time.monotonic_ns()
        save_manifest(args.out, manifest)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0, deadline - time.monotonic())))
        for stream in manifest["streams"]:
            offered = node.get_publishers_info_by_topic(stream["topic"])
            stream["offered_qos_at_end"] = [qos_record(ep) for ep in offered[:32]]
            stream["offered_publishers_at_end"] = len(offered)
        manifest["missing_streams"] = [s["name"] for s in manifest["streams"] if not s["samples"]]
        incomplete = (manifest["missing_streams"] or manifest["unconfigured_streams"]
                      or any(s["rejected"] for s in manifest["streams"]))
        manifest["status"] = "incomplete" if incomplete else "captured"
        return 1 if incomplete else 0
    except Exception as exc:
        manifest["status"] = "error"
        manifest["error"] = f"{type(exc).__name__}: {exc}"[:1024]
        print(manifest["error"], file=sys.stderr, flush=True)
        return 2
    finally:
        manifest["end_wall_ns"] = time.time_ns()
        manifest["end_monotonic_ns"] = time.monotonic_ns()
        save_manifest(args.out, manifest)
        # Save evidence before cleanup, which can itself block in middleware.
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
