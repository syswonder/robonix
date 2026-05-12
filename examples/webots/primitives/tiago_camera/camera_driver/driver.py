#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportArgumentType=false
"""Tiago camera primitive — Capability-based driver.

Owns `primitive/camera/*`. Subscribes to RGB + depth image topics, exposes:

  primitive/camera/rgb            topic_out  (declared but only as metadata —
                                              the JPEG-snapshot is the LLM-
                                              facing tool, see snapshot below)
  primitive/camera/depth          topic_out
  primitive/camera/extrinsics     topic_out  (latched TransformStamped)
  primitive/camera/snapshot       rpc        (MCP, RGB JPEG)
  primitive/camera/depth_snapshot rpc        (MCP, depth as 8-bit JPEG)
  primitive/camera/driver         rpc        (gRPC lifecycle)

Init waits up to 60 s for the first RGB frame — webots's image publisher is
racy on cold boot (frames sometimes don't arrive for 20-40 s after rclpy
spins up).
"""
from __future__ import annotations

import os
import threading
import time
from io import BytesIO

import numpy as np

from robonix_api import Primitive, Ok, Err, Deferred

cap = Primitive(id="tiago_camera", namespace="robonix/primitive/camera")

# ── shared state ─────────────────────────────────────────────────────────────
state_lock = threading.Lock()
latest_rgb_jpeg: bytes | None = None
latest_depth_jpeg: bytes | None = None
extrinsics_pub = None  # rclpy publisher for the latched TF


# ── image conversion ─────────────────────────────────────────────────────────
def ros_image_to_jpeg(msg) -> bytes:
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    if enc == "rgb8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    elif enc == "bgr8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)[:, :, ::-1]
    elif enc == "rgba8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
    elif enc == "bgra8":
        # Webots head camera publishes BGRA8 — drop alpha + swap to RGB.
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3][:, :, ::-1]
    elif enc == "mono8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        arr = np.stack([arr, arr, arr], axis=-1)
    elif enc == "16uc1":
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        arr = (raw / raw.max() * 255).astype(np.uint8) if raw.max() > 0 else np.zeros((h, w), np.uint8)
        arr = np.stack([arr, arr, arr], axis=-1)
    elif enc == "32fc1":
        raw = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
        valid = np.isfinite(raw)
        if valid.any():
            mn, mx = raw[valid].min(), raw[valid].max()
            norm = np.where(valid, (raw - mn) / max(mx - mn, 1e-6) * 255, 0).astype(np.uint8)
        else:
            norm = np.zeros((h, w), np.uint8)
        arr = np.stack([norm, norm, norm], axis=-1)
    else:
        raise ValueError(f"unsupported image encoding: {enc}")
    from PIL import Image as PILImage
    buf = BytesIO()
    PILImage.fromarray(np.ascontiguousarray(arr)).save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def on_rgb(msg):
    global latest_rgb_jpeg
    try:
        with state_lock:
            latest_rgb_jpeg = ros_image_to_jpeg(msg)
    except Exception as e:
        print(f"[tiago_camera] RGB conversion error: {e}")


def on_depth(msg):
    global latest_depth_jpeg
    try:
        with state_lock:
            latest_depth_jpeg = ros_image_to_jpeg(msg)
    except Exception as e:
        print(f"[tiago_camera] Depth conversion error: {e}")


# ── MCP snapshot tools (typed against codegen MCP dataclasses) ──────────────
import builtin_interfaces_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from sensor_msgs_mcp import Image  # noqa: E402
from std_msgs_mcp import Empty  # noqa: E402


def now_header(frame_id: str) -> std_msgs_mcp.Header:
    now = time.time()
    sec = int(now)
    ns = int((now % 1) * 1e9) % 1_000_000_000
    return std_msgs_mcp.Header(
        stamp=builtin_interfaces_mcp.Time(sec=sec, nanosec=ns),
        frame_id=frame_id,
    )


def jpeg_to_image_mcp(jpg: bytes, frame_id: str) -> Image:
    from PIL import Image as PILImage
    im = PILImage.open(BytesIO(jpg))
    w, h = im.size
    return Image(
        header=now_header(frame_id),
        height=h, width=w,
        encoding="jpeg",
        is_bigendian=0,
        step=len(jpg),
        data=jpg,
    )


def image_error(message: str) -> Image:
    raw = message.encode("utf-8")
    return Image(
        header=now_header("tiago_camera_error"),
        height=0, width=0, encoding="error", is_bigendian=0,
        step=len(raw), data=raw,
    )


@cap.mcp("robonix/primitive/camera/snapshot")
def snapshot(msg: Empty) -> Image:
    """PRIMARY perception tool. Use freely — between every chassis/cmd
    burst — to see what's in front of the robot and decide what to do
    next. Returns the current RGB head-camera frame as a JPEG-encoded
    sensor_msgs/Image (`data` is base64).
    Contract: robonix/primitive/camera/snapshot."""
    _ = msg
    with state_lock:
        data = latest_rgb_jpeg
    if data is None:
        return image_error("no RGB image received yet")
    return jpeg_to_image_mcp(
        data, os.environ.get("TIAGO_RGB_FRAME_ID", "head_front_camera_rgb_optical_frame")
    )


@cap.mcp("robonix/primitive/camera/depth_snapshot")
def depth_snapshot(msg: Empty) -> Image:
    """Get the current depth head-camera frame as a JPEG-encoded
    sensor_msgs/Image (depth normalized to grayscale; binary `data` is
    base64). Use to gauge stand-off distance / find open space.
    Contract: robonix/primitive/camera/depth_snapshot."""
    _ = msg
    with state_lock:
        data = latest_depth_jpeg
    if data is None:
        return image_error("no depth image received yet")
    return jpeg_to_image_mcp(
        data, os.environ.get("TIAGO_DEPTH_FRAME_ID", "head_front_camera_depth_optical_frame")
    )


# ── extrinsics: tf2 lookup once at startup, republish on a latched topic ────
def publish_extrinsics_when_ready(base_frame: str, cam_frame: str, topic: str) -> None:
    """Resolve `base_frame → cam_frame` from tf2, publish on latched extrinsics
    topic, exit. tf2 is used here purely as the local-to-this-primitive
    mechanism for reading the URDF chain — consumers never touch tf2; they
    go through the declared `primitive/camera/extrinsics` contract."""
    from rclpy.duration import Duration  # type: ignore
    from rclpy.time import Time  # type: ignore
    from tf2_ros import Buffer, TransformListener  # type: ignore
    from robonix_api.ros import RosBackend
    node = RosBackend.get().node
    tf_buf = Buffer()
    TransformListener(tf_buf, node)
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        try:
            tf = tf_buf.lookup_transform(base_frame, cam_frame, Time(), Duration(seconds=0.5))
        except Exception:  # noqa: BLE001
            time.sleep(0.5)
            continue
        tf.header.frame_id = base_frame
        tf.child_frame_id = cam_frame
        if extrinsics_pub is not None:
            extrinsics_pub.publish(tf)
        t = tf.transform.translation
        print(f"[tiago_camera] published extrinsics {base_frame}→{cam_frame}: "
              f"({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) → {topic}")
        return
    print(f"[tiago_camera] WARN: extrinsics publish gave up — tf2 chain "
          f"{base_frame}→{cam_frame} not resolvable.")


# ── lifecycle ────────────────────────────────────────────────────────────────
@cap.on_init
def init(cfg):
    global extrinsics_pub
    rgb_topic = cfg.get("rgb_topic") or os.environ.get(
        "TIAGO_RGB_TOPIC", "/head_front_camera/rgb/image_raw")
    depth_topic = cfg.get("depth_topic") or os.environ.get(
        "TIAGO_DEPTH_TOPIC", "/head_front_camera/depth_registered/image_raw")
    extrinsics_topic = cfg.get("extrinsics_topic") or os.environ.get(
        "TIAGO_CAMERA_EXTRINSICS_TOPIC", "/tiago/camera/extrinsics")
    base_frame = cfg.get("base_frame") or os.environ.get("TIAGO_BASE_FRAME", "base_link")
    cam_frame = cfg.get("cam_frame") or os.environ.get(
        "TIAGO_RGB_FRAME_ID", "head_front_camera_rgb_optical_frame")
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 60.0))

    # subscribe RGB + depth (we own both contracts; declare manually below)
    cap.create_subscription("robonix/primitive/camera/rgb",
                            topic=rgb_topic, msg_type="Image",
                            callback=on_rgb, qos="best_effort", declare=False)
    cap.create_subscription("robonix/primitive/camera/depth",
                            topic=depth_topic, msg_type="Image",
                            callback=on_depth, qos="best_effort", declare=False)

    # latched extrinsics publisher
    from geometry_msgs.msg import TransformStamped  # type: ignore
    extrinsics_pub = cap.create_publisher(
        "robonix/primitive/camera/extrinsics",
        topic=extrinsics_topic, msg_type=TransformStamped, qos="latched",
    )
    threading.Thread(
        target=publish_extrinsics_when_ready,
        args=(base_frame, cam_frame, extrinsics_topic),
        daemon=True,
    ).start()

    # gate INIT on first RGB arriving — same generosity as before.
    if not cap.wait_for_topic(rgb_topic, "Image", sentinel_timeout):
        return Err(f"no RGB on {rgb_topic} within {sentinel_timeout:.1f}s")

    # data interfaces are ready — declare them on atlas
    cap.declare_ros2_topic("robonix/primitive/camera/rgb",   rgb_topic,   qos="best_effort")
    cap.declare_ros2_topic("robonix/primitive/camera/depth", depth_topic, qos="best_effort")
    return Ok()


if __name__ == "__main__":
    cap.run()
