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
  primitive/camera/intrinsics     topic_out  (latched CameraInfo)
  primitive/camera/snapshot       rpc        (MCP, RGB JPEG)
  primitive/camera/depth_snapshot rpc        (MCP, depth as 8-bit JPEG)
  primitive/camera/driver         rpc        (gRPC lifecycle)

Init waits up to 60 s for the first RGB frame — webots's image publisher is
racy on cold boot (frames sometimes don't arrive for 20-40 s after rclpy
spins up).
"""
from __future__ import annotations

import os
import math
import subprocess
import threading
import time
from io import BytesIO

import numpy as np

from robonix_api import Primitive, Ok, Err, Deferred

tiago_camera = Primitive(id="tiago_camera", namespace="robonix/primitive/camera")

# ── shared state ─────────────────────────────────────────────────────────────
state_lock = threading.Lock()
intrinsics_lock = threading.Lock()
latest_rgb_jpeg: bytes | None = None
latest_depth_jpeg: bytes | None = None
extrinsics_pub = None  # rclpy publisher for the latched TF
intrinsics_pub = None  # rclpy publisher for the latched CameraInfo
intrinsics_published = False  # true after the first valid K sample is published
camera_info_seen = False
configured_intrinsics_msg = None
configured_intrinsics_k: list[float] = []
latest_intrinsics_msg = None
latest_intrinsics_k: list[float] = []
intrinsics_publish_interval_s = 0.5
configured_intrinsics_logged = False
last_intrinsics_publish = 0.0

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
        publish_intrinsics_if_needed("rgb")
    except Exception as e:
        print(f"[tiago_camera] RGB conversion error: {e}", flush=True)


def on_depth(msg):
    global latest_depth_jpeg
    try:
        with state_lock:
            latest_depth_jpeg = ros_image_to_jpeg(msg)
    except Exception as e:
        print(f"[tiago_camera] Depth conversion error: {e}", flush=True)


def publish_intrinsics_if_needed(reason: str, *, force: bool = False) -> None:
    """Publish the best available CameraInfo on the intrinsics contract.

    Scene subscribes after providers register and may miss a one-shot CameraInfo
    sample. Keep the contract alive by periodically republishing the last valid
    camera K. A live CameraInfo source always wins; deployment-configured K is a
    fallback for simulators such as Webots that do not stream camera_info.
    """
    global configured_intrinsics_logged, intrinsics_published, last_intrinsics_publish
    pub = intrinsics_pub
    with intrinsics_lock:
        if pub is None:
            return
        if latest_intrinsics_msg is not None:
            msg = latest_intrinsics_msg
            k = list(latest_intrinsics_k)
            source = "camera_info"
        elif configured_intrinsics_msg is not None:
            msg = configured_intrinsics_msg
            k = list(configured_intrinsics_k)
            source = "configured"
        else:
            return
        now = time.monotonic()
        elapsed = now - last_intrinsics_publish
        if not force and elapsed < intrinsics_publish_interval_s:
            return
        last_intrinsics_publish = now
        log_configured = source == "configured" and not configured_intrinsics_logged
        log_first = not intrinsics_published
    try:
        pub.publish(msg)
    except Exception as e:  # noqa: BLE001
        print(f"[tiago_camera] WARN: intrinsics publish failed: {e}", flush=True)
        return
    with intrinsics_lock:
        intrinsics_published = True
        if log_configured:
            configured_intrinsics_logged = True
    if log_first or log_configured:
        prefix = "publishing configured intrinsics" if source == "configured" else "publishing intrinsics"
        print(
            f"[tiago_camera] {prefix} via {reason}: "
            f"fx={k[0]:.1f} fy={k[1]:.1f} cx={k[2]:.1f} cy={k[3]:.1f} "
            f"{msg.width}x{msg.height}",
            flush=True,
        )


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


@tiago_camera.mcp("robonix/primitive/camera/snapshot")
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
        raise RuntimeError("no RGB image received yet")
    return jpeg_to_image_mcp(
        data, os.environ.get("TIAGO_RGB_FRAME_ID", "head_front_camera_rgb_optical_frame")
    )


@tiago_camera.mcp("robonix/primitive/camera/depth_snapshot")
def depth_snapshot(msg: Empty) -> Image:
    """Get the current depth head-camera frame as a JPEG-encoded
    sensor_msgs/Image (depth normalized to grayscale; binary `data` is
    base64). Use to gauge stand-off distance / find open space.
    Contract: robonix/primitive/camera/depth_snapshot."""
    _ = msg
    with state_lock:
        data = latest_depth_jpeg
    if data is None:
        raise RuntimeError("no depth image received yet")
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


def topic_has_sample(topic: str, timeout_s: float) -> bool:
    """Return true if ROS CLI can read one sample from a topic.

    This is a fallback for CI boots where the shared rclpy sentinel subscription
    misses a callback even though Webots is already publishing frames.
    """
    timeout = max(1, int(timeout_s))
    try:
        proc = subprocess.run(
            ["timeout", str(timeout), "ros2", "topic", "echo", "--once", topic, "--field", "header"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except OSError:
        return False
    return proc.returncode == 0


def _cfg_float(cfg: dict, key: str, env_key: str, default: float = 0.0) -> float:
    value = cfg.get(key)
    if value is None:
        value = os.environ.get(env_key)
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _cfg_int(cfg: dict, key: str, env_key: str, default: int) -> int:
    value = cfg.get(key)
    if value is None:
        value = os.environ.get(env_key)
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


# ── lifecycle ────────────────────────────────────────────────────────────────
@tiago_camera.on_init
def init(cfg):
    global extrinsics_pub, intrinsics_pub, intrinsics_published, camera_info_seen
    global configured_intrinsics_msg, configured_intrinsics_k
    global latest_intrinsics_msg, latest_intrinsics_k
    global intrinsics_publish_interval_s, configured_intrinsics_logged
    global last_intrinsics_publish
    cfg = cfg or {}
    rgb_topic = cfg.get("rgb_topic") or os.environ.get(
        "TIAGO_RGB_TOPIC", "/head_front_camera/rgb/image_raw")
    depth_topic = cfg.get("depth_topic") or os.environ.get(
        "TIAGO_DEPTH_TOPIC", "/head_front_camera/depth_registered/image_raw")
    extrinsics_topic = cfg.get("extrinsics_topic") or os.environ.get(
        "TIAGO_CAMERA_EXTRINSICS_TOPIC", "/tiago/camera/extrinsics")
    camera_info_topic = cfg.get("camera_info_topic") or os.environ.get(
        "TIAGO_CAMERA_INFO_TOPIC", "/head_front_camera/rgb/camera_info")
    intrinsics_topic = cfg.get("intrinsics_topic") or os.environ.get(
        "TIAGO_CAMERA_INTRINSICS_TOPIC", "/tiago/camera/intrinsics")
    base_frame = cfg.get("base_frame") or os.environ.get("TIAGO_BASE_FRAME", "base_link")
    cam_frame = cfg.get("cam_frame") or os.environ.get(
        "TIAGO_RGB_FRAME_ID", "head_front_camera_rgb_optical_frame")
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 60.0))
    intrinsics_source_timeout_s = _cfg_float(
        cfg, "camera_info_source_timeout_s", "TIAGO_CAMERA_INFO_SOURCE_TIMEOUT_S", 1.0
    )
    intrinsics_publish_interval_s = _cfg_float(
        cfg, "intrinsics_publish_interval_s", "TIAGO_CAMERA_INTRINSICS_PUBLISH_INTERVAL_S", 0.5
    )

    # subscribe RGB + depth (we own both contracts; declare manually below)
    tiago_camera.create_subscription("robonix/primitive/camera/rgb",
                            topic=rgb_topic, msg_type="Image",
                            callback=on_rgb, qos="best_effort", declare=False)
    tiago_camera.create_subscription("robonix/primitive/camera/depth",
                            topic=depth_topic, msg_type="Image",
                            callback=on_depth, qos="best_effort", declare=False)

    # latched extrinsics publisher
    from geometry_msgs.msg import TransformStamped  # type: ignore
    extrinsics_pub = tiago_camera.create_publisher(
        "robonix/primitive/camera/extrinsics",
        topic=extrinsics_topic, msg_type=TransformStamped, qos="latched",
    )
    threading.Thread(
        target=publish_extrinsics_when_ready,
        args=(base_frame, cam_frame, extrinsics_topic),
        daemon=True,
    ).start()

    # Intrinsics contract publisher. Prefer the camera's own CameraInfo stream
    # when present. Webots' Tiago camera plugin publishes images but not a
    # CameraInfo topic, so this primitive also supports deployment-provided K
    # from its package config and republishes it on the same contract.
    from sensor_msgs.msg import CameraInfo  # type: ignore
    intrinsics_pub = tiago_camera.create_publisher(
        "robonix/primitive/camera/intrinsics",
        topic=intrinsics_topic, msg_type=CameraInfo, qos="latched",
    )
    with intrinsics_lock:
        intrinsics_published = False
        camera_info_seen = False
        configured_intrinsics_msg = None
        configured_intrinsics_k = []
        latest_intrinsics_msg = None
        latest_intrinsics_k = []
        intrinsics_publish_interval_s = max(0.1, intrinsics_publish_interval_s)
        configured_intrinsics_logged = False
        last_intrinsics_publish = 0.0

    def configured_camera_info() -> tuple[CameraInfo | None, list[float]]:
        width = _cfg_int(cfg, "width", "TIAGO_CAMERA_WIDTH", 0)
        height = _cfg_int(cfg, "height", "TIAGO_CAMERA_HEIGHT", 0)
        fx = _cfg_float(cfg, "fx", "TIAGO_CAMERA_FX")
        fy = _cfg_float(cfg, "fy", "TIAGO_CAMERA_FY")
        cx = _cfg_float(cfg, "cx", "TIAGO_CAMERA_CX")
        cy = _cfg_float(cfg, "cy", "TIAGO_CAMERA_CY")
        if fx <= 0 or fy <= 0:
            horizontal_fov = _cfg_float(
                cfg, "horizontal_fov_rad", "TIAGO_CAMERA_HORIZONTAL_FOV_RAD"
            )
            if horizontal_fov > 0:
                fx = width / (2.0 * math.tan(horizontal_fov / 2.0))
                fy = fx if fy <= 0 else fy
        if min(width, height, fx, fy, cx, cy) <= 0:
            return None, []
        msg = CameraInfo()
        msg.header.frame_id = cam_frame
        msg.width = width
        msg.height = height
        msg.distortion_model = "plumb_bob"
        msg.d = []
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg, [fx, fy, cx, cy]

    configured, configured_k = configured_camera_info()
    with intrinsics_lock:
        configured_intrinsics_msg = configured
        configured_intrinsics_k = configured_k

    def on_camera_info(msg, _topic=intrinsics_topic):
        global camera_info_seen, latest_intrinsics_msg, latest_intrinsics_k
        # Validate K before relaying: skip zero/partial CameraInfo so we
        # never forward garbage K.
        k = list(msg.k) if hasattr(msg, "k") else list(getattr(msg, "K", []))
        if len(k) < 6 or k[0] <= 0 or k[4] <= 0:
            return
        with intrinsics_lock:
            camera_info_seen = True
            latest_intrinsics_msg = msg
            latest_intrinsics_k = [float(k[0]), float(k[4]), float(k[2]), float(k[5])]
        publish_intrinsics_if_needed(_topic, force=True)

    def publish_intrinsics_loop() -> None:
        try:
            warned_missing = False
            while True:
                publish_intrinsics_if_needed("timer", force=True)
                with intrinsics_lock:
                    has_any_intrinsics = latest_intrinsics_msg is not None or configured_intrinsics_msg is not None
                    has_published = intrinsics_published
                if not warned_missing and not has_any_intrinsics and not has_published:
                    # Keep the old delayed warning behavior so a wrong source
                    # topic is visible without racing normal startup.
                    time.sleep(max(0.0, intrinsics_source_timeout_s))
                    with intrinsics_lock:
                        has_any_intrinsics = latest_intrinsics_msg is not None or configured_intrinsics_msg is not None
                        has_published = intrinsics_published
                    if not has_any_intrinsics and not has_published:
                        print("[tiago_camera] WARN: no usable CameraInfo source and no "
                              "configured camera intrinsics", flush=True)
                        warned_missing = True
                time.sleep(max(0.1, intrinsics_publish_interval_s))
        except Exception as e:  # noqa: BLE001
            print(f"[tiago_camera] WARN: intrinsics publish thread exited: {e}", flush=True)

    tiago_camera.create_subscription(
        "robonix/primitive/camera/intrinsics",
        topic=camera_info_topic, msg_type="CameraInfo",
        callback=on_camera_info, qos="best_effort", declare=False,
    )
    threading.Thread(target=publish_intrinsics_loop, daemon=True).start()

    # Give-up watchdog: if no usable CameraInfo lands on `camera_info_topic`
    # the intrinsics contract is advertised but never publishes, and scene
    # would wait forever for K. Warn after a deadline — mirrors the
    # extrinsics give-up WARN — so a wrong topic name is visible.
    def warn_if_no_intrinsics(source_topic: str, deadline_s: float = 60.0) -> None:
        end = time.monotonic() + deadline_s
        while time.monotonic() < end:
            if intrinsics_published:
                return
            time.sleep(0.5)
        if not intrinsics_published:
            print(f"[tiago_camera] WARN: intrinsics never published — no "
                  f"usable CameraInfo seen on {source_topic}", flush=True)

    threading.Thread(
        target=warn_if_no_intrinsics,
        args=(camera_info_topic,),
        daemon=True,
    ).start()

    # Gate INIT on first RGB arriving. Keep the in-process rclpy sentinel, but
    # fall back to a short ROS CLI sample check so a missed transient callback
    # does not leave CI stuck with an ERROR camera while Webots is publishing.
    rclpy_wait_s = min(sentinel_timeout, 20.0)
    if not tiago_camera.wait_for_topic(rgb_topic, "Image", rclpy_wait_s):
        fallback_wait_s = max(1.0, min(30.0, sentinel_timeout - rclpy_wait_s))
        if not topic_has_sample(rgb_topic, fallback_wait_s):
            waited = rclpy_wait_s + fallback_wait_s
            return Err(f"no RGB on {rgb_topic} within {waited:.1f}s")
        print(f"[tiago_camera] RGB sample confirmed via ros2 CLI fallback on {rgb_topic}")

    # data interfaces are ready — declare them on atlas
    tiago_camera.declare_ros2_topic("robonix/primitive/camera/rgb",   rgb_topic,   qos="best_effort")
    tiago_camera.declare_ros2_topic("robonix/primitive/camera/depth", depth_topic, qos="best_effort")
    return Ok()


@tiago_camera.on_shutdown
def shutdown():
    return Ok()


if __name__ == "__main__":
    tiago_camera.run()
