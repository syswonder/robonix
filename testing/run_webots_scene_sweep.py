#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Drive a bounded Webots sweep and record RGB-D visibility evidence.

Run this script inside the Webots simulator container.  It uses only live ROS
topics, publishes a conservative lidar-gated ``/cmd_vel`` route, and projects
WBT-resolved target centers through the live map-to-camera transform.  A target
is visible only when registered metric depth near its projected footprint is
consistent with the target range.  The resulting truth-id set lets the outer
evaluator distinguish a real miss from an object the camera never observed.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformListener

from webots_scene_motion import (
    OdometryContinuity,
    SynchronizedPoseAgreement,
    choose_motion,
    publish_stop_if_available,
    sector_clearance_m,
)
from webots_scene_visibility import rgbd_visibility_evidence


def _depth_array(message: Image) -> np.ndarray | None:
    encoding = str(message.encoding or "").lower()
    if encoding == "32fc1":
        dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
        scale = 1.0
    elif encoding in {"16uc1", "mono16"}:
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        scale = 0.001
    else:
        return None
    item_size = dtype.itemsize
    if message.step < message.width * item_size:
        return None
    array = np.ndarray(
        shape=(message.height, message.width),
        dtype=dtype,
        buffer=message.data,
        strides=(message.step, item_size),
    )
    return np.asarray(array, dtype=np.float32) * scale


def _rotation_matrix(quaternion: Any) -> np.ndarray:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def _transform_matrix(transform: Any) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = _rotation_matrix(transform.rotation)
    matrix[:3, 3] = [
        float(transform.translation.x),
        float(transform.translation.y),
        float(transform.translation.z),
    ]
    return matrix


def _yaw(quaternion: Any) -> float:
    """Return planar yaw from a normalized quaternion."""

    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _align_truths(
    truths: list[dict[str, Any]],
    *,
    translation_m: tuple[float, float, float],
    yaw_rad: float,
) -> list[dict[str, Any]]:
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    aligned = []
    for truth in truths:
        x, y, z = (float(value) for value in truth["center_m"])
        copy = dict(truth)
        copy["center_m"] = [
            translation_m[0] + cosine * x - sine * y,
            translation_m[1] + sine * x + cosine * y,
            translation_m[2] + z,
        ]
        copy["yaw_rad"] = math.atan2(
            math.sin(float(truth.get("yaw_rad", 0.0)) + yaw_rad),
            math.cos(float(truth.get("yaw_rad", 0.0)) + yaw_rad),
        )
        aligned.append(copy)
    return aligned


def _sector(scan: LaserScan | None, low: float, high: float) -> float:
    if scan is None:
        return float("nan")
    return sector_clearance_m(
        ranges=scan.ranges,
        angle_min=scan.angle_min,
        angle_increment=scan.angle_increment,
        range_min=scan.range_min,
        range_max=scan.range_max,
        low=low,
        high=high,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--truth-file", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=72.0)
    parser.add_argument("--min-visible-frames", type=int, default=2)
    parser.add_argument("--linear-speed-mps", type=float, default=0.14)
    parser.add_argument("--angular-speed-rps", type=float, default=0.4)
    parser.add_argument("--robot-frame", default="base_link")
    parser.add_argument("--alignment-timeout-s", type=float, default=20.0)
    parser.add_argument("--max-initial-map-offset-m", type=float, default=0.15)
    parser.add_argument("--max-initial-map-yaw-rad", type=float, default=0.10)
    parser.add_argument("--max-odom-step-m", type=float, default=0.25)
    parser.add_argument(
        "--max-odom-speed-mps",
        type=float,
        default=0.60,
        help=(
            "reject a large odometry step only when its stamped interval also "
            "implies a physically implausible translation speed"
        ),
    )
    parser.add_argument(
        "--passive",
        action="store_true",
        help=(
            "record RGB-D visibility without publishing /cmd_vel; use when "
            "Navigation or Explore owns robot motion"
        ),
    )
    parser.add_argument(
        "--panorama",
        action="store_true",
        help=(
            "rotate in place for the full trial; this produces a repeatable "
            "fixed-position RGB-D panorama for perception A/B tests"
        ),
    )
    args = parser.parse_args()
    if args.passive and args.panorama:
        parser.error("--passive and --panorama are mutually exclusive")
    truth_payload = json.loads(args.truth_file.read_text(encoding="utf-8"))
    relative_truths = list(truth_payload["truths"])

    rclpy.init()
    node = rclpy.create_node("webots_scene_visibility_sweep")
    publisher = (
        None
        if args.passive
        else node.create_publisher(Twist, "/cmd_vel", 10)
    )
    latest: dict[str, Any] = {
        "scan": None,
        "odom": None,
        "wheel_odom": None,
        "ground_truth_odom": None,
        "depth": None,
        "info": None,
    }
    latest_lock = threading.Lock()
    odometry_tracks = {
        name: OdometryContinuity(
            max_step_gate_m=args.max_odom_step_m,
            max_speed_gate_mps=args.max_odom_speed_mps,
        )
        for name in ("localized", "fused", "wheel", "ground_truth")
    }
    localized_pose_agreement = SynchronizedPoseAgreement(max_time_delta_s=0.10)

    def receive_odometry(latest_key: str, track_key: str):
        def callback(message: Odometry) -> None:
            stamp = message.header.stamp
            position = message.pose.pose.position
            with latest_lock:
                latest[latest_key] = message
                odometry_tracks[track_key].observe(
                    stamp_s=(
                        float(stamp.sec)
                        + float(stamp.nanosec) / 1_000_000_000.0
                    ),
                    receipt_s=time.monotonic(),
                    x_m=float(position.x),
                    y_m=float(position.y),
                )

        return callback

    def receive_latest(key: str):
        def callback(message: Any) -> None:
            with latest_lock:
                latest[key] = message

        return callback

    node.create_subscription(
        LaserScan,
        "/scanner_normalized",
        receive_latest("scan"),
        10,
    )
    node.create_subscription(
        Odometry,
        "/odom",
        receive_odometry("odom", "fused"),
        50,
    )
    node.create_subscription(
        Odometry,
        "/wheel_odom",
        receive_odometry("wheel_odom", "wheel"),
        50,
    )
    node.create_subscription(
        Odometry,
        "/webots/ground_truth/odom",
        receive_odometry("ground_truth_odom", "ground_truth"),
        50,
    )
    node.create_subscription(
        Image,
        "/head_front_camera/depth_registered/image_raw",
        receive_latest("depth"),
        1,
    )
    node.create_subscription(
        CameraInfo,
        "/head_front_camera/depth_registered/camera_info",
        receive_latest("info"),
        1,
    )
    tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
    _listener = TransformListener(tf_buffer, node, spin_thread=False)
    # Visibility projection is deliberately expensive.  A single manual
    # ``spin_once`` per outer loop lets high-rate odometry monopolize the
    # callback queue and can starve both RGB-D and TF completely.  Keep ROS
    # callbacks draining independently while the main thread evaluates the
    # latest immutable messages.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin,
        name="webots-scene-sweep-executor",
        daemon=True,
    )
    executor_thread.start()

    def shutdown_ros() -> None:
        executor.shutdown(timeout_sec=2.0)
        executor_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    alignment_deadline = time.monotonic() + args.alignment_timeout_s
    initial_robot_to_map = None
    while time.monotonic() < alignment_deadline:
        if publisher is not None:
            publisher.publish(Twist())
        try:
            initial_robot_to_map = tf_buffer.lookup_transform(
                "map",
                args.robot_frame,
                Time(),
                timeout=Duration(seconds=0.15),
            )
        except Exception:
            time.sleep(0.05)
            continue
        break
    if initial_robot_to_map is None:
        shutdown_ros()
        raise RuntimeError(f"timed out waiting for map <- {args.robot_frame} alignment")
    transform = initial_robot_to_map.transform
    alignment_translation = (
        float(transform.translation.x),
        float(transform.translation.y),
        float(transform.translation.z),
    )
    alignment_yaw = _yaw(transform.rotation)
    initial_offset = math.hypot(
        alignment_translation[0],
        alignment_translation[1],
    )
    if (
        initial_offset > args.max_initial_map_offset_m
        or abs(alignment_yaw) > args.max_initial_map_yaw_rad
    ):
        shutdown_ros()
        raise RuntimeError(
            "fresh-run guard rejected non-initial simulator pose: "
            f"map<-{args.robot_frame} offset={initial_offset:.3f} m "
            f"yaw={alignment_yaw:.3f} rad; recreate the isolated Webots "
            "container before evaluating WBT truth"
        )
    truths = _align_truths(
        relative_truths,
        translation_m=alignment_translation,
        yaw_rad=alignment_yaw,
    )
    print(
        "truth alignment "
        f"map<-{args.robot_frame} "
        f"xyz=({alignment_translation[0]:.3f},"
        f"{alignment_translation[1]:.3f},"
        f"{alignment_translation[2]:.3f}) "
        f"yaw={alignment_yaw:.3f}",
        flush=True,
    )
    visible_counts = {str(truth["identity"]): 0 for truth in truths}
    first_visible_s: dict[str, float] = {}
    last_visible_s: dict[str, float] = {}
    visibility_diagnostics: dict[str, dict[str, Any]] = {
        str(truth["identity"]): {
            "max_projected_area_px": 0,
            "max_consistent_depth_pixels": 0,
            "max_consistent_depth_fraction": 0.0,
            "last_reason": "not_evaluated",
        }
        for truth in truths
    }
    start = time.monotonic()
    last_visibility = 0.0
    last_print = start
    progress_anchor_xy: tuple[float, float] | None = None
    progress_anchor_time = start
    stalled_turn_sign = 0.0
    aborted_reason = ""
    odometry_abort_source = ""
    visibility_samples = 0
    transform_failures = 0
    localized_transform_failures = 0
    depth_failures = 0
    motion_mode = "passive" if publisher is None else "waiting"

    try:
        while time.monotonic() - start < args.duration_s:
            now = time.monotonic()
            elapsed = now - start
            try:
                localized_pose = tf_buffer.lookup_transform(
                    "map",
                    args.robot_frame,
                    Time(),
                    timeout=Duration(seconds=0.0),
                )
                localized_stamp = localized_pose.header.stamp
                localized_translation = localized_pose.transform.translation
                with latest_lock:
                    odometry_tracks["localized"].observe(
                        stamp_s=(
                            float(localized_stamp.sec)
                            + float(localized_stamp.nanosec) / 1_000_000_000.0
                        ),
                        receipt_s=now,
                        x_m=float(localized_translation.x),
                        y_m=float(localized_translation.y),
                    )
                    ground_truth_pose = latest["ground_truth_odom"]
                if ground_truth_pose is not None:
                    ground_truth_stamp = ground_truth_pose.header.stamp
                    ground_truth_position = ground_truth_pose.pose.pose.position
                    localized_pose_agreement.observe(
                        localized_stamp_s=(
                            float(localized_stamp.sec)
                            + float(localized_stamp.nanosec) / 1_000_000_000.0
                        ),
                        localized_x_m=float(localized_translation.x),
                        localized_y_m=float(localized_translation.y),
                        localized_yaw_rad=_yaw(
                            localized_pose.transform.rotation
                        ),
                        reference_stamp_s=(
                            float(ground_truth_stamp.sec)
                            + float(ground_truth_stamp.nanosec)
                            / 1_000_000_000.0
                        ),
                        reference_x_m=float(ground_truth_position.x),
                        reference_y_m=float(ground_truth_position.y),
                        reference_yaw_rad=_yaw(
                            ground_truth_pose.pose.pose.orientation
                        ),
                    )
            except Exception:
                localized_transform_failures += 1
            with latest_lock:
                odometry = latest["odom"]
                fused_track = odometry_tracks["fused"]
                discontinuous_tracks = [
                    (
                        name,
                        float(
                            odometry_tracks[name].discontinuities[0][
                                "stamp_s"
                            ]
                        ),
                        odometry_tracks[name].max_step_m,
                        odometry_tracks[name].max_speed_mps,
                    )
                    for name in ("ground_truth", "localized", "wheel", "fused")
                    if odometry_tracks[name].discontinuities
                ]
            if odometry is not None:
                current_xy = (
                    float(odometry.pose.pose.position.x),
                    float(odometry.pose.pose.position.y),
                )
                if progress_anchor_xy is None:
                    progress_anchor_xy = current_xy
                    progress_anchor_time = now
                elif math.dist(current_xy, progress_anchor_xy) >= 0.12:
                    progress_anchor_xy = current_xy
                    progress_anchor_time = now
                    stalled_turn_sign = 0.0
            if discontinuous_tracks:
                (
                    odometry_abort_source,
                    _first_stamp_s,
                    discontinuous_max_step_m,
                    discontinuous_max_speed_mps,
                ) = min(
                    discontinuous_tracks,
                    key=lambda item: item[1],
                )
                aborted_reason = (
                    f"{odometry_abort_source} odometry discontinuity "
                    f"(max step {discontinuous_max_step_m:.3f} m, "
                    f"max speed {discontinuous_max_speed_mps:.3f} m/s)"
                )
                publish_stop_if_available(publisher, Twist)
                break

            with latest_lock:
                scan = latest["scan"]
            front = _sector(scan, -0.42, 0.42)
            if publisher is not None:
                left = _sector(scan, 0.45, 2.40)
                right = _sector(scan, -2.40, -0.45)
                rear = min(
                    _sector(scan, 2.50, math.pi),
                    _sector(scan, -math.pi, -2.50),
                )
                stalled_s = max(0.0, now - progress_anchor_time)
                if (
                    stalled_s >= 8.0
                    and not stalled_turn_sign
                    and math.isfinite(left)
                    and math.isfinite(right)
                ):
                    stalled_turn_sign = 1.0 if left >= right else -1.0
                command = Twist()
                if args.panorama:
                    # Fixed-position continuous rotation deliberately avoids
                    # lidar-threshold route bifurcations.  A 120 s Office run
                    # covers several complete turns, so small scheduling
                    # differences cannot change the visibility union used as
                    # the A/B denominator.
                    command.angular.z = args.angular_speed_rps
                    motion_mode = "panorama"
                else:
                    decision = choose_motion(
                        elapsed_s=elapsed,
                        front_m=front,
                        left_m=left,
                        right_m=right,
                        rear_m=rear,
                        linear_speed_mps=args.linear_speed_mps,
                        angular_speed_rps=args.angular_speed_rps,
                        stalled_s=stalled_s,
                        stalled_turn_sign=stalled_turn_sign,
                    )
                    command.linear.x = decision.linear_x_mps
                    command.angular.z = decision.angular_z_rps
                    motion_mode = decision.mode
                publisher.publish(command)
            else:
                left = right = rear = float("nan")

            if now - last_visibility >= 0.45:
                last_visibility = now
                with latest_lock:
                    depth_message = latest["depth"]
                    info = latest["info"]
                if depth_message is None or info is None:
                    depth_failures += 1
                else:
                    depth = _depth_array(depth_message)
                    if depth is None:
                        depth_failures += 1
                    else:
                        depth_frame = str(depth_message.header.frame_id or "").strip()
                        info_frame = str(info.header.frame_id or "").strip()
                        camera_frame = info_frame or depth_frame
                        if not camera_frame or (
                            depth_frame and info_frame and depth_frame != info_frame
                        ):
                            transform_failures += 1
                            continue
                        transform = None
                        observation_time = Time.from_msg(
                            depth_message.header.stamp
                        )
                        # This evaluator spins in one thread.  A registered
                        # depth sample can arrive a few milliseconds before
                        # the matching dynamic TF; blocking inside tf2 cannot
                        # receive that TF because no executor is spinning.
                        # Keep the exact same depth frame and briefly spin
                        # between retries.  Never replace timestamped truth
                        # projection with the latest transform.
                        last_transform_error = None
                        for _ in range(60):
                            try:
                                transform = tf_buffer.lookup_transform(
                                    camera_frame,
                                    "map",
                                    observation_time,
                                    timeout=Duration(seconds=0.0),
                                )
                                break
                            except Exception as error:
                                last_transform_error = error
                                time.sleep(0.025)
                        map_to_camera_matrix = None
                        if transform is not None:
                            map_to_camera_matrix = _transform_matrix(
                                transform.transform
                            )
                        else:
                            # Mapping's map→odom correction is commonly
                            # published behind the newest RGB-D stamp. Compose
                            # that slow latest correction with the exact-time
                            # high-rate odom→camera pose, matching Scene's
                            # runtime projection without using latest robot
                            # motion or an out-of-band camera transform.
                            try:
                                camera_to_odom = tf_buffer.lookup_transform(
                                    "odom",
                                    camera_frame,
                                    observation_time,
                                    timeout=Duration(seconds=0.0),
                                )
                                odom_to_map = tf_buffer.lookup_transform(
                                    "map",
                                    "odom",
                                    Time(),
                                    timeout=Duration(seconds=0.0),
                                )
                                camera_to_map_matrix = (
                                    _transform_matrix(odom_to_map.transform)
                                    @ _transform_matrix(camera_to_odom.transform)
                                )
                                map_to_camera_matrix = np.linalg.inv(
                                    camera_to_map_matrix
                                )
                            except Exception as error:
                                last_transform_error = error
                        if map_to_camera_matrix is None:
                            transform_failures += 1
                            if transform_failures <= 3:
                                print(
                                    "timestamped TF unavailable for "
                                    f"{camera_frame} <- map at "
                                    f"{depth_message.header.stamp.sec}."
                                    f"{depth_message.header.stamp.nanosec:09d}: "
                                    f"{last_transform_error}",
                                    flush=True,
                                )
                        else:
                            visibility_samples += 1
                            fx, fy = float(info.k[0]), float(info.k[4])
                            cx, cy = float(info.k[2]), float(info.k[5])
                            for truth in truths:
                                evidence = rgbd_visibility_evidence(
                                    depth_m=depth,
                                    center_m=truth["center_m"],
                                    size_m=truth["size_m"],
                                    yaw_rad=float(
                                        truth.get("yaw_rad", 0.0)
                                    ),
                                    map_to_camera=map_to_camera_matrix,
                                    fx=fx,
                                    fy=fy,
                                    cx=cx,
                                    cy=cy,
                                )
                                identity = str(truth["identity"])
                                diagnostics = visibility_diagnostics[identity]
                                diagnostics["max_projected_area_px"] = max(
                                    int(
                                        diagnostics[
                                            "max_projected_area_px"
                                        ]
                                    ),
                                    evidence.projected_area_px,
                                )
                                diagnostics[
                                    "max_consistent_depth_pixels"
                                ] = max(
                                    int(
                                        diagnostics[
                                            "max_consistent_depth_pixels"
                                        ]
                                    ),
                                    evidence.consistent_depth_pixels,
                                )
                                diagnostics[
                                    "max_consistent_depth_fraction"
                                ] = max(
                                    float(
                                        diagnostics[
                                            "max_consistent_depth_fraction"
                                        ]
                                    ),
                                    evidence.consistent_depth_fraction,
                                )
                                diagnostics["last_reason"] = evidence.reason
                                # Which depths the projection expected, and
                                # what the camera actually returned there. A
                                # count of zero consistent pixels says the
                                # test failed but not whether the object was
                                # occluded, the depth frame was empty, or the
                                # projection put the object at the wrong
                                # range — three failures that need three
                                # different fixes.
                                if evidence.depth_interval_m is not None:
                                    diagnostics["last_depth_interval_m"] = [
                                        round(float(evidence.depth_interval_m[0]), 3),
                                        round(float(evidence.depth_interval_m[1]), 3),
                                    ]
                                if evidence.projected_area_px > int(
                                    diagnostics.get("_depth_sample_area_px") or 0
                                ):
                                    finite = depth[np.isfinite(depth)]
                                    diagnostics["_depth_sample_area_px"] = (
                                        evidence.projected_area_px
                                    )
                                    diagnostics["depth_frame_stats_m"] = {
                                        "finite_fraction": round(
                                            float(finite.size)
                                            / float(max(depth.size, 1)),
                                            4,
                                        ),
                                        "min": (
                                            round(float(finite.min()), 3)
                                            if finite.size
                                            else None
                                        ),
                                        "median": (
                                            round(float(np.median(finite)), 3)
                                            if finite.size
                                            else None
                                        ),
                                        "max": (
                                            round(float(finite.max()), 3)
                                            if finite.size
                                            else None
                                        ),
                                    }
                                if not evidence.visible:
                                    continue
                                visible_counts[identity] += 1
                                first_visible_s.setdefault(identity, elapsed)
                                last_visible_s[identity] = elapsed

            if now - last_print >= 5.0:
                with latest_lock:
                    fused_track = odometry_tracks["fused"]
                    position = fused_track.last_xy or (
                        float("nan"),
                        float("nan"),
                    )
                    path_length_m = fused_track.path_length_m
                visible_now = sum(
                    count >= args.min_visible_frames
                    for count in visible_counts.values()
                )
                print(
                    f"t={elapsed:.1f}s odom=({position[0]:.2f},{position[1]:.2f}) "
                    f"path={path_length_m:.2f}m "
                    f"visible={visible_now}/{len(truths)} "
                    f"clearance=f{front:.2f}/l{left:.2f}/"
                    f"r{right:.2f}/b{rear:.2f} motion={motion_mode}",
                    flush=True,
                )
                last_print = now
            time.sleep(0.04)
    finally:
        if publisher is not None:
            stop = Twist()
            for _ in range(10):
                publisher.publish(stop)
                time.sleep(0.03)
        shutdown_ros()

    visible_truth_ids = sorted(
        identity
        for identity, count in visible_counts.items()
        if count >= args.min_visible_frames
    )
    fused_track = odometry_tracks["fused"]
    output = {
        "world_id": truth_payload["world_id"],
        "truth_count": len(truths),
        "duration_s": round(time.monotonic() - start, 3),
        "path_length_m": round(fused_track.path_length_m, 3),
        "odom_discontinuity_count": len(fused_track.discontinuities),
        "max_odom_step_m": round(fused_track.max_step_m, 6),
        "max_odom_speed_mps": round(fused_track.max_speed_mps, 6),
        "odometry_sources": {
            name: track.as_dict()
            for name, track in odometry_tracks.items()
        },
        "localized_pose_agreement": localized_pose_agreement.as_dict(),
        "odometry_abort_source": odometry_abort_source,
        "aborted_reason": aborted_reason,
        "visibility_samples": visibility_samples,
        "transform_failures": transform_failures,
        "localized_transform_failures": localized_transform_failures,
        "depth_failures": depth_failures,
        "min_visible_frames": args.min_visible_frames,
        "truth_alignment": {
            "source_frame": args.robot_frame,
            "target_frame": "map",
            "translation_m": list(alignment_translation),
            "yaw_rad": alignment_yaw,
        },
        "visible_truth_ids": visible_truth_ids,
        "visibility": {
            identity: {
                "frames": visible_counts[identity],
                "first_s": round(first_visible_s[identity], 3),
                "last_s": round(last_visible_s[identity], 3),
            }
            for identity in visible_truth_ids
        },
        "visibility_diagnostics": visibility_diagnostics,
    }
    args.output.write_text(
        json.dumps(output, separators=(",", ":"), sort_keys=True),
        encoding="utf-8",
    )
    print(
        f"done path={fused_track.path_length_m:.2f}m "
        f"visible={len(visible_truth_ids)}/{len(truths)} "
        f"output={args.output}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
