# SPDX-License-Identifier: MulanPSL-2.0
"""Publish the simulator's own answer for where the robot is.

Webots knows the robot's exact pose; nothing in this deployment was passing
that out. The URDF carries a `<gazebo>` block declaring a ground-truth
odometry plugin, but Webots ignores `<gazebo>` entirely, so the topic never
existed and every consumer saw an empty stream. Scoring perception needs a
reference the estimator cannot influence: without one, a scene benchmark
cannot tell "the object is where we said" from "the robot thinks it is
somewhere else".

This is a reference signal for evaluation, not a localization source. Nothing
in the navigation or mapping stack should consume it — doing so would hide the
drift the reference exists to measure.
"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry

DEFAULT_TOPIC = "/webots/ground_truth/odom"
DEFAULT_FRAME_ID = "world"
DEFAULT_CHILD_FRAME_ID = "base_footprint"


def _quaternion_from_webots_orientation(
    orientation: list[float],
) -> tuple[float, float, float, float]:
    """Convert Webots' row-major 3x3 rotation matrix to (x, y, z, w).

    Uses Shepperd's method: pick the branch whose divisor is largest so the
    square root never runs against a near-zero denominator. The naive
    trace-only formula loses all precision near a 180-degree rotation, which
    the robot reaches routinely just by driving in a loop.
    """
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = orientation
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)


class GroundTruthOdometry:
    """webots_ros2_driver plugin publishing the supervisor's robot pose.

    `init` is called once with the driver's Webots node; `step` runs every
    simulation step. Requires `supervisor TRUE` on the robot node — without it
    `getSelf()` returns None and the plugin publishes nothing rather than
    raising, because taking down the whole driver would cost the deployment
    its sensors to gain a diagnostic.
    """

    def init(self, webots_node, properties):  # noqa: D102 - driver-called hook
        self._robot = webots_node.robot
        self._node = rclpy.create_node("webots_ground_truth_odometry")
        # Stamps must come from the same clock as every other topic in the
        # deployment, or consumers that synchronize by timestamp reject every
        # sample they receive.
        self._node.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", value=True)]
        )

        self._topic = properties.get("topic", DEFAULT_TOPIC)
        self._frame_id = properties.get("frameId", DEFAULT_FRAME_ID)
        self._child_frame_id = properties.get("childFrameId", DEFAULT_CHILD_FRAME_ID)

        self._self_node = None
        get_self = getattr(self._robot, "getSelf", None)
        if get_self is not None:
            self._self_node = get_self()
        if self._self_node is None:
            self._publisher = None
            self._node.get_logger().warn(
                "ground-truth odometry disabled: this robot is not a supervisor "
                "(set `supervisor TRUE` on the robot node to enable it)"
            )
            return

        self._publisher = self._node.create_publisher(Odometry, self._topic, 10)
        self._node.get_logger().info(
            f"publishing ground-truth odometry on {self._topic} "
            f"({self._frame_id} -> {self._child_frame_id})"
        )

    def step(self):  # noqa: D102 - driver-called hook
        rclpy.spin_once(self._node, timeout_sec=0)
        if self._publisher is None:
            return

        position = self._self_node.getPosition()
        orientation = self._self_node.getOrientation()
        qx, qy, qz, qw = _quaternion_from_webots_orientation(orientation)

        message = Odometry()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.child_frame_id = self._child_frame_id
        message.pose.pose.position.x = float(position[0])
        message.pose.pose.position.y = float(position[1])
        message.pose.pose.position.z = float(position[2])
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        # Twist is left zero: the supervisor exposes velocity only for physics
        # -enabled solids, and consumers of this topic compare positions.
        self._publisher.publish(message)
