# SPDX-License-Identifier: MulanPSL-2.0
"""Webots evaluation ground-truth plugin.

Publishes the robot's supervisor-read pose as nav_msgs/Odometry on
``/webots/ground_truth/odom``. The Scene benchmark sweep synchronizes
Scene's localized pose against this stream and derives object
visibility from it; without it every evaluation gate fails with
"ground_truth odometry has 0 samples".

Reconstruction of an unversioned original: the node name, topic, rate,
and frame contract are taken from the recorded trial logs
(``[webots_ground_truth] ... publishing on /webots/ground_truth/odom``)
and from ``testing/run_webots_scene_sweep.py``, the consumer.

Requires ``supervisor TRUE`` on the robot node in the world file; when
the supervisor API is unavailable the plugin logs once and stays
silent instead of crashing the controller.
"""
import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry


def _yaw_to_quaternion(axis_angle):
    import math
    # Webots getOrientation() returns a 3x3 rotation matrix (row-major).
    r = axis_angle
    yaw = math.atan2(r[3], r[0])
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        rclpy.init(args=None)
        self.__node = rclpy.create_node("webots_ground_truth")
        # The mapping stack runs on simulated time; stamping ground truth
        # with the wall clock would make every timestamp pairing fail.
        from rclpy.parameter import Parameter
        self.__node.set_parameters([Parameter("use_sim_time", value=True)])
        self.__publisher = self.__node.create_publisher(
            Odometry, "/webots/ground_truth/odom", 10
        )
        self.__self_node = None
        try:
            self.__self_node = self.__robot.getSelf()
        except Exception:  # noqa: BLE001
            self.__self_node = None
        if self.__self_node is None:
            self.__node.get_logger().warning(
                "supervisor API unavailable (robot node needs "
                "'supervisor TRUE'); ground truth will not be published"
            )
        else:
            self.__node.get_logger().info(
                "Webots evaluation ground truth publishing on "
                "/webots/ground_truth/odom"
            )
        # The Webots driver publishes TF for its native device frames
        # ("Astra depth", "Astra rgb") but the camera messages carry the
        # conventional optical frame ids. Bridge them with static TF so
        # evaluators can project map points through the optical frames.
        # Webots device frames are FLU (x fwd, y left, z up); the ROS
        # optical convention is z fwd, x right, y down.
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import StaticTransformBroadcaster

        self.__static_tf = StaticTransformBroadcaster(self.__node)
        transforms = []
        for parent, child in (
            ("Astra depth", "head_front_camera_depth_optical_frame"),
            ("Astra rgb", "head_front_camera_rgb_optical_frame"),
        ):
            t = TransformStamped()
            t.header.stamp = self.__node.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = child
            # FLU -> optical: quaternion (x,y,z,w) = (-0.5, 0.5, -0.5, 0.5)
            t.transform.rotation.x = -0.5
            t.transform.rotation.y = 0.5
            t.transform.rotation.z = -0.5
            t.transform.rotation.w = 0.5
            transforms.append(t)
        self.__static_tf.sendTransform(transforms)
        # rmw_zenoh's transient_local latching is unreliable: a listener
        # that joins after the one-shot broadcast may never receive it
        # (observed as visibility=0 runs alternating with healthy ones).
        # Re-broadcasting a static transform is harmless, so do it
        # periodically from step().
        self.__static_transforms = transforms
        self.__last_static_pub = -10.0

        # ~25 Hz at the usual 8 ms basic timestep.
        self.__period_s = 0.04
        self.__last_pub = -1.0

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        now_s = self.__robot.getTime()
        if now_s - self.__last_static_pub >= 2.0:
            self.__last_static_pub = now_s
            stamp = self.__node.get_clock().now().to_msg()
            for t in self.__static_transforms:
                t.header.stamp = stamp
            self.__static_tf.sendTransform(self.__static_transforms)
        if self.__self_node is None:
            return
        now = self.__robot.getTime()
        if self.__last_pub >= 0.0 and (now - self.__last_pub) < self.__period_s:
            return
        self.__last_pub = now
        position = self.__self_node.getPosition()
        orientation = self.__self_node.getOrientation()
        msg = Odometry()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = float(position[2])
        msg.pose.pose.orientation = _yaw_to_quaternion(orientation)
        self.__publisher.publish(msg)
