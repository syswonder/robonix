import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseWithCovarianceStamped
from mcp.server.fastmcp import FastMCP

from tf2_ros import Buffer, TransformListener

from robonix.manager.eaios_decorators import eaios
from typing import Optional, Tuple, Iterable

import math

def euler_from_quaternion(q: Iterable[float]) -> Tuple[float, float, float]:
    """
    Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)
    in radians. Convention matches tf_transformations: XYZ (roll, pitch, yaw).
    """
    x, y, z, w = q

    # (optional) normalize to avoid drift; safe for nearly-normalized inputs
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0.0:
        raise ValueError("Invalid quaternion with zero norm")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm

    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    # clamp for numerical safety (|sinp| could be 1+epsilon)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw



class TfPoseGetter(Node):
    def __init__(self):
        super().__init__('tf_pose_getter')
        self.target_frame = 'map'
        self.source_frame = 'base_link'  # or 'base_footprint'
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def try_lookup(self, per_try_timeout_sec: float = 0.2) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),  # now
                timeout=Duration(seconds=per_try_timeout_sec)
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            return (t.x, t.y, yaw)
        except Exception:
            return None


@eaios.api
def get_pose(timeout_sec: float = 20.0) -> Optional[Tuple[float, float, float]]:
    """
    Get current robot pose (x, y, yaw) in map frame.
    Follows ROS2 tf standard: X forward, Y left, Z up, yaw in radians.
    Returns None on timeout.
    """
    init_here = not rclpy.ok()
    if init_here:
        rclpy.init()

    node = TfPoseGetter()

    end_time_ns = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
    result: Optional[Tuple[float, float, float]] = None

    while rclpy.ok() and node.get_clock().now().nanoseconds < end_time_ns:
        rclpy.spin_once(node, timeout_sec=0.05)
        result = node.try_lookup(per_try_timeout_sec=0.15)
        if result is not None:
            break

    node.destroy_node()
    if init_here:
        rclpy.shutdown()

    return result
