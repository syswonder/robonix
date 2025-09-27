import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseWithCovarianceStamped
from mcp.server.fastmcp import FastMCP
from tf_transformations import euler_from_quaternion

from tf2_ros import Buffer, TransformListener

from robonix.manager.eaios_decorators import eaios
from typing import Optional, Tuple


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
