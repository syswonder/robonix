# SPDX-License-Identifier: MulanPSL-2.0
"""Example camera vendor: implements prm::camera (rgb, depth, rgbd, intrinsics).
Does NOT implement ir - hardware does not support.

Code structure:
- [Generated] ridlc generates: create_*_publisher, RobonixRuntimeStub, msg types
- [You write] In main(): connect to meta, create publishers, publish(msg) in timer callback
- [Hook] Stream has no execute; just call publisher.publish(msg) in loop/timer
"""

import os
import sys

# [Optional] Only needed when running directly with python (not via rbnx start); rbnx start sources install, skip this
def _setup_path():
    try:
        import robonix_runtime_pb2_grpc  # noqa: F401
    except ImportError:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_root = os.path.dirname(os.path.dirname(script_dir))
        rust_dir = os.path.dirname(pkg_root)
        python_pkg = os.path.join(
            rust_dir, "robonix-server", "target", "rclrs_interfaces_ws", "python_pkg"
        )
        if os.path.exists(python_pkg) and python_pkg not in sys.path:
            sys.path.insert(0, python_pkg)


_setup_path()

import grpc
import rclpy

from robonix.prm.camera.rgb_stream import create_rgb_publisher
from robonix.prm.camera.depth_stream import create_depth_publisher
from robonix.prm.camera.rgbd_stream import create_rgbd_publisher
from robonix.prm.camera.intrinsics_stream import create_intrinsics_publisher
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from sensor_msgs.msg import Image, CameraInfo
from robonix_msgs.msg import RGBD


def main() -> None:
    endpoint = os.environ.get("ROBONIX_META_GRPC_ENDPOINT", "127.0.0.1:50051")
    node_id = os.environ.get("ROBONIX_NODE_ID", "com.robonix.example.camera")

    # [Required] 1) Init ROS2  2) Connect to meta gRPC  3) Get runtime_client (for channel registration)
    rclpy.init()
    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)

    # [Generated] create_*_publisher returns publisher; call publish(msg) directly
    rgb_pub = create_rgb_publisher(runtime_client, node_id=node_id)
    depth_pub = create_depth_publisher(runtime_client, node_id=node_id)
    rgbd_pub = create_rgbd_publisher(runtime_client, node_id=node_id)
    intrinsics_pub = create_intrinsics_publisher(runtime_client, node_id=node_id)
    # NOT registering ir - hardware does not support

    # [You write] Timer callback: capture frame, build msg, publish
    def publish_mock_frame():
        # Real impl: rgb, depth = camera.capture()
        rgb_msg = Image()
        rgb_msg.header.frame_id = "camera_optical_frame"
        rgb_msg.header.stamp = rgb_pub.get_clock().now().to_msg()
        rgb_msg.height = 480
        rgb_msg.width = 640
        rgb_msg.encoding = "rgb8"
        rgb_msg.step = 640 * 3
        rgb_msg.data = bytes(640 * 480 * 3)

        depth_msg = Image()
        depth_msg.header = rgb_msg.header
        depth_msg.height = 480
        depth_msg.width = 640
        depth_msg.encoding = "16UC1"
        depth_msg.step = 640 * 2
        depth_msg.data = bytes(640 * 480 * 2)

        rgb_pub.publish(rgb_msg)
        depth_pub.publish(depth_msg)
        rgbd_pub.publish(RGBD(rgb=rgb_msg, depth=depth_msg))

        info_msg = CameraInfo()
        info_msg.header = rgb_msg.header
        info_msg.height = 480
        info_msg.width = 640
        info_msg.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        intrinsics_pub.publish(info_msg)

    timer = rgb_pub.create_timer(0.5, publish_mock_frame)

    try:
        rclpy.spin(rgb_pub)
    finally:
        timer.cancel()
        rgb_pub.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
