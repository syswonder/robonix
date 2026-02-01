#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Transform Scan Service
#
# Serves many callers: single-call CONVERT or per-client stream. Each stream client
# gets a dedicated output_topic and stream_id; STOP_STREAM releases resources.

import math
import time
import uuid
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
from robonix_sdk.srv import TransformScan

ACTION_CONVERT = 0
ACTION_START_STREAM = 1
ACTION_STOP_STREAM = 2

STREAM_NS = "/demo_service/transform_scan/stream"


def pointcloud_to_laserscan(cloud: PointCloud2, angle_min=-math.pi, angle_max=math.pi,
                            num_bins=360, range_min=0.1, range_max=10.0) -> LaserScan:
    """Convert PointCloud2 to LaserScan by projecting to 2D and binning by angle (min range per bin)."""
    try:
        points = point_cloud2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=True
        )
        pts = np.array(list(points), dtype=np.float64)
    except Exception:
        return None
    if pts.size == 0:
        return None
    x, y = pts[:, 0], pts[:, 1]
    ranges = np.sqrt(x * x + y * y)
    angles = np.arctan2(y, x)
    angle_inc = (angle_max - angle_min) / num_bins
    out = LaserScan()
    out.header = cloud.header
    out.angle_min = angle_min
    out.angle_max = angle_max
    out.angle_increment = angle_inc
    out.time_increment = 0.0
    out.scan_time = 0.0
    out.range_min = range_min
    out.range_max = range_max
    out.ranges = [float("inf")] * num_bins
    for i in range(num_bins):
        a_lo = angle_min + i * angle_inc
        a_hi = angle_min + (i + 1) * angle_inc
        mask = (angles >= a_lo) & (angles < a_hi) & (ranges >= range_min) & (ranges <= range_max)
        if np.any(mask):
            out.ranges[i] = float(np.min(ranges[mask]))
    return out


class StreamSlot:
    """One stream client: sub to input_topic, pub to dedicated output_topic."""

    def __init__(self, node: Node, stream_id: str, input_topic: str):
        self.stream_id = stream_id
        self.input_topic = input_topic
        self.output_topic = f"{STREAM_NS}/{stream_id}"
        self._node = node
        self._pub = node.create_publisher(LaserScan, self.output_topic, 10)
        self._sub = node.create_subscription(
            PointCloud2, input_topic, self._on_cloud, 10
        )

    def _on_cloud(self, msg: PointCloud2):
        scan = pointcloud_to_laserscan(msg)
        if scan is not None:
            self._pub.publish(scan)

    def destroy(self):
        self._node.destroy_publisher(self._pub)
        self._node.destroy_subscription(self._sub)


class TransformScanService(Node):
    """Transform scan: CONVERT (one shot), START_STREAM (per-client output_topic), STOP_STREAM (release)."""

    def __init__(self):
        super().__init__("demo_transform_scan_service")
        self._streams = {}  # stream_id -> StreamSlot
        self._latest_cloud = None
        self._single_sub = None
        self._single_topic = None
        self.srv = self.create_service(
            TransformScan, "/demo_service/transform_scan/convert", self.srv_callback
        )
        self.get_logger().info(
            "Transform scan service: /demo_service/transform_scan/convert "
            "(action 0=CONVERT 1=START_STREAM 2=STOP_STREAM)"
        )

    def srv_callback(self, request, response):
        response.success = False
        response.scan = LaserScan()
        response.output_topic = ""
        response.stream_id = ""

        if request.action == ACTION_CONVERT:
            return self._do_convert(request, response)
        if request.action == ACTION_START_STREAM:
            return self._do_start_stream(request, response)
        if request.action == ACTION_STOP_STREAM:
            return self._do_stop_stream(request, response)

        self.get_logger().warn(f"Unknown action {request.action}")
        return response

    def _do_convert(self, request, response):
        cloud = None
        if request.input_topic and request.input_topic.strip():
            topic = request.input_topic.strip()
            if self._single_topic != topic:
                if self._single_sub is not None:
                    self.destroy_subscription(self._single_sub)
                    self._single_sub = None
                self._latest_cloud = None
                self._single_topic = topic
                self._single_sub = self.create_subscription(
                    PointCloud2, topic, self._cloud_cb, 10
                )
            deadline = time.monotonic() + 5.0
            while self._latest_cloud is None and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            cloud = self._latest_cloud
        else:
            if request.pointcloud.width * request.pointcloud.height > 0:
                cloud = request.pointcloud
        if cloud is None:
            return response
        scan = pointcloud_to_laserscan(cloud)
        if scan is None:
            return response
        response.success = True
        response.scan = scan
        return response

    def _do_start_stream(self, request, response):
        input_topic = (request.input_topic or "").strip()
        if not input_topic:
            self.get_logger().warn("START_STREAM requires input_topic")
            return response
        stream_id = str(uuid.uuid4())
        try:
            slot = StreamSlot(self, stream_id, input_topic)
            self._streams[stream_id] = slot
            response.success = True
            response.output_topic = slot.output_topic
            response.stream_id = stream_id
            self.get_logger().info(
                f"START_STREAM stream_id={stream_id} input={input_topic} output={slot.output_topic}"
            )
        except Exception as e:
            self.get_logger().error(f"START_STREAM failed: {e}")
        return response

    def _do_stop_stream(self, request, response):
        stream_id = (request.stream_id or "").strip()
        if not stream_id:
            self.get_logger().warn("STOP_STREAM requires stream_id")
            return response
        slot = self._streams.pop(stream_id, None)
        if slot is None:
            self.get_logger().warn(f"STOP_STREAM unknown stream_id={stream_id}")
            return response
        slot.destroy()
        response.success = True
        self.get_logger().info(f"STOP_STREAM released stream_id={stream_id}")
        return response

    def _cloud_cb(self, msg):
        self._latest_cloud = msg


def main(args=None):
    rclpy.init(args=args)
    node = TransformScanService()
    try:
        rclpy.spin(node)
    finally:
        for slot in list(node._streams.values()):
            slot.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
