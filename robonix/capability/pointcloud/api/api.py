#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from robonix.manager.eaios_decorators import eaios


class PointCloudGetter(Node):
    """Node to get pointcloud data from ROS2 topics"""

    def __init__(
        self, topic_name="/scanner/cloud", max_frames=10, collection_time=30.0
    ):
        super().__init__("pointcloud_getter")
        self.pointcloud_data_list = []
        self.max_frames = max_frames
        self.collection_time = collection_time
        self.start_time = None
        self._event = threading.Event()
        self.subscription = self.create_subscription(
            PointCloud2, topic_name, self.pointcloud_callback, 10
        )

    def pointcloud_callback(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        self.pointcloud_data_list.append(msg)
        self.get_logger().info(
            f"Collected frame {len(self.pointcloud_data_list)} with {len(msg.data)} bytes"
        )

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        if (
            len(self.pointcloud_data_list) >= self.max_frames
            or elapsed_time >= self.collection_time
        ):
            self.get_logger().info(
                f"Finished collecting {len(self.pointcloud_data_list)} frames in {elapsed_time:.2f}s"
            )
            self._event.set()


def write_ply_file(filename, points, colors=None):
    """Write pointcloud data to PLY file format"""
    try:
        with open(filename, "w") as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")

            if colors is not None:
                f.write("property uchar red\n")
                f.write("property uchar green\n")
                f.write("property uchar blue\n")

            f.write("end_header\n")
            for i in range(len(points)):
                x, y, z = points[i]
                if colors is not None:
                    r, g, b = colors[i]
                    f.write(f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n")
                else:
                    f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

        return True
    except Exception as e:
        print(f"Error writing PLY file: {e}")
        return False


@eaios.api
def cap_pointcloud_to_file(filename, max_frames=5, collection_time=60.0):
    """Convert current pointcloud to a .ply 3D model file"""
    topic = "/scanner/cloud"
    max_frames = min(max_frames, 25)

    node = PointCloudGetter(
        topic, max_frames=max_frames, collection_time=collection_time
    )

    timeout_sec = collection_time + 10.0
    end_time = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)

    while (
        rclpy.ok()
        and not node._event.is_set()
        and node.get_clock().now().nanoseconds < end_time
    ):
        rclpy.spin_once(node, timeout_sec=0.1)

    if hasattr(node, "subscription"):
        node.destroy_subscription(node.subscription)

    if len(node.pointcloud_data_list) == 0:
        node.get_logger().error("Timeout waiting for pointcloud data")
        node.destroy_node()
        return {"success": False, "points_collected": 0}

    try:
        all_points = []
        all_intensities = []

        node.get_logger().info(
            f"Processing {len(node.pointcloud_data_list)} pointcloud frames..."
        )

        for i, pointcloud_msg in enumerate(node.pointcloud_data_list):
            points = point_cloud2.read_points_list(
                pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True
            )

            if len(points) > 0:
                all_points.extend(points)
                node.get_logger().info(f"Frame {i+1}: {len(points)} points")

            try:
                intensity_points = point_cloud2.read_points_list(
                    pointcloud_msg,
                    field_names=("x", "y", "z", "intensity"),
                    skip_nans=True,
                )
                if len(intensity_points) > 0:
                    intensities = [p[3] for p in intensity_points]
                    all_intensities.extend(intensities)
            except Exception as e:
                node.get_logger().warn(
                    f"Could not extract intensity data from frame {i+1}: {e}"
                )

        points_array = np.array(all_points)

        if len(points_array) == 0:
            node.get_logger().error("No valid points found in any pointcloud frame")
            node.destroy_node()
            return {"success": False, "points_collected": 0}

        colors = None
        if len(all_intensities) > 0 and len(all_intensities) == len(points_array):
            intensities = np.array(all_intensities)
            min_intensity = np.min(intensities)
            max_intensity = np.max(intensities)
            if max_intensity > min_intensity:
                normalized_intensity = (
                    (intensities - min_intensity)
                    / (max_intensity - min_intensity)
                    * 255
                )
                colors = np.column_stack(
                    [normalized_intensity, normalized_intensity, normalized_intensity]
                )

        success = write_ply_file(filename, points_array, colors)

        if success:
            node.get_logger().info(
                f"Successfully saved {len(points_array)} points from {len(node.pointcloud_data_list)} frames to {filename}"
            )
        else:
            node.get_logger().error(f"Failed to save PLY file: {filename}")

        node.destroy_node()
        return {"success": success, "points_collected": len(points_array)}

    except Exception as e:
        node.get_logger().error(f"Error processing pointcloud: {e}")
        node.destroy_node()
        return {"success": False, "points_collected": 0}
