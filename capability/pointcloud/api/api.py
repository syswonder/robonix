#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from Robonix.manager.eaios_decorators import eaios


class PointCloudGetter(Node):
    """Node to get pointcloud data from ROS2 topics"""

    def __init__(self, topic_name="/livox/lidar"):
        super().__init__('pointcloud_getter')
        self.pointcloud_data = None
        self._event = threading.Event()
        self.subscription = self.create_subscription(
            PointCloud2,
            topic_name,
            self.pointcloud_callback,
            10
        )

    def pointcloud_callback(self, msg):
        """Callback function to receive pointcloud data"""
        self.pointcloud_data = msg
        self.get_logger().info(f"Got pointcloud with {len(msg.data)} bytes")
        self._event.set()
        self.destroy_subscription(self.subscription)


def write_ply_file(filename, points, colors=None):
    """
    Write pointcloud data to PLY file format

    Args:
        filename (str): Output PLY filename
        points (numpy.ndarray): Nx3 array of XYZ coordinates
        colors (numpy.ndarray, optional): Nx3 array of RGB colors (0-255)

    Returns:
        bool: True if successful, False otherwise
    """
    try:
        with open(filename, 'w') as f:
            # Write PLY header
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

            # Write vertex data
            for i in range(len(points)):
                x, y, z = points[i]
                if colors is not None:
                    r, g, b = colors[i]
                    f.write(
                        f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n")
                else:
                    f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

        return True
    except Exception as e:
        print(f"Error writing PLY file: {e}")
        return False


@eaios.api
def cap_pointcloud_to_file(filename):
    """
    Convert current pointcloud to a .ply 3D model file

    Args:
        filename (str): Output filename for the PLY file

    Returns:
        dict: {"success": bool} indicating if the operation was successful
    """
    # Default topic for mid360 pointcloud data
    topic = "/livox/lidar"

    # Create node to get pointcloud data
    node = PointCloudGetter(topic)

    # Wait for pointcloud data with timeout
    timeout_sec = 10.0
    end_time = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)

    while rclpy.ok() and node.pointcloud_data is None and node.get_clock().now().nanoseconds < end_time:
        rclpy.spin_once(node, timeout_sec=1.0)
        if node._event.is_set():
            break

    if node.pointcloud_data is None:
        node.get_logger().error("Timeout waiting for pointcloud data")
        node.destroy_node()
        return {"success": False}

    try:
        # Convert PointCloud2 to numpy arrays
        points = point_cloud2.read_points_list(
            node.pointcloud_data,
            field_names=("x", "y", "z"),
            skip_nans=True
        )

        # Convert to numpy array
        points_array = np.array(points)

        if len(points_array) == 0:
            node.get_logger().error("No valid points found in pointcloud")
            node.destroy_node()
            return {"success": False}

        # Try to extract intensity/reflectivity for coloring
        colors = None
        try:
            intensity_points = point_cloud2.read_points_list(
                node.pointcloud_data,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True
            )
            if len(intensity_points) > 0:
                # Normalize intensity to 0-255 for coloring
                intensities = np.array([p[3] for p in intensity_points])
                if len(intensities) > 0:
                    # Normalize intensity to 0-255 range
                    min_intensity = np.min(intensities)
                    max_intensity = np.max(intensities)
                    if max_intensity > min_intensity:
                        normalized_intensity = ((intensities - min_intensity) /
                                                (max_intensity - min_intensity) * 255)
                        # Create grayscale colors based on intensity
                        colors = np.column_stack([
                            normalized_intensity,
                            normalized_intensity,
                            normalized_intensity
                        ])
        except Exception as e:
            node.get_logger().warn(f"Could not extract intensity data: {e}")

        # Write PLY file
        success = write_ply_file(filename, points_array, colors)

        if success:
            node.get_logger().info(
                f"Successfully saved {len(points_array)} points to {filename}")
        else:
            node.get_logger().error(f"Failed to save PLY file: {filename}")

        node.destroy_node()
        return {"success": success}

    except Exception as e:
        node.get_logger().error(f"Error processing pointcloud: {e}")
        node.destroy_node()
        return {"success": False}
