#!/usr/bin/env python3
"""
Test script for the cap_pointcloud_to_file function
This script tests the PLY export functionality
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import time

def create_test_pointcloud():
    """Create a simple test pointcloud for testing purposes"""
    # Create a simple pointcloud with a few points
    points = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 1.0]
    ]
    
    # Create PointCloud2 message
    cloud = PointCloud2()
    cloud.header.frame_id = "test_frame"
    cloud.header.stamp.sec = int(time.time())
    cloud.header.stamp.nanosec = 0
    
    # Set point fields
    from sensor_msgs.msg import PointField
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
    ]
    
    # Set point data
    cloud.point_step = 16  # 4 bytes per field * 4 fields
    cloud.row_step = cloud.point_step * len(points)
    cloud.width = len(points)
    cloud.height = 1
    cloud.is_dense = True
    
    # Pack point data
    data = []
    for i, point in enumerate(points):
        data.extend([point[0], point[1], point[2], float(i)])  # x, y, z, intensity
    
    cloud.data = np.array(data, dtype=np.float32).tobytes()
    
    return cloud

def test_ply_export():
    """Test the PLY export functionality"""
    print("Testing PLY export functionality...")
    
    # Import our function
    from api import cap_pointcloud_to_file
    
    # Test with a simple filename
    test_filename = "/tmp/test_pointcloud.ply"
    
    print(f"Attempting to export pointcloud to {test_filename}")
    result = cap_pointcloud_to_file(test_filename)
    
    if result["success"]:
        print(f"✅ Successfully exported pointcloud to {test_filename}")
        # Check if file exists and has content
        import os
        if os.path.exists(test_filename):
            file_size = os.path.getsize(test_filename)
            print(f"File size: {file_size} bytes")
            if file_size > 0:
                print("✅ File has content")
                # Show first few lines
                with open(test_filename, 'r') as f:
                    lines = f.readlines()
                    print("First few lines of PLY file:")
                    for i, line in enumerate(lines[:10]):
                        print(f"  {i+1}: {line.strip()}")
            else:
                print("❌ File is empty")
        else:
            print("❌ File was not created")
    else:
        print("❌ Failed to export pointcloud")

if __name__ == "__main__":
    # Initialize ROS2
    rclpy.init()
    
    try:
        test_ply_export()
    finally:
        rclpy.shutdown()
