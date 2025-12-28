#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# RGB Publisher
#
# Demo RGB camera package that publishes random color images.
""""""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import random
import time


class RGBPublisher(Node):
    """Publishes random color images to simulate an RGB camera."""

    def __init__(self):
        super().__init__('demo_rgb_provider')
        self.publisher_ = self.create_publisher(Image, '/demo_rgb/image', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Demo RGB camera package started. Publishing to /demo_rgb/image')

    def timer_callback(self):
        """Generate and publish a random color image."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        
        # Image dimensions: 640x480 RGB
        width = 640
        height = 480
        channels = 3
        
        # Generate random color (same color for entire image for simplicity)
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        
        # Create image data (row-major order)
        msg.width = width
        msg.height = height
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = width * channels  # bytes per row
        
        # Fill image with random color
        image_data = np.zeros((height, width, channels), dtype=np.uint8)
        image_data[:, :, 0] = r  # Red channel
        image_data[:, :, 1] = g  # Green channel
        image_data[:, :, 2] = b  # Blue channel
        
        msg.data = image_data.tobytes()
        
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published RGB image: R={r}, G={g}, B={b}')


def main(args=None):
    rclpy.init(args=args)
    rgb_publisher = RGBPublisher()
    
    try:
        rclpy.spin(rgb_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        rgb_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

