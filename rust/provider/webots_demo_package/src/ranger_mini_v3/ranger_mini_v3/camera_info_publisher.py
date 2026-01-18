#!/usr/bin/env python3
"""
Camera info publisher for webots cameras.
Publishes camera_info messages for RGB and depth cameras.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # Front camera RGB info publisher
        self.front_rgb_info_pub = self.create_publisher(
            CameraInfo,
            '/head_front_camera/rgb/camera_info',
            10
        )
        
        # Front camera depth info publisher
        self.front_depth_info_pub = self.create_publisher(
            CameraInfo,
            '/head_front_camera/depth_registered/camera_info',
            10
        )
        
        # Back camera RGB info publisher
        self.back_rgb_info_pub = self.create_publisher(
            CameraInfo,
            '/head_back_camera/rgb/camera_info',
            10
        )
        
        # Back camera depth info publisher
        self.back_depth_info_pub = self.create_publisher(
            CameraInfo,
            '/head_back_camera/depth_registered/camera_info',
            10
        )
        
        # Create timer to publish camera info periodically
        self.timer = self.create_timer(0.1, self.publish_camera_info)
        
        # Typical camera parameters (adjust based on your camera)
        # These are example values, should match your actual camera
        self.image_width = 640
        self.image_height = 480
        self.fx = 525.0  # Focal length in pixels
        self.fy = 525.0
        self.cx = 320.0  # Principal point
        self.cy = 240.0
        
        self.get_logger().info('Camera info publisher started')
    
    def create_camera_info(self, frame_id):
        """Create a CameraInfo message with typical parameters"""
        msg = CameraInfo()
        msg.header = Header()
        msg.header.frame_id = frame_id
        msg.width = self.image_width
        msg.height = self.image_height
        
        # Camera matrix (3x3)
        msg.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0
        ]
        
        # Distortion model (plumb_bob)
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        
        # Rectification matrix (identity)
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Projection matrix (3x4)
        msg.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        return msg
    
    def publish_camera_info(self):
        """Publish camera info for all cameras"""
        now = self.get_clock().now()
        
        # Front RGB camera
        front_rgb_info = self.create_camera_info('head_front_camera_rgb_optical_frame')
        front_rgb_info.header.stamp = now.to_msg()
        self.front_rgb_info_pub.publish(front_rgb_info)
        
        # Front depth camera
        front_depth_info = self.create_camera_info('head_front_camera_depth_optical_frame')
        front_depth_info.header.stamp = now.to_msg()
        self.front_depth_info_pub.publish(front_depth_info)
        
        # Back RGB camera
        back_rgb_info = self.create_camera_info('head_back_camera_rgb_optical_frame')
        back_rgb_info.header.stamp = now.to_msg()
        self.back_rgb_info_pub.publish(back_rgb_info)
        
        # Back depth camera
        back_depth_info = self.create_camera_info('head_back_camera_depth_optical_frame')
        back_depth_info.header.stamp = now.to_msg()
        self.back_depth_info_pub.publish(back_depth_info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
