#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from driver import BM01Driver


class BM01Node(Node):
    """ROS2 Node for BM01 Pressure Sensor"""
    
    def __init__(self):
        super().__init__('bm01_node')
        self.declare_parameter('gpio_pin', 20)
        self.gpio_pin = self.get_parameter('gpio_pin').value
        self.pressure_pub = self.create_publisher(Bool, 'pressure_detected', 10)
        self.pressure_count = 0
        self.last_pressure_state = False
        try:
            self.driver = BM01Driver(gpio_pin=self.gpio_pin, callback=self.pressure_callback, debounce_ms=5)
            self.get_logger().info(f'BM01 driver initialized on GPIO {self.gpio_pin}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BM01 driver: {e}')
            raise
        
        self.timer = self.create_timer(0.0001, self.check_sensor_state)
        
        self.get_logger().info('BM01 ROS2 node started - waiting for pressure detection...')
    
    def pressure_callback(self):
        self.get_logger().info(f'Pressure detected!')
    
    def check_sensor_state(self):
        try:
            current_state = self.driver.read_digital()
            if current_state != self.last_pressure_state:
                self.get_logger().info(f'Sensor state changed: {self.last_pressure_state} -> {current_state}')
                self.last_pressure_state = current_state
                # if current_state is True, publish a message
                if current_state:
                    self.pressure_pub.publish(Bool(data=True))
                else:
                    self.pressure_pub.publish(Bool(data=False))
        except Exception as e:
            self.get_logger().error(f'Error reading sensor state: {e}')
    
    def cleanup(self):
        if hasattr(self, 'driver'):
            self.driver.cleanup()


def main(args=None):
    rclpy.init(args=args)
    
    node = BM01Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 