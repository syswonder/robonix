#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


import time

class HelloWorldNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.count = 0
        self.name = "World"
        self.running = True
        
        # 创建文件写入定时器（每秒触发）
        self.timer = self.create_timer(2.0, self.write_to_file)
        
        # 创建服务接口
        self.modify_name_srv = self.create_service(
            Trigger, 
            'modify_name', 
            self.modify_name_callback
        )
        
        self.get_count_srv = self.create_service(
            Trigger, 
            'get_count', 
            self.get_count_callback
        )
        
        self.shutdown_srv = self.create_service(
            Trigger, 
            'shutdown_node', 
            self.shutdown_callback
        )

    def write_to_file(self):
        if not self.running:
            self.destroy_node()
            return
        content = f"hello {self.name} {self.count}"
        with open('output.txt', 'a') as f:
            f.write(content + '\n')
        self.count += 1
        self.get_logger().info(f"Written to file: {content}")

    # 服务回调函数
    def modify_name_callback(self, req, res):
        # 直接访问字符串字段 req.new_name
        self.name = "ROS2"
        res.success = True
        res.message = f"Name updated to {self.name}"
        self.get_logger().info(f"Received new name: {self.name}")
        return res

    def get_count_callback(self, req, res):
        res.success = True
        res.message = f"Current count: {self.count}"
        return res

    def shutdown_callback(self, req, res):
        self.running = False
        res.success = True
        res.message = "Shutdown signal received"
        
        return res

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()