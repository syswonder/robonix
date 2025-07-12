#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from mcp.server.fastmcp import FastMCP

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
import sys
import time


mcp = FastMCP(name="demo_hello",
                host="127.0.0.1",
                port=8001,
                sse_path="/sse",
                message_path="/messages/")


class NodeController(Node):
    def __init__(self):
        super().__init__('node_controller')


    def call_service(self, service_name, request):
        if service_name == "get_count_hello":
            client = self.create_client(Trigger, service_name)
        elif service_name == "modify_name_hello":
            client = self.create_client(Trigger, service_name)
        elif service_name == "shutdown_node_hello":
            client = self.create_client(Trigger, service_name)
        
        # 调用服务
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Service {service_name} not available')
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Service {service_name} called successfully')
            return future.result()
        else:
            self.get_logger().error(f'Service {service_name} call failed')
            return None

@mcp.tool()
def api_get_hello_status() -> str:
    """统计hello的状态
    Args:
        None
    """
    func_status = "hello api_get_hello_status call ok"

    return func_status

@mcp.tool()
def api_change_hello(name: str) -> str:
    """修改hello的对象
    Args:
        name: 新的hello名称
    """
    func_status = "hello api_change_hello call ok"
    return func_status

@mcp.tool()
def api_close_hello() -> str:
    """关闭hello的node对象
    Args:
        None
    """
    func_status = "hello api_close_hello call ok"
    return func_status

if __name__ == "__main__":
    # 初始化并运行 server
    mcp.run(transport="sse")