#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from mcp.server.fastmcp import FastMCP

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import Range
import sys

import sys
root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
if root_dir not in sys.path:
    sys.path.append(root_dir)
print(root_dir)
from DeepEmbody.manager.eaios_decorators import eaios

#TODO memory
mamory = {}

@eaios.api
@eaios.caller
def test_nv():
    # res = nv_test()
    # print("lhe debug in skill test nv res",res)
    # return res
    return nv_test()

#315曹老师办公室 28.3 0.1 0
if __name__ == "__main__":
    # 初始化并运行 server
   test_nv()