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

from robonix.manager.eaios_decorators import eaios
from robonix.uapi.graph.entity import Entity

from typing import Optional, Tuple

#TODO memory
memory = {}

@eaios.api
@eaios.caller
def move_to_goal(self_entity, goal_name:str) -> str:
    if goal_name in memory.keys():
        return move_to_ab_pos(self_entity, memory[goal_name])
    else:
        return f"Service setmove_to_goal_goal response: {False}, message: goal not in memory"

@eaios.api
@eaios.caller
def move_to_ab_pos(self_entity, x, y, yaw) -> str:
    result = self_entity.cap_set_goal(x=x,y=y,yaw=yaw)
    if result is True:
        return "Movement command sent successfully"
    elif result is False:
        return "Movement command failed"
    else:
        return str(result) if result is not None else "Movement command sent"

@eaios.api
@eaios.caller
def move_to_rel_pos(self_entity, dx,dy,dyaw) -> str:
    current_pos = self_entity.cap_get_pose()
    if current_pos is None:
        return "Failed to get current pose"
    # current_pos is a tuple (x, y, yaw)
    x, y, yaw = current_pos
    result = self_entity.cap_set_goal(x=x + dx, y=y + dy, yaw=yaw + dyaw)
    if result is True:
        return "Movement command sent successfully"
    elif result is False:
        return "Movement command failed"
    else:
        return str(result) if result is not None else "Movement command sent"

def test():
    rclpy.init()
    node = NodeController()

    # ros2 service list
    # ros2 service type /get_count
    # ros2 service call get_count std_srvs/srv/Trigger

    req = Trigger.Request()
    res = node.call_service('get_count', req)
    print(f"Service get_count response: {res.success}, message: {res.message}")

    req = Trigger.Request()
    res = node.call_service('modify_name', req)
    print(f"Service modify_name response: {res.success}, message: {res.message}")

    req = Trigger.Request()
    res = node.call_service('shutdown_node', req)
    print(f"Service shutdown_node response: {res.success}, message: {res.message}")
    
    node.destroy_node()
    rclpy.shutdown()
    
#315曹老师办公室 28.3 0.1 0
if __name__ == "__main__":
    # 初始化并运行 server
    move_to_ab_pos(-11.7,-6.8,0)