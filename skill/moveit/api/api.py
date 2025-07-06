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

from DeepEmbody.manager.eaios_decorators import eaios

#TODO memory
mamory = {}

@eaios.api
@eaios.caller
def move_to_goal(goal_name:str) -> str:
    """通过预存名称移动到指定位置
    Args:
        goal_name: 预存位置点的名称
    Returns:
        移动操作的结果状态字符串
    """
    if goal_name in memory.keys():
        return move_to_ab_pos(memory[goal_name])
    else:
        return f"Service setmove_to_goal_gaol response: {False}, message: goal not in memory"

@eaios.api
@eaios.caller
def move_to_ab_pos(x, y, yaw) -> str:
    """移动到绝对坐标位置
    Args:
        x: 目标点X坐标
        y: 目标点Y坐标
        yaw: 目标点偏航角
    Returns:
        移动操作的结果状态字符串
    """
    #TODO how read dep
    if "set_goal" in dep["move"][1].keys():
        set_goal = dep["move"][1]["set_goal"]
        return set_goal(x,y,yaw)
    if "simple_go" in dep["move"][1].keys():
        simple_go = dep["move"][1]["simple_go"]
        pos = get_pos()
        while pos.x != x or pos.y != y or pos.yaw != yaw:
            simple_go(x-pos.x,y-pos.y,yaw-pos.yaw)
            #TODO
            if timeout:
                return f"Service move_to_goal response: {False}, message: TimeOut"
    return f"Service move_to_goal response: {True}, message: None"

@eaios.api
@eaios.caller
def move_to_rel_pos(dx,dy,dyaw) -> str:
    """相对当前位置移动指定偏移量
    Args:
        dx: X方向偏移量
        dy: Y方向偏移量
        dyaw: 偏航角偏移量
    Returns:
        移动操作的结果状态字符串
    """
    if "set_goal" in dep["move"][1].keys():
        set_goal = dep["move"][1]["set_goal"]
        pos = get_pos()
        return set_goal(pos.x + dx,pos.y + dy,pos.yaw + dyaw)
    if "simple_go" in dep["move"][1].keys():
        simple_go = dep["move"][1]["simple_go"]
        pos = get_pos()
        return simple_go(dx,dy,dyaw)
    return f"Service move_to_goal response: {True}, message: None"

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


if __name__ == "__main__":
    # 初始化并运行 server
    mcp.run(transport='stdio')