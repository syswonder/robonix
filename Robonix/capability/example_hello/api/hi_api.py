#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from mcp.server.fastmcp import FastMCP
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
import sys
import time


def get_hi_tools():
    hi_tools = [
        {
            "fn": api_get_hi_status,
            "name": "api_get_hi_status",
            "description":"check hi status; Args: None"
        },
        {
            "fn": api_change_hi,
            "name": "api_change_hi",
            "description": "change hi object; Args: name: new hi name"
        },
        {
            "fn": api_close_hi,
            "name": "api_close_hi",
            "description": "close hi node object; Args: None"
        }
    ]
    return hi_tools

def api_get_hi_status() -> str:
    """统计hi的状态
    Args:
        None
    """
    func_status = "hi api_get_hi_status call ok"

    return func_status

def api_change_hi(name: str) -> str:
    """修改hi的对象
    Args:
        name: 新的hi名称
    """
    func_status = "hi api_change_hi call ok"
    return func_status

def api_close_hi() -> str:
    """关闭hi的node对象
    Args:
        None
    """
    func_status = "hi api_close_hi call ok"
    return func_status
