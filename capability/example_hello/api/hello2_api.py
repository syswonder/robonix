#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from mcp.server.fastmcp import FastMCP
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
import sys
import time

def get_hello2_tools():
    hello_tools = [
        {
            "fn": api_get_hello_status,
            "name": "api_get_hello_status",
            "description":"check hello status; Args: None"
        },
        {
            "fn": api_change_hello,
            "name": "api_change_hello",
            "description": "change hello object; Args: name: new hello name"
        },
        {
            "fn": api_close_hello,
            "name": "api_close_hello",
            "description": "close hello node object; Args: None"
        }
    ]
    return hello_tools

def api_get_hello_status() -> str:
    """统计hello的状态
    Args:
        None
    """
    func_status = "hello api_get_hello_status call ok"

    return func_status

def api_change_hello(name: str) -> str:
    """修改hello的对象
    Args:
        name: 新的hello名称
    """
    func_status = "hello api_change_hello call ok"
    return func_status

def api_close_hello() -> str:
    """关闭hello的node对象
    Args:
        None
    """
    func_status = "hello api_close_hello call ok"
    return func_status
