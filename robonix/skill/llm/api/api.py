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

import requests
import json

@eaios.caller
def skl_ask_llm(self_entity, api_key, prompt=None, request=""):
    """
    Query DeepSeek API with the given parameters
    
    Parameters:
    api_key: DeepSeek API key for authentication
    prompt: Context/content for the conversation, if None read from prompt.txt
    request: User's question or request
    
    Returns:
    API response content as string
    """
    
    # If prompt is empty, read context from prompt.txt
    if prompt is None:
        try:
            prompt_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config/prompt.txt')
            with open(prompt_path, 'r', encoding='utf-8') as f:
                prompt = f.read()
        except FileNotFoundError:
            print("Warning: prompt.txt file not found, using empty context")
            prompt = ""
    
    # Construct messages list for the API
    messages = []
    
    # Add context as system message (if available)
    if prompt.strip():
        messages.append({
            "role": "system",
            "content": prompt
        })
    
    # Add user request
    messages.append({
        "role": "user",
        "content": request
    })
    
    # API endpoint (adjust according to DeepSeek official documentation)
    api_url = "https://api.deepseek.com/v1/chat/completions"
    
    # Request headers
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}"
    }
    
    # Request payload
    data = {
        "model": "deepseek-chat",  # Adjust based on available models
        "messages": messages,
        "stream": False,
        "temperature": 0.7,
        "max_tokens": 2048
    }
    
    try:
        # Send API request
        response = requests.post(api_url, headers=headers, json=data, timeout=30)
        response.raise_for_status()  # Raise exception for bad status codes
        
        # Parse response
        result = response.json()
        return result['choices'][0]['message']['content']
        
    except requests.exceptions.RequestException as e:
        return f"Request failed: {str(e)}"
    except KeyError as e:
        return f"Failed to parse response: {str(e)}"
    except Exception as e:
        return f"Error occurred: {str(e)}"