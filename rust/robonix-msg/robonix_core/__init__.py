"""Robonix Core - ROS2 message and service definitions for Robonix system."""

# This package provides ROS2 message and service definitions.
# Users should implement their own service clients using standard ROS2 patterns.

# Import the high-level client for convenience
from .client import RobonixClient, create_client

__all__ = ['RobonixClient', 'create_client']

