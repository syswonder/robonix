# SPDX-License-Identifier: MulanPSL-2.0
"""
Python client for robonix/system/debug/ping query.
Calls the ping service via robonix-server meta API + ROS2 (Zenoh).
Must be run via rbnx start (not direct python).
"""

import sys

import grpc

from robonix.system.debug.ping_query import create_ping_client
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix_interfaces_ros2.srv import SystemDebugPing
from std_msgs.msg import String


def main():
    endpoint = "127.0.0.1:50051"
    target = "robonix-server"
    requester_id = "python_ping_client"
    payload = sys.argv[1] if len(sys.argv) > 1 else "hello"

    channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(channel)

    client = create_ping_client(runtime_client, requester_id, target)

    req = SystemDebugPing.Request()
    req.data = String()
    req.data.data = payload

    response = client.call(req, timeout_sec=10.0)
    print(response.data.data)


if __name__ == "__main__":
    main()
