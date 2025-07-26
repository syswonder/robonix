# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import sys

sys.path.append(".")
sys.path.append("./simulator/genesis")

import grpc
import robot_control_pb2 as robot_control_pb2
import robot_control_pb2_grpc as robot_control_pb2_grpc
import math
import time
from grpc import RpcError


# gRPC client setup
def wait_for_server(max_retries=30, retry_interval=2.0):
    print("[driver] waiting for server to start...")
    for attempt in range(max_retries):
        try:
            channel = grpc.insecure_channel("localhost:50051")
            stub = robot_control_pb2_grpc.RobotControlStub(channel)
            # Try to call a simple RPC to test connection
            stub.GetPose(robot_control_pb2.GetPoseRequest())
            print(
                f"[driver] server connected successfully! (Attempt {attempt + 1}/{max_retries})"
            )
            return channel, stub
        except RpcError as e:
            if attempt < max_retries - 1:
                print(
                    f"[driver] server not ready, retrying in {retry_interval} seconds... (Attempt {attempt + 1}/{max_retries})"
                )
                time.sleep(retry_interval)
            else:
                print(
                    "[driver] server connection failed, please check if server is running"
                )
                raise e

    raise ConnectionError("Unable to connect to server")


# Initialize connection
try:
    channel, stub = wait_for_server()
except Exception as e:
    print(f"[driver] connection error: {e}")
    sys.exit(1)


def get_pose():
    global channel, stub
    try:
        response = stub.GetPose(robot_control_pb2.GetPoseRequest())
        return response.x, response.y, response.z, response.yaw
    except RpcError as e:
        print(f"[driver] failed to get pose: {e}")
        print("[driver] attempting to reconnect to server...")
        try:
            channel, stub = wait_for_server(max_retries=5, retry_interval=1.0)
            response = stub.GetPose(robot_control_pb2.GetPoseRequest())
            return response.x, response.y, response.z, response.yaw
        except Exception as reconnect_error:
            print(f"[driver] reconnection failed: {reconnect_error}")
            raise


def rotate_to_yaw(target_yaw_deg, tolerance=2.0):
    global channel, stub
    while True:
        try:
            _, _, _, current_yaw = get_pose()
            diff = (target_yaw_deg - current_yaw + 180) % 360 - 180
            if abs(diff) < tolerance:
                stub.Stop(robot_control_pb2.StopRequest())
                break
            angle = math.radians(diff)
            stub.Rotate(robot_control_pb2.RotateRequest(angle=angle))
            time.sleep(0.1)
            stub.Stop(robot_control_pb2.StopRequest())
            time.sleep(0.05)
        except RpcError as e:
            print(f"[driver] rotation operation failed: {e}")
            print("[driver] attempting to reconnect...")
            try:
                channel, stub = wait_for_server(max_retries=5, retry_interval=1.0)
            except Exception as reconnect_error:
                print(f"[driver] reconnection failed: {reconnect_error}")
                raise


def move_forward(distance):
    global channel, stub
    try:
        stub.MoveTo(robot_control_pb2.MoveToRequest(forward=distance, lateral=0.0))
    except RpcError as e:
        print(f"[driver] forward movement failed: {e}")
        print("[driver] attempting to reconnect...")
        try:
            channel, stub = wait_for_server(max_retries=5, retry_interval=1.0)
            stub.MoveTo(robot_control_pb2.MoveToRequest(forward=distance, lateral=0.0))
        except Exception as reconnect_error:
            print(f"[driver] reconnection failed: {reconnect_error}")
            raise


def move_to_point(target_x, target_y):
    try:
        x, y, z, yaw = get_pose()
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)
        if distance < 1e-3:
            return
        target_yaw_deg = math.degrees(math.atan2(dx, dy)) % 360
        rotate_to_yaw(target_yaw_deg)
        move_forward(distance)
        time.sleep(max(distance, 1.0))
    except Exception as e:
        print(f"[driver] failed to move to point ({target_x}, {target_y}): {e}")
        raise


if __name__ == "__main__":
    # print("move to (1.0, 1.0)")
    # move_to_point(1.0, 1.0)
    # print("move to (2.5, 3.0)")
    # move_to_point(2.5, 3.0)
    # print("move to (1.5, 0.0)")
    # move_to_point(1.5, 0.0)
    # print("move to (0.0, 0.0)")
    # move_to_point(0.0, 0.0)
    # print("goodbye")
    move_to_point(-2.2, 1.8)
    move_to_point(0, 0)
