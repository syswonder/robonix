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
import numpy as np
from grpc import RpcError
import cv2
import io
from PIL import Image

TARGET_SERVER_IP = "127.0.0.1" # if you are running genesis simulator locally
# TARGET_SERVER_IP = "100.71.152.94" # if you are running genesis simulator on a remote server

TARGET_SERVER_PORT = 50051 # the default port for gRPC

# gRPC client setup
def wait_for_server(max_retries=30, retry_interval=2.0):
    print("[driver] waiting for server to start...")
    for attempt in range(max_retries):
        try:
            channel = grpc.insecure_channel(f"{TARGET_SERVER_IP}:{TARGET_SERVER_PORT}")
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
                    f"[driver] server {TARGET_SERVER_IP}:{TARGET_SERVER_PORT} not ready, retrying in {retry_interval} seconds... (Attempt {attempt + 1}/{max_retries})"
                )
                time.sleep(retry_interval)
            else:
                print(
                    f"[driver] server {TARGET_SERVER_IP}:{TARGET_SERVER_PORT} connection failed, please check if server is running"
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


def get_rgb_image(width=None, height=None):
    """
    Get RGB image from the robot camera.

    Args:
        width (int, optional): Desired image width
        height (int, optional): Desired image height

    Returns:
        tuple: (image_array, timestamp) where image_array is a numpy array with shape (H, W, 3)
    """
    global channel, stub
    try:
        request = robot_control_pb2.GetImageRequest()
        if width is not None:
            request.width = width
        if height is not None:
            request.height = height

        response = stub.GetRGBImage(request)

        # Convert bytes to numpy array
        if response.format.lower() == "jpeg":
            # Decode JPEG data
            image_array = np.frombuffer(response.image_data, dtype=np.uint8)
            image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
            
            # Debug: Save received image and check colors
            try:
                debug_filename = "rgb_received_debug.jpg"
                # cv2.imwrite(debug_filename, image)  # Save as BGR (OpenCV format)
                # print(f"[driver] Saved received image for debugging: {debug_filename}")
                
                # Check center pixel colors
                if image.size > 0:
                    center_y, center_x = image.shape[0] // 2, image.shape[1] // 2
                    center_color = image[center_y, center_x]
                    print(f"[driver] Received image center pixel BGR values: B={center_color[0]}, G={center_color[1]}, R={center_color[2]}")
            except Exception as e:
                print(f"[driver] Failed to save debug image: {e}")
            
            # Convert BGR to RGB
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Debug: Check converted RGB values
            if image.size > 0:
                center_y, center_x = image.shape[0] // 2, image.shape[1] // 2
                center_color = image[center_y, center_x]
                print(f"[driver] Converted image center pixel RGB values: R={center_color[0]}, G={center_color[1]}, B={center_color[2]}")
                
                # Save converted image for comparison
                try:
                    converted_filename = "rgb_converted_debug.jpg"
                    # Convert back to BGR for saving
                    image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                    # cv2.imwrite(converted_filename, image_bgr)
                    print(f"[driver] Saved converted image for debugging: {converted_filename}")
                except Exception as e:
                    print(f"[driver] Failed to save converted debug image: {e}")
        elif response.format.lower() == "png":
            # Decode PNG data
            image = Image.open(io.BytesIO(response.image_data))
            image = np.array(image)
        else:
            raise ValueError(f"Unsupported image format: {response.format}")

        return image, response.timestamp

    except RpcError as e:
        print(f"[driver] failed to get RGB image: {e}")
        print("[driver] attempting to reconnect to server...")
        try:
            channel, stub = wait_for_server(max_retries=5, retry_interval=1.0)
            response = stub.GetRGBImage(request)

            # Convert bytes to numpy array
            if response.format.lower() == "jpeg":
                image_array = np.frombuffer(response.image_data, dtype=np.uint8)
                image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            elif response.format.lower() == "png":
                image = Image.open(io.BytesIO(response.image_data))
                image = np.array(image)
            else:
                raise ValueError(f"Unsupported image format: {response.format}")

            return image, response.timestamp
        except Exception as reconnect_error:
            print(f"[driver] reconnection failed: {reconnect_error}")
            raise


def get_depth_image(width=None, height=None):
    """
    Get depth image from the robot camera.

    Args:
        width (int, optional): Desired image width
        height (int, optional): Desired image height

    Returns:
        tuple: (depth_array, min_depth, max_depth, timestamp) where depth_array is a numpy array with shape (H, W)
    """
    global channel, stub
    try:
        request = robot_control_pb2.GetImageRequest()
        if width is not None:
            request.width = width
        if height is not None:
            request.height = height

        response = stub.GetDepthImage(request)

        # Convert bytes to numpy array (float32)
        depth_array = np.frombuffer(response.depth_data, dtype=np.float32)
        depth_array = depth_array.reshape(response.height, response.width)

        return depth_array, response.min_depth, response.max_depth, response.timestamp

    except RpcError as e:
        print(f"[driver] failed to get depth image: {e}")
        print("[driver] attempting to reconnect to server...")
        try:
            channel, stub = wait_for_server(max_retries=5, retry_interval=1.0)
            response = stub.GetDepthImage(request)

            # Convert bytes to numpy array (float32)
            depth_array = np.frombuffer(response.depth_data, dtype=np.float32)
            depth_array = depth_array.reshape(response.height, response.width)

            return (
                depth_array,
                response.min_depth,
                response.max_depth,
                response.timestamp,
            )
        except Exception as reconnect_error:
            print(f"[driver] reconnection failed: {reconnect_error}")
            raise


def save_rgb_image(filename, width=None, height=None):
    """
    Capture and save RGB image to file.

    Args:
        filename (str): Output filename (should include extension like .jpg or .png)
        width (int, optional): Desired image width
        height (int, optional): Desired image height
    """
    try:
        image, timestamp = get_rgb_image(width, height)

        # Determine format from filename extension
        if filename.lower().endswith(".jpg") or filename.lower().endswith(".jpeg"):
            # Convert RGB to BGR for OpenCV
            image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imwrite(filename, image_bgr)
        elif filename.lower().endswith(".png"):
            # Convert RGB to BGR for OpenCV
            image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imwrite(filename, image_bgr)
        else:
            # Default to JPEG
            image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imwrite(filename, image_bgr)

        print(f"[driver] RGB image saved to {filename} (timestamp: {timestamp})")

    except Exception as e:
        print(f"[driver] failed to save RGB image: {e}")
        raise


def save_depth_image(filename, width=None, height=None):
    """
    Capture and save depth image to file.

    Args:
        filename (str): Output filename (should include extension like .npy or .png)
        width (int, optional): Desired image width
        height (int, optional): Desired image height
    """
    try:
        depth_array, min_depth, max_depth, timestamp = get_depth_image(width, height)

        if filename.lower().endswith(".npy"):
            # Save as numpy array
            np.save(filename, depth_array)
        elif filename.lower().endswith(".png"):
            # Normalize depth to 0-255 range for visualization
            depth_normalized = (
                (depth_array - min_depth) / (max_depth - min_depth) * 255
            ).astype(np.uint8)
            cv2.imwrite(filename, depth_normalized)
        else:
            # Default to numpy array
            np.save(filename, depth_array)

        print(
            f"[driver] Depth image saved to {filename} (timestamp: {timestamp}, depth range: {min_depth:.3f}-{max_depth:.3f}m)"
        )

    except Exception as e:
        print(f"[driver] failed to save depth image: {e}")
        raise


if __name__ == "__main__":
    # Example usage of image capture functionality
    try:
        print("[driver] Testing image capture functionality...")

        # Capture and save RGB image
        print("[driver] Capturing RGB image...")
        save_rgb_image("robot_rgb.jpg", width=640, height=480)

        # Capture and save depth image
        print("[driver] Capturing depth image...")
        save_depth_image("robot_depth.npy", width=640, height=480)
        save_depth_image("robot_depth_vis.png", width=640, height=480)

        # Get image data for processing
        rgb_image, rgb_timestamp = get_rgb_image()
        depth_image, min_depth, max_depth, depth_timestamp = get_depth_image()

        print(f"[driver] RGB image shape: {rgb_image.shape}")
        print(f"[driver] Depth image shape: {depth_image.shape}")
        print(f"[driver] Depth range: {min_depth:.3f} - {max_depth:.3f} meters")

        # Example movement with image capture
        print("[driver] Moving to position and capturing images...")
        move_to_point(1.0, 1.0)
        save_rgb_image("position_1_rgb.jpg")
        save_depth_image("position_1_depth.npy")

        move_to_point(0, 0)
        save_rgb_image("position_2_rgb.jpg")
        save_depth_image("position_2_depth.npy")

        print("[driver] Image capture test completed successfully!")

    except Exception as e:
        print(f"[driver] Error during image capture test: {e}")
        # Fallback to original movement test
        print("[driver] Falling back to movement test...")
        move_to_point(-2.2, 1.8)
        move_to_point(0, 0)
