# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from pynput import keyboard
from . import robot_control_pb2_grpc
from . import robot_control_pb2

from robonix.manager.log import logger

class RobotControlService(robot_control_pb2_grpc.RobotControlServicer):
    def __init__(self, car, keyboard_device, scene_lock, camera_manager=None):
        self.car = car
        self.keyboard_device = keyboard_device
        self._lock = threading.Lock()
        self._scene_lock = scene_lock
        self._camera_manager = camera_manager

    def Move(self, request, context):
        """Move vehicle forward or backward"""
        distance = request.distance
        key = None
        if distance > 0:
            key = keyboard.Key.up
        elif distance < 0:
            key = keyboard.Key.down
            
        if key:
            self.keyboard_device.on_press(key)
            # Sleep time proportional to distance
            time.sleep(min(abs(distance), 1.0))
            self.keyboard_device.on_release(key)
            
        return robot_control_pb2.MoveReply(status="ok")

    def Rotate(self, request, context):
        """Rotate vehicle"""
        angle = request.angle  # angle in radians
        current_yaw = getattr(self.car, "_my_yaw", 0.0)
        target_yaw = (current_yaw + angle) % (2 * np.pi)
        max_speed = 2.0  # maximum angular speed (rad/s)
        accel = 4.0  # angular acceleration (rad/s^2)
        dt = 1.0 / 60.0  # time step (60 FPS)
        
        yaw = current_yaw
        speed = 0.0
        diff = (target_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
        direction = 1 if diff > 0 else -1
        
        while True:
            diff = (target_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
            if abs(diff) < 0.01:
                break
                
            if abs(diff) > 0.2:
                speed = min(speed + accel * dt, max_speed)
            else:
                speed = max(speed - accel * dt, 0.2)
                
            step_angle = direction * min(abs(diff), speed * dt)
            yaw = (yaw + step_angle) % (2 * np.pi)
            self.car._my_yaw = yaw
            quat = R.from_euler("x", yaw).as_quat()
            self.car.set_quat(quat)
            
            # Note: scene object needed here for stepping
            # In refactored code, scene stepping is handled by main loop
                
            time.sleep(dt)
            
        return robot_control_pb2.MoveReply(status="ok")

    def Stop(self, request, context):
        """Stop vehicle"""
        self.keyboard_device.pressed_keys.clear()
        if hasattr(self.car, "_move_to_target"):
            self.car._move_to_target["active"] = False
        return robot_control_pb2.MoveReply(status="stopped")

    def Reset(self, request, context):
        """Reset vehicle to initial position and orientation"""
        try:
            with self._scene_lock:
                # Reset position
                if hasattr(self.car, "_initial_pos"):
                    qpos = self.car.get_qpos()
                    if hasattr(qpos, "cpu"):
                        qpos = qpos.cpu().numpy()
                    qpos_new = qpos.copy()
                    qpos_new[0] = self.car._initial_pos[0]  # x
                    qpos_new[1] = self.car._initial_pos[1]  # y
                    qpos_new[2] = self.car._initial_pos[2]  # z
                    self.car.set_qpos(qpos_new)
                    
                    # Reset orientation
                    self.car._my_yaw = 0.0
                    quat = R.from_euler("x", 0.0).as_quat()
                    self.car.set_quat(quat)
                    
                    # Clear any active keyboard input
                    self.keyboard_device.pressed_keys.clear()
                    
                    # Clear MoveTo state if exists
                    if hasattr(self.car, "_move_to_target"):
                        self.car._move_to_target["active"] = False
                        
                    logger.info("Vehicle reset to initial position via gRPC")
                    return robot_control_pb2.MoveReply(status="reset")
                else:
                    logger.warning("Vehicle initial state not found")
                    return robot_control_pb2.MoveReply(status="error: no initial state")
        except Exception as e:
            logger.error(f"Reset failed: {e}")
            return robot_control_pb2.MoveReply(status=f"error: {str(e)}")

    def GetPose(self, request, context):
        """Get vehicle pose"""
        car_x, car_y, car_z = self.car.get_pos()
        yaw = getattr(self.car, "_my_yaw", 0.0)
        yaw_deg = np.degrees(yaw) % 360
        return robot_control_pb2.PoseReply(
            x=float(car_x), y=float(car_y), z=float(car_z), yaw=float(yaw_deg)
        )

    def MoveTo(self, request, context):
        """Move to specified position"""
        with self._lock:
            qpos = self.car.get_qpos()
            if hasattr(qpos, "cpu"):
                qpos = qpos.cpu().numpy()
            start_x, start_y, _ = qpos[:3]
            yaw = getattr(self.car, "_my_yaw", 0.0)

            dx = request.forward * np.sin(yaw) + request.lateral * np.cos(yaw)
            dy = request.forward * np.cos(yaw) - request.lateral * np.sin(yaw)
            target_x = start_x + dx
            target_y = start_y + dy

            self.car._move_to_target = {
                "forward": request.forward,
                "lateral": request.lateral,
                "active": True,
                "start_time": None,
                "max_speed": 1.0,
                "accel": 2.0,
                "decel": 3.0,
                "tolerance": 0.02,
                "target_x": target_x,
                "target_y": target_y,
            }

        return robot_control_pb2.MoveReply(status="ok")

    def GetRGBImage(self, request, context):
        """Get RGB image"""
        try:
            if self._camera_manager is not None:
                # Use camera manager to get image
                image_data, width, height = self._camera_manager.get_rgb_image(
                    request.width, request.height
                )
                
                return robot_control_pb2.RGBImageReply(
                    image_data=image_data,
                    width=width,
                    height=height,
                    format="jpeg",
                    timestamp=int(time.time() * 1000),
                )
            else:
                # Return empty data if no camera manager
                return robot_control_pb2.RGBImageReply(
                    image_data=b"",
                    width=0,
                    height=0,
                    format="jpeg",
                    timestamp=int(time.time() * 1000),
                )

        except Exception as e:
            logger.error(f"error getting rgb image: {e}")
            return robot_control_pb2.RGBImageReply(
                image_data=b"",
                width=0,
                height=0,
                format="jpeg",
                timestamp=int(time.time() * 1000),
            )

    def GetDepthImage(self, request, context):
        """Get depth image"""
        try:
            if self._camera_manager is not None:
                # Use camera manager to get image
                depth_data, width, height, min_depth, max_depth = self._camera_manager.get_depth_image(
                    request.width, request.height
                )
                
                return robot_control_pb2.DepthImageReply(
                    depth_data=depth_data,
                    width=width,
                    height=height,
                    min_depth=min_depth,
                    max_depth=max_depth,
                    timestamp=int(time.time() * 1000),
                )
            else:
                # Return empty data if no camera manager
                return robot_control_pb2.DepthImageReply(
                    depth_data=b"",
                    width=0,
                    height=0,
                    min_depth=0.0,
                    max_depth=0.0,
                    timestamp=int(time.time() * 1000),
                )

        except Exception as e:
            logger.error(f"error getting depth image: {e}")
            return robot_control_pb2.DepthImageReply(
                depth_data=b"",
                width=0,
                height=0,
                min_depth=0.0,
                max_depth=0.0,
                timestamp=int(time.time() * 0),
            )


def serve_grpc(car, keyboard_device, scene_lock, camera_manager=None, port=50051):
    """Start gRPC server"""
    import grpc
    from concurrent import futures
    
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    robot_control_pb2_grpc.add_RobotControlServicer_to_server(
        RobotControlService(car, keyboard_device, scene_lock, camera_manager), server
    )
    server.add_insecure_port(f"[::]:{port}")
    logger.info(f"[gRPC] RobotControl server started on port {port}")
    server.start()
    return server
