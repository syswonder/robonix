# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import numpy as np
from scipy.spatial.transform import Rotation as R
from pynput import keyboard

from robonix.manager.log import logger

class CarController:
    def __init__(self, car, keyboard_device, dt, max_speed, max_rot_speed, accel=3.0, rot_accel=5.0, scene_manager=None):
        self.car = car
        self.keyboard_device = keyboard_device
        self.dt = dt
        self.max_speed = max_speed
        self.max_rot_speed = max_rot_speed
        self.accel = accel
        self.rot_accel = rot_accel
        self.scene_manager = scene_manager

        # Initialize car state
        if not hasattr(car, "_my_yaw"):
            car._my_yaw = np.pi * 0.93

        # Velocity state
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # Target velocity
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        # Position record
        self.last_pos = None
        self.last_yaw = None

        # MoveTo state
        self.move_to_vx = 0.0
        self.move_to_vy = 0.0
        self.move_to_active = False

    def update_controls(self):
        """Update control input"""
        keys = self.keyboard_device.get_keys()
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        if keyboard.Key.up in keys:
            self.target_vx += self.max_speed
        if keyboard.Key.down in keys:
            self.target_vx -= self.max_speed
        if keyboard.Key.left in keys:
            self.target_vy -= self.max_speed
        if keyboard.Key.right in keys:
            self.target_vy += self.max_speed
        if keyboard.KeyCode.from_char("[") in keys:
            self.target_wz += self.max_rot_speed
        if keyboard.KeyCode.from_char("]") in keys:
            self.target_wz -= self.max_rot_speed
        if keyboard.KeyCode.from_char("-") in keys:
            self.reset_car()

        return keyboard.Key.esc in keys

    def update_move_to(self):
        """Update MoveTo function"""
        keyboard_active = any(
            [self.target_vx != 0, self.target_vy != 0, self.target_wz != 0])

        if (hasattr(self.car, "_move_to_target") and
            self.car._move_to_target.get("active", False) and
                not keyboard_active):

            self.move_to_active = True
            target = self.car._move_to_target

            if target["start_time"] is None:
                target["start_time"] = 0.0
            else:
                target["start_time"] += self.dt

            qpos = self.car.get_qpos()
            if hasattr(qpos, "cpu"):
                qpos = qpos.cpu().numpy()
            curr_x, curr_y, _ = qpos[:3]
            target_x = target["target_x"]
            target_y = target["target_y"]

            rem_x = target_x - curr_x
            rem_y = target_y - curr_y
            rem_dist = np.hypot(rem_x, rem_y)

            if rem_dist < target["tolerance"]:
                self.move_to_vx = 0.0
                self.move_to_vy = 0.0
                self.move_to_active = False
                self.car._move_to_target["active"] = False
                logger.info(
                    f"move to completed: reached target within {target['tolerance']:.3f}m")
            else:
                if rem_dist > 1e-6:
                    dir_x = rem_x / rem_dist
                    dir_y = rem_y / rem_dist
                else:
                    dir_x = dir_y = 0.0

                current_speed = np.hypot(self.move_to_vx, self.move_to_vy)
                stopping_dist = ((current_speed**2) / (2 * target["decel"])
                                 if target["decel"] > 0 else 0.0)

                if rem_dist > stopping_dist + 0.1:
                    target_speed = min(current_speed + target["accel"] * self.dt,
                                       target["max_speed"])
                else:
                    # Decelerate
                    target_speed = max(
                        current_speed - target["decel"] * self.dt, 0.0)

                self.move_to_vx = dir_x * target_speed
                self.move_to_vy = dir_y * target_speed
        else:
            self.move_to_active = False
            self.move_to_vx = 0.0
            self.move_to_vy = 0.0

            if keyboard_active and hasattr(self.car, "_move_to_target"):
                self.car._move_to_target["active"] = False
                logger.info("move to interrupted by keyboard input")

    def update_velocities(self):
        """Update velocities"""
        if not self.move_to_active:
            self.vx = self._approach(self.vx, self.target_vx, self.accel)
            self.vy = self._approach(self.vy, self.target_vy, self.accel)
            self.wz = self._approach(self.wz, self.target_wz, self.rot_accel)

    def _approach(self, current, target, acceleration):
        """Smoothly approach target value"""
        if current < target:
            return min(current + acceleration * self.dt, target)
        elif current > target:
            return max(current - acceleration * self.dt, target)
        else:
            return current

    def update_position(self):
        """Update car position"""
        qpos = self.car.get_qpos()
        if hasattr(qpos, "cpu"):
            qpos = qpos.cpu().numpy()
        x, y, z = qpos[:3]

        if self.move_to_active:
            dx = self.move_to_vx * self.dt
            dy = self.move_to_vy * self.dt
            dtheta = 0.0
        else:
            dx = (self.vx * np.sin(self.car._my_yaw) * self.dt +
                  self.vy * np.cos(self.car._my_yaw) * self.dt)
            dy = (self.vx * np.cos(self.car._my_yaw) * self.dt -
                  self.vy * np.sin(self.car._my_yaw) * self.dt)
            dtheta = self.wz * self.dt

        # Update position
        qpos_new = qpos.copy()
        qpos_new[0] = x + dx
        qpos_new[1] = y + dy
        self.car.set_qpos(qpos_new)

        # Update orientation
        self.car._my_yaw -= dtheta
        quat = R.from_euler("x", self.car._my_yaw).as_quat()
        self.car.set_quat(quat)

    def print_position(self):

        # deprecated
        return

        """Print car position information"""
        car_x, car_y, car_z = self.car.get_pos()
        car_x = float(car_x)
        car_y = float(car_y)
        car_z = float(car_z)
        pos = (round(car_x, 4), round(car_y, 4), round(car_z, 4))
        yaw_deg = np.degrees(getattr(self.car, "_my_yaw", 0.0)) % 360
        yaw_deg_rounded = round(yaw_deg, 2)

        if pos != self.last_pos or yaw_deg_rounded != self.last_yaw:
            logger.info(
                f"car position: x={car_x:.4f}, y={car_y:.4f}, z={car_z:.4f}, yaw={yaw_deg:.2f}°")

        self.last_pos = pos
        self.last_yaw = yaw_deg_rounded
        return pos, yaw_deg_rounded

    def reset_car(self):
        """Reset car to initial position and orientation"""
        try:
            # Get initial position and quaternion from scene manager
            if self.scene_manager is not None:
                initial_pos = self.scene_manager.car_initial_pos
                initial_quat = self.scene_manager.car_initial_quat
            else:
                # Fallback to hardcoded initial values from scene_manager.py
                initial_pos = (-0.38341576, 0.4632059, 0.14989224)
                initial_quat = None  # Will use identity quaternion

            # Reset position
            if initial_pos is not None:
                self.car.set_pos(initial_pos)
                logger.info(f"Car position reset to: {initial_pos}")

            # reset _my_yaw to np.pi * 0.93
            self.car._my_yaw = np.pi * 0.93
            logger.info(f"Car orientation reset to yaw: {self.car._my_yaw}")

            # Reset velocities
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0

            # Reset MoveTo state
            self.move_to_vx = 0.0
            self.move_to_vy = 0.0
            self.move_to_active = False

            # Clear any active MoveTo target
            if hasattr(self.car, '_move_to_target'):
                self.car._move_to_target["active"] = False

            logger.info("Car reset completed successfully")

        except Exception as e:
            logger.error(f"Failed to reset car: {e}")
            import traceback
            logger.error(traceback.format_exc())

    def step(self):
        """Execute a control step"""
        # Check exit condition
        if self.update_controls():
            return False

        # Update MoveTo function
        self.update_move_to()

        # Update velocities
        self.update_velocities()

        # Update position
        self.update_position()

        # Print position information
        self.print_position()

        return True
