# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import threading
import time

# 尝试导入uapi.log，如果失败则使用标准logging
try:
    from uapi.log import logger
except ImportError:
    import logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    logger = logging.getLogger(__name__)

from car_controller import CarController
from camera_manager import CameraManager


class MainControlLoop:
    def __init__(self, car, keyboard_device, scene_lock, camera_manager=None):
        self.car = car
        self.keyboard_device = keyboard_device
        self.scene_lock = scene_lock
        self.camera_manager = camera_manager
        
        # 控制参数
        self.dt = 1.0 / 60.0  # 60 FPS
        self.speed = 1.5  # 平移速度
        self.rot_speed = 2.5  # 旋转速度
        
        # 创建车辆控制器
        self.car_controller = CarController(
            car, keyboard_device, self.dt, self.speed, self.rot_speed
        )
        
    def start_camera(self):
        """启动相机管理器"""
        if self.camera_manager is not None:
            self.camera_manager.start_camera_thread()
            
    def stop_camera(self):
        """停止相机管理器"""
        if self.camera_manager is not None:
            self.camera_manager.stop_camera_thread()
            
    def run(self, scene_step_func):
        """运行主控制循环"""
        logger.info("\nKeyboard Controls:")
        logger.info("Arrow Up/Down: Forward/Backward")
        logger.info("Arrow Left/Right: Left/Right")
        logger.info("[: Rotate left")
        logger.info("]: Rotate right")
        logger.info("ESC: Quit")
        
        try:
            while True:
                # 执行车辆控制步骤
                if not self.car_controller.step():
                    logger.info("Exiting simulation.")
                    break
                    
                # 场景步进
                with self.scene_lock:
                    try:
                        scene_step_func()
                    except Exception as e:
                        logger.info(f"Viewer closed or scene step failed: {e}")
                        break
                        
                # 控制循环频率
                time.sleep(self.dt)
                
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
        finally:
            # 停止相机
            self.stop_camera()
            logger.info("Main control loop stopped")
