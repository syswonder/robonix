# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import sys
import os
import signal
import threading

PROJECT_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
sys.path.insert(0, PROJECT_ROOT)

from robonix.manager.log import logger, init_logger

from simulator.genesis.keyboard_device import KeyboardDevice
from simulator.genesis.scene_manager import SceneManager
from simulator.genesis.camera_manager import CameraManager
from simulator.genesis.main_loop import MainControlLoop
from simulator.genesis.grpc_service import serve_grpc


class RobotSimulator:
    def __init__(self):
        self.scene_manager = None
        self.keyboard_device = None
        self.camera_manager = None
        self.main_loop = None
        self.grpc_server = None
        self.scene_lock = threading.Lock()

    def setup_signal_handlers(self):
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        logger.info(f"received signal {signum}, shutting down gracefully...")
        self.cleanup()
        sys.exit(0)

    def create_scene_with_camera(self):
        self.scene_manager = SceneManager()
        scene, car, camera = self.scene_manager.create_scene_with_camera()
        return scene, car, camera

    def setup_camera_manager(self, camera, car):
        # Create camera manager with 0.5 second capture interval for testing
        self.camera_manager = CameraManager(
            camera, car, self.scene_lock, output_dir="cam_output", capture_interval=0.5
        )

    def setup_keyboard(self):
        self.keyboard_device = KeyboardDevice()
        self.keyboard_device.start()

    def setup_grpc(self, car, camera):
        self.grpc_server = serve_grpc(
            car, self.keyboard_device, self.scene_lock, self.camera_manager
        )

    def setup_main_loop(self, car):
        self.main_loop = MainControlLoop(
            car, self.keyboard_device, self.scene_lock, self.camera_manager, self.scene_manager
        )

    def run(self):
        try:
            scene, car, camera = self.create_scene_with_camera()
            self.setup_camera_manager(camera, car)
            self.setup_keyboard()
            self.setup_grpc(car, camera)
            self.setup_main_loop(car)
            self.main_loop.start_camera()
            self.main_loop.run(self.scene_manager.step)

        except Exception as e:
            logger.error(f"simulation error: {e}")
            raise
        finally:
            self.cleanup()

    def cleanup(self):
        try:
            if self.grpc_server is not None:
                self.grpc_server.stop(0)
                logger.info("gRPC server stopped")
        except Exception as e:
            logger.warning(f"error stopping gRPC server: {e}")

        try:
            if self.keyboard_device is not None:
                self.keyboard_device.stop()
                logger.info("keyboard device stopped")
        except Exception as e:
            logger.warning(f"error stopping keyboard device: {e}")

        try:
            if self.main_loop is not None:
                self.main_loop.stop_camera()
        except Exception as e:
            logger.warning(f"error stopping camera: {e}")

        try:
            if self.scene_manager is not None and self.scene_manager.scene is not None:
                self.scene_manager.scene = None
                logger.info("genesis scene cleaned up")
        except Exception as e:
            logger.warning(f"error cleaning up genesis scene: {e}")


def main():
    # Initialize logger first
    init_logger()
    
    simulator = RobotSimulator()
    simulator.setup_signal_handlers()
    simulator.run()


if __name__ == "__main__":
    main()
