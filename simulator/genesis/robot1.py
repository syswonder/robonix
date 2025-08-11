# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import sys
import os
import signal
import threading
import time

# Add project root to Python path
PROJECT_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
sys.path.insert(0, PROJECT_ROOT)

# Try to import uapi.log, fallback to standard logging if failed
try:
    from uapi.log import logger
except ImportError:
    import logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    logger = logging.getLogger(__name__)

# Import custom modules
from keyboard_device import KeyboardDevice
from scene_manager import SceneManager
from camera_manager import CameraManager
from main_loop import MainControlLoop
from grpc_service import serve_grpc


class RobotSimulator:
    def __init__(self):
        self.scene_manager = None
        self.keyboard_device = None
        self.camera_manager = None
        self.main_loop = None
        self.grpc_server = None
        self.scene_lock = threading.Lock()
        
    def setup_signal_handlers(self):
        """Setup signal handlers"""
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        """Signal handler"""
        logger.info(f"Received signal {signum}, shutting down gracefully...")
        self.cleanup()
        sys.exit(0)
        
    def create_scene_with_camera(self):
        """Create scene with camera (before build)"""
        self.scene_manager = SceneManager()
        scene, car, camera = self.scene_manager.create_scene_with_camera()
        return scene, car, camera
        
    def setup_camera_manager(self, camera, car):
        """Setup camera manager"""
        # Create camera manager with 0.5 second capture interval for testing
        self.camera_manager = CameraManager(
            camera, car, self.scene_lock, 
            output_dir="cam_output", 
            capture_interval=0.5
        )
        
    def setup_keyboard(self):
        """Setup keyboard control"""
        self.keyboard_device = KeyboardDevice()
        self.keyboard_device.start()
        
    def setup_grpc(self, car, camera):
        """Setup gRPC service"""
        self.grpc_server = serve_grpc(
            car, self.keyboard_device, self.scene_lock, self.camera_manager
        )
        
    def setup_main_loop(self, car):
        """Setup main control loop"""
        self.main_loop = MainControlLoop(
            car, self.keyboard_device, self.scene_lock, self.camera_manager
        )
        
    def run(self):
        """Run simulator"""
        try:
            # Create scene with camera (before build)
            scene, car, camera = self.create_scene_with_camera()
            
            # Setup camera manager
            self.setup_camera_manager(camera, car)
            
            # Setup keyboard control
            self.setup_keyboard()
            
            # Setup gRPC service
            self.setup_grpc(car, camera)
            
            # Setup main control loop
            self.setup_main_loop(car)
            
            # Start camera
            self.main_loop.start_camera()
            
            # Run main control loop
            self.main_loop.run(self.scene_manager.step)
            
        except Exception as e:
            logger.error(f"Simulation error: {e}")
            raise
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Cleanup resources"""
        try:
            # Stop gRPC server
            if self.grpc_server is not None:
                self.grpc_server.stop(0)
                logger.info("gRPC server stopped")
        except Exception as e:
            logger.warning(f"Error stopping gRPC server: {e}")
            
        try:
            # Stop keyboard device
            if self.keyboard_device is not None:
                self.keyboard_device.stop()
                logger.info("Keyboard device stopped")
        except Exception as e:
            logger.warning(f"Error stopping keyboard device: {e}")
            
        try:
            # Stop camera
            if self.main_loop is not None:
                self.main_loop.stop_camera()
        except Exception as e:
            logger.warning(f"Error stopping camera: {e}")
            
        try:
            # Cleanup scene
            if self.scene_manager is not None and self.scene_manager.scene is not None:
                self.scene_manager.scene = None
                logger.info("Genesis scene cleaned up")
        except Exception as e:
            logger.warning(f"Error cleaning up Genesis scene: {e}")


def main():
    """Main function"""
    simulator = RobotSimulator()
    simulator.setup_signal_handlers()
    simulator.run()


if __name__ == "__main__":
    main()
