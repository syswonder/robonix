# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import os

from numba.core.types import NoneType
import genesis as gs
import numpy as np

from robonix.manager.log import logger

class SceneManager:
    def __init__(self):
        self.scene: gs.Scene = None
        self.chair = None
        self.chair_initial_pos = None
        self.chair_initial_quat = None
        self.chair_last_pos = None
        self.chair_last_quat = None
        self.car = None
        self.car_initial_pos = None
        self.car_initial_quat = None
        self.car_last_pos = None
        self.car_last_quat = None

    def create_scene(self):
        """Create Genesis scene"""
        gs.init(backend=gs.gpu)

        self.scene = gs.Scene(
            show_viewer=True,
            sim_options=gs.options.SimOptions(
                dt=0.01,
                gravity=(0, 0, -9.81),
            ),
            viewer_options=gs.options.ViewerOptions(
                res=(1366, 768),
                camera_pos=(-4.0, 2.5, 3.0),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=45,
                max_FPS=60,
            ),
            vis_options=gs.options.VisOptions(
                show_world_frame=False,
                world_frame_size=1.0,
                show_link_frame=False,
                show_cameras=False,
                plane_reflection=False,
                ambient_light=(0.3, 0.3, 0.3),
            ),
            renderer=gs.renderers.Rasterizer(),
        )

        self.scene.profiling_options.show_FPS = False
        logger.info("genesis scene created successfully")
        return self.scene

    def create_scene_with_camera(self):
        """Create scene and add camera (before build)"""
        scene = self.create_scene()

        # Add scene elements
        self.add_floor()
        self.add_room_walls()
        car = self.add_car()
        self.add_furniture()

        # Add camera before build
        camera = self.add_camera(
            res=(800, 600),
            pos=(0.3, 0.0, 0.2),
            lookat=(1.0, 0.0, 0.2),
            fov=60,
            gui=False,
        )

        # Build scene
        self.build_scene()

        # Set view to follow vehicle
        self.follow_entity(car)

        return scene, car, camera

    def add_camera(self, res=(800, 600), pos=(0.3, 0.0, 0.2),
                   lookat=(1.0, 0.0, 0.2), fov=60, gui=False):
        """Add camera to scene"""
        if self.scene is None:
            raise RuntimeError("Scene not created yet")

        camera = self.scene.add_camera(
            res=res,
            pos=pos,
            lookat=lookat,
            fov=fov,
            GUI=gui,
        )
        logger.info(f"camera added to scene at position {pos}")
        return camera

    def add_floor(self):
        """Add floor"""
        if self.scene is None:
            raise RuntimeError("Scene not created yet")

        floor = self.scene.add_entity(
            gs.morphs.Plane(pos=(0.0, 0.0, 0.0)),
            surface=gs.surfaces.Rough(
                roughness=0.9,
                diffuse_texture=gs.textures.ImageTexture(
                    image_path=self._get_sim_asset(
                        "texture_tiles_0024/tiles_0024_color_1k.jpg")
                ),
                normal_texture=gs.textures.ImageTexture(
                    image_path=self._get_sim_asset(
                        "texture_tiles_0024/tiles_0024_normal_opengl_1k.png"
                    )
                ),
            ),
        )
        logger.info("floor added to scene")
        return floor

    def add_room_walls(self):
        """Add room walls"""
        if self.scene is None:
            raise RuntimeError("Scene not created yet")

        walls = []

        # North wall
        wall_north = self.scene.add_entity(
            gs.morphs.Box(pos=(0.0, 4.0, -0.1),
                          size=(8.0, 0.2, 3.0), fixed=True),
            surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9)),
        )
        walls.append(wall_north)

        # South wall
        wall_south = self.scene.add_entity(
            gs.morphs.Box(pos=(0.0, -4.0, -0.1),
                          size=(8.0, 0.2, 3.0), fixed=True),
            surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9)),
        )
        walls.append(wall_south)

        # East wall
        wall_east = self.scene.add_entity(
            gs.morphs.Box(pos=(4.0, 0.0, -0.1),
                          size=(0.2, 8.0, 3.0), fixed=True),
            surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9)),
        )
        walls.append(wall_east)

        # West wall
        wall_west = self.scene.add_entity(
            gs.morphs.Box(pos=(-4.0, 0.0, -0.1),
                          size=(0.2, 8.0, 3.0), fixed=True),
            surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9)),
        )
        walls.append(wall_west)

        logger.info("room walls added to scene")
        return walls

    def add_car(self, size=(0.3, 0.5, 0.3)):
        """Add car to scene"""

        # Car position changed - Pos: [-0.38341576  0.4632059   0.14989224], Q: [0.9906616  0.         0.         0.13634254]

        self.car_initial_pos = (-0.38341576, 0.4632059, 0.14989224)
        # self.car_initial_quat = (0.9906616, 0.0, 0.0, 0.13634254)
        self.car_initial_quat = None

        if self.scene is None:
            raise RuntimeError("Scene not created yet")

        car = self.scene.add_entity(
            gs.morphs.Box(pos=self.car_initial_pos, size=size, fixed=False,
                          quat=self.car_initial_quat),
            surface=gs.surfaces.Iron(color=(0.2, 0.2, 0.8)),
        )

        self.car = car

        # self._initial_yaw = self.car_initial_quat[2]
        # self._my_yaw = self.car_initial_quat[2]

        logger.info(f"car added to scene at position {self.car_initial_pos}")
        return car

    def add_furniture(self):
        """Add furniture and decoration"""
        if self.scene is None:
            raise RuntimeError("Scene not created yet")

        furniture = []

        # Chair
        chair_pos = (0.29952443, -2.2542348, 0.00328773)
        chair_quat = (-0.00091311, -0.00104396,
                      0.70632267, 0.7078886)  # w, x, y, z
        chair = self.scene.add_entity(
            morph=gs.morphs.Mesh(
                file=self._get_sim_asset("chair1/MAD_QUEEN_CHAIR.obj"),
                scale=0.007,
                pos=chair_pos,
                quat=chair_quat,
            ),
            surface=gs.surfaces.Default(
                diffuse_texture=gs.textures.ImageTexture(
                    image_path=self._get_sim_asset(
                        "chair1/MAD_QUEEN_CHAIR/3_1_2_d.jpg"),
                ),
                roughness_texture=gs.textures.ImageTexture(
                    image_path=self._get_sim_asset(
                        "chair1/MAD_QUEEN_CHAIR/14_2_8_r.jpg"),
                ),
            ),
        )
        furniture.append(chair)

        # Store chair reference and initial state
        self.chair = chair
        self.chair_initial_pos = chair_pos
        self.chair_initial_quat = chair_quat

        logger.info(
            f"furniture added to scene, chair at position {chair_pos}, quat {chair_quat}")
        logger.info(f"Chair monitoring enabled - will log position changes")
        return furniture

    def build_scene(self):
        """Build scene"""
        if self.scene is None:
            raise RuntimeError("Scene not created yet")

        self.scene.build()
        logger.info("scene built successfully")

    def follow_entity(self, entity):
        """Set view to follow entity"""
        if self.scene is None or self.scene.viewer is None:
            raise RuntimeError("Scene or viewer not available")

        self.scene.viewer.follow_entity(entity)
        logger.info(f"viewer following entity: {entity}")

    def monitor_chair_position(self):
        """Monitor chair position and log changes"""
        if self.chair is None:
            logger.debug("Chair is None, skipping position monitoring")
            return

        try:
            # Get current position and quaternion directly from the chair
            current_pos = self.chair.get_pos()
            current_quat = self.chair.get_quat()

            car_pos = self.car.get_pos()
            car_quat = self.car.get_quat()

            # Convert tensors to CPU and then to numpy if they are tensors
            if hasattr(current_pos, 'cpu'):
                current_pos = current_pos.cpu().numpy()
            if hasattr(current_quat, 'cpu'):
                current_quat = current_quat.cpu().numpy()

            if hasattr(car_pos, 'cpu'):
                car_pos = car_pos.cpu().numpy()
            if hasattr(car_quat, 'cpu'):
                car_quat = car_quat.cpu().numpy()

            # Check chair movement compared to last position
            if self.chair_last_pos is not None and self.chair_last_quat is not None:
                chair_pos_changed = np.linalg.norm(
                    np.array(current_pos) - np.array(self.chair_last_pos)) > 0.01
                chair_quat_changed = np.linalg.norm(
                    np.array(current_quat) - np.array(self.chair_last_quat)) > 0.01

                if chair_pos_changed or chair_quat_changed:
                    logger.info(
                        f"Chair position changed - Pos: {current_pos}, Q: {current_quat}")
            else:
                # First time, log initial position
                logger.info(
                    f"Chair initial position - Pos: {current_pos}, Q: {current_quat}")

            # Check car movement compared to last position
            if self.car_last_pos is not None and self.car_last_quat is not None:
                car_pos_changed = np.linalg.norm(
                    np.array(car_pos) - np.array(self.car_last_pos)) > 0.01
                car_quat_changed = np.linalg.norm(
                    np.array(car_quat) - np.array(self.car_last_quat)) > 0.01

                if car_pos_changed or car_quat_changed:
                    logger.info(
                        f"Car position changed - Pos: {car_pos}, Q: {car_quat}")
            else:
                # First time, log initial position
                logger.info(
                    f"Car initial position - Pos: {car_pos}, Q: {car_quat}")

            # Update last positions for next comparison
            self.chair_last_pos = current_pos.copy()
            self.chair_last_quat = current_quat.copy()
            self.car_last_pos = car_pos.copy()
            self.car_last_quat = car_quat.copy()

        except Exception as e:
            logger.error(f"Failed to monitor chair position: {e}")
            import traceback
            logger.error(traceback.format_exc())

    def step(self):
        """Scene step"""
        if self.scene is None:
            raise RuntimeError("Scene not available")

        try:
            self.scene.step()
            # Monitor chair position changes
            self.monitor_chair_position()
        except Exception as e:
            logger.info(f"scene step failed: {e}")
            raise

    def _get_sim_asset(self, path):
        """Get simulation asset path"""
        this_file_dir = os.path.dirname(os.path.abspath(__file__))
        target_file = os.path.join(this_file_dir, "assets", path)
        logger.info(f"get_sim_asset: {target_file}")
        return target_file
