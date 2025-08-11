# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import os
import genesis as gs

try:
    from uapi.log import logger
except ImportError:
    import logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
    logger = logging.getLogger(__name__)


class SceneManager:
    def __init__(self):
        self.scene = None
        
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
        logger.info("Genesis scene created successfully")
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
        logger.info(f"Camera added to scene at position {pos}")
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
                    image_path=self._get_sim_asset("texture_tiles_0024/tiles_0024_color_1k.jpg")
                ),
                normal_texture=gs.textures.ImageTexture(
                    image_path=self._get_sim_asset(
                        "texture_tiles_0024/tiles_0024_normal_opengl_1k.png"
                    )
                ),
            ),
        )
        logger.info("Floor added to scene")
        return floor
        
    def add_room_walls(self):
        """Add room walls"""
        if self.scene is None:
            raise RuntimeError("Scene not created yet")
            
        walls = []
        
        # North wall
        wall_north = self.scene.add_entity(
            gs.morphs.Box(pos=(0.0, 4.0, -0.1), size=(8.0, 0.2, 3.0), fixed=True),
            surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9)),
        )
        walls.append(wall_north)
        
        # South wall
        wall_south = self.scene.add_entity(
            gs.morphs.Box(pos=(0.0, -4.0, -0.1), size=(8.0, 0.2, 3.0), fixed=True),
            surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9)),
        )
        walls.append(wall_south)
        
        # East wall
        wall_east = self.scene.add_entity(
            gs.morphs.Box(pos=(4.0, 0.0, -0.1), size=(0.2, 8.0, 3.0), fixed=True),
            surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9)),
        )
        walls.append(wall_east)
        
        # West wall
        wall_west = self.scene.add_entity(
            gs.morphs.Box(pos=(-4.0, 0.0, -0.1), size=(0.2, 8.0, 3.0), fixed=True),
            surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9)),
        )
        walls.append(wall_west)
        
        logger.info("Room walls added to scene")
        return walls
        
    def add_car(self, pos=(0.0, -2, 0.15), size=(0.3, 0.5, 0.3)):
        """Add car to scene"""
        if self.scene is None:
            raise RuntimeError("Scene not created yet")
            
        car = self.scene.add_entity(
            gs.morphs.Box(pos=pos, size=size, fixed=False),
            surface=gs.surfaces.Iron(color=(0.2, 0.2, 0.8)),
        )
        logger.info(f"Car added to scene at position {pos}")
        return car
        
    def add_furniture(self):
        """Add furniture and decoration"""
        if self.scene is None:
            raise RuntimeError("Scene not created yet")
            
        furniture = []
        
        # Chair
        chair = self.scene.add_entity(
            morph=gs.morphs.Mesh(
                file=self._get_sim_asset("chair1/MAD_QUEEN_CHAIR.obj"),
                scale=0.007,
                pos=(0.3, 2, 0.8),
            ),
            surface=gs.surfaces.Default(
                diffuse_texture=gs.textures.ImageTexture(
                    image_path=self._get_sim_asset("chair1/MAD_QUEEN_CHAIR/3_1_2_d.jpg"),
                ),
                roughness_texture=gs.textures.ImageTexture(
                    image_path=self._get_sim_asset("chair1/MAD_QUEEN_CHAIR/14_2_8_r.jpg"),
                ),
            ),
        )
        furniture.append(chair)
        
        logger.info("Furniture added to scene")
        return furniture
        
    def build_scene(self):
        """Build scene"""
        if self.scene is None:
            raise RuntimeError("Scene not created yet")
            
        self.scene.build()
        logger.info("Scene built successfully")
        
    def follow_entity(self, entity):
        """Set view to follow entity"""
        if self.scene is None or self.scene.viewer is None:
            raise RuntimeError("Scene or viewer not available")
            
        self.scene.viewer.follow_entity(entity)
        logger.info(f"Viewer following entity: {entity}")
        
    def step(self):
        """Scene step"""
        if self.scene is None:
            raise RuntimeError("Scene not available")
            
        try:
            self.scene.step()
        except Exception as e:
            logger.info(f"Scene step failed: {e}")
            raise
            
    def _get_sim_asset(self, path):
        """Get simulation asset path"""
        this_file_dir = os.path.dirname(os.path.abspath(__file__))
        target_file = os.path.join(this_file_dir, "assets", path)
        logger.info(f"get_sim_asset: {target_file}")
        return target_file
