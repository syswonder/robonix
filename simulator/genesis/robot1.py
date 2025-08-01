# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import robot_control_pb2_grpc as robot_control_pb2_grpc
import robot_control_pb2 as robot_control_pb2
from concurrent import futures
import grpc
from scipy.spatial.transform import Rotation as R
import threading
from pynput import keyboard
import numpy as np
import genesis as gs
import sys
import os
import cv2
import time

# Add the project root to Python path to import uapi
PROJECT_ROOT = os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, PROJECT_ROOT)

from uapi.log import logger

GLOBAL_SCENE = None


class KeyboardDevice:
    def __init__(self):
        self.pressed_keys = set()
        self.lock = threading.Lock()
        self.listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        )

    def start(self):
        self.listener.start()

    def stop(self):
        self.listener.stop()
        self.listener.join()

    def on_press(self, key):
        with self.lock:
            self.pressed_keys.add(key)

    def on_release(self, key):
        with self.lock:
            self.pressed_keys.discard(key)

    def get_keys(self):
        with self.lock:
            return set(self.pressed_keys)


def print_car_position(car, last_pos, last_yaw):
    car_x, car_y, car_z = car.get_pos()
    car_x = float(car_x)
    car_y = float(car_y)
    car_z = float(car_z)
    pos = (round(car_x, 4), round(car_y, 4), round(car_z, 4))
    yaw_deg = np.degrees(getattr(car, "_my_yaw", 0.0)) % 360
    yaw_deg_rounded = round(yaw_deg, 2)
    if pos != last_pos or yaw_deg_rounded != last_yaw:
        logger.info(
            f"car position: x={car_x:.4f}, y={car_y:.4f}, z={car_z:.4f}, yaw={yaw_deg:.2f}°"
        )
    return pos, yaw_deg_rounded


def camera_update_loop(camera, car, stop_event, scene_lock):
    """Camera update thread that runs every 0.5 seconds"""
    while not stop_event.is_set():
        try:
            with scene_lock:
                # Update camera position to follow car
                car_pos = car.get_pos()
                car_x, car_y, car_z = float(car_pos[0]), float(car_pos[1]), float(car_pos[2])
                car_yaw = getattr(car, "_my_yaw", 0.0)
                
                # Calculate camera position at car front (0.25 units forward from car center)
                camera_offset_x = 0.25 * np.cos(car_yaw)
                camera_offset_y = 0.25 * np.sin(car_yaw)
                camera_x = car_x + camera_offset_x
                camera_y = car_y + camera_offset_y
                camera_z = car_z + 0.15  # Slightly above car center
                
                # Calculate lookat point (forward direction)
                lookat_x = camera_x + np.cos(car_yaw)
                lookat_y = camera_y + np.sin(car_yaw)
                lookat_z = camera_z
                
                # Update camera position and orientation
                camera.set_pose(pos=(camera_x, camera_y, camera_z), lookat=(lookat_x, lookat_y, lookat_z))
                
                rgb, depth, segmentation, normal = camera.render(depth=True, segmentation=True, normal=True)
            cv2.imshow("rgb", rgb)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break
        except Exception as e:
            logger.error(f"Camera update error: {e}")
        
        time.sleep(0.5)  # Update every 0.5 seconds


def control_car_loop(
    car, keyboard_device, dt, max_speed, max_rot_speed, accel=3.0, rot_accel=5.0, camera=None, scene_lock=None
):
    # Start camera thread if camera is provided
    camera_thread = None
    stop_event = threading.Event()
    
    if camera is not None:
        camera_thread = threading.Thread(target=camera_update_loop, args=(camera, car, stop_event, scene_lock))
        camera_thread.daemon = True
        camera_thread.start()

    if not hasattr(car, "_my_yaw"):
        car._my_yaw = 0.0
    vx = 0.0
    vy = 0.0
    wz = 0.0
    target_vx = 0.0
    target_vy = 0.0
    target_wz = 0.0
    last_pos = None
    last_yaw = None
    move_to_vx = 0.0
    move_to_vy = 0.0
    move_to_active = False

    try:
        while True:
            keys = keyboard_device.get_keys()
            target_vx = 0.0
            target_vy = 0.0
            target_wz = 0.0
            if keyboard.Key.up in keys:
                target_vx += max_speed
            if keyboard.Key.down in keys:
                target_vx -= max_speed
            if keyboard.Key.left in keys:
                target_vy -= max_speed
            if keyboard.Key.right in keys:
                target_vy += max_speed
            if keyboard.KeyCode.from_char("[") in keys:
                target_wz += max_rot_speed
            if keyboard.KeyCode.from_char("]") in keys:
                target_wz -= max_rot_speed
            if keyboard.Key.esc in keys:
                logger.info("Exiting simulation.")
                keyboard_device.stop()
                break
            keyboard_active = any([target_vx != 0, target_vy != 0, target_wz != 0])
            if (
                hasattr(car, "_move_to_target")
                and car._move_to_target.get("active", False)
                and not keyboard_active
            ):
                move_to_active = True
                target = car._move_to_target
                if target["start_time"] is None:
                    target["start_time"] = 0.0
                else:
                    target["start_time"] += dt
                qpos = car.get_qpos()
                if hasattr(qpos, "cpu"):
                    qpos = qpos.cpu().numpy()
                curr_x, curr_y, _ = qpos[:3]
                target_x = target["target_x"]
                target_y = target["target_y"]
                rem_x = target_x - curr_x
                rem_y = target_y - curr_y
                rem_dist = np.hypot(rem_x, rem_y)
                if rem_dist < target["tolerance"]:
                    move_to_vx = 0.0
                    move_to_vy = 0.0
                    move_to_active = False
                    car._move_to_target["active"] = False
                    logger.info(
                        f"MoveTo completed: reached target within {target['tolerance']:.3f}m"
                    )
                else:
                    if rem_dist > 1e-6:
                        dir_x = rem_x / rem_dist
                        dir_y = rem_y / rem_dist
                    else:
                        dir_x = dir_y = 0.0
                    current_speed = np.hypot(move_to_vx, move_to_vy)
                    stopping_dist = (
                        (current_speed**2) / (2 * target["decel"])
                        if target["decel"] > 0
                        else 0.0
                    )
                    if rem_dist > stopping_dist + 0.1:
                        target_speed = min(
                            current_speed + target["accel"] *
                            dt, target["max_speed"]
                        )
                    else:
                        # Decelerate
                        target_speed = max(
                            current_speed - target["decel"] * dt, 0.0)
                    move_to_vx = dir_x * target_speed
                    move_to_vy = dir_y * target_speed
            else:
                move_to_active = False
                move_to_vx = 0.0
                move_to_vy = 0.0
                if keyboard_active and hasattr(car, "_move_to_target"):
                    car._move_to_target["active"] = False
                    logger.info("MoveTo interrupted by keyboard input")

                def approach(current, target, a):
                    if current < target:
                        return min(current + a * dt, target)
                    elif current > target:
                        return max(current - a * dt, target)
                    else:
                        return current
                vx = approach(vx, target_vx, accel)
                vy = approach(vy, target_vy, accel)
                wz = approach(wz, target_wz, rot_accel)
            qpos = car.get_qpos()
            if hasattr(qpos, "cpu"):
                qpos = qpos.cpu().numpy()
            x, y, z = qpos[:3]
            if move_to_active:
                dx = move_to_vx * dt
                dy = move_to_vy * dt
                dtheta = 0.0
            else:
                dx = vx * np.sin(car._my_yaw) * dt + vy * np.cos(car._my_yaw) * dt
                dy = vx * np.cos(car._my_yaw) * dt - vy * np.sin(car._my_yaw) * dt
                dtheta = wz * dt

            qpos_new = qpos.copy()
            qpos_new[0] = x + dx
            qpos_new[1] = y + dy
            car.set_qpos(qpos_new)

            car._my_yaw -= dtheta
            quat = R.from_euler("x", car._my_yaw).as_quat()
            car.set_quat(quat)

            last_pos, last_yaw = print_car_position(car, last_pos, last_yaw)

            with scene_lock:
                global GLOBAL_SCENE
                GLOBAL_SCENE.step()
    finally:
        # Stop camera thread
        if camera_thread is not None:
            stop_event.set()
            camera_thread.join(timeout=1.0)
            cv2.destroyAllWindows()


class RobotControlService(robot_control_pb2_grpc.RobotControlServicer):
    def __init__(self, car, keyboard_device, scene_lock):
        self.car = car
        self.keyboard_device = keyboard_device
        self._lock = threading.Lock()
        self._scene_lock = scene_lock

    def Move(self, request, context):
        # Move the car forward or backward by simulating key press
        distance = request.distance
        # For simplicity, use a fixed step and duration
        key = None
        if distance > 0:
            key = keyboard.Key.up
        elif distance < 0:
            key = keyboard.Key.down
        if key:
            self.keyboard_device.on_press(key)
            # Sleep time proportional to distance (tune as needed)
            import time

            time.sleep(min(abs(distance), 1.0))
            self.keyboard_device.on_release(key)
        return robot_control_pb2.MoveReply(status="ok")

    def Rotate(self, request, context):
        import time

        angle = request.angle  # angle in radians
        current_yaw = getattr(self.car, "_my_yaw", 0.0)
        target_yaw = (current_yaw + angle) % (2 * np.pi)
        max_speed = 2.0  # Maximum angular speed (rad/s)
        accel = 4.0  # Angular acceleration (rad/s^2)
        dt = 1.0 / 60.0  # Time step (60 FPS)
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
            with self._scene_lock:
                global GLOBAL_SCENE
                GLOBAL_SCENE.step()
            time.sleep(dt)
        return robot_control_pb2.MoveReply(status="ok")

    def Stop(self, request, context):
        self.keyboard_device.pressed_keys.clear()
        if hasattr(self.car, "_move_to_target"):
            self.car._move_to_target["active"] = False
        return robot_control_pb2.MoveReply(status="stopped")

    def GetPose(self, request, context):
        car_x, car_y, car_z = self.car.get_pos()
        yaw = getattr(self.car, "_my_yaw", 0.0)
        yaw_deg = np.degrees(yaw) % 360
        return robot_control_pb2.PoseReply(
            x=float(car_x), y=float(car_y), z=float(car_z), yaw=float(yaw_deg)
        )

    def MoveTo(self, request, context):
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


def serve_grpc(car, keyboard_device, scene_lock, port=50051):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    robot_control_pb2_grpc.add_RobotControlServicer_to_server(
        RobotControlService(car, keyboard_device, scene_lock), server
    )
    server.add_insecure_port(f"[::]:{port}")
    logger.info(f"[gRPC] RobotControl server started on port {port}")
    server.start()
    return server


def get_sim_asset(path):
    this_file_dir = os.path.dirname(os.path.abspath(__file__))
    target_file = os.path.join(this_file_dir, "assets", path)
    logger.info(f"get_sim_asset: {target_file}")
    return target_file


def main():

    gs.init(backend=gs.gpu)

    global GLOBAL_SCENE
    GLOBAL_SCENE = gs.Scene(
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
            show_world_frame=True,
            world_frame_size=2.0,
            show_link_frame=False,
            show_cameras=False,
            plane_reflection=True,
            ambient_light=(0.3, 0.3, 0.3),
        ),
        # renderer=gs.renderers.RayTracer(
        #     env_surface=gs.surfaces.Emission(
        #         emissive_texture=gs.textures.ImageTexture(
        #             image_path="textures/indoor_bright.png",
        #         ),
        #     ),
        #     env_radius=12.0,
        #     env_euler=(0, 0, 180),
        #     lights=[
        #         {"pos": (0.0, 0.0, 6.0), "radius": 1.5,
        #          "color": (8.0, 8.0, 8.0)},
        #     ],
        # )
        renderer=gs.renderers.Rasterizer(),
    )

    GLOBAL_SCENE.profiling_options.show_FPS = False

    cam = GLOBAL_SCENE.add_camera(
        res    = (800, 600),
        pos    = (0.25, 0.0, 0.15),  # Initial position at car front
        lookat = (1.0, 0.0, 0.15),   # Look forward
        fov    = 60,                 # Wider FOV for better view
        GUI    = False
    )

    # Create floor
    floor = GLOBAL_SCENE.add_entity(
        gs.morphs.Plane(pos=(0.0, 0.0, 0.0)),
        surface=gs.surfaces.Rough(
            roughness=0.5,
            diffuse_texture=gs.textures.ImageTexture(
                image_path=get_sim_asset(
                    "texture_tiles_0024/tiles_0024_color_1k.jpg")
            ),
            normal_texture=gs.textures.ImageTexture(
                image_path=get_sim_asset(
                    "texture_tiles_0024/tiles_0024_normal_opengl_1k.png")
            ),
        )
    )

    # Create room walls
    room_wall_north = GLOBAL_SCENE.add_entity(
        gs.morphs.Box(pos=(0.0, 4.0, -0.1), size=(8.0, 0.2, 3.0), fixed=True),
        surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9)),
    )
    room_wall_south = GLOBAL_SCENE.add_entity(
        gs.morphs.Box(pos=(0.0, -4.0, -0.1), size=(8.0, 0.2, 3.0), fixed=True),
        surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9)),
    )
    room_wall_east = GLOBAL_SCENE.add_entity(
        gs.morphs.Box(pos=(4.0, 0.0, -0.1), size=(0.2, 8.0, 3.0), fixed=True),
        surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9)),
    )
    room_wall_west = GLOBAL_SCENE.add_entity(
        gs.morphs.Box(pos=(-4.0, 0.0, -0.1), size=(0.2, 8.0, 3.0), fixed=True),
        surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9)),
    )

    # Create the car (box)
    car = GLOBAL_SCENE.add_entity(
        gs.morphs.Box(pos=(0.0, 0.0, 0.15), size=(0.3, 0.5, 0.3), fixed=False),
        surface=gs.surfaces.Iron(color=(0.2, 0.2, 0.8)),
    )

    # car = GLOBAL_SCENE.add_entity(
    #     morph=gs.morphs.URDF(file="urdf/go2/urdf/go2.urdf", pos=(-1, 0.0, 0.5)),
    # )

    GLOBAL_SCENE.viewer.follow_entity(car)

    # # Add furniture and decorations (unchanged)
    # table = GLOBAL_SCENE.add_entity(
    #     gs.morphs.Box(pos=(-2.0, 2.0, 0.05), size=(1.2, 0.8, 0.1), fixed=True),
    #     surface=gs.surfaces.Rough(color=(0.4, 0.2, 0.1)),
    # )
    # chair = GLOBAL_SCENE.add_entity(
    #     gs.morphs.Box(pos=(-2.0, 1.0, 0.4), size=(0.5, 0.5, 0.8), fixed=True),
    #     surface=gs.surfaces.Rough(color=(0.6, 0.4, 0.2)),
    # )
    # plant_pot = GLOBAL_SCENE.add_entity(
    #     gs.morphs.Cylinder(pos=(2.0, -2.0, 0.2), height=0.4,
    #                        radius=0.3, fixed=True),
    #     surface=gs.surfaces.Rough(color=(0.6, 0.4, 0.2)),
    # )
    # book1 = GLOBAL_SCENE.add_entity(
    #     gs.morphs.Box(pos=(-2.2, 1.8, 0.1),
    #                   size=(0.15, 0.2, 0.05), fixed=True),
    #     surface=gs.surfaces.Rough(color=(0.8, 0.2, 0.2)),
    # )
    # book2 = GLOBAL_SCENE.add_entity(
    #     gs.morphs.Box(pos=(-1.8, 1.8, 0.1),
    #                   size=(0.15, 0.2, 0.05), fixed=True),
    #     surface=gs.surfaces.Rough(color=(0.2, 0.6, 0.8)),
    # )

    # add 3D models
    ########################## materials ##########################
    mat_elastic = gs.materials.PBD.Elastic()

    ########################## entities ##########################
    dragon = GLOBAL_SCENE.add_entity(
        # material=mat_elastic,
        morph=gs.morphs.Mesh(
            file="meshes/dragon/dragon.obj",
            scale=0.007,
            pos=(1, 1, 0.8),
        ),
        surface=gs.surfaces.Default(
            # vis_mode='recon',
        ),
    )

    chair = GLOBAL_SCENE.add_entity(
        morph=gs.morphs.Mesh(
            file=get_sim_asset("chair1/MAD_QUEEN_CHAIR.obj"),
            scale=0.007,
            pos=(-1, -1, 0.8),
        ),
        surface=gs.surfaces.Default(
            diffuse_texture=gs.textures.ImageTexture(
                image_path=get_sim_asset("chair1/MAD_QUEEN_CHAIR/3_1_2_d.jpg"),
            ),
            roughness_texture=gs.textures.ImageTexture(
                image_path=get_sim_asset("chair1/MAD_QUEEN_CHAIR/14_2_8_r.jpg"),
            ),
        ),
    )

    GLOBAL_SCENE.build()
    # rgb, depth, segmentation, normal = cam.render(depth=True, segmentation=True, normal=True)

    # Keyboard control setup
    keyboard_device = KeyboardDevice()
    keyboard_device.start()

    # Control parameters
    dt = 1.0 / 60.0  # 60 FPS
    speed = 1.5  # translation speed
    rot_speed = 2.5  # rotation speed

    logger.info("\nKeyboard Controls:")
    logger.info("Arrow Up/Down: Forward/Backward")
    logger.info("Arrow Left/Right: Left/Right")
    logger.info("[: Rotate left")
    logger.info("]: Rotate right")
    logger.info("ESC: Quit")

    # Create shared scene lock
    scene_lock = threading.Lock()
    
    grpc_server = serve_grpc(car, keyboard_device, scene_lock)
    try:
        control_car_loop(car, keyboard_device, dt, speed, rot_speed, camera=cam, scene_lock=scene_lock)
    finally:
        grpc_server.stop(0)


if __name__ == "__main__":
    main()
