import sys
import os
import genesis as gs
import numpy as np
from pynput import keyboard
import threading
from scipy.spatial.transform import Rotation as R
import grpc
from concurrent import futures
import robot_control_pb2 as robot_control_pb2
import robot_control_pb2_grpc as robot_control_pb2_grpc

# Keyboard input device using pynput


class KeyboardDevice:
    def __init__(self):
        self.pressed_keys = set()
        self.lock = threading.Lock()
        self.listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release)

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

# Initialize Genesis


gs.init(backend=gs.gpu)

scene = gs.Scene(
    show_viewer=True,
    viewer_options=gs.options.ViewerOptions(
        res=(1000, 700),
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
    renderer=gs.renderers.Rasterizer()
)

scene.profiling_options.show_FPS = False

# Create floor
floor = scene.add_entity(
    gs.morphs.Plane(
        pos=(0.0, 0.0, 0.0)
    ),
    surface=gs.surfaces.Rough(color=(0.8, 0.7, 0.6))
)

# Create room walls
room_wall_north = scene.add_entity(
    gs.morphs.Box(
        pos=(0.0, 4.0, 0.5),
        size=(8.0, 0.2, 1.0),
        fixed=True
    ),
    surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9))
)
room_wall_south = scene.add_entity(
    gs.morphs.Box(
        pos=(0.0, -4.0, 0.5),
        size=(8.0, 0.2, 1.0),
        fixed=True
    ),
    surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9))
)
room_wall_east = scene.add_entity(
    gs.morphs.Box(
        pos=(4.0, 0.0, 0.5),
        size=(0.2, 8.0, 1.0),
        fixed=True
    ),
    surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9))
)
room_wall_west = scene.add_entity(
    gs.morphs.Box(
        pos=(-4.0, 0.0, 0.5),
        size=(0.2, 8.0, 1.0),
        fixed=True
    ),
    surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9))
)

# Create the car (box)
car = scene.add_entity(
    gs.morphs.Box(
        pos=(0.0, 0.0, 0.15),
        size=(0.3, 0.5, 0.3),
        fixed=False
    ),
    surface=gs.surfaces.Iron(color=(0.2, 0.2, 0.8))
)

scene.viewer.follow_entity(car)

# Add furniture and decorations (unchanged)
table = scene.add_entity(
    gs.morphs.Box(
        pos=(-2.0, 2.0, 0.05),
        size=(1.2, 0.8, 0.1),
        fixed=True
    ),
    surface=gs.surfaces.Rough(color=(0.4, 0.2, 0.1))
)
chair = scene.add_entity(
    gs.morphs.Box(
        pos=(-2.0, 1.0, 0.4),
        size=(0.5, 0.5, 0.8),
        fixed=True
    ),
    surface=gs.surfaces.Rough(color=(0.6, 0.4, 0.2))
)
plant_pot = scene.add_entity(
    gs.morphs.Cylinder(
        pos=(2.0, -2.0, 0.2),
        height=0.4,
        radius=0.3,
        fixed=True
    ),
    surface=gs.surfaces.Rough(color=(0.6, 0.4, 0.2))
)
book1 = scene.add_entity(
    gs.morphs.Box(
        pos=(-2.2, 1.8, 0.1),
        size=(0.15, 0.2, 0.05),
        fixed=True
    ),
    surface=gs.surfaces.Rough(color=(0.8, 0.2, 0.2))
)
book2 = scene.add_entity(
    gs.morphs.Box(
        pos=(-1.8, 1.8, 0.1),
        size=(0.15, 0.2, 0.05),
        fixed=True
    ),
    surface=gs.surfaces.Rough(color=(0.2, 0.6, 0.8))
)

scene.build()

# Keyboard control setup
keyboard_device = KeyboardDevice()
keyboard_device.start()

# Control parameters
dt = 1.0 / 60.0  # 60 FPS
speed = 1.5      # translation speed
rot_speed = 2.5  # rotation speed

print("\nKeyboard Controls:")
print("Arrow Up/Down: Forward/Backward")
print("Arrow Left/Right: Left/Right")
print("[: Rotate left")
print("]: Rotate right")
print("ESC: Quit")


def print_car_position(car, last_pos, last_yaw):
    """
    Print the car's current position and yaw angle in degrees (0~360), formatted to 4 decimal places for position and 2 decimal places for yaw. Only prints if the position or yaw has changed.
    Returns the new position tuple and yaw.
    """
    car_x, car_y, car_z = car.get_pos()
    car_x = float(car_x)
    car_y = float(car_y)
    car_z = float(car_z)
    pos = (round(car_x, 4), round(car_y, 4), round(car_z, 4))
    yaw_deg = np.degrees(getattr(car, '_my_yaw', 0.0)) % 360
    yaw_deg_rounded = round(yaw_deg, 2)
    if pos != last_pos or yaw_deg_rounded != last_yaw:
        print(
            f"Car position: x={car_x:.4f}, y={car_y:.4f}, z={car_z:.4f}, yaw={yaw_deg:.2f}°")
    return pos, yaw_deg_rounded


def control_car_loop(car, keyboard_device, dt, max_speed, max_rot_speed, accel=3.0, rot_accel=5.0):
    """
    Main control loop for the car. Uses acceleration for smooth control.
    Args:
        car: The car entity to control.
        keyboard_device: Instance of KeyboardDevice for input.
        dt: Time step per frame.
        max_speed: Maximum linear speed.
        max_rot_speed: Maximum rotational speed.
        accel: Linear acceleration (m/s^2).
        rot_accel: Rotational acceleration (rad/s^2).
    """
    if not hasattr(car, '_my_yaw'):
        car._my_yaw = 0.0
    # Current velocities
    vx = 0.0
    vy = 0.0
    wz = 0.0
    # Target velocities
    target_vx = 0.0
    target_vy = 0.0
    target_wz = 0.0
    last_pos = None
    last_yaw = None
    while True:
        keys = keyboard_device.get_keys()
        # Set target velocities based on key input
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
        if keyboard.KeyCode.from_char('[') in keys:
            target_wz += max_rot_speed
        if keyboard.KeyCode.from_char(']') in keys:
            target_wz -= max_rot_speed
        if keyboard.Key.esc in keys:
            print("Exiting simulation.")
            keyboard_device.stop()
            break

        # Smoothly update velocities using acceleration limits
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

        # Get current position and orientation
        qpos = car.get_qpos()
        if hasattr(qpos, 'cpu'):
            qpos = qpos.cpu().numpy()
        x, y, z = qpos[:3]

        # Calculate new position and orientation
        dx = vx * np.sin(car._my_yaw) * dt + vy * np.cos(car._my_yaw) * dt
        dy = vx * np.cos(car._my_yaw) * dt - vy * np.sin(car._my_yaw) * dt
        dtheta = wz * dt

        qpos_new = qpos.copy()
        qpos_new[0] = x - dx
        qpos_new[1] = y - dy
        car.set_qpos(qpos_new)

        car._my_yaw -= dtheta
        quat = R.from_euler('x', car._my_yaw).as_quat()
        car.set_quat(quat)

        # Print current position if changed
        last_pos, last_yaw = print_car_position(car, last_pos, last_yaw)

        # Step the simulation
        scene.step()


class RobotControlService(robot_control_pb2_grpc.RobotControlServicer):
    def __init__(self, car, keyboard_device):
        self.car = car
        self.keyboard_device = keyboard_device
        self._lock = threading.Lock()

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
        """
        Rotate the car to the target yaw (current + angle), simulating acceleration and deceleration for a smooth rotation.
        """
        import time
        angle = request.angle  # angle in radians
        current_yaw = getattr(self.car, '_my_yaw', 0.0)
        target_yaw = (current_yaw + angle) % (2 * np.pi)
        max_speed = 2.0  # Maximum angular speed (rad/s)
        accel = 4.0      # Angular acceleration (rad/s^2)
        dt = 1.0 / 60.0  # Time step (60 FPS)
        yaw = current_yaw
        speed = 0.0
        # Determine rotation direction (+1 for CCW, -1 for CW)
        diff = (target_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
        direction = 1 if diff > 0 else -1
        while True:
            diff = (target_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
            if abs(diff) < 0.01:
                break
            # Acceleration or deceleration phase
            if abs(diff) > 0.2:
                speed = min(speed + accel * dt, max_speed)
            else:
                speed = max(speed - accel * dt, 0.2)
            # Limit step to not overshoot
            step_angle = direction * min(abs(diff), speed * dt)
            yaw = (yaw + step_angle) % (2 * np.pi)
            self.car._my_yaw = yaw
            quat = R.from_euler('x', yaw).as_quat()
            self.car.set_quat(quat)
            scene.step()
            time.sleep(dt)
        return robot_control_pb2.MoveReply(status="ok")

    def Stop(self, request, context):
        # Release all keys to stop the car
        self.keyboard_device.pressed_keys.clear()
        return robot_control_pb2.MoveReply(status="stopped")

    def GetPose(self, request, context):
        # Return the current pose of the car, with yaw in degrees (0~360)
        car_x, car_y, car_z = self.car.get_pos()
        yaw = getattr(self.car, '_my_yaw', 0.0)
        yaw_deg = np.degrees(yaw) % 360
        return robot_control_pb2.PoseReply(x=float(car_x), y=float(car_y), z=float(car_z), yaw=float(yaw_deg))

    def MoveTo(self, request, context):
        """
        Move the car forward/backward and laterally (left/right) by a specified distance,
        simulating smooth acceleration, cruising, and deceleration.
        """
        import time
        forward = request.forward  # meters
        lateral = request.lateral  # meters
        max_speed = 1.5  # Maximum linear speed (m/s)
        accel = 3.0      # Linear acceleration (m/s^2)
        dt = 1.0 / 60.0  # Time step (60 FPS)
        # Get current position and yaw
        qpos = self.car.get_qpos()
        if hasattr(qpos, 'cpu'):
            qpos = qpos.cpu().numpy()
        x, y, z = qpos[:3]
        yaw = getattr(self.car, '_my_yaw', 0.0)
        # Compute target position in world frame
        dx = forward * np.sin(yaw) + lateral * np.cos(yaw)
        dy = forward * np.cos(yaw) - lateral * np.sin(yaw)
        target_x = x + dx
        target_y = y + dy
        # Movement loop with smooth acceleration and deceleration
        v = 0.0
        while True:
            qpos = self.car.get_qpos()
            if hasattr(qpos, 'cpu'):
                qpos = qpos.cpu().numpy()
            curr_x, curr_y, _ = qpos[:3]
            rem_x = target_x - curr_x
            rem_y = target_y - curr_y
            rem_dist = np.hypot(rem_x, rem_y)
            if rem_dist < 0.01:
                break
            # Compute stopping distance for smooth deceleration
            stopping_dist = (v ** 2) / (2 * accel) if accel > 0 else 0.0
            if rem_dist > stopping_dist:
                # Accelerate or cruise
                v = min(v + accel * dt, max_speed)
            else:
                # Decelerate
                v = max(v - accel * dt, 0.0)
            # Direction in world frame
            if rem_dist > 1e-6:
                dir_x = rem_x / rem_dist
                dir_y = rem_y / rem_dist
            else:
                dir_x = dir_y = 0.0
            # Update position using velocity integration
            step_x = curr_x + dir_x * v * dt
            step_y = curr_y + dir_y * v * dt
            qpos_new = qpos.copy()
            qpos_new[0] = step_x
            qpos_new[1] = step_y
            self.car.set_qpos(qpos_new)
            scene.step()
            time.sleep(dt)
        return robot_control_pb2.MoveReply(status="ok")


def serve_grpc(car, keyboard_device, port=50051):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    robot_control_pb2_grpc.add_RobotControlServicer_to_server(
        RobotControlService(car, keyboard_device), server)
    server.add_insecure_port(f'[::]:{port}')
    print(f"[gRPC] RobotControl server started on port {port}")
    server.start()
    return server


# Start gRPC server in background
grpc_server = serve_grpc(car, keyboard_device)
try:
    # Start the main control loop for the car (with acceleration control)
    control_car_loop(car, keyboard_device, dt, speed, rot_speed)
finally:
    grpc_server.stop(0)
