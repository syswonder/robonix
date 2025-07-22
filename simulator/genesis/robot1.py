import genesis as gs
import numpy as np
from pynput import keyboard
import threading
from scipy.spatial.transform import Rotation as R

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
        res=(1280, 960),
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
    renderer=gs.renderers.RayTracer(
        env_surface=gs.surfaces.Emission(
            emissive_texture=gs.textures.ImageTexture(
                image_path="textures/indoor_bright.png",
            ),
        ),
        env_radius=12.0,
        env_euler=(0, 0, 180),
        lights=[
            {"pos": (0.0, 0.0, 6.0), "radius": 1.5,
             "color": (8.0, 8.0, 8.0)},
        ],
    )
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

if not hasattr(car, '_my_yaw'):
    car._my_yaw = 0.0

while True:
    keys = keyboard_device.get_keys()
    vx, vy, wz = 0.0, 0.0, 0.0
    if keyboard.Key.up in keys:
        vx += speed
    if keyboard.Key.down in keys:
        vx -= speed
    if keyboard.Key.left in keys:
        vy += speed
    if keyboard.Key.right in keys:
        vy -= speed
    if keyboard.KeyCode.from_char('[') in keys:
        wz += rot_speed
    if keyboard.KeyCode.from_char(']') in keys:
        wz -= rot_speed
    if keyboard.Key.esc in keys:
        print("Exiting simulation.")
        keyboard_device.stop()
        break

    # Get current pose (assume [x, y, z, roll, pitch, yaw, ...])
    qpos = car.get_qpos()
    if hasattr(qpos, 'cpu'):
        qpos = qpos.cpu().numpy()
    x, y, z = qpos[:3]

    # Omnidirectional mecanum drive kinematics (no yaw in dx/dy)
    dx = vx * np.sin(car._my_yaw) * dt
    dy = vx * np.cos(car._my_yaw) * dt
    dtheta = wz * dt

    # Update position (translation only)
    qpos_new = qpos.copy()
    qpos_new[0] = x + dx
    qpos_new[1] = y + dy
    car.set_qpos(qpos_new)

    # --- Robust yaw rotation: try set_quat ---
    # Maintain a yaw state variable
    car._my_yaw -= dtheta
    quat = R.from_euler('x', car._my_yaw).as_quat()
    car.set_quat(quat)

    scene.step()
