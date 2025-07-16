import genesis as gs
import numpy as np

gs.init(
    backend=gs.gpu,
)

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

# Create floor with wooden color
floor = scene.add_entity(
    gs.morphs.Plane(
        pos=(0.0, 0.0, 0.0)
    ),
    surface=gs.surfaces.Rough(color=(0.8, 0.7, 0.6))
)

# Create a square room (8x8 meters) with lower walls
# North wall
room_wall_north = scene.add_entity(
    gs.morphs.Box(
        pos=(0.0, 4.0, 1.5),
        size=(8.0, 0.2, 3.0),
        fixed=True
    ),
    surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9))
)

# South wall
room_wall_south = scene.add_entity(
    gs.morphs.Box(
        pos=(0.0, -4.0, 1.5),
        size=(8.0, 0.2, 3.0),
        fixed=True
    ),
    surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9))
)

# East wall
room_wall_east = scene.add_entity(
    gs.morphs.Box(
        pos=(4.0, 0.0, 1.5),
        size=(0.2, 8.0, 3.0),
        fixed=True
    ),
    surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9))
)

# West wall
room_wall_west = scene.add_entity(
    gs.morphs.Box(
        pos=(-4.0, 0.0, 1.5),
        size=(0.2, 8.0, 3.0),
        fixed=True
    ),
    surface=gs.surfaces.Smooth(color=(0.9, 0.9, 0.9))
)

# Create ANYmal C robot
try:
    anymal_robot = scene.add_entity(
        gs.morphs.URDF(
            file='urdf/anymal_c/urdf/anymal_c.urdf',
            pos=(0.0, 0.0, 0.0),
            scale=1.0
        )
    )
    print("Using ANYmal C robot model")
except Exception as e:
    print(f"Failed to load ANYmal C model: {e}")
    print("Using fallback robot model")
    
    # Fallback to a simple robot
    robot_body = scene.add_entity(
        gs.morphs.Box(
            pos=(0.0, 0.0, 0.3),
            size=(0.8, 0.4, 0.3),
            fixed=False
        ),
        surface=gs.surfaces.Iron(color=(0.2, 0.2, 0.2))
    )

# Add some furniture to the room
# Table
table = scene.add_entity(
    gs.morphs.Box(
        pos=(-2.0, 2.0, 0.05),
        size=(1.2, 0.8, 0.1),
        fixed=True
    ),
    surface=gs.surfaces.Rough(color=(0.4, 0.2, 0.1))
)

# Chair
chair = scene.add_entity(
    gs.morphs.Box(
        pos=(-2.0, 1.0, 0.4),
        size=(0.5, 0.5, 0.8),
        fixed=True
    ),
    surface=gs.surfaces.Rough(color=(0.6, 0.4, 0.2))
)

# Add some decorative objects
# Plant pot
plant_pot = scene.add_entity(
    gs.morphs.Cylinder(
        pos=(2.0, -2.0, 0.2),
        height=0.4,
        radius=0.3,
        fixed=True
    ),
    surface=gs.surfaces.Rough(color=(0.6, 0.4, 0.2))
)

# Books on table
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

# Main loop
while True:
    scene.step()
