# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import sys
import os

sys.path.append(
    os.path.dirname(
        os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        )
    )
)
spatiallm_path = os.path.join(os.path.dirname(__file__), "spatiallm")
sys.path.insert(0, spatiallm_path)

print(f"added robonix root to sys.path")
print(f"added spatiallm path: {spatiallm_path}")
print(f"sys.path: {sys.path[:3]}...")

from robonix.manager.eaios_decorators import eaios
from robonix.uapi.graph.entity import Entity
from robonix.uapi.specs.types import (
    EOS_TYPE_SpatialLM_WorldResult,
    EOS_TYPE_SpatialLM_Wall,
    EOS_TYPE_SpatialLM_Door,
    EOS_TYPE_SpatialLM_Window,
    EOS_TYPE_SpatialLM_Bbox,
)

import torch
import numpy as np
from tqdm import tqdm
from threading import Thread
from transformers import AutoTokenizer, AutoModelForCausalLM
from transformers import TextIteratorStreamer, set_seed
import re
import math

from spatiallm.layout.layout import Layout
from spatiallm.pcd import (
    load_o3d_pcd,
    get_points_and_colors,
    cleanup_pcd,
    Compose,
)

DETECT_TYPE_PROMPT = {
    "all": "Detect walls, doors, windows, boxes.",
    "arch": "Detect walls, doors, windows.",
    "object": "Detect boxes.",
}


@eaios.caller
def skl_spatiallm_detect_local(self_entity: Entity, ply_model: bytes) -> dict:
    import tempfile

    model_path = "manycore-research/SpatialLM1.1-Qwen-0.5B"
    detect_type = "all"
    inference_dtype = "bfloat16"
    temperature = 0.6
    top_p = 0.95
    top_k = 10
    max_new_tokens = 4096

    tokenizer = AutoTokenizer.from_pretrained(model_path)
    model = AutoModelForCausalLM.from_pretrained(
        model_path, torch_dtype=getattr(torch, inference_dtype)
    )
    model.to("cuda")
    model.set_point_backbone_dtype(torch.float32)
    model.eval()

    num_bins = model.config.point_config["num_bins"]
    grid_size = Layout.get_grid_size(num_bins)

    with tempfile.NamedTemporaryFile(suffix=".ply", delete=False) as tmp_file:
        tmp_file.write(ply_model)
        tmp_ply_path = tmp_file.name

    try:
        point_cloud = load_o3d_pcd(tmp_ply_path)
        point_cloud = cleanup_pcd(point_cloud, voxel_size=grid_size)
        points, colors = get_points_and_colors(point_cloud)
        min_extent = np.min(points, axis=0)

        input_pcd = preprocess_point_cloud(points, colors, grid_size, num_bins)

        layout = generate_layout(
            model=model,
            point_cloud=input_pcd,
            tokenizer=tokenizer,
            code_template_file=os.path.join(
                os.path.dirname(__file__), "spatiallm", "code_template.txt"
            ),
            top_k=top_k,
            top_p=top_p,
            temperature=temperature,
            max_new_tokens=max_new_tokens,
            detect_type=detect_type,
            categories=[],
        )

        layout.translate(min_extent)

        result = {
            "walls": [],
            "doors": [],
            "windows": [],
            "objects": [],
            "layout_string": layout.to_language_string(),
        }

        for wall in layout.walls:
            result["walls"].append(
                {
                    "id": wall.id,
                    "start": [wall.ax, wall.ay, wall.az],
                    "end": [wall.bx, wall.by, wall.bz],
                    "height": wall.height,
                    "thickness": wall.thickness,
                }
            )

        for door in layout.doors:
            result["doors"].append(
                {
                    "id": door.id,
                    "wall_id": door.wall_id,
                    "position": [door.position_x, door.position_y, door.position_z],
                    "width": door.width,
                    "height": door.height,
                }
            )

        for window in layout.windows:
            result["windows"].append(
                {
                    "id": window.id,
                    "wall_id": window.wall_id,
                    "position": [
                        window.position_x,
                        window.position_y,
                        window.position_z,
                    ],
                    "width": window.width,
                    "height": window.height,
                }
            )

        for bbox in layout.bboxes:
            result["objects"].append(
                {
                    "id": bbox.id,
                    "class": bbox.class_name,
                    "position": [bbox.position_x, bbox.position_y, bbox.position_z],
                    "rotation": bbox.angle_z,
                    "scale": [bbox.scale_x, bbox.scale_y, bbox.scale_z],
                }
            )

        return result

    finally:
        if os.path.exists(tmp_ply_path):
            os.unlink(tmp_ply_path)


def preprocess_point_cloud(points, colors, grid_size, num_bins):
    transform = Compose(
        [
            dict(type="PositiveShift"),
            dict(type="NormalizeColor"),
            dict(
                type="GridSample",
                grid_size=grid_size,
                hash_type="fnv",
                mode="test",
                keys=("coord", "color"),
                return_grid_coord=True,
                max_grid_coord=num_bins,
            ),
        ]
    )

    point_cloud = transform(
        {
            "name": "pcd",
            "coord": points.copy(),
            "color": colors.copy(),
        }
    )

    coord = point_cloud["grid_coord"]
    xyz = point_cloud["coord"]
    rgb = point_cloud["color"]
    point_cloud = np.concatenate([coord, xyz, rgb], axis=1)
    return torch.as_tensor(np.stack([point_cloud], axis=0))


def generate_layout(
    model,
    point_cloud,
    tokenizer,
    code_template_file,
    top_k=10,
    top_p=0.95,
    temperature=0.6,
    max_new_tokens=4096,
    detect_type="all",
    categories=[],
):
    with open(code_template_file, "r") as f:
        code_template = f.read()

    task_prompt = DETECT_TYPE_PROMPT[detect_type]
    if detect_type != "arch" and categories:
        task_prompt = task_prompt.replace("boxes", ", ".join(categories))

    prompt = f"<|point_start|><|point_pad|><|point_end|>{task_prompt} the reference code is as followed: {code_template}"

    if model.config.model_type == "spatiallm_qwen":
        conversation = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": prompt},
        ]
    else:
        conversation = [{"role": "user", "content": prompt}]

    input_ids = tokenizer.apply_chat_template(
        conversation, add_generation_prompt=True, return_tensors="pt"
    )
    input_ids = input_ids.to(model.device)

    with torch.no_grad():
        outputs = model.generate(
            input_ids=input_ids,
            point_clouds=point_cloud,
            max_new_tokens=max_new_tokens,
            do_sample=True,
            use_cache=True,
            temperature=temperature,
            top_p=top_p,
            top_k=top_k,
            num_beams=1,
        )

    generated_text = tokenizer.decode(
        outputs[0][len(input_ids[0]) :], skip_special_tokens=True
    )

    layout = Layout(generated_text)
    layout.undiscretize_and_unnormalize(num_bins=model.config.point_config["num_bins"])

    return layout


if __name__ == "__main__":
    import sys

    ply_file_path = os.path.join(os.path.dirname(__file__), "..", "pointcloud.ply")

    if not os.path.exists(ply_file_path):
        print(f"PLY file not found: {ply_file_path}")
        sys.exit(1)

    with open(ply_file_path, "rb") as f:
        ply_data = f.read()

    print("starting spatiallm detection test...")
    print(f"PLY file size: {len(ply_data)} bytes")

    try:
        result = skl_spatiallm_detect_local.__wrapped__(None, ply_data)

        print("detection completed!")
        print(f"detected {len(result['walls'])} walls")
        print(f"detected {len(result['doors'])} doors")
        print(f"detected {len(result['windows'])} windows")
        print(f"detected {len(result['objects'])} objects")

        if result["walls"]:
            print("\nwalls:")
            for wall in result["walls"]:
                print(f"  ID: {wall['id']}, start: {wall['start']}, end: {wall['end']}")

        if result["doors"]:
            print("\ndoors:")
            for door in result["doors"]:
                print(
                    f"  ID: {door['id']}, position: {door['position']}, size: {door['width']}x{door['height']}"
                )

        if result["windows"]:
            print("\nwindows:")
            for window in result["windows"]:
                print(
                    f"  ID: {window['id']}, position: {window['position']}, size: {window['width']}x{window['height']}"
                )

        if result["objects"]:
            print("\nobjects:")
            for obj in result["objects"]:
                print(
                    f"  ID: {obj['id']}, class: {obj['class']}, position: {obj['position']}"
                )

        print(f"\nlayout string length: {len(result['layout_string'])} characters")

    except Exception as e:
        print(f"test failed: {e}")
        import traceback

        traceback.print_exc()


def parse_spatiallm_txt(spatiallm_txt: str):
    """
    Parse SpatialLM txt output into structured data
    
    Args:
        spatiallm_txt: The txt output from SpatialLM containing object definitions
        
    Returns:
        dict: Parsed objects organized by type
    """
    walls = []
    doors = []
    windows = []
    bboxes = []
    
    lines = spatiallm_txt.strip().split('\n')
    
    for line in lines:
        line = line.strip()
        if not line:
            continue
            
        # Parse wall: wall_0=Wall(-3.941,-2.9770000000000003,-0.491,2.909,-2.9770000000000003,-0.491,3.0600000000000005,0.0)
        wall_match = re.match(r'(\w+)=Wall\(([^)]+)\)', line)
        if wall_match:
            wall_id, params = wall_match.groups()
            coords = [float(x) for x in params.split(',')]
            if len(coords) == 8:
                walls.append({
                    'id': wall_id,
                    'ax': coords[0], 'ay': coords[1], 'az': coords[2],
                    'bx': coords[3], 'by': coords[4], 'bz': coords[5],
                    'height': coords[6], 'thickness': coords[7]
                })
            continue
        
        # Parse door: door_0=Door(wall_1,-3.941,-2.3770000000000002,0.684,0.9800000000000001,2.44)
        door_match = re.match(r'(\w+)=Door\(([^)]+)\)', line)
        if door_match:
            door_id, params = door_match.groups()
            parts = params.split(',')
            if len(parts) == 6:
                doors.append({
                    'id': door_id,
                    'wall_id': parts[0],
                    'position_x': float(parts[1]), 'position_y': float(parts[2]), 'position_z': float(parts[3]),
                    'width': float(parts[4]), 'height': float(parts[5])
                })
            continue
        
        # Parse window: window_0=Window(wall_1,-3.941,-0.002000000000000224,0.8590000000000001,1.22,0.9800000000000001)
        window_match = re.match(r'(\w+)=Window\(([^)]+)\)', line)
        if window_match:
            window_id, params = window_match.groups()
            parts = params.split(',')
            if len(parts) == 6:
                windows.append({
                    'id': window_id,
                    'wall_id': parts[0],
                    'position_x': float(parts[1]), 'position_y': float(parts[2]), 'position_z': float(parts[3]),
                    'width': float(parts[4]), 'height': float(parts[5])
                })
            continue
        
        # Parse bbox: bbox_0=Bbox(tv_cabinet,2.609,-0.9770000000000003,-0.316,-1.5708000000000002,4.0,0.578125,0.53125)
        bbox_match = re.match(r'(\w+)=Bbox\(([^)]+)\)', line)
        if bbox_match:
            bbox_id, params = bbox_match.groups()
            parts = params.split(',')
            if len(parts) == 8:
                bboxes.append({
                    'id': bbox_id,
                    'class_name': parts[0],
                    'position_x': float(parts[1]), 'position_y': float(parts[2]), 'position_z': float(parts[3]),
                    'angle_z': float(parts[4]),
                    'scale_x': float(parts[5]), 'scale_y': float(parts[6]), 'scale_z': float(parts[7])
                })
            continue
    
    return {
        'walls': walls,
        'doors': doors,
        'windows': windows,
        'bboxes': bboxes
    }


def transform_robot_to_world_coords(robot_x: float, robot_y: float, robot_z: float, robot_yaw: float,
                                  local_x: float, local_y: float, local_z: float):
    """
    Transform coordinates from robot frame to world frame.
    Follows ROS2 tf standard: X forward, Y left, Z up, yaw in radians.
    """
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    
    world_x = robot_x + cos_yaw * local_x - sin_yaw * local_y
    world_y = robot_y + sin_yaw * local_x + cos_yaw * local_y
    world_z = robot_z + local_z
    
    return world_x, world_y, world_z


@eaios.caller
def skl_spatiallm_to_world_pose(self_entity: Entity, spatiallm_txt: str) -> EOS_TYPE_SpatialLM_WorldResult:
    """
    Convert SpatialLM detection results from robot coordinates to world coordinates.
    """
    robot_pose = self_entity.cap_get_pose()
    if robot_pose is None:
        raise ValueError("Failed to get robot pose")
    
    robot_x, robot_y, robot_yaw = robot_pose
    robot_z = 0.0  # Assume robot is on ground level
    
    parsed_objects = parse_spatiallm_txt(spatiallm_txt)
    world_walls = []
    for wall in parsed_objects['walls']:
        center_x = (wall['ax'] + wall['bx']) / 2
        center_y = (wall['ay'] + wall['by']) / 2
        center_z = (wall['az'] + wall['bz']) / 2
        
        world_center_x, world_center_y, world_center_z = transform_robot_to_world_coords(
            robot_x, robot_y, robot_z, robot_yaw, center_x, center_y, center_z
        )
        
        world_walls.append(EOS_TYPE_SpatialLM_Wall(
            id=wall['id'],
            ax=wall['ax'], ay=wall['ay'], az=wall['az'],
            bx=wall['bx'], by=wall['by'], bz=wall['bz'],
            height=wall['height'], thickness=wall['thickness'],
            world_center_x=world_center_x,
            world_center_y=world_center_y,
            world_center_z=world_center_z
        ))
    
    world_doors = []
    for door in parsed_objects['doors']:
        world_pos_x, world_pos_y, world_pos_z = transform_robot_to_world_coords(
            robot_x, robot_y, robot_z, robot_yaw,
            door['position_x'], door['position_y'], door['position_z']
        )
        
        world_doors.append(EOS_TYPE_SpatialLM_Door(
            id=door['id'],
            wall_id=door['wall_id'],
            position_x=door['position_x'], position_y=door['position_y'], position_z=door['position_z'],
            width=door['width'], height=door['height'],
            world_center_x=world_pos_x,
            world_center_y=world_pos_y,
            world_center_z=world_pos_z
        ))
    
    world_windows = []
    for window in parsed_objects['windows']:
        world_pos_x, world_pos_y, world_pos_z = transform_robot_to_world_coords(
            robot_x, robot_y, robot_z, robot_yaw,
            window['position_x'], window['position_y'], window['position_z']
        )
        
        world_windows.append(EOS_TYPE_SpatialLM_Window(
            id=window['id'],
            wall_id=window['wall_id'],
            position_x=window['position_x'], position_y=window['position_y'], position_z=window['position_z'],
            width=window['width'], height=window['height'],
            world_center_x=world_pos_x,
            world_center_y=world_pos_y,
            world_center_z=world_pos_z
        ))
    
    world_bboxes = []
    for bbox in parsed_objects['bboxes']:
        world_pos_x, world_pos_y, world_pos_z = transform_robot_to_world_coords(
            robot_x, robot_y, robot_z, robot_yaw,
            bbox['position_x'], bbox['position_y'], bbox['position_z']
        )
        
        world_bboxes.append(EOS_TYPE_SpatialLM_Bbox(
            id=bbox['id'],
            class_name=bbox['class_name'],
            position_x=bbox['position_x'], position_y=bbox['position_y'], position_z=bbox['position_z'],
            angle_z=bbox['angle_z'],
            scale_x=bbox['scale_x'], scale_y=bbox['scale_y'], scale_z=bbox['scale_z'],
            world_center_x=world_pos_x,
            world_center_y=world_pos_y,
            world_center_z=world_pos_z
        ))
    
    return EOS_TYPE_SpatialLM_WorldResult(
        walls=world_walls,
        doors=world_doors,
        windows=world_windows,
        bboxes=world_bboxes,
        robot_pose=(robot_x, robot_y, robot_z, robot_yaw)
    )
