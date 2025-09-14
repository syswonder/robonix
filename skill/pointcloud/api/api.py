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

print(f"Added Robonix root to sys.path")
print(f"Added spatiallm path: {spatiallm_path}")
print(f"sys.path: {sys.path[:3]}...")

from Robonix.manager.eaios_decorators import eaios
from Robonix.uapi.graph.entity import Entity

import torch
import numpy as np
from tqdm import tqdm
from threading import Thread
from transformers import AutoTokenizer, AutoModelForCausalLM
from transformers import TextIteratorStreamer, set_seed

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

    prompt = f"<|point_start|><|point_pad|><|point_end|>{task_prompt} The reference code is as followed: {code_template}"

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

    print("Starting SpatialLM detection test...")
    print(f"PLY file size: {len(ply_data)} bytes")

    try:
        result = skl_spatiallm_detect_local.__wrapped__(None, ply_data)

        print("Detection completed!")
        print(f"Detected {len(result['walls'])} walls")
        print(f"Detected {len(result['doors'])} doors")
        print(f"Detected {len(result['windows'])} windows")
        print(f"Detected {len(result['objects'])} objects")

        if result["walls"]:
            print("\nWalls:")
            for wall in result["walls"]:
                print(f"  ID: {wall['id']}, start: {wall['start']}, end: {wall['end']}")

        if result["doors"]:
            print("\nDoors:")
            for door in result["doors"]:
                print(
                    f"  ID: {door['id']}, position: {door['position']}, size: {door['width']}x{door['height']}"
                )

        if result["windows"]:
            print("\nWindows:")
            for window in result["windows"]:
                print(
                    f"  ID: {window['id']}, position: {window['position']}, size: {window['width']}x{window['height']}"
                )

        if result["objects"]:
            print("\nObjects:")
            for obj in result["objects"]:
                print(
                    f"  ID: {obj['id']}, class: {obj['class']}, position: {obj['position']}"
                )

        print(f"\nLayout string length: {len(result['layout_string'])} characters")

    except Exception as e:
        print(f"Test failed: {e}")
        import traceback

        traceback.print_exc()
