"""
Skill Specifications Module
===========================

This module defines the skill specifications for the robonix OS system.
It contains the complete specification of all capabilities and skills available
in the system, including their input/output types and dependencies.

The specifications follow a standardized format where:
- Capabilities are atomic operations provided by skill providers
- Skills are composite operations that depend on one or more capabilities
- Input/output types are strictly defined for type checking
"""

# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

from typing import Tuple, Dict, Any, Optional
from .types import *

# documentation:
# For input and output fields, the argument and return values are standardized to dict or class.
# - For simple types, use {<str>: <class>} (e.g., {"x": float, "y": float, "z": float})
# - For complex types, define a dataclass/class (e.g., EOS_SPEC_Image) and use the class directly as the value.
# The TYPE here is the actual Python type symbol, like str, int, float, or a dataclass/class.
# For fields other than input and output, the above rules are not applied!
# Example:
#   { "input": None, "output": EOS_SPEC_Image }
#   { "input": {"target": str, "distance": float}, "output": {"success": bool} }
# This enables strict type checking, including for nested/complex structures.
# Also noted that EOS_SPEC is <class> for complex types, not just <Dict>.
# - wheatfox 2025.7.25

# "skill_name": {
#     "description": <str>,
#     "type": <str>
#     "input": <Dict[str, Any]>, // Any is Class or Dict
#     "output": <Dict[str, Any]>,
#     "dependencies": <List[str]>, // List of skill names, when type is capability, this field should not exist
# }

EOS_SKILL_SPECS = {
    ##########################
    ###### CAPABILITIES ######
    ##########################
    "cap_pointcloud_to_file": {
        "description": "Convert pointcloud to a .ply 3D model file",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"filename": str},
        "output": {"success": bool},
    },
    "cap_space_getpos": {
        # DEPRECATED !!!
        "description": "Get the position of the entity",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": Optional[Tuple[float, float, float]],  # (x,y,z) tuple
    },
    "cap_space_move": {
        # DEPRECATED !!!
        "description": "Move the entity to the given position",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"x": float, "y": float, "z": float},  # (x,y,z) tuple
        "output": {"success": bool},
    },
    # "skl_space_move2entity": {
    #     "description": "Move the entity to vicinity of another entity",
    #     "type": EOS_SkillType.SKILL,
    #     "input": {
    #         # TODO: use list to support multiple options for passing target_entity (with different Types)
    #         "target_entity": [
    #             EntityPathAndRequired,
    #             EntityPath,
    #         ],  # Use EntityPath (str)，and EntityPathAndRequired is a wrapper of EntityPath (str) with required skills, or just EntityPath (str) without skill checks
    #         # e.g. robot1.skl_space_move2entity(target_entity={entity="/room1/book1",required="cap_space_getpos"}, distance=0.5)
    #         # e.g. robot1.skl_space_move2entity(target_entity="/room1/book1", distance=0.5)
    #         "distance": float,
    #     },
    #     "output": {"success": bool},
    #     "dependencies": ["cap_space_move", "cap_space_getpos"],
    # },
    "cap_camera_rgb": {
        "description": "Get the RGB image from the specified camera",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"camera_name": str, "timeout_sec": float},
        "output": Any,  # opencv image (numpy array)
    },
    "cap_camera_dep_rgb": {
        "description": "Get the RGB and depth images from the specified camera",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"camera_name": str, "timeout_sec": float},
        "output": Tuple[Any, Any],  # tuple with rgb and depth images
    },
    "cap_camera_info": {
        "description": "Get the camera info of the specified camera",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"camera_name": str, "timeout_sec": float},
        "output": Dict[str, Any],
    },
    "cap_save_rgb_image": {
        "description": "Capture and save RGB image to file",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"filename": str, "camera_name": str, "width": int, "height": int},
        "output": {"success": bool},
    },
    "cap_save_depth_image": {
        "description": "Capture and save depth image to file",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"filename": str, "camera_name": str, "width": int, "height": int},
        "output": {"success": bool},
    },
    "cap_get_robot_pose": {
        # TODO: rename this
        "description": "Get the current pose of the robot",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"timeout_sec": float},
        "output": Optional[Tuple[float, float, float, float]],  # (x,y,z,yaw) tuple
    },
    "cap_set_goal": {
        "description": "Set the goal of the robot",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"x": float, "y": float, "yaw": float},
        "output": str,
    },
    "cap_stop_goal": {
        "description": "Stop the goal of the robot",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": str,
    },
    "cap_get_object_global_pos": {
        "description": "Calculate the global position of an object based on robot pose, pixel coordinates, depth and camera parameters",
        "type": EOS_SkillType.CAPABILITY,
        "input": {
            "pixel_x": float,
            "pixel_y": float,
            "depth": float,
            "camera_info": Dict[str, Any],
            "robot_pose": {"x": float, "y": float, "z": float, "yaw": float},
        },
        "output": Optional[
            Tuple[float, float, float]
        ],  # (global_x, global_y, global_z) tuple
    },
    "cap_get_pose": {
        "description": "Get the current pose of the robot",
        "type": EOS_SkillType.CAPABILITY,
        "input": [
            None,
            {"timeout_sec": float},
        ],  # no arg or timeout_sec(float), the list is for multiple alternative types - wheatfox
        "output": Optional[Tuple[float, float, float]],  # (x,y,yaw) tuple
    },
    "cap_tf_transform": {
        "description": "Transform the coordinates from the source frame to the target frame",
        "type": EOS_SkillType.CAPABILITY,
        "input": {
            "source_frame": str,
            "target_frame": str,
            "x": float,
            "y": float,
            "z": float,
        },
        "output": Tuple[float, float, float],  # (x,y,z) tuple
    },
    ####################
    ###### SKILLS ######
    ####################
    "skl_spatiallm_detect": {
        "description": "Detect walls, doors, objects with bounding boxes using spatiallm",
        "type": EOS_SkillType.SKILL,
        "input": {"ply_model": bytes},
        "output": {
            "txt": str,  # .txt file that marked each object with its bounding box and label
            "rrd": bytes,  # .rrd file used for visualization
        },
    },
    "skl_debug_test_skill": {
        "description": "Test skill",
        "type": EOS_SkillType.SKILL,
        "input": {"input_val": int},
        "output": float,
    },
    "skl_detect_objs": {
        "description": "Detect objects in the current view of the specified camera",
        "type": EOS_SkillType.SKILL,
        "input": {"camera_name": str},
        "output": Dict[str, Tuple[float, float, float]],
        "dependencies": [
            "cap_camera_dep_rgb",
            "cap_camera_info",
            "cap_get_object_global_pos",
            "cap_get_robot_pose",
        ],
    },
    "skl_move_to_goal": {
        "description": "Move the robot to the goal",
        "type": EOS_SkillType.SKILL,
        "input": {"goal_name": str},
        "output": str,
        "dependencies": ["cap_set_goal"],
    },
    "skl_move_to_ab_pos": {
        "description": "Move the robot to the absolute position",
        "type": EOS_SkillType.SKILL,
        "input": {"x": float, "y": float, "yaw": float},
        "output": str,
        "dependencies": ["cap_set_goal"],
    },
    "skl_move_to_rel_pos": {
        "description": "Move the robot to the relative position",
        "type": EOS_SkillType.SKILL,
        "input": {"dx": float, "dy": float, "dyaw": float},
        "output": str,
        "dependencies": ["cap_set_goal", "cap_get_pos"],
    },
    "skl_update_map": {
        "description": "Update the semantic map",
        "type": EOS_SkillType.SKILL,
        "input": {"camera_name": str},
        "output": bool,
        "dependencies": ["skl_detect_objs"],
    },
    "skl_spatiallm_to_world_pose": {
        "description": "Convert SpatialLM detection results from robot coordinates to world coordinates",
        "type": EOS_SkillType.SKILL,
        "input": {"spatiallm_txt": str},
        "output": EOS_TYPE_SpatialLM_WorldResult,
        "dependencies": ["cap_get_pose"],
    },
}
