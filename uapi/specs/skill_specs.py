# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

from enum import Enum
from typing import List, Tuple, Dict, Any
from dataclasses import dataclass
import datetime

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
#     "dependencies": <List[str]>, // List of skill names, use [] for no dependencies
# }


class EOS_TYPE_ImageFormat(Enum):
    JPEG = "jpeg"
    PNG = "png"
    BMP = "bmp"
    TIFF = "tiff"
    WEBP = "webp"


class EOS_TYPE_CameraType(Enum):
    RGB = "rgb"
    DEPTH = "depth"
    INFRARED = "infrared"


class EOS_SkillType(Enum):
    CAPABILITY = "capability"
    SKILL = "skill"


class EOS_TYPE_Datetime:
    timestamp: datetime.datetime


# TODO: add recursive type check in entity
@dataclass
class EOS_TYPE_ImageMetadata:
    width: int
    height: int
    format: EOS_TYPE_ImageFormat
    camera_type: EOS_TYPE_CameraType

    def __str__(self):
        from rich.pretty import pretty_repr

        return pretty_repr(self)


@dataclass
class EOS_TYPE_Image:
    image_raw: bytes
    metadata: EOS_TYPE_ImageMetadata

    def __str__(self):
        from rich.pretty import pretty_repr

        meta_str = pretty_repr(self.metadata)
        if self.image_raw is not None:
            preview = self.image_raw[:8]
            preview_str = f"bytes[{len(self.image_raw)}]: {preview!r}..."
        else:
            preview_str = "None"
        return f"EOS_Image(\n  image_raw={preview_str},\n  metadata={meta_str}\n)"


# --- Additional Data Types for Capabilities ---


@dataclass
class EOS_TYPE_AudioBuffer:
    data: bytes
    sample_rate: int
    channels: int


@dataclass
class EOS_TYPE_DepthFrame:
    data: bytes
    width: int
    height: int
    format: str  # e.g. 'raw', 'png', etc.


@dataclass
class EOS_TYPE_ImagePair:
    rgb_image: "EOS_TYPE_Image"
    depth_image: "EOS_TYPE_DepthFrame"


@dataclass
class EOS_TYPE_BBox:
    x: int
    y: int
    w: int
    h: int


@dataclass
class EOS_TYPE_IMUData:
    accel: Tuple[float, float, float]
    gyro: Tuple[float, float, float]
    mag: Tuple[float, float, float]


@dataclass
class EOS_TYPE_LidarScan:
    ranges: List[float]
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float


@dataclass
class EOS_TYPE_Pose2D:
    x: float
    y: float
    theta: float


@dataclass
class EOS_TYPE_Pose6D:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass
class EOS_TYPE_DetectedObject:
    label: str
    bbox: EOS_TYPE_BBox
    confidence: float


@dataclass
class EOS_TYPE_MaskMap:
    mask: EOS_TYPE_Image


@dataclass
class EOS_TYPE_GridMap:
    data: List[List[int]]
    resolution: float
    origin: EOS_TYPE_Pose2D


@dataclass
class EOS_TYPE_Transform:
    translation: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]  # Quaternion


@dataclass
class EOS_TYPE_Path:
    poses: List[EOS_TYPE_Pose2D]


@dataclass
class EOS_TYPE_Feature:
    name: str
    position: EOS_TYPE_Pose2D


@dataclass
class EOS_TYPE_FeatureList:
    features: List[EOS_TYPE_Feature]


EntityPath = str  # Represents the path of an entity.

EntityPathAndRequired = {
    "entity": EntityPath,
    "required": List[str],
}

EOS_SKILL_SPECS = {
    # naming rules: [c/s]_<name>
    # c: capability, s: skill
    # dependencies: list of skill that THIS entity should bind before using this skill
    "c_space_getpos": {
        "description": "Get the position of the entity",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": {"x": float, "y": float, "z": float},
        "dependencies": [],
    },
    "c_space_move": {
        "description": "Move the entity to the given position",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"x": float, "y": float, "z": float},
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_image_capture": {
        # cap_camera_rgb, cap_camera_depth, cap_camera_ir
        "description": "Capture an image, should be implemented on camera or something",
        "type": EOS_SkillType.CAPABILITY,
        "input": None,
        "output": EOS_TYPE_Image,  # Use the dataclass
        "dependencies": [],
    },
    "s_space_move2entity": {
        "description": "Move the entity to vicinity of another entity",
        "type": EOS_SkillType.SKILL,
        "input": {
            # TODO: use list to support multiple options for passing target_entity (with different Types)
            "target_entity": [
                EntityPathAndRequired,
                EntityPath,
            ],  # Use EntityPath (str)，and EntityPathAndRequired is a wrapper of EntityPath (str) with required skills, or just EntityPath (str) without skill checks
            # e.g. robot1.s_space_move2entity(target_entity={entity="/room1/book1",required="c_space_getpos"}, distance=0.5)
            # e.g. robot1.s_space_move2entity(target_entity="/room1/book1", distance=0.5)
            "distance": float,
        },
        "output": {"success": bool},
        "dependencies": ["c_space_move", "c_space_getpos"],
    },
    "c_camera_rgb": {
        # according to capability/vision/api/api.py:c_camera_rgb
        "description": "Get the RGB image from the specified camera",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"camera_name": str},
        "output": Any,  # opencv image
    },
    "c_camera_dep_rgb": {
        # according to capability/vision/api/api.py:c_camera_dep_rgb
        "description": "Get the RGB image from the specified camera",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"camera_name": str},
        "output": (
            Any,  # opencv image
            Any,  # opencv image
        ),  # use Any to skip type check, but this is just a workaround for development, don't use it in production!!! - wheatfox
        "dependencies": [],
    },
    "c_camera_info": {
        # according to capability/vision/api/api.py:c_camera_info
        "description": "Get the camera info of the specified camera",
        "type": EOS_SkillType.CAPABILITY,
        "input": {"camera_name": str},
        "output": Dict[str, Any],
        "dependencies": [],
    },
    "c_tf_transform": {
        # according to capability/vision/api/api.py:c_tf_transform
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
        "dependencies": [],
    },
    "s_detect_objs": {
        # according to skill/vision/api/api.py:s_detect_objs
        "description": "Detect objects in the current view of the specified camera",
        "type": EOS_SkillType.SKILL,
        "input": {"camera_name": str},
        "output": Dict[str, Tuple[float, float, float]],
        "dependencies": ["c_camera_dep_rgb", "c_camera_info", "c_tf_transform"],
    },
}
