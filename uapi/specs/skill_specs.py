# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

from enum import Enum

EOS_None = None


class EOS_ImageFormat(Enum):
    JPEG = "jpeg"
    PNG = "png"
    BMP = "bmp"
    TIFF = "tiff"
    WEBP = "webp"


class EOS_CameraType(Enum):
    RGB = "rgb"
    DEPTH = "depth"
    INFRARED = "infrared"


class EOS_SkillType(Enum):
    CAPABILITY = "capability"
    SKILL = "skill"


EOS_PosXYZ = {"x": float, "y": float, "z": float}

# TODO: add recursive type check in entity
EOS_Image = {
    "image_raw": bytes,
    "metadata": {
        "width": int,
        "height": int,
        "format": EOS_ImageFormat,
        "camera_type": EOS_CameraType,
    },
}

EntityPath = str  # Represents the path of an entity.

EOS_SKILL_SPECS = {
    # naming rules: [c/s]_<category>_<name>
    # c: capability, s: skill
    # dependencies: list of skill that THIS entity should bind before using this skill
    "c_space_getpos": {
        "description": "Get the position of the entity",
        "type": EOS_SkillType.CAPABILITY,
        "input": EOS_None,
        "output": EOS_PosXYZ,
        "dependencies": [],
    },
    "c_space_move": {
        "description": "Move the entity to the given position",
        "type": EOS_SkillType.CAPABILITY,
        "input": EOS_PosXYZ,
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_image_caputre": {
        # cap_camera_rgb, cap_camera_depth, cap_camera_ir
        "description": "Capture an image, should be implemented on camera or something",
        "type": EOS_SkillType.CAPABILITY,
        "input": EOS_None,
        "output": EOS_Image,
        "dependencies": [],
    },
    "s_space_move2entity": {
        "description": "Move the entity to vicinity of another entity",
        "type": EOS_SkillType.SKILL,
        "input": {
            "target_entity": EntityPath,  # Use EntityPath (str)
            "distance": float,
        },
        "output": {"success": bool},
        "dependencies": ["c_space_move", "c_space_getpos"],
    },
}
