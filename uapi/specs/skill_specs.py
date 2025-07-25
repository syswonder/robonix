# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

from enum import Enum
from typing import List

EOS_TYPE_None = None


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


EOS_SPEC_PosXYZ = {"x": float, "y": float, "z": float}

# TODO: add recursive type check in entity
EOS_SPEC_Image = {
    "image_raw": bytes,
    "metadata": {
        "width": int,
        "height": int,
        "format": EOS_TYPE_ImageFormat,
        "camera_type": EOS_TYPE_CameraType,
    },
}

EntityPath = str  # Represents the path of an entity.

EntityPathAndRequired = {
    "entity": EntityPath,
    "required": List[str],
}

# documentation:
# for input and output fields, the argument and return values are standardized to dict
# the key is the NAME of the argument or return value, and the value is the TYPE of the argument or return value
# the TYPE here is the actually python type symbol, like str, int, float (which is actually a CLASS) so:
# { <str>: <class>/<Dict> } where Dict support complex data types like Image
# for fields other than input and output, the above rules are not applied! - wheatfox 2025.7.25

# also noted that EOS_SPEC is <Dict>, EOS_TYPE is <class>

EOS_SKILL_SPECS = {
    # naming rules: [c/s]_<category>_<name>
    # c: capability, s: skill
    # dependencies: list of skill that THIS entity should bind before using this skill
    "c_space_getpos": {
        "description": "Get the position of the entity",
        "type": EOS_SkillType.CAPABILITY,
        "input": EOS_TYPE_None,
        "output": EOS_SPEC_PosXYZ,
        "dependencies": [],
    },
    "c_space_move": {
        "description": "Move the entity to the given position",
        "type": EOS_SkillType.CAPABILITY,
        "input": EOS_SPEC_PosXYZ,
        "output": {"success": bool},
        "dependencies": [],
    },
    "c_image_caputre": {
        # cap_camera_rgb, cap_camera_depth, cap_camera_ir
        "description": "Capture an image, should be implemented on camera or something",
        "type": EOS_SkillType.CAPABILITY,
        "input": EOS_TYPE_None,
        "output": EOS_SPEC_Image,
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
}
