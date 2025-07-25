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

EOS_SKILL_SPECS = {
    "getpos": {
        "args": EOS_None,
        "returns": EOS_PosXYZ,
    },
    "move": {
        "args": EOS_PosXYZ,
        "returns": {"success": bool},
    },
    "capture": {
        "args": EOS_None,
        "returns": EOS_Image,
    },
}
