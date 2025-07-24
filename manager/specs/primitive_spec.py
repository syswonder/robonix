# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

from enum import Enum

EAIOS_TYPE_NONE = None


class EAIOS_IMAGE_FORMAT(Enum):
    JPEG = "jpeg"
    PNG = "png"
    BMP = "bmp"
    TIFF = "tiff"
    WEBP = "webp"


# types are defined as python dicts, and used when calling entity's binded primitives

EAIOS_TYPE_XYZ = {"x": float, "y": float, "z": float}

# TODO: add recursive type check in entity
EAIOS_TYPE_IMAGE = {
    "image_raw": bytes,
    "metadata": {
        "width": int,
        "height": int,
        "format": EAIOS_IMAGE_FORMAT,
    },
}

PRIMITIVE_SPECS = {
    "getpos": {
        "args": EAIOS_TYPE_NONE,
        "returns": EAIOS_TYPE_XYZ,
    },
    "move": {
        "args": EAIOS_TYPE_XYZ,
        "returns": {"success": bool},
    },
    "capture": {
        "args": EAIOS_TYPE_NONE,
        "returns": EAIOS_TYPE_IMAGE,
    },
}
