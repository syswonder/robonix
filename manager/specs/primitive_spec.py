# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

from enum import Enum

EAIOS_PRIMITIVE_NO_ARGS = None


class EAIOS_IMAGE_FORMAT(Enum):
    JPEG = "jpeg"
    PNG = "png"
    BMP = "bmp"
    TIFF = "tiff"
    WEBP = "webp"


EAIOS_XYZ = {"x": float, "y": float, "z": float}

# TODO: add recursive type check in entity
EAIOS_IMAGE = {
    "image_raw": bytes,
    "metadata": {
        "width": int,
        "height": int,
        "format": EAIOS_IMAGE_FORMAT,
    },
}

PRIMITIVE_SPECS = {
    "getpos": {
        "args": EAIOS_PRIMITIVE_NO_ARGS,
        "returns": EAIOS_XYZ,
    },
    "move": {
        "args": EAIOS_XYZ,
        "returns": {"success": bool},
    },
    "capture": {
        "args": EAIOS_PRIMITIVE_NO_ARGS,
        "returns": EAIOS_IMAGE,
    },
}
