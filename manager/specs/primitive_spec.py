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


PRIMITIVE_SPECS = {
    "getpos": {
        "args": EAIOS_PRIMITIVE_NO_ARGS,
        "returns": {"x": float, "y": float, "z": float},
    },
    "move": {
        "args": {"x": float, "y": float, "z": float},
        "returns": {"success": bool},
    },
    "capture": {
        "args": EAIOS_PRIMITIVE_NO_ARGS,
        "returns": {
            "image_raw": bytes,
            "metadata": {
                "width": int,
                "height": int,
                "format": EAIOS_IMAGE_FORMAT,
            },
        },
    },
}
