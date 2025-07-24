# primitive_spec.py
# This file defines the standard primitive specifications for Entity.
# Each primitive has its argument list and return value specification.

from enum import Enum

NO_ARGS = None


class EAIOS_IMAGE_FORMAT(Enum):
    JPEG = "jpeg"
    PNG = "png"
    BMP = "bmp"
    TIFF = "tiff"
    WEBP = "webp"


PRIMITIVE_SPECS = {
    "getpos": {
        "args": NO_ARGS,
        "returns": {"x": float, "y": float, "z": float},
    },
    "move": {
        "args": {"x": float, "y": float, "z": float},
        "returns": {"success": bool},
    },
    "capture": {
        "args": NO_ARGS,
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
