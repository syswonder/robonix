# primitive_spec.py
# This file defines the standard primitive specifications for Entity.
# Each primitive has its argument list and return value specification.

PRIMITIVE_SPECS = {
    "getpos": {
        "args": [],  # No arguments
        "returns": {"x": float, "y": float, "z": float},  # Returns a dict with x, y, z coordinates
    },
    "move": {
        "args": {"x": float, "y": float, "z": float},  # Requires x, y, z as arguments
        "returns": {"success": bool},  # Returns a dict indicating success
    },
} 