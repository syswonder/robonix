# SPDX-License-Identifier: MulanPSL-2.0
"""Robonix provider for the existing Nova5 RGB capture script."""
import asyncio
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading

import camera_pb2
import camera_mcp
import grpc
import sensor_msgs_pb2
import sensor_msgs_mcp
import std_msgs_mcp
import builtin_interfaces_mcp
from robonix_api import Err, Ok, Primitive


camera = Primitive(id="nova5_wrist_rgb", namespace="robonix/primitive/camera")
_config = None
_capture = Path(__file__).resolve().parents[3] / "capture_rgb.py"
_lock = threading.Lock()


@camera.on_init
def initialize(config):
    global _config
    calibration_root = Path(config.get("calibration_root", ""))
    camera_python = Path(config.get("camera_python", ""))
    if not (calibration_root / "config.json").is_file() or not camera_python.is_file():
        return Err("Nova5 RGB calibration or Python interpreter is missing")
    _config = (calibration_root, camera_python)
    return Ok()


def capture():
    if _config is None:
        raise RuntimeError("Camera is not initialized")
    calibration_root, camera_python = _config
    with _lock, tempfile.TemporaryDirectory(prefix="nova5-rgb-") as temporary:
        output = Path(temporary)
        env = os.environ.copy()
        env.pop("PYTHONPATH", None)
        env.pop("PYTHONHOME", None)
        result = subprocess.run([str(camera_python), str(_capture), str(calibration_root),
                                 str(output)], capture_output=True, text=True, timeout=25,
                                env=env)
        if result.returncode:
            raise RuntimeError(f"RGB capture failed: {(result.stderr or result.stdout)[-1600:]}")
        return json.loads((output / "capture.json").read_text()), {
            "raw": (output / "rgb.bgr").read_bytes(),
            "jpeg": (output / "rgb.jpg").read_bytes(),
        }


@camera.grpc("robonix/primitive/camera/snapshot")
def snapshot(request, context):
    try:
        metadata, pixels = capture()
    except (RuntimeError, subprocess.SubprocessError, OSError, ValueError) as error:
        context.abort(grpc.StatusCode.UNAVAILABLE, str(error))
    image = sensor_msgs_pb2.Image(
        height=metadata["height"], width=metadata["width"], encoding="bgr8",
        step=metadata["width"] * 3, data=pixels["raw"])
    image.header.frame_id = "nova5_wrist_camera_optical"
    image.header.stamp.sec = metadata["stamp_sec"]
    image.header.stamp.nanosec = metadata["stamp_nanosec"]
    return camera_pb2.GetCameraImage_Response(image=image)


@camera.mcp("robonix/primitive/camera/snapshot")
async def snapshot_for_pilot(request: camera_mcp.GetCameraImage_Request) -> sensor_msgs_mcp.Image:
    """Give Pilot one stationary RGB image of the current Nova5 work surface."""
    metadata, pixels = await asyncio.to_thread(capture)
    # MCP carries a compressed frame for the model; gRPC keeps raw bgr8.
    jpeg = pixels["jpeg"]
    return sensor_msgs_mcp.Image(
        header=std_msgs_mcp.Header(
            stamp=builtin_interfaces_mcp.Time(sec=metadata["stamp_sec"],
                                             nanosec=metadata["stamp_nanosec"]),
            frame_id="nova5_wrist_camera_optical"),
        height=metadata["height"], width=metadata["width"], encoding="jpeg",
        step=len(jpeg), data=jpeg)


if __name__ == "__main__":
    camera.run()
