# SPDX-License-Identifier: MulanPSL-2.0
"""Acquire RGB paired with stationary Nova5 feedback; never submit motion."""
import json
from pathlib import Path
import sys


def main():
    calibration_root = Path(sys.argv[1]).resolve()
    output = Path(sys.argv[2]).resolve()
    sys.path.insert(0, str(calibration_root))
    from handeye.camera import Camera
    from handeye.cli import _robot, _stationary_snapshot, _save_image
    from handeye.workflow import (load_config, load_intrinsics, load_handeye,
                                  load_plane, check_camera_identity, save_json)

    config = load_config(calibration_root / "config.json")
    intrinsic = load_intrinsics(config)
    load_handeye(config)
    load_plane(config)
    with Camera(config["camera"]) as camera, _robot(config, feedback_only=True) as robot:
        check_camera_identity(camera, intrinsic)
        frame, sample = _stationary_snapshot(camera, robot)
        _save_image(output / "rgb.png", frame)
        import cv2
        if not cv2.imwrite(str(output / "rgb.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 90]):
            raise RuntimeError("Failed to encode Nova5 RGB JPEG")
        (output / "rgb.bgr").write_bytes(frame.tobytes())
        stamp = sample["host_frame_received_at"]
        metadata = {"sample": sample, "status": robot.status(), "motion_sent": False,
                    "width": frame.shape[1], "height": frame.shape[0],
                    "stamp_sec": int(stamp), "stamp_nanosec": int((stamp % 1) * 1e9)}
        save_json(output / "capture.json", metadata)
        print(json.dumps({"image": str(output / "rgb.png"), "motion_sent": False}))


if __name__ == "__main__":
    main()
