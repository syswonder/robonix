# SPDX-License-Identifier: MulanPSL-2.0
"""One RGB-guided Nova5 pick, return-to-board and return-home cycle."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile


HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent


def path(value):
    # Keep venv/bin/python as the executable path: resolving its symlink
    # selects the base interpreter and loses the venv's installed packages.
    return Path(os.path.abspath(REPO / value))


def command(args, log, *, env=None, input_text=None, timeout=120):
    result = subprocess.run(args, cwd=REPO, env=env, input=input_text,
                            capture_output=True, text=True, timeout=timeout,
                            stdin=subprocess.DEVNULL if input_text is None else None)
    log.write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(f"{args[0]} failed; inspect {log}: {(result.stdout + result.stderr)[-1200:]}")
    return result.stdout


def alignment_targets(start_tcp, target_rotation):
    import numpy as np
    from scipy.spatial.transform import Rotation, Slerp

    start, target = start_tcp[:3, :3], target_rotation
    total = Rotation.from_matrix(start.T @ target).magnitude()
    # With the required 165 mm TCP, 30 degrees moves the flange at most
    # 85.4 mm, leaving margin below the worker's 100 mm / 70 degree bounds.
    segments = max(1, math.ceil(math.degrees(total) / 30))
    interpolation = Slerp([0, 1], Rotation.from_matrix([start, target]))
    targets = np.repeat(start_tcp[None, :, :], segments, axis=0)
    targets[:, :3, :3] = interpolation(
        [step / segments for step in range(1, segments + 1)]).as_matrix()
    return targets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--settings", type=Path, default=HERE / "settings.json")
    args = parser.parse_args()
    settings = json.loads(args.settings.read_text(encoding="utf-8"))
    files = {name: path(settings[name]) for name in (
        "calibration_root", "camera_python", "runtime_python", "hand_python",
        "generated_proto", "pgi_module", "atlas_binary")}
    if any(not item.exists() for item in files.values()):
        raise ValueError("A configured runtime or calibration path does not exist")
    if not Path(settings["serial_port"]).exists():
        raise ValueError("Configured PGI serial port does not exist")
    from model_api import decide as model_decide
    import numpy as np
    import cv2
    from scipy.spatial.transform import Rotation
    sys.path.insert(0, str(files["calibration_root"]))
    from handeye.geometry import (intersect_pixel_plane, pose_to_transform,
                                  target_flange, transform_to_pose)
    from handeye.test_motion import table_z
    from handeye.workflow import (load_config, load_handeye, load_intrinsics,
                                  load_plane, tcp_transform)

    config = load_config(files["calibration_root"] / "config.json")
    intrinsics, handeye, plane = load_intrinsics(config), load_handeye(config), load_plane(config)
    tcp = tcp_transform(config)
    if not np.allclose(tcp[:3, 3], [0, 0, 165], atol=.01):
        raise ValueError("This cycle requires the measured 165 mm flange-to-tip TCP")
    height = float(settings["object_height_mm"])
    if not 40 <= height <= 60:
        raise ValueError("Measured target height must be 40..60 mm for this grasp")
    home = [float(v) for v in settings["home_flange_mm_deg"]]
    if len(home) != 6 or not all(math.isfinite(v) for v in home):
        raise ValueError("Invalid recorded home flange pose")
    artifacts = HERE / ".artifacts/nova5_rgb"
    artifacts.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix="pipeline-", dir=artifacts))
    trace = []

    common = os.environ.copy()
    common["PYTHONPATH"] = os.pathsep.join((str(REPO / "pylib/robonix-api"),
                                            str(files["generated_proto"])))
    hand_env = common.copy()
    hand_env["PYTHONPATH"] += os.pathsep + str(files["pgi_module"])

    def record(stage, data):
        trace.append({"stage": stage, "data": data})
        (run / "trace.json").write_text(json.dumps(trace, indent=2, ensure_ascii=False) + "\n")
        print(stage, json.dumps(data, ensure_ascii=False), flush=True)

    def hand_status():
        output = command([str(files["hand_python"]), str(HERE / "read_hand.py"),
                          "--port", settings["serial_port"]], run / "hand-status.log",
                         env=hand_env, timeout=10)
        return json.loads(output)

    def hand_move(target):
        before = hand_status()
        if before["position_raw"] == target:
            return before
        output = command([str(files["hand_python"]), str(HERE / "hand_step.py"),
                          "--port", settings["serial_port"], "--target-raw", str(target),
                          "--expected-start-raw", str(before["position_raw"]),
                          "--atlas-binary", str(files["atlas_binary"])],
                         run / f"hand-{len(trace):02d}.log", env=hand_env, timeout=115)
        result = json.loads(output)
        record("hand", {"target_raw": target, "actual": result["final_state"],
                        "evidence": result["evidence"]})
        return result["final_state"]

    def observe():
        output = command([str(files["runtime_python"]), str(HERE / "observe.py"),
                          "--calibration-root", str(files["calibration_root"]),
                          "--camera-python", str(files["camera_python"]),
                          "--atlas-binary", str(files["atlas_binary"])],
                         run / f"observe-{len(trace):02d}.log", env=common, timeout=45)
        evidence = json.loads(output)
        sample = json.loads(Path(evidence["sample"]).read_text(encoding="utf-8"))
        pose = transform_to_pose(sample["sample"]["T_base_flange"])
        record("observe", {"image": evidence["image"], "flange": pose})
        return Path(evidence["image"]), pose, sample

    def decide(stage, images, schema, prompt):
        decision = model_decide(images, HERE / schema,
            "You are making a visual decision for one real robot grasp. Use only the attached "
            "RGB images and the facts in this prompt. Return JSON matching the supplied schema. "
            "If the object is missing, a finger may collide, or evidence is insufficient, "
            "choose abort.\n\n" + prompt, run / f"{stage}.json")
        record(stage, decision)
        if decision["decision"] != "proceed":
            raise RuntimeError(f"Model API stopped at {stage}: {decision['reason']}")
        return decision

    def locate(image, pose, stage):
        decision = decide(stage, [image], "vision.schema.json",
            f"Image size is 1280x720. Target: {settings['object_description']}. "
            f"Its previously measured height is {height:.0f} mm. Identify the four visible "
            "corners of its BLACK TOP FACE, in any order. Ignore the black checker squares "
            "and gripper. Proceed only if a single target top face is clear.")
        pixels = np.asarray(decision["top_corners_px"], dtype=float)
        if pixels.shape != (4, 2) or not np.all(np.isfinite(pixels)):
            raise ValueError("Model did not return four finite top corners")
        if np.any(pixels < 0) or np.any(pixels[:, 0] >= 1280) or np.any(pixels[:, 1] >= 720):
            raise ValueError("Model top corners outside the calibrated image")
        center = pixels.mean(axis=0)
        angles = np.arctan2(pixels[:, 1] - center[1], pixels[:, 0] - center[0])
        pixels = pixels[np.argsort(angles)]
        if abs(cv2.contourArea(pixels.astype(np.float32))) < 150:
            raise ValueError("Target polygon is too small for a grasp")
        camera = pose_to_transform(pose) @ np.asarray(handeye["T_flange_camera"])
        points = np.asarray([intersect_pixel_plane(pixel, intrinsics["K"], intrinsics["dist"],
                            camera, plane["normal"], plane["offset_mm"] - height)
                            for pixel in pixels])
        xy = points[:, :2].mean(axis=0)
        limits = np.asarray(settings["allowed_xy_mm"], dtype=float)
        if limits.shape != (2, 2) or np.any(xy < limits[:, 0]) or np.any(xy > limits[:, 1]):
            raise ValueError(f"Target {xy.tolist()} outside configured work area")
        edges = np.roll(points[:, :2], -1, axis=0) - points[:, :2]
        widths = np.linalg.norm(edges, axis=1)
        if widths.min() < 20 or widths.max() > 90:
            raise ValueError("Detected top dimensions outside the box envelope")
        # An unevenly drawn corner can make one long edge appear shortest.
        # Opposite edges together distinguish the two box axes more reliably.
        opposite_pairs = [(0, 2), (1, 3)]
        first, second = min(opposite_pairs,
                            key=lambda pair: float(np.mean(widths[list(pair)])))
        edge = (edges[first] - edges[second]) / 2
        yaw = math.degrees(math.atan2(edge[0], -edge[1]))
        yaw = min((yaw, yaw + 180, yaw - 180), key=lambda value: abs((value - 146 + 180) % 360 - 180))
        record("target", {"xy_mm": xy.tolist(), "yaw_deg": yaw,
                          "top_edges_mm": widths.tolist(), "source": stage})
        return xy, yaw

    def tcp_target(start, xy, clearance):
        target = pose_to_transform(start) @ tcp
        target[:3, 3] = [float(xy[0]), float(xy[1]), table_z(plane, *xy) + clearance]
        return transform_to_pose(target_flange(target, tcp))

    def move(purpose, start, target):
        folder = run / f"move-{len(trace):02d}"
        folder.mkdir()
        plan = folder / "plan.json"
        plan.write_text(json.dumps({"purpose": purpose, "start_flange_mm_deg": start,
                                    "target_flange_mm_deg": target, "speed_mm_s": 50}, indent=2))
        invocation = [str(files["runtime_python"]), str(HERE / "arm_step.py"),
                      "--plan", str(plan), "--calibration-root", str(files["calibration_root"]),
                      "--sdk-python", str(files["camera_python"]),
                      "--atlas-binary", str(files["atlas_binary"])]
        log = folder / "command.log"
        command(invocation, log, env=common, timeout=55)
        arrival = json.loads((folder / "arrival.json").read_text(encoding="utf-8"))
        record("move", {"purpose": purpose, "arrival": arrival, "plan": str(plan)})
        return arrival["actual_flange_mm_deg"]

    def align(start, purpose, *, yaw=None, orientation=None):
        start_tcp = pose_to_transform(start) @ tcp
        if orientation is None:
            orientation = Rotation.from_euler("xyz", [180, 0, yaw], degrees=True).as_matrix()
        targets = alignment_targets(start_tcp, orientation)
        pose = start
        for step, target in enumerate(targets):
            stage = purpose if step == len(targets) - 1 else "Wrist transition"
            pose = move(stage, pose, transform_to_pose(target_flange(target, tcp)))
        return pose

    if not all(os.environ.get(name) for name in ("VLM_BASE_URL", "VLM_API_KEY")):
        raise ValueError("Set VLM_BASE_URL and VLM_API_KEY before running")
    status = hand_status()
    if (status["initialization_raw"] != 1 or status["grip_status_raw"] != 1
            or status["io_mode_raw"] != 0 or status["configuration"] != {
                "force_setpoint_percent": 20,
                "target_position_raw": status["position_raw"],
                "speed_setpoint_percent": 1}):
        raise RuntimeError(f"PGI must be initialized, idle and empty before starting: {status}")
    if status["position_raw"] != 1000:
        hand_move(1000)
    initial_image, pose, sample = observe()
    if not (sample["status"]["enabled"] and sample["status"]["mode"] == 5):
        raise RuntimeError("Nova5 must be enabled and in idle TCP mode")
    original_xy, yaw = locate(initial_image, pose, "initial-target")
    current_tcp = pose_to_transform(pose) @ tcp
    if current_tcp[2, 3] - table_z(plane, *current_tcp[:2, 3]) < 199:
        raise RuntimeError("Start pose needs at least 200 mm table clearance")
    pose = move("Object hover observation", pose, tcp_target(pose, original_xy, 200))
    pose = align(pose, "Grasp alignment", yaw=yaw)
    alignment_image, pose, _ = observe()
    corrected_xy, corrected_yaw = locate(alignment_image, pose, "aligned-target")
    if np.linalg.norm(corrected_xy - original_xy) > 15:
        raise RuntimeError("Second RGB view disagrees with the initial target")
    if abs((corrected_yaw - yaw + 180) % 360 - 180) > 15:
        first_edges = next(row["data"]["top_edges_mm"] for row in trace
                           if row["stage"] == "target")
        if max(first_edges) / min(first_edges) > 1.15:
            raise RuntimeError("Two clear RGB views disagree on box orientation")
        pose = align(pose, "Grasp alignment", yaw=corrected_yaw)
        alignment_image, pose, _ = observe()
        verified_xy, verified_yaw = locate(alignment_image, pose, "verified-target")
        if (np.linalg.norm(verified_xy - corrected_xy) > 15
                or abs((verified_yaw - corrected_yaw + 180) % 360 - 180) > 15):
            raise RuntimeError("Third RGB view disagrees after resolving ambiguous box edges")
        corrected_xy, corrected_yaw = verified_xy, verified_yaw
    xy = corrected_xy
    pose = move("Grasp centering", pose, tcp_target(pose, xy, 200))
    pose = move("Pregrasp descent", pose, tcp_target(pose, xy, 75))
    image, pose, _ = observe()
    try:
        decide("pregrasp-check", [alignment_image, image], "check.schema.json",
               "These images show the same open gripper and box from 200 mm and then 75 mm "
               "TCP clearance above the board, with a strictly vertical descent between them. "
               f"The box top is about {height:.0f} mm above the board. Use the wider first "
               "view to assess whether the open fingers straddle the box laterally, and the "
               "second to check for any new obstacle. The open fingers are the large dark "
               "mechanical shapes at the FAR left and FAR right of the image top; the "
               "alternating black/white tiles and diagonal board edge beside the box are "
               "CHECKERBOARD, not fingers. The camera is mounted between the fingers, so "
               "the near view is partly occluded. Proceed only if both views together "
               "support a clear vertical path and side clearance.")
    except RuntimeError:
        move("Grasp clearance retreat", pose, tcp_target(pose, xy, 200))
        raise
    pose = move("Grasp descent", pose, tcp_target(pose, xy, 35))
    before_lift, pose, _ = observe()
    gripped = hand_move(300)
    if gripped["grip_status_raw"] != 2 or hand_status()["grip_status_raw"] != 2:
        raise RuntimeError("No stable object contact; do not lift")
    pose = move("Lift object", pose, tcp_target(pose, xy, 65))
    after_lift, pose, _ = observe()
    if hand_status()["grip_status_raw"] != 2:
        raise RuntimeError("Gripper lost contact during lift")
    # Place before the visual-model call to limit how long the gripper holds the box.
    pose = move("Place object", pose, tcp_target(pose, xy, 35))
    if hand_status()["grip_status_raw"] != 2:
        raise RuntimeError("Gripper lost contact before release")
    released = hand_move(1000)
    if released["grip_status_raw"] != 1 or released["position_raw"] != 1000:
        raise RuntimeError("PGI did not reach its open state after placing")
    pose = move("Postplace retreat", pose, tcp_target(pose, xy, 75))
    pose = move("Grasp clearance retreat", pose, tcp_target(pose, xy, 200))
    decide("lift-check",
           [before_lift, after_lift], "check.schema.json",
           "These images are before then after a 30 mm vertical lift. The target box has a "
           "gray front face with the black Robonix wordmark directly beneath the dark wrist "
           "housing near the top center; this face is the OBJECT, not the gripper. PGI reported "
           "stable contact status 2. Compare this box face against the camera while the "
           "checkerboard moves away. Proceed only if visible evidence confirms the box rose "
           "with the wrist; otherwise abort.")
    final_image, pose, _ = observe()
    decide("place-check", [initial_image, final_image], "check.schema.json",
           "The first image shows the initial box position. The second image is after "
           "returning the box to that board position "
           "and opening the gripper. Is the box visibly resting on the board near the "
           "initial location? Abort if missing or obviously displaced.")
    home_orientation = pose_to_transform(home) @ tcp
    pose = align(pose, "Return alignment", orientation=home_orientation[:3, :3])
    pose = move("Return home", pose, home)
    if hand_status()["grip_status_raw"] != 1:
        raise RuntimeError("Final gripper status is not empty")
    record("complete", {"home_flange_mm_deg": pose, "model": "gpt-6-astra",
                        "reasoning_effort": os.environ.get("VLM_REASONING_EFFORT", "high"), "speed_mm_s": 50,
                        "trace": str(run / "trace.json")})


if __name__ == "__main__":
    main()
