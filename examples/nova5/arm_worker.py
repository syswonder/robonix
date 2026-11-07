# SPDX-License-Identifier: MulanPSL-2.0
"""Execute one reviewed flange target using the existing Dobot SDK adapter."""
import json
from pathlib import Path
import signal
import sys
import threading


def main():
    root, plan_path = Path(sys.argv[1]), Path(sys.argv[2])
    sys.path.insert(0, str(root))
    import numpy as np
    from handeye.geometry import pose_to_transform, transform_to_pose
    from handeye.robot import DobotRobot, _reply, _numbers
    from handeye.workflow import load_config, load_plane, pose_distance, tcp_transform, save_json
    from handeye.test_motion import table_z

    plan = json.loads(plan_path.read_text())
    config = load_config(root / "config.json")
    plane = load_plane(config)
    expected = pose_to_transform(plan["start_flange_mm_deg"])
    target = pose_to_transform(plan["target_flange_mm_deg"])
    distance, angle = pose_distance(expected, target)
    purpose = plan["purpose"]
    max_distance = {"Object hover observation":600,
                    "Grasp alignment":100,"Wrist transition":100,"Grasp centering":15,
                    "Pregrasp descent":140,"Grasp descent":65,"Lift object":50,
                    "Place object":50,"Postplace retreat":50,
                    "Grasp clearance retreat":140,"Return alignment":100,
                    "Return home":600}.get(purpose)
    if max_distance is None:
        raise ValueError("Unsupported motion purpose")
    if distance > max_distance or angle > (70 if purpose in ("Grasp alignment", "Wrist transition", "Return alignment") else .1):
        raise ValueError("Motion exceeds the planned distance or rotation bound")
    if purpose in ("Grasp alignment", "Wrist transition", "Return alignment"):
        start_tcp, target_tcp = expected@tcp_transform(config), target@tcp_transform(config)
        if np.linalg.norm(start_tcp[:3,3]-target_tcp[:3,3]) > .2:
            raise ValueError("Alignment must preserve TCP position")
        if purpose == "Grasp alignment" and not np.allclose(target[:3,2],[0,0,-1],atol=1e-5):
            raise ValueError("Grasp alignment must end vertically downward")
    if purpose in ("Pregrasp descent", "Grasp descent", "Lift object",
                   "Place object", "Postplace retreat", "Grasp clearance retreat"):
        start_tcp, target_tcp = expected@tcp_transform(config), target@tcp_transform(config)
        if (np.linalg.norm(start_tcp[:2,3]-target_tcp[:2,3]) > .2
                or not np.allclose(target[:3,2],[0,0,-1],atol=1e-5)):
            raise ValueError("Grasp moves must be vertical with the gripper downward")
        start_height = start_tcp[2,3]-table_z(plane,*start_tcp[:2,3])
        end_height = target_tcp[2,3]-table_z(plane,*target_tcp[:2,3])
        bounds = {"Pregrasp descent": (199,70,80),
                  "Grasp descent": (70,25,35),"Lift object": (25,50,85),
                  "Place object": (50,25,35),
                  "Postplace retreat": (25,70,80),
                  "Grasp clearance retreat": (70,199,205)}
        min_start, min_end, max_end = bounds[purpose]
        if start_height < min_start - .5 or not min_end - .5 <= end_height <= max_end + .5:
            raise ValueError("Grasp height outside planned clearance bounds")
        if (purpose in ("Lift object", "Postplace retreat", "Grasp clearance retreat")
                and end_height <= start_height):
            raise ValueError("Retreat must move upward")
        if purpose == "Place object" and end_height >= start_height:
            raise ValueError("Place must move downward")
    else:
        for transform in (expected, target):
            tcp = transform @ tcp_transform(config)
            if tcp[2, 3] - table_z(plane, *tcp[:2, 3]) < 199:
                raise ValueError("Observation motion requires at least 200 mm table clearance")
    cancel = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: cancel.set())
    signal.signal(signal.SIGTERM, lambda *_: cancel.set())
    with DobotRobot(config["robot"]) as robot:
        current = robot.wait_enabled_still(cancel_event=cancel)
        moved, rotated = pose_distance(current, expected)
        if moved > 1.0 or rotated > .3:
            raise ValueError("Arm changed since observation plan")
        robot.verify_feedback_pose()
        joints = robot.read_joints()
        raw = robot._sdk.robot_control.InverseKin(plan["target_flange_mm_deg"],
                use_joint_near=1, joint_near=joints, user=0, tool=0)
        _numbers(_reply(raw, "InverseKin"), 6, "InverseKin")
        robot.move_flange(plan["target_flange_mm_deg"], 50, timeout_s=30,
                          cancel_event=cancel, pose_stability=True,
                          expected_start=robot.read_flange_pose())
        save_json(plan_path.with_name("arrival.json"), {
            "actual_flange_mm_deg": robot.read_flange_pose(), "speed_mm_s": 50,
            "gripper_command_sent": False, "purpose": purpose})


if __name__ == "__main__":
    main()
