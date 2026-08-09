# SPDX-License-Identifier: MulanPSL-2.0
"""Pure-Python regression tests for Scene's Atlas/ROS observation wiring."""

import importlib.util
import os
from pathlib import Path
import sys
import unittest


MODULE_PATH = os.path.join(
    os.path.dirname(__file__),
    "..",
    "scene_service",
    "ingest",
    "ros_subscribers.py",
)
SPEC = importlib.util.spec_from_file_location("scene_ros_subscribers", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules["scene_ros_subscribers"] = MODULE
SPEC.loader.exec_module(MODULE)


class RosContractQosTests(unittest.TestCase):
    def policy(self, kind: str, profile: str = "default"):
        return MODULE.topic_qos_policy(
            MODULE.TopicSpec(kind, "/test", "Image", profile)
        )

    def test_atlas_best_effort_is_preserved(self):
        self.assertEqual(
            self.policy("rgb", "best_effort"),
            ("best_effort", "volatile", 1),
        )
        self.assertEqual(
            self.policy("lidar3d", "best_effort"),
            ("best_effort", "volatile", 2),
        )

    def test_atlas_reliable_and_latched_are_preserved(self):
        self.assertEqual(
            self.policy("odom", "reliable"),
            ("reliable", "volatile", 10),
        )
        self.assertEqual(
            self.policy("occupancy_grid", "latched"),
            ("reliable", "transient_local", 1),
        )
        self.assertEqual(
            self.policy("occupancy_grid", "reliable"),
            ("reliable", "transient_local", 1),
        )

    def test_default_sensor_subscription_is_compatible(self):
        self.assertEqual(
            self.policy("lidar3d"),
            ("best_effort", "volatile", 2),
        )
        self.assertEqual(
            self.policy("intrinsics"),
            ("best_effort", "volatile", 1),
        )

    def test_scene_uses_canonical_lidar_and_chassis_odom_contracts(self):
        service = (Path(MODULE_PATH).parents[1] / "service.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("robonix/primitive/lidar/lidar3d", service)
        self.assertIn("robonix/primitive/chassis/odom", service)
        self.assertNotIn("robonix/primitive/lidar/pointcloud", service)

    def test_docker_runtime_supports_interface_bound_cyclonedds(self):
        scene_root = Path(MODULE_PATH).parents[2]
        dockerfile = (scene_root / "docker" / "Dockerfile").read_text(
            encoding="utf-8"
        )
        start = (scene_root / "scripts" / "start.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("ros-$ROS_DISTRO-rmw-cyclonedds-cpp", dockerfile)
        self.assertIn('-e CYCLONEDDS_URI="${CYCLONEDDS_URI:-}"', start)
        self.assertIn(
            '-e ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}"',
            start,
        )
        self.assertIn(
            '-e ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}"', start
        )
        self.assertIn(
            '-e SCENE_ANNOTATIONS_DIR="${SCENE_ANNOTATIONS_DIR:-/data/robonix/scene_annotations}"',
            start,
        )


if __name__ == "__main__":
    unittest.main()
