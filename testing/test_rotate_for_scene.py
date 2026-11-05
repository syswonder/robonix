# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location("rotate_for_scene", Path(__file__).with_name("rotate_for_scene.py"))
rotation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rotation)


class RotationTests(unittest.TestCase):
    def run_rotation(self, *, stale=False, collision=False, missing_counter=False):
        clock = [0.0]
        yaw = [2.9]
        commands = []
        callbacks = {}
        node = Mock()
        node.create_subscription.side_effect = lambda _kind, topic, callback, _qos: callbacks.update({topic: callback})
        node.create_publisher.return_value.publish.side_effect = lambda msg: commands.append(msg.angular.z)
        ros = Mock()
        ros.create_node.return_value = node

        def spin(_node, timeout_sec):
            clock[0] += timeout_sec
            speed = commands[-1] if commands else 0.0
            yaw[0] += speed * timeout_sec * .5
            if not stale or not commands:
                q = NS(x=0.0, y=0.0, z=math.sin(yaw[0] / 2), w=math.cos(yaw[0] / 2))
                callbacks["/odom"](NS(pose=NS(pose=NS(orientation=q))))
            if not missing_counter:
                callbacks["/webots/collision_count"](NS(data=int(collision and yaw[0] > 3.2)))

        ros.spin_once.side_effect = spin
        modules = {
            "rclpy": ros,
            "rclpy.qos": NS(DurabilityPolicy=NS(TRANSIENT_LOCAL=1),
                            ReliabilityPolicy=NS(BEST_EFFORT=1, RELIABLE=2),
                            QoSProfile=lambda **_kw: None),
            "geometry_msgs.msg": NS(Twist=lambda: NS(angular=NS(z=0.0))),
            "nav_msgs.msg": NS(Odometry=object),
            "std_msgs.msg": NS(UInt32=object),
        }
        error = None
        with patch.dict("sys.modules", modules), \
             patch.object(rotation.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(rotation.signal, "signal"), patch("builtins.print"):
            try:
                rotation.main()
            except RuntimeError as exc:
                error = str(exc)
        self.assertEqual(commands[-1], 0.0)
        node.destroy_node.assert_called_once()
        ros.shutdown.assert_called_once()
        return clock[0], yaw[0] - 2.9, error

    def test_slow_simulation_completes_a_full_measured_turn(self):
        wall, angle, error = self.run_rotation()
        self.assertIsNone(error)
        self.assertGreaterEqual(angle, 2 * math.pi)
        self.assertLess(angle, 2 * math.pi + .05)
        self.assertGreater(wall, 30)

    def test_stale_odometry_stops_the_robot(self):
        wall, angle, error = self.run_rotation(stale=True)
        self.assertIn("lost odometry", error)
        self.assertLess(wall, 4)
        self.assertLess(angle, 1)

    def test_collision_stops_the_robot(self):
        wall, angle, error = self.run_rotation(collision=True)
        self.assertIn("collision", error)
        self.assertLess(angle, 1)
        self.assertLess(wall, 6)

    def test_missing_collision_counter_never_starts_motion(self):
        wall, angle, error = self.run_rotation(missing_counter=True)
        self.assertIn("collision counter did not arrive", error)
        self.assertEqual(angle, 0)
        self.assertLess(wall, 31)


if __name__ == "__main__":
    unittest.main()
