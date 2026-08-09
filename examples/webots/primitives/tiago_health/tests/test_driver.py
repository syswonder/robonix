#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Focused tests for the deterministic Webots health profile."""

import unittest

from tiago_health.driver import HealthSettings, build_health_state


class HealthDriverTests(unittest.TestCase):
    def test_normal_profile_contains_nominal_component_values(self):
        """The default frame covers every Soma component without fault codes."""
        state = build_health_state(HealthSettings())
        readings = {reading.name: reading for reading in state.readings}
        self.assertAlmostEqual(state.voltage, 24.8, places=5)
        self.assertEqual(readings["body/state"].current_a, 0.0)
        self.assertEqual(readings["body/base/left_wheel/error"].current_a, 0.0)
        self.assertEqual(readings["body/base/right_wheel/communication_ok"].current_a, 1.0)
        self.assertAlmostEqual(
            readings["body/base/battery"].battery_percent, 82.0, places=5
        )

    def test_unknown_scenario_is_rejected(self):
        """Reserved scenarios fail initialization until their data is implemented."""
        with self.assertRaisesRegex(ValueError, "only 'normal' is implemented"):
            HealthSettings.from_config({"scenario": "wheel_fault"})

    def test_full_profile_contains_arm_and_gripper(self):
        """Full TIAGo reports every described arm joint and its gripper."""
        settings = HealthSettings.from_config({"variant": "full"})
        state = build_health_state(settings)
        readings = {reading.name: reading for reading in state.readings}
        for joint_index in range(1, 8):
            component_id = f"body/arm/joint_{joint_index}"
            self.assertIn(component_id, readings)
            self.assertEqual(readings[f"{component_id}/enabled"].current_a, 1.0)
        self.assertIn("body/arm/gripper", readings)
        self.assertEqual(readings["body/arm/gripper/error"].current_a, 0.0)
        self.assertIn("body/arm/gripper/actuator", readings)
        self.assertEqual(
            readings["body/arm/gripper/actuator/enabled"].current_a,
            1.0,
        )

    def test_lite_profile_omits_nonexistent_arm(self):
        """Default TIAGo Lite telemetry never invents arm components."""
        state = build_health_state(HealthSettings())
        self.assertFalse(
            any(reading.name.startswith("body/arm") for reading in state.readings)
        )

    def test_unknown_variant_is_rejected(self):
        """Variant typos fail initialization instead of desynchronizing Soma."""
        with self.assertRaisesRegex(ValueError, "choose 'lite' or 'full'"):
            HealthSettings.from_config({"variant": "tiago++"})


if __name__ == "__main__":
    unittest.main()
