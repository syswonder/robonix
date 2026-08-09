#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0

"""Focused tests for the deterministic Piper health profile."""

import json
from pathlib import Path
import tempfile
import unittest

from piper_health.driver import (
    HealthSettings,
    build_health_state,
    load_fault_targets,
    normalize_fault_targets,
)


class PiperHealthDriverTests(unittest.TestCase):
    def test_normal_profile_covers_arm_joints_and_gripper(self):
        """The normal frame marks every described Piper actuator healthy."""
        state = build_health_state(HealthSettings())
        readings = {reading.name: reading for reading in state.readings}
        self.assertEqual(readings["body/state"].current_a, 0.0)
        self.assertEqual(readings["body/arm/online"].current_a, 1.0)
        for joint_index in range(1, 7):
            component_id = f"body/arm/joint_{joint_index}"
            self.assertIn(component_id, readings)
            self.assertEqual(readings[f"{component_id}/enabled"].current_a, 1.0)
            self.assertEqual(readings[f"{component_id}/error"].current_a, 0.0)
        self.assertIn("body/arm/gripper", readings)
        self.assertEqual(readings["body/arm/gripper/error"].current_a, 0.0)
        self.assertIn("body/arm/gripper/actuator", readings)
        self.assertEqual(
            readings["body/arm/gripper/actuator/enabled"].current_a, 1.0
        )

    def test_configuration_controls_voltage_and_interval(self):
        """Lifecycle values are reflected without changing component topology."""
        settings = HealthSettings.from_config(
            {"scenario": "normal", "interval_s": 1.0, "voltage": 25.2}
        )
        state = build_health_state(settings)
        readings = {reading.name: reading for reading in state.readings}
        self.assertAlmostEqual(state.voltage, 25.2, places=5)
        self.assertAlmostEqual(
            readings["body/arm/joint_1"].voltage, 25.2, places=5
        )

    def test_selected_joints_and_gripper_report_injected_faults(self):
        """Fault injection changes only the requested actuator controls."""
        state = build_health_state(
            HealthSettings(),
            frozenset({"joint_2", "joint_5", "gripper"}),
        )
        readings = {reading.name: reading for reading in state.readings}
        for target in ("joint_2", "joint_5"):
            component_id = f"body/arm/{target}"
            self.assertEqual(readings[f"{component_id}/enabled"].current_a, 0.0)
            self.assertEqual(
                readings[f"{component_id}/communication_ok"].current_a, 0.0
            )
            self.assertEqual(readings[f"{component_id}/error"].current_a, 17.0)
            self.assertEqual(readings[component_id].temp_c, 86.0)
        self.assertEqual(
            readings["body/arm/joint_3/communication_ok"].current_a,
            1.0,
        )
        self.assertEqual(readings["body/arm/joint_3/error"].current_a, 0.0)
        self.assertEqual(readings["body/arm/gripper/online"].current_a, 0.0)
        self.assertEqual(readings["body/arm/gripper/error"].current_a, 23.0)
        self.assertEqual(
            readings["body/arm/gripper/actuator/error"].current_a,
            0.0,
        )

    def test_runtime_control_file_supports_fault_and_recovery(self):
        """An atomic demo profile maps to targets and normal clears them."""
        with tempfile.TemporaryDirectory() as temporary:
            control_file = Path(temporary) / "piper_health.json"
            control_file.write_text(
                json.dumps(
                    {"schemaVersion": 1, "mode": "fault", "targets": ["joint_4"]}
                ),
                encoding="utf-8",
            )
            self.assertEqual(load_fault_targets(control_file), {"joint_4"})
            control_file.write_text(
                json.dumps({"schemaVersion": 1, "mode": "normal", "targets": []}),
                encoding="utf-8",
            )
            self.assertEqual(load_fault_targets(control_file), set())

    def test_unknown_fault_target_is_rejected(self):
        """Typos cannot silently inject a fault into an unintended component."""
        with self.assertRaisesRegex(ValueError, "unsupported Piper fault target"):
            normalize_fault_targets(["joint_7"])

    def test_unknown_scenario_is_rejected(self):
        """Reserved fault profiles fail until their behavior is implemented."""
        with self.assertRaisesRegex(ValueError, "only 'normal' is implemented"):
            HealthSettings.from_config({"scenario": "joint_fault"})

    def test_invalid_numeric_configuration_is_rejected(self):
        """Invalid timing and power values cannot create misleading telemetry."""
        with self.assertRaisesRegex(ValueError, "interval_s"):
            HealthSettings.from_config({"interval_s": 0})
        with self.assertRaisesRegex(ValueError, "voltage"):
            HealthSettings.from_config({"voltage": 0})


if __name__ == "__main__":
    unittest.main()
