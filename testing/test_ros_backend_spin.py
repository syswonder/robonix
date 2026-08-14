# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


ROS_BACKEND = (
    Path(__file__).parents[1]
    / "pylib"
    / "robonix-api"
    / "robonix_api"
    / "ros.py"
)


def load_ros_backend_module():
    spec = importlib.util.spec_from_file_location("robonix_api_ros_test", ROS_BACKEND)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class InvalidHandle(Exception):
    pass


class RosBackendSpinTest(unittest.TestCase):
    def test_transient_destroyed_handle_does_not_stop_executor(self) -> None:
        rclpy = types.ModuleType("rclpy")
        rclpy_pybind = types.ModuleType("rclpy._rclpy_pybind11")
        rclpy_pybind.InvalidHandle = InvalidHandle
        running = True

        def ok() -> bool:
            return running

        rclpy.ok = ok

        class Executor:
            calls = 0

            def spin_once(self, *, timeout_sec: float) -> None:
                nonlocal running
                self.calls += 1
                self.timeout_sec = timeout_sec
                if self.calls == 1:
                    raise InvalidHandle(
                        "cannot use Destroyable because destruction was requested"
                    )
                running = False

        with mock.patch.dict(
            sys.modules,
            {"rclpy": rclpy, "rclpy._rclpy_pybind11": rclpy_pybind},
        ), mock.patch("time.sleep"):
            module = load_ros_backend_module()
            backend = module.RosBackend()
            backend._executor = Executor()
            backend._spin()

        self.assertEqual(backend._executor.calls, 2)
        self.assertEqual(backend._executor.timeout_sec, 0.05)


if __name__ == "__main__":
    unittest.main()
