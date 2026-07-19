#!/usr/bin/env python3
"""Regression tests for the Webots LaserScan normalization relay."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import types
import unittest


SCRIPT = Path(__file__).parents[1] / "scripts" / "scan_normalize.py"


class Stamp:
    def __init__(self, sec: int = 0, nanosec: int = 0):
        self.sec = sec
        self.nanosec = nanosec


class Header:
    def __init__(self, frame_id: str = "", sec: int = 0, nanosec: int = 0):
        self.frame_id = frame_id
        self.stamp = Stamp(sec, nanosec)


class LaserScan:
    def __init__(self):
        self.header = Header()
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.time_increment = 0.0
        self.scan_time = 0.0
        self.range_min = 0.0
        self.range_max = 0.0
        self.ranges = []
        self.intensities = []


class Logger:
    def info(self, _message: str) -> None:
        pass


class Node:
    def get_logger(self) -> Logger:
        return Logger()


class Publisher:
    def __init__(self):
        self.message = None

    def publish(self, message: LaserScan) -> None:
        self.message = message


def load_normalizer_module():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.spin = lambda _node: None
    rclpy.shutdown = lambda: None

    node_module = types.ModuleType("rclpy.node")
    node_module.Node = Node

    qos_module = types.ModuleType("rclpy.qos")
    qos_module.QoSProfile = lambda **kwargs: kwargs
    qos_module.ReliabilityPolicy = types.SimpleNamespace(RELIABLE="reliable")
    qos_module.DurabilityPolicy = types.SimpleNamespace(VOLATILE="volatile")
    qos_module.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.LaserScan = LaserScan

    modules = {
        "rclpy": rclpy,
        "rclpy.node": node_module,
        "rclpy.qos": qos_module,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
    }
    previous = {name: sys.modules.get(name) for name in modules}
    sys.modules.update(modules)
    try:
        spec = importlib.util.spec_from_file_location("scan_normalize_under_test", SCRIPT)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        for name, value in previous.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value


MODULE = load_normalizer_module()


def normalize(message: LaserScan) -> LaserScan:
    node = MODULE.Normalizer.__new__(MODULE.Normalizer)
    node._warned = False
    node.pub = Publisher()
    node._cb(message)
    assert node.pub.message is not None
    return node.pub.message


def scan(*, reversed_angles: bool, scan_time: float, time_increment: float) -> LaserScan:
    message = LaserScan()
    message.header = Header("base_laser_link", 123, 456_789_012)
    message.angle_min = 1.5 if reversed_angles else -1.5
    message.angle_max = -1.5 if reversed_angles else 1.5
    message.angle_increment = -0.5 if reversed_angles else 0.5
    message.scan_time = scan_time
    message.time_increment = time_increment
    message.range_min = 0.12
    message.range_max = 30.0
    message.ranges = [1.0, 2.0, 3.0, 4.0]
    message.intensities = [10.0, 20.0, 30.0, 40.0]
    return message


class ScanNormalizeTest(unittest.TestCase):
    def assert_timing_unchanged(self, source: LaserScan, result: LaserScan) -> None:
        self.assertEqual(result.header.frame_id, source.header.frame_id)
        self.assertEqual(result.header.stamp.sec, source.header.stamp.sec)
        self.assertEqual(result.header.stamp.nanosec, source.header.stamp.nanosec)
        self.assertEqual(result.scan_time, source.scan_time)
        self.assertEqual(result.time_increment, source.time_increment)

    def test_reversed_angles_and_samples_are_normalized(self) -> None:
        source = scan(reversed_angles=True, scan_time=0.125, time_increment=0.001)
        result = normalize(source)

        self.assertEqual(result.angle_min, source.angle_max)
        self.assertEqual(result.angle_max, source.angle_min)
        self.assertEqual(result.angle_increment, -source.angle_increment)
        self.assertEqual(result.ranges, [4.0, 3.0, 2.0, 1.0])
        self.assertEqual(result.intensities, [40.0, 30.0, 20.0, 10.0])
        self.assertEqual(result.range_min, source.range_min)
        self.assertEqual(result.range_max, source.range_max)
        self.assert_timing_unchanged(source, result)

    def test_standard_scan_is_passed_through(self) -> None:
        source = scan(reversed_angles=False, scan_time=0.125, time_increment=0.001)
        result = normalize(source)

        self.assertEqual(result.angle_min, source.angle_min)
        self.assertEqual(result.angle_max, source.angle_max)
        self.assertEqual(result.angle_increment, source.angle_increment)
        self.assertEqual(result.ranges, source.ranges)
        self.assertEqual(result.intensities, source.intensities)
        self.assert_timing_unchanged(source, result)

    def test_zero_timing_fields_remain_zero(self) -> None:
        source = scan(reversed_angles=True, scan_time=0.0, time_increment=0.0)
        result = normalize(source)

        self.assertEqual(result.scan_time, 0.0)
        self.assertEqual(result.time_increment, 0.0)
        self.assert_timing_unchanged(source, result)

    def test_synthetic_timing_logic_is_absent(self) -> None:
        source = SCRIPT.read_text(encoding="utf-8")
        for forbidden in ("_scan_period_s", "_last_stamp_s", "offset_s"):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
