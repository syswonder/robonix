# SPDX-License-Identifier: MulanPSL-2.0
"""Real ROS 2 tests; source generated navigation interfaces first."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import uuid

from robonix_api import Transport
from robonix_api.atlas import _Atlas
from robonix_api.atlas_types import Ros2Params
from test_channel import ROOT, free_port, wait_port

SPEED = "robonix/service/navigation/get_speed_limit"
NAV = "robonix/service/navigation/navigate"
ODOM = "robonix/primitive/chassis/odom"
COMMAND = "robonix/primitive/chassis/twist_in"


def eventually(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("ROS 2 observation timed out")


@unittest.skipUnless(importlib.util.find_spec("rclpy")
                     and importlib.util.find_spec("navigation"), "source generated ROS 2 navigation interfaces")
class Ros2ChannelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from unittest import mock
        local_ros = mock.patch.dict(os.environ, {"ROS_LOCALHOST_ONLY": "1"})
        local_ros.start()
        cls.addClassCleanup(local_ros.stop)
        cls.temp = tempfile.TemporaryDirectory()
        cls.work = Path(cls.temp.name)
        cls.control = cls.work / "control.json"
        cls.control.write_text("{}")
        cls.log = (cls.work / "servers.log").open("w")
        subprocess.run([sys.executable, "-m", "grpc_tools.protoc",
            f"-I{ROOT / 'system/atlas/proto'}", f"--python_out={cls.work}",
            f"--grpc_python_out={cls.work}", "atlas.proto"], check=True)
        sys.path.insert(0, str(cls.work))
        port = free_port()
        cls.atlas_process = subprocess.Popen([str(ROOT / "target/debug/robonix-atlas"),
            "--listen", f"127.0.0.1:{port}", "--capabilities", str(ROOT / "capabilities")],
            stdout=cls.log, stderr=cls.log)
        cls.addClassCleanup(cls.atlas_process.wait, timeout=10)
        cls.addClassCleanup(cls.atlas_process.terminate)
        wait_port(cls.atlas_process, port)
        cls.prefix = "channel_" + uuid.uuid4().hex
        cls.ros_process = subprocess.Popen([sys.executable, str(Path(__file__).with_name("ros2_server.py")),
            "--prefix", cls.prefix, "--control", str(cls.control)], stdout=cls.log, stderr=cls.log)
        cls.addClassCleanup(cls.ros_process.wait, timeout=10)
        cls.addClassCleanup(cls.ros_process.terminate)
        eventually(lambda: cls.control.with_suffix(".ready").exists())
        cls.atlas = _Atlas(f"127.0.0.1:{port}")
        cls.atlas.register_service("ros2-test", "robonix/service")
        for cid, suffix in ((SPEED, "speed_limit"), (NAV, "navigate"), (NAV + "/status", "status"),
                            (NAV + "/cancel", "cancel"), (ODOM, "odom"), (COMMAND, "command")):
            cls.atlas.declare_capability("ros2-test", cid, Transport.ROS2, f"/{cls.prefix}/{suffix}",
                                        params=Ros2Params("latched" if cid == ODOM else ""))

    @classmethod
    def tearDownClass(cls):
        for channel in list(cls.atlas._channel_client.channels.values()):
            channel.close()
        from robonix_api.ros import RosBackend
        RosBackend.get().shutdown()
        for process in (cls.ros_process, cls.atlas_process):
            process.terminate()
            process.wait(timeout=10)
        cls.atlas._channel.close()
        cls.log.close()
        sys.path.remove(str(cls.work))
        cls.temp.cleanup()

    def setUp(self):
        self.control.write_text("{}")

    def connect(self, cid):
        return self.atlas.connect_channel(consumer_id="ros2-consumer", provider_id="ros2-test",
                                         contract_id=cid, transport=Transport.ROS2)

    def test_services_and_async_cancellation(self):
        from navigation.srv import GetNavigationSpeedLimit
        with self.connect(SPEED) as channel:
            self.assertIsInstance(channel.call({}), GetNavigationSpeedLimit.Response)
        with self.connect(NAV) as channel:
            self.assertTrue(channel.call({}).accepted)
            self.assertEqual(channel.run_id, "ros-run")
            self.assertEqual(channel.status().state, "RUNNING")
            self.assertTrue(channel.cancel().accepted)
            self.assertEqual(channel.status().state, "CANCELED")
        channel = self.connect(SPEED)
        self.control.write_text(json.dumps({"delay": 0.2}))
        with self.assertRaisesRegex(RuntimeError, "retained"):
            channel.call({}, timeout=0.01)
        self.assertIs(self.atlas.get_channel(channel.channel_id), channel)
        channel.close(timeout=2)
        self.assertNotIn(channel.channel_id, self.atlas.inspect()["channels"])

    def test_topics_qos_and_shared_backend(self):
        from rclpy.qos import DurabilityPolicy
        first, second = threading.Event(), threading.Event()
        with self.connect(ODOM) as channel, self.connect(ODOM) as other:
            channel.subscribe(lambda m: first.set() if m.pose.pose.position.x == 42 else None)
            other.subscribe(lambda m: second.set())
            self.assertTrue(first.wait(3))
            self.assertTrue(second.wait(3))
            self.assertEqual(channel._execution.protocol.entity.qos_profile.durability,
                             DurabilityPolicy.TRANSIENT_LOCAL)
            with self.assertRaisesRegex(RuntimeError, "only one"):
                channel.subscribe(lambda m: None)
            channel.close()
            second.clear()
            self.assertTrue(second.wait(3))
        with self.connect(COMMAND) as channel:
            from geometry_msgs.msg import Twist
            eventually(lambda: channel._execution.protocol.entity.get_subscription_count() > 0)
            channel.publish({"linear": {"x": 4.5}})
            eventually(lambda: self.control.with_suffix(".received").exists())
            self.assertEqual(float(self.control.with_suffix(".received").read_text()), 4.5)
            message = Twist()
            message.linear.x = 7.0
            channel.publish(message)
            eventually(lambda: self.control.with_suffix(".received").read_text() == "7.0")

    def test_close_waits_for_callback_and_setup_failure_cleans_allocation(self):
        entered, release = threading.Event(), threading.Event()
        channel = self.connect(ODOM)
        channel.subscribe(lambda message: (entered.set(), release.wait(3)))
        self.assertTrue(entered.wait(3))
        with self.assertRaisesRegex(RuntimeError, "callbacks still running"):
            channel.close(timeout=0.01)
        self.assertIs(self.atlas.get_channel(channel.channel_id), channel)
        release.set()
        channel.close()
        from unittest import mock
        from robonix_api.ros import RosBackend
        before = set(self.atlas.inspect()["channels"])
        with mock.patch.object(RosBackend.get(), "create_publisher", side_effect=RuntimeError("setup failed")):
            with self.assertRaisesRegex(RuntimeError, "setup failed"):
                self.connect(COMMAND)
        self.assertEqual(set(self.atlas.inspect()["channels"]), before)
