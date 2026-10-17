# SPDX-License-Identifier: MulanPSL-2.0

import contextlib
import importlib.util
import io
import json
from pathlib import Path
from types import ModuleType, SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location("reference_ros", Path(__file__).parents[1] / "reference_ros.py")
reader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reader)


class ReferenceReaderTest(unittest.TestCase):
    def test_zero_pause_publisher_loss_and_owner_eof(self):
        node = Mock()
        callbacks = []
        node.create_subscription.side_effect = lambda _, topic, callback, qos: callbacks.append(callback)
        ros = ModuleType("rclpy")
        ros.init = Mock()
        ros.create_node = Mock(return_value=node)
        ros.shutdown = Mock()
        qos = ModuleType("rclpy.qos")
        qos.QoSProfile = lambda **kw: kw
        qos.ReliabilityPolicy = SimpleNamespace(BEST_EFFORT=1)
        messages = ModuleType("rosgraph_msgs.msg")
        messages.Clock = object
        now = [0]
        turns = iter([(0, 1, False), (.6, 1, True), (1.2, 1, False), (1.8, 0, False), (2.4, 1, False), (3, 1, True)])

        def spin_once(_, timeout_sec):
            now[0], publishers, deliver = next(turns)
            node.count_publishers.return_value = publishers
            if deliver:
                callbacks[0](SimpleNamespace(clock=SimpleNamespace(sec=0, nanosec=0)))

        ros.spin_once = spin_once
        output = io.StringIO()
        with patch.dict(sys.modules, {"rclpy": ros, "rclpy.qos": qos, "rosgraph_msgs.msg": messages}), \
                patch.object(reader.select, "select", side_effect=[([], [], [])] * 6 + [([0], [], [])]), \
                patch.object(reader.os, "read", return_value=b""), \
                patch.object(reader.sys, "stdin", SimpleNamespace(fileno=lambda: 0)), \
                patch.object(reader.time, "monotonic", side_effect=lambda: now[0]), \
                contextlib.redirect_stdout(output):
            reader.main()
        events = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertEqual(events[0], {"ready": True})
        self.assertEqual(events[1], {"timestamp_ns": None, "available": False})
        self.assertEqual(events[4], {"timestamp_ns": 0, "available": True})
        self.assertEqual(events[5], {"timestamp_ns": None, "available": False})
        self.assertEqual(events[6], {"timestamp_ns": None, "available": False})
        self.assertEqual(events[7], {"timestamp_ns": 0, "available": True})
        node.destroy_node.assert_called_once()
        ros.shutdown.assert_called_once()

    def test_consumer_close_during_sample_delivery_does_not_read_again(self):
        import asyncio
        from unittest.mock import AsyncMock
        from robonix_api import _reference_time as reference
        clock = reference._ReferenceClock({}, 'test/reference')
        clock._release = Mock()
        delivered = []
        async def read():
            if delivered:
                self.fail('closed consumer attempted another stream read')
            delivered.append(True)
            clock._closed.set()
            return SimpleNamespace(available=True, timestamp_ns=0, generation=1)
        call = SimpleNamespace(read=read, cancel=Mock())
        clock._stub_type = lambda channel: SimpleNamespace(ReferenceTime=lambda request: call)
        atlas = SimpleNamespace(_wire_stub=Mock(), _wire_pb=Mock())
        atlas._wire_stub.ConnectCapability.return_value = SimpleNamespace(endpoint='127.0.0.1:1')
        with patch.object(reference, 'ATLAS', atlas), \
                patch.object(reference.grpc.aio, 'insecure_channel', return_value=AsyncMock()):
            asyncio.run(clock._follow())
        self.assertEqual(delivered, [True])
        call.cancel.assert_called_once()
        self.assertEqual(clock._release.call_count, 2)


if __name__ == "__main__":
    unittest.main()
