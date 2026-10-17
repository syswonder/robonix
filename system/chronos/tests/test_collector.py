# SPDX-License-Identifier: MulanPSL-2.0
"""Dependency-free checks of device extraction and fail-closed identity handling."""
import importlib.util
import io
import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace as Message
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).parents[1]))
from chronos_query import QuerySample, capture_query

spec = importlib.util.spec_from_file_location("collector", Path(__file__).parents[1] / "collector.py")
collector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(collector)


class CollectorTests(unittest.TestCase):
    def test_callback_captures_before_lock_and_extraction(self):
        """Execute subscription and IPC forwarding with delays inserted at locking and extraction."""
        clock = dict(steady=100, reference=1_700_000_000_000_000_000)
        node, executor, condition = MagicMock(), MagicMock(), MagicMock()

        def delay():
            clock["steady"] += 1_000
            clock["reference"] += 1_000

        def subscribe(_type, _topic, callback, _qos):
            node.callback = callback

        sample = collector.Source.sample

        def delayed_sample(source, message):
            delay()
            return sample(source, message)

        condition.__enter__.side_effect = delay
        node.create_subscription.side_effect = subscribe
        thread = MagicMock()
        thread.start.side_effect = lambda: node.callback(self.message(123))
        modules = {
            "rclpy": Message(init=MagicMock(), create_node=lambda _name: node, ok=lambda: True, shutdown=MagicMock()),
            "rclpy.executors": Message(SingleThreadedExecutor=lambda: executor),
            "rclpy.qos": Message(QoSProfile=MagicMock(),
                ReliabilityPolicy=Message(BEST_EFFORT=0, RELIABLE=1),
                DurabilityPolicy=Message(TRANSIENT_LOCAL=0, VOLATILE=1)),
            "rosidl_runtime_py": Message(),
            "rosidl_runtime_py.utilities": Message(get_message=lambda _type: Message),
        }
        plan = dict(domain_id="camera", timestamp_field="header.stamp", qos="best_effort",
                    msg_type="sensor_msgs/msg/Image", topic="/camera")
        output = io.StringIO()
        with patch.dict(collector.sys.modules, modules), \
                patch.object(collector.sys, "stdin", io.StringIO(json.dumps([plan]) + "\n")), \
                patch.object(collector.sys, "stdout", output), \
                patch.object(collector.threading, "Condition", return_value=condition), \
                patch.object(collector.threading, "Thread", return_value=thread), \
                patch.object(collector.time, "monotonic_ns", side_effect=lambda: clock["steady"]), \
                patch.object(collector.time, "time_ns", side_effect=lambda: clock["reference"]), \
                patch.object(collector.Source, "sample", delayed_sample):
            collector.run()
        ready, observation = map(json.loads, output.getvalue().splitlines())
        self.assertEqual(ready, dict(ready=True))
        self.assertEqual(observation["device_timestamp_ns"], 123)
        self.assertEqual(observation["received_reference_ns"], 1_700_000_000_000_000_000)
        self.assertEqual(observation["received_steady_ns"], 100)
        self.assertEqual(observation["received_uncertainty_ns"], collector.REFERENCE_RESOLUTION_NS)

    def test_receive_clock_pair_is_bracketed(self):
        """Callback evidence keeps integer precision and accounts for time spent reading clocks."""
        with patch.object(collector.time, "monotonic_ns", side_effect=[100, 107]), \
                patch.object(collector.time, "time_ns", return_value=1_700_000_000_123_456_789):
            received = collector.receive_time()
        self.assertEqual(received["received_reference_ns"], 1_700_000_000_123_456_789)
        self.assertEqual(received["received_steady_ns"], 103)
        self.assertEqual(received["received_uncertainty_ns"], 4 + collector.REFERENCE_RESOLUTION_NS)
        with patch.object(collector.time, "monotonic_ns", side_effect=[107, 100]):
            with self.assertRaisesRegex(RuntimeError, "regressed"):
                collector.receive_time()

    def source(self, **fields):
        return collector.Source(dict(domain_id="camera", timestamp_field="header.stamp", **fields))

    def message(self, nanos, **fields):
        return Message(header=Message(stamp=Message(sec=nanos // 10**9, nanosec=nanos % 10**9)), **fields)

    def test_exact_timestamp_and_capture_deduplication(self):
        """Integer conversion keeps large timestamps exact and cache repeats contribute nothing."""
        source = self.source()
        nanos = 1_700_000_000_123_456_789
        first = source.sample(self.message(nanos))
        self.assertEqual(first["device_timestamp_ns"], nanos)
        self.assertIsNone(source.sample(self.message(nanos)))
        second = source.sample(self.message(nanos + 1))
        self.assertEqual(second["session_id"], first["session_id"])
        self.assertEqual(second["sequence_id"], first["sequence_id"] + 1)

    def test_regression_requires_explicit_identity_change(self):
        """Timestamp-only sources cannot turn an out-of-order sample into a guessed reset."""
        source = self.source()
        source.sample(self.message(100))
        with self.assertRaisesRegex(ValueError, "regressed"):
            source.sample(self.message(99))
        native = self.source(session_id_field="session", device_generation_field="generation", sequence_id_field="sequence")
        native.sample(self.message(100, session="a", generation=1, sequence=10))
        restarted = native.sample(self.message(1, session="b", generation=1, sequence=1))
        self.assertEqual(restarted["session_id"], "b")
        generation = native.sample(self.message(0, session="b", generation=2, sequence=0))
        self.assertEqual(generation["device_generation"], 2)

    def test_native_sequence_and_reused_identity(self):
        """Older captures are dropped and reusing a capture identity with new data is an error."""
        source = self.source(session_id_field="session", device_generation_field="generation", sequence_id_field="sequence")
        source.sample(self.message(100, session="a", generation=1, sequence=10))
        self.assertIsNone(source.sample(self.message(99, session="a", generation=1, sequence=9)))
        with self.assertRaisesRegex(ValueError, "reused"):
            source.sample(self.message(101, session="a", generation=1, sequence=10))

    def test_partial_identity_is_rejected_before_subscribing(self):
        """A synthetic counter must not restart inside a preserved native device session."""
        with self.assertRaisesRegex(ValueError, "all three"):
            self.source(session_id_field="session")

    def test_native_cached_frame_on_new_helper_is_identical(self):
        """Native identity stays unchanged across helpers so the core can reject cached replay."""
        fields = dict(session_id_field="session", device_generation_field="generation", sequence_id_field="sequence")
        message = self.message(100, session="a", generation=1, sequence=10)
        self.assertEqual(self.source(**fields).sample(message), self.source(**fields).sample(message))

    def test_invalid_values_are_not_coerced(self):
        """Reject floats, booleans, malformed ROS Time and RPC integer overflow."""
        for value in (True, 1.5, -1, 2**63, Message(sec=0, nanosec=10**9)):
            with self.subTest(value=value), self.assertRaises((ValueError, AttributeError)):
                collector.timestamp(value)
        for value in (True, 0, 2**64):
            with self.subTest(value=value), self.assertRaises(ValueError):
                collector.integer(value, "generation", 1)


class QueryTests(unittest.TestCase):
    def source(self, **query):
        """Build a fake adapter through the real module factory without loading any hardware SDK."""
        adapter = Message(read=MagicMock(), close=MagicMock())
        module = Message(create=MagicMock(return_value=adapter))
        plan = dict(domain_id="camera", sampling_interval_ns=20_000_000,
                    wrap_period_ns=2**32 * 1000, max_drift_ppm=500,
                    query=dict(adapter="fake_clock:create", tick_ns=1000,
                               timeout_ns=50_000_000, options=dict(serial="123"), **query))
        with patch.dict(sys.modules, fake_clock=module):
            source = collector.QuerySource(plan)
        module.create.assert_called_once_with(dict(serial="123"))
        return source, adapter

    def sample(self, ticks, generation=1, before=100, after=200, reference=1_700_000_000_000_000_000):
        """Construct exact same-host query brackets with controllable raw counter and restart evidence."""
        return QuerySample(ticks, reference + before, reference + after, before, after, generation)

    def test_capture_query_brackets_actual_read(self):
        """Clock reads surround the vendor operation, not later IPC or extraction."""
        with patch("chronos_query.time.monotonic_ns", side_effect=[10, 40]), \
                patch("chronos_query.time.time_ns", side_effect=[100, 130]):
            sample = capture_query(lambda: 2**32 - 1, device_generation=2)
        self.assertEqual(sample, QuerySample(2**32 - 1, 100, 130, 10, 40, 2))

    def test_query_u32_wrap_and_explicit_restart(self):
        """Raw u32 microseconds remain raw after scaling; confirmed reboot changes generation."""
        source, adapter = self.source()
        with patch.object(collector.time, "monotonic_ns", return_value=2000):
            adapter.read.return_value = self.sample(2**32 - 1)
            first = source.sample_query()
            adapter.read.return_value = self.sample(1, before=1100, after=1200)
            wrapped = source.sample_query()
            adapter.read.return_value = self.sample(0, generation=2, before=1300, after=1400)
            restarted = source.sample_query()
        self.assertEqual(first["device_timestamp_ns"], (2**32 - 1) * 1000)
        self.assertEqual(wrapped["device_timestamp_ns"], 1000)
        self.assertEqual(wrapped["device_generation"], 1)
        self.assertEqual(restarted["device_generation"], 2)
        self.assertEqual(first["received_steady_ns"], 150)
        self.assertEqual(first["received_reference_ns"], 1_700_000_000_000_000_150)
        self.assertEqual(first["received_uncertainty_ns"], 51 + collector.REFERENCE_RESOLUTION_NS)

    def test_invalid_queries_fail_closed(self):
        """Reject wrong sample types, overflow, stale/future brackets, long reads and wall-clock steps."""
        bad = [
            123, self.sample(True), self.sample(1.5), self.sample(2**64),
            self.sample(2**32), self.sample(1, generation=0),
            self.sample(1, after=99), self.sample(1, after=50_000_101),
            self.sample(1, after=2_000_000_000),
            QuerySample(1, 100, 2_000_100, 100, 200),
            QuerySample(1, 100, 200, 100, 200, uncertainty_ns=2**64 - 1),
        ]
        for sample in bad:
            source, adapter = self.source()
            adapter.read.return_value = sample
            with self.subTest(sample=sample), patch.object(collector.time, "monotonic_ns", return_value=1000), \
                    self.assertRaises((ValueError, TypeError)):
                source.sample_query()
        source, adapter = self.source()
        adapter.read.return_value = self.sample(1)
        with patch.object(collector.time, "monotonic_ns", return_value=200_000_000), \
                self.assertRaisesRegex(ValueError, "stale"):
            source.sample_query()
        source.plan["query"]["tick_ns"] = 2**63
        with patch.object(collector.time, "monotonic_ns", return_value=1000), \
                self.assertRaisesRegex(ValueError, "timestamp"):
            source.sample_query()

    def test_wrap_gap_and_unconfirmed_regression_are_rejected(self):
        """Do not guess missed cycles or a reboot without generation evidence."""
        source, adapter = self.source()
        with patch.object(collector.time, "monotonic_ns", return_value=2000):
            adapter.read.return_value = self.sample(100)
            source.sample_query()
            adapter.read.return_value = self.sample(99, before=1100, after=1200)
            with self.assertRaisesRegex(ValueError, "ambiguous"):
                source.sample_query()
        adapter.read.return_value = self.sample(101, before=3_000_000_000_000, after=3_000_000_000_100)
        with patch.object(collector.time, "monotonic_ns", return_value=3_000_000_000_200), \
                self.assertRaisesRegex(ValueError, "gap"):
            source.sample_query()

    def test_query_only_process_errors_and_timeouts_need_no_ros(self):
        """A real helper process reports failed/hung SDK reads and exits promptly without ROS fallback."""
        folder = Path(__file__).parent
        for mode in ("error", "hang"):
            plan = dict(domain_id="camera", sampling_interval_ns=20_000_000,
                        wrap_period_ns=2**32 * 1000, max_drift_ppm=500,
                        query=dict(adapter="query_fixture:create", options=dict(mode=mode),
                                   tick_ns=1000, timeout_ns=20_000_000))
            environment = dict(os.environ, PYTHONPATH=str(folder))
            result = subprocess.run([sys.executable, str(folder.parent / "collector.py")],
                input=json.dumps([plan]) + "\n", text=True, capture_output=True,
                env=environment, timeout=3)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("all configured observation domains have failed" if mode == "error" else "query timed out", result.stderr)
            self.assertEqual(result.stdout.strip(), '{"ready": true}')


    def test_invalid_adapter_does_not_corrupt_another_domain(self):
        """Overflow is contained before IPC while another configured adapter continues sampling."""
        plans = [dict(domain_id=domain, sampling_interval_ns=20_000_000,
                    wrap_period_ns=2**32 * 1000, max_drift_ppm=500,
                    query=dict(adapter="query_fixture:create", options=dict(mode=mode),
                               tick_ns=1000, timeout_ns=50_000_000))
                 for domain, mode in [("bad", "overflow"), ("good", "normal")]]
        folder = Path(__file__).parent
        result = subprocess.run([sys.executable, str(folder.parent / "collector.py")],
            input=json.dumps(plans) + "\n" + "\n" * 5, text=True, capture_output=True,
            env=dict(os.environ, PYTHONPATH=str(folder)), timeout=3)
        self.assertEqual(result.returncode, 0, result.stderr)
        observations = [json.loads(line) for line in result.stdout.splitlines()[1:]]
        self.assertGreaterEqual(len(observations), 5)
        self.assertTrue(all(row["domain_id"] == "good" for row in observations))
        self.assertIn("combined query uncertainty", result.stderr)


if __name__ == "__main__":
    unittest.main()
