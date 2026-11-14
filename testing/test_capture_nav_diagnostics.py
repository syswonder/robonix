# SPDX-License-Identifier: MulanPSL-2.0
import array
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import capture_nav_diagnostics as capture


class Message:
    def __init__(self, **fields):
        self.__dict__.update(fields)

    def get_fields_and_field_types(self):
        return self.__dict__


def grid(data=(-1, 0, 37, 100)):
    return Message(
        header=Message(frame_id="map", stamp=Message(sec=12, nanosec=34)),
        info=Message(width=2, height=2, resolution=0.05,
                     origin=Message(position=Message(x=-3.0, y=1.0, z=0.0))),
        data=array.array("b", data),
    )


class CaptureTests(unittest.TestCase):
    def test_numpy_fixed_pose_covariance_is_captured_with_existing_bounds(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest("NumPy is required for generated fixed-size ROS arrays")
        covariance = np.arange(36, dtype=np.float64)
        msg = Message(header=Message(frame_id="map", stamp=Message(sec=12, nanosec=34)),
                      pose=Message(covariance=covariance))
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            stream = {"name": "pose", "received": 0, "rejected": 0, "samples": {}}
            capture.write_sample(out, stream, msg, 123, 456)
            self.assertEqual(stream["rejected"], 0)
            saved = json.loads((out / "pose-first.json").read_text())
            self.assertEqual(saved["message"]["pose"]["covariance"], covariance.tolist())
            self.assertEqual(saved["message"]["header"]["frame_id"], "map")
        with self.assertRaisesRegex(ValueError, "budget"):
            capture.message_values(covariance, budget=[4])
        with patch.object(capture, "MAX_RECORD_BYTES", 1):
            with self.assertRaisesRegex(ValueError, "budget"):
                capture.message_values(covariance)
        covariance[0] = np.nan
        with self.assertRaisesRegex(ValueError, "non-finite"):
            capture.sample_record(msg, 123, 456)

    def test_interleaved_tf_retains_rare_map_edge_and_each_edges_first_latest(self):
        def transform(parent, child, stamp):
            return Message(header=Message(frame_id=parent, stamp=Message(sec=stamp, nanosec=0)),
                           child_frame_id=child,
                           transform=Message(translation=Message(x=float(stamp))))

        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            stream = {"name": "tf", "received": 0, "rejected": 0, "samples": {}}
            for index, transforms in enumerate([
                [transform("map", "odom", 1)],
                [transform("odom", "base_link", 2)],
                [transform("odom", "base_link", 3)],
            ]):
                capture.write_sample(out, stream, Message(transforms=transforms),
                                     index, index * 10_000_000)
            frames = json.loads((out / "tf-frames.json").read_text())["frame_pairs"]
            self.assertEqual(len(frames), 2)
            self.assertEqual(frames[0]["latest"]["transform"]["header"]["frame_id"], "map")
            self.assertEqual(frames[1]["first"]["transform"]["header"]["stamp"]["sec"], 2)
            self.assertEqual(frames[1]["latest"]["transform"]["header"]["stamp"]["sec"], 3)
            capture.save_manifest(out, {"streams": [stream]})
            self.assertNotIn("_tf_frames", json.loads((out / "manifest.json").read_text())["streams"][0])

    def test_preserves_raw_costs_frame_source_and_receipt_times(self):
        sample = json.loads(capture.sample_record(grid(), 123, 456))
        self.assertEqual(sample["message"]["data"], [-1, 0, 37, 100])
        self.assertEqual(sample["message"]["header"],
                         {"frame_id": "map", "stamp": {"sec": 12, "nanosec": 34}})
        self.assertEqual(sample["receipt_wall_ns"], 123)
        self.assertEqual(sample["receipt_monotonic_ns"], 456)

    def test_first_latest_and_rejected_sample_are_auditable(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            stream = {"name": "map", "received": 0, "rejected": 0, "samples": {}}
            for count in range(4):
                capture.write_sample(out, stream, grid(), count, count * 10**9)
            self.assertEqual(len(list(out.iterdir())), 2)
            self.assertEqual(json.loads((out / "map-first.json").read_text())["receipt_wall_ns"], 0)
            self.assertEqual(json.loads((out / "map-latest.json").read_text())["receipt_wall_ns"], 3)
            with patch.object(capture, "MAX_RECORD_BYTES", 1):
                capture.write_sample(out, stream, grid(), 4, 4 * 10**9)
            self.assertEqual(stream["rejected"], 1)
            self.assertIn("byte limit", stream["last_error"])
            self.assertEqual(json.loads((out / "map-latest.json").read_text())["receipt_wall_ns"], 3)

    def test_capacity_checks_precede_message_conversion(self):
        with patch.object(capture, "MAX_CELLS", 3):
            with self.assertRaisesRegex(ValueError, "cell limit"):
                capture.sample_record(grid(), 0, 0)
        with patch.object(capture, "MAX_TRANSFORMS", 1):
            with self.assertRaisesRegex(ValueError, "transform limit"):
                capture.sample_record(Message(transforms=[Message(), Message()]), 0, 0)
        with self.assertRaisesRegex(ValueError, "dimensions"):
            capture.sample_record(grid([0]), 0, 0)
        with self.assertRaisesRegex(ValueError, "string"):
            capture.sample_record(Message(frame_id="x" * 1025), 0, 0)

    def test_total_output_limit_retains_previous_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            stream = {"name": "map", "received": 0, "rejected": 0, "samples": {}}
            capture.write_sample(out, stream, grid(), 1, 10**9)
            with patch.object(capture, "MAX_OUTPUT_BYTES", 1):
                capture.write_sample(out, stream, grid(), 2, 2 * 10**9)
            self.assertEqual(stream["rejected"], 1)
            self.assertEqual(len(list(out.iterdir())), 1)

    def test_missing_ros_dependency_leaves_error_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / "capture"
            argv = ["--out", str(out)]
            for name in ("map", "global-costmap", "pose", "tf", "tf-static", "clock"):
                argv.extend([f"--{name}-topic", "/" + name])
            with patch.dict("sys.modules", {"rclpy": None}), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(capture.main(argv), 2)
            manifest = json.loads((out / "manifest.json").read_text())
            self.assertEqual(manifest["status"], "error")
            self.assertIn("ModuleNotFoundError", manifest["error"])
            self.assertGreaterEqual(manifest["end_monotonic_ns"], manifest["start_monotonic_ns"])

    def test_unconfigured_pose_and_local_costmap_remain_explicit(self):
        node = SimpleNamespace(
            get_publishers_info_by_topic=lambda topic: [],
            create_subscription=lambda msg_type, topic, callback, qos: callback(Message(value=1)),
            destroy_node=lambda: None,
        )
        ros = SimpleNamespace(init=lambda: None, create_node=lambda name: node,
                              spin_once=lambda *args, **kwargs: None,
                              ok=lambda: True, shutdown=lambda: None)
        modules = {
            "rclpy": ros,
            "rclpy.qos": SimpleNamespace(
                QoSProfile=lambda **kwargs: SimpleNamespace(**kwargs),
                DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL="transient_local", VOLATILE="volatile"),
                ReliabilityPolicy=SimpleNamespace(RELIABLE="reliable", BEST_EFFORT="best_effort")),
            "geometry_msgs.msg": SimpleNamespace(PoseWithCovarianceStamped=Message),
            "nav_msgs.msg": SimpleNamespace(OccupancyGrid=Message),
            "rosgraph_msgs.msg": SimpleNamespace(Clock=Message),
            "tf2_msgs.msg": SimpleNamespace(TFMessage=Message),
        }
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / "capture"
            argv = ["--out", str(out), "--duration", "0.001"]
            for name in ("map", "global-costmap", "tf", "tf-static", "clock"):
                argv.extend([f"--{name}-topic", "/" + name])
            with patch.dict("sys.modules", modules), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(capture.main(argv), 1)
            manifest = json.loads((out / "manifest.json").read_text())
            self.assertEqual(manifest["status"], "incomplete")
            self.assertEqual(manifest["unconfigured_streams"], ["pose", "local_costmap"])
            self.assertEqual(manifest["missing_streams"], [])
            self.assertEqual(len(manifest["streams"]), 5)


if __name__ == "__main__":
    unittest.main()
