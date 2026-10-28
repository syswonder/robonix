# SPDX-License-Identifier: MulanPSL-2.0

import concurrent.futures
import ctypes
from pathlib import Path
import os
import subprocess
import sys
import tempfile
import unittest

import robonix_chronos_clock as clock


class ClockTests(unittest.TestCase):
    def test_reference_configuration_discards_previous_ros_reading(self):
        try:
            clock.configure_reference("ros_time")
            clock.update_reference_time(5_000_000_000, 1, True)
            self.assertEqual(clock.reference_timestamp_ns(), 5_000_000_000)
            clock.configure_reference("ros_time")
            self.assertFalse(clock.reference_time().available)
            clock.update_reference_time(0, 1, True)
            self.assertEqual(clock.reference_timestamp_ns(), 0)
        finally:
            clock.configure_reference("linux_system")

    def test_wall_clock_has_epoch_time_and_quality(self):
        self.assertGreater(clock.wall_timestamp_ns(), 0)
        reading = clock.wall_time()
        self.assertIsInstance(reading, clock.VirtualReferenceTime)
        self.assertGreater(reading.timestamp_ns, 0)
        self.assertGreater(reading.uncertainty_ns, 0)

    def test_snapshot_preserves_integer_times_and_read_only_quality(self):
        before = clock.steady_time().timestamp_ns
        snapshot = clock.snapshot()
        after = clock.steady_time().timestamp_ns
        self.assertIsInstance(snapshot, clock.VirtualClockSnapshot)
        self.assertIsInstance(snapshot.reference.timestamp_ns, int)
        self.assertLessEqual(before, snapshot.steady.timestamp_ns)
        self.assertLessEqual(snapshot.steady.timestamp_ns, after)
        self.assertGreater(snapshot.reference.uncertainty_ns, 0)
        self.assertFalse(snapshot.reference.verified)
        self.assertGreater(snapshot.reference.timestamp_ns, 0)
        with self.assertRaises(AttributeError):
            snapshot.reference.timestamp_ns = 0
        with self.assertRaises(AttributeError):
            snapshot.steady = None

    def test_threads_share_reference_generation(self):
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
            readings = list(executor.map(lambda _: clock.reference_time(), range(32)))
        self.assertTrue(all(value.generation == readings[0].generation for value in readings))

    @unittest.skipUnless(sys.platform == "linux", "requires Linux clock interposition")
    def test_read_failures_quality_and_reference_steps(self):
        with tempfile.TemporaryDirectory() as directory:
            library = Path(directory) / "clock_failure.so"
            fixture = Path(__file__).resolve().parent / "support/clock_failure.c"
            subprocess.run([
                "cc", "-shared", "-fPIC", "-std=c11", "-Wall", "-Wextra", "-Werror",
                str(fixture), "-ldl", "-o", str(library),
            ], check=True)
            result = subprocess.run([
                sys.executable, str(Path(__file__).resolve()), "--fault-case",
            ], env={**os.environ, "LD_PRELOAD": str(library)}, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


def fault_case():
    mode = ctypes.CDLL(None).robonix_test_clock_mode
    mode.argtypes = [ctypes.c_int]
    mode.restype = None
    case = unittest.TestCase()
    first = clock.reference_time()
    mode(1)
    with case.assertRaises(OSError):
        clock.reference_time()
    case.assertGreater(clock.steady_time().timestamp_ns, 0)
    mode(16)
    with case.assertRaises(OSError):
        clock.steady_time()
    case.assertGreater(clock.steady_time().timestamp_ns, 0)
    mode(4)
    reading = clock.reference_time()
    case.assertFalse(reading.available)
    case.assertGreater(reading.uncertainty_ns, 1_000_000)
    with case.assertRaises(OSError):
        clock.reference_timestamp_ns()
    case.assertFalse(clock.wall_time().available)
    case.assertGreater(clock.wall_timestamp_ns(), 0)
    mode(2)
    case.assertLess(clock.reference_time().timestamp_ns, 0)
    with case.assertRaises(OSError):
        clock.reference_timestamp_ns()
    mode(0)
    recovered = clock.reference_time()
    case.assertGreater(recovered.timestamp_ns, 0)
    case.assertGreater(recovered.generation, first.generation)
    mode(8)
    stepped = clock.snapshot().reference
    case.assertTrue(stepped.generation_changed)
    case.assertGreater(stepped.generation, recovered.generation)
    case.assertEqual(clock.reference_time().generation, stepped.generation)


if __name__ == "__main__":
    if sys.argv[1:] == ["--fault-case"]:
        fault_case()
    else:
        unittest.main()
