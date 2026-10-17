# SPDX-License-Identifier: MulanPSL-2.0
"""Test-only hardware counter with a real clock bracket and a u32 wrap after 50ms."""
from dataclasses import replace
import os
from pathlib import Path
import sys
import time
from chronos_query import capture_query


class Clock:
    """Emulate unsigned microsecond ticks without depending on ROS or an actual vendor SDK."""

    def __init__(self, options):
        self.started = None
        self.mode = options.get("mode", "normal")

    def read(self):
        """Exercise polling, read errors, oversized stdout and blocking SDK calls."""
        if self.mode == "error":
            raise RuntimeError("fixture hardware query failed")
        if self.mode == "hang":
            time.sleep(5)
        if self.mode == "oversized_observation":
            time.sleep(0.1)
            oversized_output()
        if self.started is None:
            self.started = time.monotonic_ns()
        sample = capture_query(lambda: (2**32 - 50_000 + (time.monotonic_ns() - self.started) // 1000) % 2**32)
        return replace(sample, uncertainty_ns=2**64 - 1) if self.mode == "overflow" else sample

    def close(self):
        pass


def oversized_output():
    """Keep stdout open after overflowing the frame so the parent must enforce its byte limit."""
    sys.stdout.write("x" * 4097)
    sys.stdout.flush()
    time.sleep(30)


def create(options):
    """Record the owned process identity and optionally overflow its startup output."""
    if options.get("pid_file"):
        Path(options["pid_file"]).write_text(str(os.getpid()))
    if options.get("mode") == "oversized_startup":
        oversized_output()
    return Clock(options)
