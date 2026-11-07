# SPDX-License-Identifier: MulanPSL-2.0
import asyncio
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch


HERE = Path(__file__).resolve().parent
for package in (HERE / "skills/pick_return", HERE / "primitives/wrist_rgb"):
    sys.path[:0] = [str(package), str(package / "rbnx-build/codegen/robonix_mcp_types")]

from pick_return import driver as pick
from wrist_rgb import driver as camera


class PickLifecycleTest(unittest.TestCase):
    def test_shutdown_cancels_and_reaps_running_workflow(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            script = root / "run.py"
            script.write_text("from pathlib import Path\nimport time\n"
                              "Path(__file__).with_suffix('.ready').touch()\n"
                              "time.sleep(60)\n")
            with patch.multiple(pick, _script=script, _config=(Path(sys.executable), root / "settings.json"),
                                _active=True, _latched=False, _run=None):
                pick.run(pick.Run_Request(target="box"))
                process = pick._run["process"]
                try:
                    deadline = time.monotonic() + 5
                    while not script.with_suffix('.ready').exists():
                        self.assertLess(time.monotonic(), deadline)
                        time.sleep(.01)
                    pick.skill._on_shutdown()
                    pick.skill._teardown()
                    process.wait(timeout=5)
                    pick.refresh()
                    self.assertEqual(pick._run["state"], "CANCELED")
                    self.assertTrue(pick._latched)
                    self.assertFalse(pick._active)
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.wait(timeout=5)

    def test_artifact_limit_refuses_a_new_workflow(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            logs = root / ".artifacts/skill-runs"
            for index in range(100):
                (logs / str(index)).mkdir(parents=True)
            with patch.multiple(pick, _script=root / "run.py", _config=(Path(sys.executable), root / "settings.json"),
                                _active=True, _latched=False, _run=None):
                with self.assertRaisesRegex(RuntimeError, "100 run records"):
                    pick.run(pick.Run_Request(target="box"))
                self.assertIsNone(pick._run)
                self.assertEqual(len(list(logs.iterdir())), 100)


class CameraLoopTest(unittest.IsolatedAsyncioTestCase):
    async def test_capture_does_not_block_mcp_event_loop(self):
        started, release = threading.Event(), threading.Event()

        def capture():
            started.set()
            if not release.wait(timeout=2):
                raise TimeoutError("MCP event loop did not resume during capture")
            return {"height": 1, "width": 1, "stamp_sec": 1, "stamp_nanosec": 0}, {"jpeg": b"test"}

        with patch.object(camera, "capture", capture):
            task = asyncio.create_task(camera.snapshot_for_pilot(None))
            try:
                for _ in range(100):
                    if started.is_set():
                        break
                    await asyncio.sleep(.01)
                self.assertTrue(started.is_set())
                self.assertFalse(task.done())
            finally:
                release.set()
            image = await asyncio.wait_for(task, timeout=2)
            self.assertEqual(image.data, b"test")


if __name__ == "__main__":
    unittest.main()
