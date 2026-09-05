# SPDX-License-Identifier: MulanPSL-2.0
import re
import unittest
from pathlib import Path

SOURCE = Path(__file__).resolve().parent / "verify_scene_map_persistence.py"


class EmbeddedProbeTests(unittest.TestCase):
    """The /map probe is a string executed in the mapping container, so a
    syntax or name error in it only surfaces mid-run on the GPU runner."""

    def test_probe_compiles_and_defines_its_callback(self):
        text = SOURCE.read_text(encoding="utf-8")
        code = re.search(r'def live_map.*?code = r"""(.*?)"""', text, re.S)
        self.assertIsNotNone(code, "live_map probe source not found")
        body = code.group(1)
        compile(body, "<live_map probe>", "exec")
        self.assertIn("def cb(", body)
        self.assertIn("create_subscription", body)


if __name__ == "__main__":
    unittest.main()
