# SPDX-License-Identifier: MulanPSL-2.0
import re
import contextlib
import io
import unittest
from pathlib import Path
from unittest.mock import patch

import verify_scene_map_persistence as verifier

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


class PersistenceCleanupTests(unittest.TestCase):
    def _cleanup(self, replacement_load, *, binding_id="loaded_a-after"):
        calls = []
        active = "loaded_a"

        def http_json(method, url, payload=None, timeout=60.0):
            nonlocal active
            operation = url.rsplit("/", 1)[-1]
            target = (payload or {}).get("map_id")
            calls.append((method, operation, target))
            if operation == "delete":
                if len(calls) == 1:
                    raise RuntimeError("HTTP 409: map loaded_a is in use")
                return {"ok": active != target}
            if operation == "save":
                # A snapshot does not change the mapping service's live identity.
                return {"ok": True, "map_id": target}
            if operation == "load":
                if replacement_load.get("ok"):
                    active = target
                return replacement_load
            if operation == "state":
                return {"map_binding": {"map_id": binding_id, "mode": "localization"}}
            raise AssertionError((method, url, payload))

        artifact = {"exists": True, "engine": "slam_toolbox", "graph_exists": True,
                    "artifact_size": 1_000_000, "preview_exists": True,
                    "preview_width": 8, "preview_height": 8,
                    "meta": {"width": 8, "height": 8}}
        argv = ["verify_scene_map_persistence.py", "--map-id", "loaded_a",
                "--skip-save", "--skip-load", "--delete-after"]
        with patch.object(verifier, "http_json", side_effect=http_json), \
                patch.object(verifier, "mapping_artifact", return_value=artifact), \
                patch("sys.argv", argv), contextlib.redirect_stdout(io.StringIO()):
            status = verifier.main()
        return status, calls, active

    def test_cleanup_loads_saved_replacement_before_deleting_original(self):
        loaded = {"ok": True, "occupancy": {"count": 2, "stamp_unix": 123.0,
                                            "width": 8, "height": 8}}
        status, calls, active = self._cleanup(loaded)
        self.assertEqual(status, 0)
        self.assertEqual(active, "loaded_a-after")
        self.assertEqual(calls, [
            ("POST", "delete", "loaded_a"),
            ("POST", "save", "loaded_a-after"),
            ("POST", "load", "loaded_a-after"),
            ("GET", "state", None),
            ("POST", "delete", "loaded_a"),
        ])

    def test_cleanup_keeps_original_on_failed_or_incomplete_activation(self):
        good_grid = {"count": 2, "stamp_unix": 123.0, "width": 8, "height": 8}
        cases = [({"ok": False}, "loaded_a-after"),
                 ({"ok": True}, "loaded_a-after"),
                 ({"ok": True, "occupancy": {**good_grid, "count": 0}}, "loaded_a-after"),
                 ({"ok": True, "occupancy": good_grid}, "loaded_a")]
        for response, binding_id in cases:
            with self.subTest(response=response, binding_id=binding_id):
                status, calls, _ = self._cleanup(response, binding_id=binding_id)
                self.assertEqual(status, 1)
                self.assertIn(("POST", "load", "loaded_a-after"), calls)
                self.assertEqual(calls.count(("POST", "delete", "loaded_a")), 1)


if __name__ == "__main__":
    unittest.main()
