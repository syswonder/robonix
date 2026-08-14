#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0

"""Tests for deterministic TIAGo world selection."""

from pathlib import Path
import tempfile
import unittest

from eaios_webots.world_variant import detect_variant, materialize_world, render_variant


LITE_WORLD = """#VRML_SIM R2025a utf8
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/pal_robotics/tiago_lite/protos/TiagoLite.proto"
TiagoLite {
  name "my_robot"
  controller "<extern>"
}
"""


class WorldVariantTests(unittest.TestCase):
    def test_all_bundled_worlds_support_full_variant(self):
        """Every shipped world has one transformable TIAGo Lite declaration."""
        worlds_dir = Path(__file__).resolve().parents[1] / "worlds"
        world_paths = sorted(worlds_dir.glob("*.wbt"))
        self.assertEqual(len(world_paths), 5)
        for world_path in world_paths:
            with self.subTest(world=world_path.name):
                world = world_path.read_text(encoding="utf-8")
                self.assertEqual(detect_variant(world), "lite")
                self.assertEqual(detect_variant(render_variant(world, "full")), "full")

    def test_full_variant_changes_only_model_markers(self):
        """Full selection preserves robot fields while switching its official PROTO."""
        rendered = render_variant(LITE_WORLD, "full")
        self.assertEqual(detect_variant(rendered), "full")
        self.assertIn("pal_robotics/tiago/protos/Tiago.proto", rendered)
        self.assertIn('name "my_robot"', rendered)
        self.assertIn('controller "<extern>"', rendered)
        self.assertNotIn("TiagoLite", rendered)

    def test_matching_variant_returns_source_verbatim(self):
        """Lite selection does not rewrite or reformat an already-lite world."""
        self.assertEqual(render_variant(LITE_WORLD, "lite"), LITE_WORLD)

    def test_ambiguous_world_is_rejected(self):
        """A second TIAGo node fails instead of selecting an arbitrary robot."""
        ambiguous = LITE_WORLD + "\nTiagoLite {\n}\n"
        with self.assertRaisesRegex(ValueError, "inconsistent model markers"):
            render_variant(ambiguous, "full")

    def test_materialized_world_is_a_sibling(self):
        """Generated worlds retain the source directory for relative assets."""
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory, "office.wbt")
            source.write_text(LITE_WORLD, encoding="utf-8")
            output = materialize_world(source, "full")
            self.assertEqual(output.parent, source.parent.resolve())
            self.assertEqual(detect_variant(output.read_text(encoding="utf-8")), "full")


if __name__ == "__main__":
    unittest.main()
