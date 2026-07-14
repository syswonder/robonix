import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parent


def entries(document, section):
    return {entry["name"]: entry for entry in document.get(section, [])}


class WebotsDeployConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.document = yaml.safe_load((ROOT / "robonix_manifest.yaml").read_text())

    def test_removed_defaults_do_not_return(self):
        self.assertNotIn("log", self.document["system"]["pilot"])
        self.assertNotIn("api_format", self.document["system"]["pilot"]["vlm"])
        self.assertNotIn("handsfree_enabled", self.document["system"]["liaison"])
        primitive = entries(self.document, "primitive")
        self.assertEqual(primitive["tiago_chassis"]["config"], {})
        self.assertEqual(primitive["audio_client_bridge"]["config"], {})

    def test_mapping_uses_deploy_owned_file(self):
        mapping = entries(self.document, "service")["mapping"]["config"]
        self.assertEqual(mapping["params_file"], "config/rtabmap_params.yaml")
        self.assertNotIn("rtabmap_profile", mapping)
        self.assertNotIn("rtabmap_params", mapping)
        params = yaml.safe_load((ROOT / mapping["params_file"]).read_text())
        self.assertEqual(params["Reg/Strategy"], 0)
        self.assertEqual(params["Rtabmap/DetectionRate"], 5.0)

    def test_navigation_uses_deploy_owned_file(self):
        navigation = entries(self.document, "service")["nav2"]["config"]
        self.assertEqual(navigation["params_file"], "config/nav2_params.yaml")
        self.assertNotIn("params_profile", navigation)
        self.assertTrue((ROOT / navigation["params_file"]).is_file())

    def test_vitals_has_a_non_conflicting_external_listener(self):
        self.assertEqual(self.document["system"]["vitals"]["listen"], "0.0.0.0:50093")
        self.assertNotEqual(
            self.document["system"]["vitals"]["listen"],
            self.document["system"]["soma"].get("listen", "127.0.0.1:50091"),
        )

    def test_documented_config_specs_exist(self):
        for relative in (
            "primitives/audio_client_bridge/config.spec",
            "primitives/audio_driver/config.spec",
            "primitives/tiago_camera/config.spec",
            "primitives/tiago_chassis/config.spec",
            "primitives/tiago_lidar/config.spec",
        ):
            self.assertTrue((ROOT / relative).is_file(), relative)


if __name__ == "__main__":
    unittest.main()
