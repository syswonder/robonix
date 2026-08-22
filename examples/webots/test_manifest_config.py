import os
import subprocess
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
        self.assertTrue((ROOT / mapping["params_file"]).is_file())

    def test_navigation_uses_deploy_owned_file(self):
        navigation = entries(self.document, "service")["nav2"]["config"]
        self.assertEqual(navigation["params_file"], "config/nav2_params.yaml")
        self.assertNotIn("params_profile", navigation)
        self.assertTrue((ROOT / navigation["params_file"]).is_file())

    def test_vitals_has_a_non_conflicting_loopback_listener(self):
        # The merge follows #188's provider-loopback direction: vitals binds
        # 127.0.0.1, not dev-next's 0.0.0.0.
        self.assertEqual(self.document["system"]["vitals"]["listen"], "127.0.0.1:50093")
        self.assertNotEqual(
            self.document["system"]["vitals"]["listen"],
            self.document["system"]["soma"].get("listen", "127.0.0.1:50091"),
        )

    def test_documented_config_specs_exist(self):
        for relative in (
            "primitives/tiago_camera/config.spec",
            "primitives/tiago_chassis/config.spec",
            "primitives/tiago_lidar/config.spec",
        ):
            self.assertTrue((ROOT / relative).is_file(), relative)

    def test_lidar_manifest_advertises_its_stream_contract(self):
        package = yaml.safe_load(
            (
                ROOT / "primitives" / "tiago_lidar" / "package_manifest.yaml"
            ).read_text()
        )
        advertised = {item["name"] for item in package["capabilities"]}
        self.assertIn("robonix/primitive/lidar/lidar", advertised)
        driver = (
            ROOT / "primitives" / "tiago_lidar" / "lidar_driver" / "driver.py"
        ).read_text()
        self.assertIn(
            'declare_ros2_topic("robonix/primitive/lidar/lidar"',
            driver,
        )

    def test_current_primitive_examples_use_canonical_implicit_shared_driver(self):
        for name in ("tiago_chassis", "tiago_camera", "tiago_lidar"):
            manifest = yaml.safe_load(
                (ROOT / "primitives" / name / "package_manifest.yaml").read_text()
            )
            drivers = {
                entry["name"]
                for entry in manifest.get("capabilities", [])
                if entry["name"].endswith("/driver")
            }
            self.assertEqual(drivers, set(), name)

    def test_audio_primitives_use_reusable_packages(self):
        primitive = entries(self.document, "primitive")
        expected = {
            "audio_driver": "https://github.com/syswonder/primitive-audio-driver-rbnx",
            "audio_client_bridge": (
                "https://github.com/syswonder/primitive-audio-client-bridge-rbnx"
            ),
        }
        for name, url in expected.items():
            self.assertEqual(primitive[name]["url"], url)
            self.assertEqual(primitive[name]["branch"], "main")
            self.assertNotIn("path", primitive[name])

    def test_primitive_builds_do_not_touch_a_running_simulator(self):
        for name in ("tiago_chassis", "tiago_camera", "tiago_lidar"):
            package = ROOT / "primitives" / name
            build = (package / "scripts/build.sh").read_text()
            start = (package / "scripts/start.sh").read_text()
            self.assertNotIn("docker exec", build, name)
            self.assertNotIn("robonix_tiago_sim", build, name)
            self.assertNotIn("--ros2", build, name)
            self.assertNotIn("ros2_idl/install/setup.bash", start, name)
            self.assertIn("run_python_codegen.sh", build, name)

    def test_bridge_network_drivers_translate_host_loopback(self):
        helper = (ROOT / "scripts" / "container_network.sh").read_text()
        self.assertIn("resolve_container_atlas_endpoint", helper)
        self.assertIn(".NetworkSettings.Networks", helper)
        self.assertIn("return 1", helper)
        for name in ("tiago_chassis", "tiago_camera", "tiago_lidar"):
            start = (ROOT / "primitives" / name / "scripts" / "start.sh").read_text()
            self.assertIn('source "$WEBOTS_SCRIPTS/container_network.sh"', start)
            self.assertIn('ROBONIX_ATLAS="$ATLAS_ENDPOINT"', start)
            self.assertIn(
                'ROBONIX_DRIVER_CONTRACT_ID="${ROBONIX_DRIVER_CONTRACT_ID-', start
            )
            self.assertIn("ROBONIX_DRIVER_ALLOW_OLD_ARTIFACT_FALLBACK", start)
            self.assertIn("robonix/lifecycle/driver", start)
            self.assertIn('append_no_proxy_hosts', start)
            self.assertIn('-e NO_PROXY="$NO_PROXY_VALUE"', start)
            self.assertIn('-e no_proxy="$no_proxy_value"', start)

    def test_container_network_policy_regressions(self):
        script = ROOT / "tests" / "test_container_network.sh"
        self.assertTrue(os.access(script, os.X_OK), f"not executable: {script}")
        completed = subprocess.run(
            [str(script)],
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(
            completed.returncode,
            0,
            f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}",
        )

    def test_retained_simple_nav_uses_shared_container_helpers(self):
        package = ROOT / "services" / "simple_nav"
        manifest = yaml.safe_load((package / "package_manifest.yaml").read_text())
        build = (package / "scripts" / "build.sh").read_text()
        start = (package / "scripts" / "start.sh").read_text()
        capability_names = {entry["name"] for entry in manifest.get("capabilities", [])}
        self.assertNotIn("robonix/lifecycle/driver", capability_names)
        self.assertIn("run_python_codegen.sh", build)
        self.assertIn("--mcp --ros2", build)
        self.assertIn('source "$WEBOTS_SCRIPTS/container_network.sh"', start)
        self.assertIn('resolve_container_atlas_endpoint "$SIM_CT"', start)
        self.assertIn('ROBONIX_ATLAS="$ATLAS_ENDPOINT"', start)
        self.assertIn(
            'ROBONIX_DRIVER_CONTRACT_ID="${ROBONIX_DRIVER_CONTRACT_ID-', start
        )
        self.assertIn("ROBONIX_DRIVER_ALLOW_OLD_ARTIFACT_FALLBACK", start)

    def test_sim_prepares_soma_runtime_mount_before_compose(self):
        launcher = (ROOT / "sim" / "start.sh").read_text()
        mkdir = launcher.index('mkdir -p "$SOMA_RUNTIME_DIR"')
        compose_up = launcher.index('"${DC[@]}" up --build -d')
        self.assertLess(mkdir, compose_up)
        self.assertIn('[[ ! -w "$SOMA_RUNTIME_DIR" ]]', launcher)

    def test_sim_does_not_write_root_owned_python_cache_to_source(self):
        compose = yaml.safe_load((ROOT / "sim" / "compose.yaml").read_text())
        environment = compose["services"]["sim"]["environment"]
        self.assertEqual(environment["PYTHONDONTWRITEBYTECODE"], "1")

    def test_bridge_network_publishes_the_zenoh_router(self):
        launcher = (ROOT / "sim" / "start.sh").read_text()
        self.assertIn('CF+=(-f compose.bridge.yaml)', launcher)
        self.assertIn('ROBONIX_SIM_ZENOH_PORT="${ROBONIX_SIM_ZENOH_PORT:-7447}"', launcher)
        expected_router = (
            "ROBONIX_ZENOH_ROUTER=tcp/127.0.0.1:${ROBONIX_SIM_ZENOH_PORT}"
        )
        self.assertIn(expected_router, launcher)
        bridge = yaml.safe_load((ROOT / "sim" / "compose.bridge.yaml").read_text())
        self.assertEqual(
            bridge["services"]["sim"]["ports"],
            ["127.0.0.1:${ROBONIX_SIM_ZENOH_PORT:-7447}:7447"],
        )

if __name__ == "__main__":
    unittest.main()
