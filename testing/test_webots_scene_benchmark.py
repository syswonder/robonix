# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import json
import math
import tempfile
import unittest
import base64
import io
import zlib
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image

from export_webots_scene_truth import _truth_payload
from evaluate_webots_occupancy import (
    Segment,
    _evaluate_loaded_grid,
    _load_scene_state_grid,
)
from evaluate_webots_scene import _load_label_judgments
from estimate_webots_explore_budget import estimate_exploration_budget_s
from freeze_webots_scene_visibility import freeze_common_visibility
from render_webots_scene_report import (
    _box_from_debug_candidate,
    _costmap_layer_payload,
    _invalid_world_payload,
    _occupancy_plane_payload,
    _saved_map_payload,
    _world_payload,
)
from run_webots_explore import _explore_endpoints
from scene_quality_ground_truth import (
    SemanticObjectGroundTruth,
    _semantic_size_bucket,
    evaluate_semantic_inventory,
    load_semantic_inventory,
    transform_semantic_inventory_planar,
)
from webots_scene_motion import (
    OdometryContinuity,
    SynchronizedPoseAgreement,
    choose_motion,
    odometry_path_agreement,
    publish_stop_if_available,
    sector_clearance_m,
)
from webots_scene_visibility import rgbd_visibility_evidence

ROOT = Path(__file__).resolve().parents[1]
BENCHMARK = ROOT / "testing/fixtures/webots_scene_benchmark.json"


class WebotsSceneBenchmarkTests(unittest.TestCase):
    def test_common_visibility_preserves_each_trial_alignment(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            first = root / "first" / "visibility.json"
            second = root / "second" / "visibility.json"
            first.parent.mkdir()
            second.parent.mkdir()
            first.write_text(
                json.dumps(
                    {
                        "visible_truth_ids": ["a", "b", "c"],
                        "truth_alignment": {"translation_m": [1, 0, 0]},
                    }
                ),
                encoding="utf-8",
            )
            second.write_text(
                json.dumps(
                    {
                        "visible_truth_ids": ["b", "c", "d"],
                        "truth_alignment": {"translation_m": [2, 0, 0]},
                    }
                ),
                encoding="utf-8",
            )

            summary = freeze_common_visibility([first, second])
            frozen_first = json.loads(
                first.with_name("visibility.common.json").read_text()
            )
            frozen_second = json.loads(
                second.with_name("visibility.common.json").read_text()
            )

        self.assertEqual(summary["visible_truth_ids"], ["b", "c"])
        self.assertEqual(frozen_first["visible_truth_ids"], ["b", "c"])
        self.assertEqual(frozen_second["visible_truth_ids"], ["b", "c"])
        self.assertEqual(
            frozen_first["truth_alignment"]["translation_m"],
            [1, 0, 0],
        )
        self.assertEqual(
            frozen_second["truth_alignment"]["translation_m"],
            [2, 0, 0],
        )

    def test_explore_budget_is_wbt_derived_and_bounded(self) -> None:
        compact = {
            "truths": [
                {"center_m": [0.0, 0.0, 0.0]},
                {"center_m": [3.0, 4.0, 0.0]},
            ]
        }
        self.assertEqual(
            estimate_exploration_budget_s(compact, max_speed_m_s=0.12),
            180,
        )

        large = {
            "truths": [
                {"center_m": [0.0, 0.0, 0.0]},
                *(
                    {"center_m": [12.0, 16.0, 0.0]}
                    for _ in range(99)
                ),
            ]
        }
        self.assertEqual(
            estimate_exploration_budget_s(large, max_speed_m_s=0.10),
            375,
        )
        self.assertEqual(
            estimate_exploration_budget_s(
                large,
                max_speed_m_s=0.01,
                maximum_s=420,
            ),
            420,
        )

    def test_explore_budget_rejects_invalid_speed(self) -> None:
        with self.assertRaisesRegex(ValueError, "max_speed_m_s"):
            estimate_exploration_budget_s(
                {"truths": []},
                max_speed_m_s=0.0,
            )

    def test_occupancy_error_is_measured_to_wall_surface(self) -> None:
        samples_x = np.linspace(-1.0, 1.0, 81)
        occupied = np.column_stack(
            (samples_x, np.full_like(samples_x, 0.10))
        )
        result = _evaluate_loaded_grid(
            np.zeros((4, 4), dtype=np.uint8),
            occupied,
            {"resolution": 0.05, "origin": [0.0, 0.0, 0.0]},
            [
                Segment(
                    name="wall",
                    start=(-1.0, 0.0),
                    end=(1.0, 0.0),
                    thickness_m=0.20,
                )
            ],
        )
        self.assertAlmostEqual(result["supported_wall_median_error_m"], 0.0)
        self.assertAlmostEqual(result["wall_coverage_0_10"], 1.0)
        self.assertLess(result["wall_angle_median_error_deg"], 0.01)

    def test_scene_state_occupancy_preserves_origin_yaw(self) -> None:
        pixels = np.asarray([[20, 128], [240, 240]], dtype=np.uint8)
        encoded = io.BytesIO()
        Image.fromarray(pixels).save(encoded, format="PNG")
        state = {
            "occupancy": {
                "width": 2,
                "height": 2,
                "resolution": 0.5,
                "origin_x": 1.0,
                "origin_y": 2.0,
                "origin_yaw": math.pi / 2.0,
                "png_b64": base64.b64encode(encoded.getvalue()).decode("ascii"),
            }
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.json"
            path.write_text(json.dumps(state), encoding="utf-8")
            image, occupied, metadata = _load_scene_state_grid(path)
        self.assertEqual(image.shape, (2, 2))
        self.assertEqual(metadata["origin"][2], math.pi / 2.0)
        self.assertEqual(occupied.shape, (1, 2))
        # Top-left occupied cell is local (0.25, 0.75). A +90 degree grid
        # origin rotation maps it to world (1 - 0.75, 2 + 0.25).
        np.testing.assert_allclose(occupied[0], (0.25, 2.25), atol=1e-9)

    def test_odometry_continuity_accepts_sparse_timestamped_motion(self) -> None:
        track = OdometryContinuity(
            max_step_gate_m=0.25,
            max_speed_gate_mps=0.60,
        )
        self.assertFalse(
            track.observe(stamp_s=1.0, receipt_s=10.0, x_m=0.0, y_m=0.0)
        )
        self.assertFalse(
            track.observe(stamp_s=4.0, receipt_s=13.0, x_m=0.59, y_m=0.0)
        )
        self.assertAlmostEqual(track.path_length_m, 0.59)
        self.assertEqual(len(track.large_steps), 1)
        self.assertEqual(track.discontinuities, [])
        self.assertAlmostEqual(track.large_steps[0]["speed_mps"], 0.196667)

    def test_odometry_continuity_rejects_implausible_stamped_jump(self) -> None:
        track = OdometryContinuity(
            max_step_gate_m=0.25,
            max_speed_gate_mps=0.60,
        )
        track.observe(stamp_s=1.0, receipt_s=10.0, x_m=0.0, y_m=0.0)
        self.assertTrue(
            track.observe(stamp_s=1.02, receipt_s=10.1, x_m=0.59, y_m=0.0)
        )
        self.assertEqual(track.path_length_m, 0.0)
        self.assertEqual(len(track.discontinuities), 1)
        self.assertGreater(track.discontinuities[0]["speed_mps"], 20.0)

    def test_odometry_path_agreement_accepts_independent_tracks(self) -> None:
        result = odometry_path_agreement(
            {
                "ground_truth": {
                    "sample_count": 100,
                    "path_length_m": 13.8787,
                },
                "wheel": {
                    "sample_count": 98,
                    "path_length_m": 13.8181,
                },
                "fused": {
                    "sample_count": 99,
                    "path_length_m": 13.8148,
                },
            }
        )
        self.assertTrue(result["valid"])
        self.assertEqual(result["failures"], [])
        self.assertLess(
            result["comparisons"]["fused"]["relative_error"],
            0.01,
        )

    def test_odometry_path_agreement_rejects_smooth_scale_drift(self) -> None:
        result = odometry_path_agreement(
            {
                "ground_truth": {
                    "sample_count": 100,
                    "path_length_m": 10.40,
                },
                "wheel": {
                    "sample_count": 98,
                    "path_length_m": 12.65,
                },
                "fused": {
                    "sample_count": 99,
                    "path_length_m": 12.64,
                },
            }
        )
        self.assertFalse(result["valid"])
        self.assertGreater(
            result["comparisons"]["fused"]["relative_error"],
            0.20,
        )
        self.assertIn("differs from ground_truth", result["failures"][0])

    def test_scene_frame_gate_keeps_wheel_slip_as_diagnostic(self) -> None:
        result = odometry_path_agreement(
            {
                "ground_truth": {
                    "sample_count": 100,
                    "path_length_m": 10.40,
                },
                "localized": {
                    "sample_count": 99,
                    "path_length_m": 10.31,
                },
                "wheel": {
                    "sample_count": 98,
                    "path_length_m": 12.65,
                },
                "fused": {
                    "sample_count": 99,
                    "path_length_m": 12.64,
                },
            },
            compared_sources=("localized", "wheel", "fused"),
            required_sources=("localized",),
        )
        self.assertTrue(result["valid"])
        self.assertEqual(result["failures"], [])
        self.assertEqual(result["required_sources"], ["localized"])
        self.assertTrue(result["comparisons"]["localized"]["required"])
        self.assertFalse(result["comparisons"]["wheel"]["required"])
        self.assertEqual(len(result["warnings"]), 2)
        self.assertTrue(
            all("differs from ground_truth" in item for item in result["warnings"])
        )

    def test_synchronized_pose_agreement_uses_relative_motion(self) -> None:
        agreement = SynchronizedPoseAgreement(max_time_delta_s=0.10)
        self.assertTrue(
            agreement.observe(
                localized_stamp_s=1.00,
                localized_x_m=10.0,
                localized_y_m=20.0,
                localized_yaw_rad=0.20,
                reference_stamp_s=1.02,
                reference_x_m=0.0,
                reference_y_m=0.0,
                reference_yaw_rad=0.10,
            )
        )
        self.assertTrue(
            agreement.observe(
                localized_stamp_s=2.00,
                localized_x_m=10.0 + math.cos(0.20),
                localized_y_m=20.0 + math.sin(0.20),
                localized_yaw_rad=0.30,
                reference_stamp_s=2.01,
                reference_x_m=math.cos(0.10),
                reference_y_m=math.sin(0.10),
                reference_yaw_rad=0.20,
            )
        )
        self.assertFalse(
            agreement.observe(
                localized_stamp_s=3.00,
                localized_x_m=12.0,
                localized_y_m=20.0,
                localized_yaw_rad=0.40,
                reference_stamp_s=3.30,
                reference_x_m=2.0,
                reference_y_m=0.0,
                reference_yaw_rad=0.30,
            )
        )
        result = agreement.as_dict()
        self.assertEqual(result["sample_count"], 2)
        self.assertEqual(result["rejected_time_pairs"], 1)
        self.assertLess(result["translation_error_m"]["p95"], 1e-6)
        self.assertLess(result["yaw_error_rad"]["p95"], 1e-6)

    def test_visibility_sweep_drains_ros_callbacks_in_background(self) -> None:
        source = (
            ROOT / "testing/run_webots_scene_sweep.py"
        ).read_text(encoding="utf-8")
        self.assertIn("MultiThreadedExecutor(num_threads=2)", source)
        self.assertIn('name="webots-scene-sweep-executor"', source)
        self.assertNotIn("rclpy.spin_once(", source)
        self.assertIn(
            'for name in ("ground_truth", "localized", "wheel", "fused")',
            source,
        )
        self.assertIn('odometry_tracks["localized"].observe(', source)
        self.assertIn("localized_pose_agreement.observe(", source)
        self.assertIn('"localized_pose_agreement"', source)
        self.assertIn('"localized_transform_failures"', source)
        self.assertIn('"odometry_abort_source": odometry_abort_source', source)

    def test_rgbd_visibility_requires_bbox_area_and_depth_fraction(self) -> None:
        camera = np.eye(4, dtype=float)
        full_depth = np.full((100, 100), 2.0, dtype=np.float32)
        visible = rgbd_visibility_evidence(
            depth_m=full_depth,
            center_m=(0.0, 0.0, 2.0),
            size_m=(0.8, 0.8, 0.8),
            yaw_rad=0.0,
            map_to_camera=camera,
            fx=100.0,
            fy=100.0,
            cx=50.0,
            cy=50.0,
        )
        self.assertTrue(visible.visible)
        self.assertGreaterEqual(
            visible.consistent_depth_pixels,
            visible.required_depth_pixels,
        )

        sparse_depth = np.full((100, 100), np.nan, dtype=np.float32)
        sparse_depth[49:50, 49:52] = 2.0
        sparse = rgbd_visibility_evidence(
            depth_m=sparse_depth,
            center_m=(0.0, 0.0, 2.0),
            size_m=(0.8, 0.8, 0.8),
            yaw_rad=0.0,
            map_to_camera=camera,
            fx=100.0,
            fy=100.0,
            cx=50.0,
            cy=50.0,
        )
        self.assertFalse(sparse.visible)
        self.assertEqual(sparse.reason, "insufficient_depth_support")
        self.assertEqual(sparse.consistent_depth_pixels, 3)

    def test_rgbd_visibility_rejects_occluder_and_tiny_projection(self) -> None:
        camera = np.eye(4, dtype=float)
        occluded = rgbd_visibility_evidence(
            depth_m=np.full((100, 100), 0.5, dtype=np.float32),
            center_m=(0.0, 0.0, 2.0),
            size_m=(0.8, 0.8, 0.8),
            yaw_rad=0.4,
            map_to_camera=camera,
            fx=100.0,
            fy=100.0,
            cx=50.0,
            cy=50.0,
        )
        self.assertFalse(occluded.visible)
        self.assertEqual(occluded.consistent_depth_pixels, 0)

        tiny = rgbd_visibility_evidence(
            depth_m=np.full((100, 100), 5.0, dtype=np.float32),
            center_m=(0.0, 0.0, 5.0),
            size_m=(0.02, 0.02, 0.02),
            yaw_rad=0.0,
            map_to_camera=camera,
            fx=100.0,
            fy=100.0,
            cx=50.0,
            cy=50.0,
        )
        self.assertFalse(tiny.visible)
        self.assertEqual(tiny.reason, "projected_area_too_small")

    def test_passive_sweep_stop_does_not_require_velocity_publisher(self) -> None:
        publish_stop_if_available(None, lambda: object())

        class Publisher:
            def __init__(self) -> None:
                self.messages: list[object] = []

            def publish(self, message: object) -> None:
                self.messages.append(message)

        publisher = Publisher()
        marker = object()
        publish_stop_if_available(publisher, lambda: marker)
        self.assertEqual(publisher.messages, [marker])

    def test_explore_helper_requires_standard_driver_and_skill_endpoints(
        self,
    ) -> None:
        driver, mcp = _explore_endpoints(
            {
                "providers": {
                    "explore": {
                        "endpoints": [
                            {
                                "contract_id": "robonix/lifecycle/driver",
                                "endpoint": "127.0.0.1:50000",
                            },
                            {
                                "contract_id": "robonix/skill/explore/explore",
                                "endpoint": "http://127.0.0.1:50001/mcp/",
                            },
                        ]
                    }
                }
            }
        )
        self.assertEqual(driver, "127.0.0.1:50000")
        self.assertEqual(mcp, "http://127.0.0.1:50001/mcp/")

    def test_benchmark_waits_for_complete_explore_endpoints(self) -> None:
        script = (ROOT / "testing/run_webots_scene_benchmark.sh").read_text()
        wait_body = script.split("wait_for_explore() {", 1)[1].split(
            "\n}\n", 1
        )[0]
        self.assertIn('"$EXPLORE_HELPER" ready', wait_body)
        self.assertNotIn("provider_id", wait_body)

    def test_benchmark_copies_pure_visibility_helper_into_simulator(self) -> None:
        script = (ROOT / "testing/run_webots_scene_benchmark.sh").read_text()
        self.assertIn(
            'docker cp "$REPO_ROOT/testing/webots_scene_visibility.py"',
            script,
        )
        self.assertIn(
            '"$SIM_CONTAINER:/tmp/webots_scene_visibility.py"',
            script,
        )

    def test_motion_policy_scans_at_normal_kitchen_start_clearance(self) -> None:
        decision = choose_motion(
            elapsed_s=5.0,
            front_m=0.55,
            left_m=0.55,
            right_m=0.74,
            rear_m=float("nan"),
            linear_speed_mps=0.10,
            angular_speed_rps=0.18,
        )
        self.assertEqual(decision.mode, "scan")
        self.assertEqual(decision.linear_x_mps, 0.0)
        self.assertEqual(decision.angular_z_rps, 0.18)

    def test_sector_treats_all_positive_infinity_as_observed_clear(self) -> None:
        clearance = sector_clearance_m(
            ranges=[float("inf")] * 5,
            angle_min=-0.4,
            angle_increment=0.2,
            range_min=0.10,
            range_max=5.0,
            low=-0.42,
            high=0.42,
        )
        self.assertEqual(clearance, 5.0)

    def test_sector_keeps_invalid_or_missing_samples_unavailable(self) -> None:
        common = {
            "angle_min": -0.4,
            "angle_increment": 0.2,
            "range_min": 0.10,
            "range_max": 5.0,
            "low": -0.42,
            "high": 0.42,
        }
        self.assertTrue(
            math.isnan(sector_clearance_m(ranges=[], **common))
        )
        self.assertTrue(
            math.isnan(
                sector_clearance_m(ranges=[float("nan")] * 5, **common)
            )
        )

    def test_sector_prefers_nearest_finite_hit_over_clear_rays(self) -> None:
        clearance = sector_clearance_m(
            ranges=[float("inf"), 1.2, 0.7, 2.0, float("inf")],
            angle_min=-0.4,
            angle_increment=0.2,
            range_min=0.10,
            range_max=5.0,
            low=-0.42,
            high=0.42,
        )
        self.assertEqual(clearance, 0.7)

    def test_motion_policy_does_not_deadlock_in_safe_narrow_corridor(self) -> None:
        decision = choose_motion(
            elapsed_s=100.0,
            front_m=0.77,
            left_m=0.45,
            right_m=0.48,
            rear_m=0.60,
            linear_speed_mps=0.14,
            angular_speed_rps=0.40,
        )
        self.assertEqual(decision.mode, "narrow_corridor_forward")
        self.assertGreater(decision.linear_x_mps, 0.0)
        self.assertEqual(decision.angular_z_rps, 0.0)

    def test_motion_policy_escapes_front_and_rear_obstacles_along_safe_axis(
        self,
    ) -> None:
        front = choose_motion(
            elapsed_s=100.0,
            front_m=0.40,
            left_m=0.50,
            right_m=0.50,
            rear_m=0.80,
            linear_speed_mps=0.14,
            angular_speed_rps=0.40,
        )
        self.assertEqual(front.mode, "front_blocked_reverse")
        self.assertLess(front.linear_x_mps, 0.0)

        rear = choose_motion(
            elapsed_s=100.0,
            front_m=0.80,
            left_m=0.50,
            right_m=0.50,
            rear_m=0.40,
            linear_speed_mps=0.14,
            angular_speed_rps=0.40,
        )
        self.assertEqual(rear.mode, "rear_blocked_forward")
        self.assertGreater(rear.linear_x_mps, 0.0)

    def test_motion_policy_does_not_treat_unobserved_rear_as_free(self) -> None:
        decision = choose_motion(
            elapsed_s=100.0,
            front_m=0.40,
            left_m=0.80,
            right_m=0.45,
            rear_m=float("nan"),
            linear_speed_mps=0.14,
            angular_speed_rps=0.40,
        )
        self.assertEqual(decision.mode, "front_blocked_turn")
        self.assertEqual(decision.linear_x_mps, 0.0)

    def test_motion_policy_breaks_safe_in_place_turn_oscillation(self) -> None:
        decision = choose_motion(
            elapsed_s=200.0,
            front_m=0.84,
            left_m=0.78,
            right_m=0.54,
            rear_m=float("nan"),
            linear_speed_mps=0.14,
            angular_speed_rps=0.40,
            stalled_s=9.0,
        )
        self.assertEqual(decision.mode, "stalled_safe_forward")
        self.assertGreater(decision.linear_x_mps, 0.0)
        self.assertEqual(decision.angular_z_rps, 0.0)

    def test_motion_policy_commits_to_one_turn_when_front_is_too_close(
        self,
    ) -> None:
        first = choose_motion(
            elapsed_s=200.0,
            front_m=0.60,
            left_m=0.58,
            right_m=0.55,
            rear_m=float("nan"),
            linear_speed_mps=0.14,
            angular_speed_rps=0.40,
            stalled_s=9.0,
            stalled_turn_sign=1.0,
        )
        flipped_sectors = choose_motion(
            elapsed_s=201.0,
            front_m=0.64,
            left_m=0.56,
            right_m=0.57,
            rear_m=float("nan"),
            linear_speed_mps=0.14,
            angular_speed_rps=0.40,
            stalled_s=10.0,
            stalled_turn_sign=1.0,
        )
        self.assertEqual(first.mode, "stalled_committed_turn")
        self.assertEqual(flipped_sectors.mode, "stalled_committed_turn")
        self.assertGreater(first.angular_z_rps, 0.0)
        self.assertGreater(flipped_sectors.angular_z_rps, 0.0)

    def test_occupancy_plane_keeps_unknown_transparent(self) -> None:
        from PIL import Image
        import io

        image = Image.new("L", (4, 4), color=205)
        image.putpixel((0, 3), 0)
        for y in range(2):
            for x in range(2, 4):
                image.putpixel((x, y), 240)
        encoded = io.BytesIO()
        image.save(encoded, format="PNG")
        plane = _occupancy_plane_payload(
            {
                "width": 4,
                "height": 4,
                "resolution": 0.5,
                "origin_x": 1.0,
                "origin_y": -2.0,
                "origin_yaw": 0.0,
                "png_b64": base64.b64encode(encoded.getvalue()).decode("ascii"),
            }
        )
        self.assertEqual(plane["occupied"], [[1.25, -1.75]])
        self.assertEqual(plane["free"], [[2.5, -0.5, 1.0, 1.0]])
        self.assertEqual(len(plane["extent"]), 4)

    def test_perspective_camera_uses_right_handed_non_mirrored_basis(self) -> None:
        renderer = (
            ROOT / "testing/render_webots_scene_report.py"
        ).read_text(encoding="utf-8")
        self.assertIn("const rx=-cy*a+sy*b", renderer)
        self.assertNotIn("const rx=cy*a-sy*b", renderer)
        # Retain grab-to-drag behavior after correcting the horizontal basis.
        self.assertIn("v.yaw+=dx*.008", renderer)

    def test_report_viewers_support_deep_zoom_pan_and_decluttered_labels(
        self,
    ) -> None:
        renderer = (
            ROOT / "testing/render_webots_scene_report.py"
        ).read_text(encoding="utf-8")
        self.assertIn('labelMode:"hover"', renderer)
        self.assertIn('<option value="hover">Hover + selected</option>', renderer)
        self.assertIn('<option value="all">All</option>', renderer)
        self.assertIn("function placeLabel(", renderer)
        self.assertIn("const candidates=", renderer)
        self.assertIn("maxZoom:1e18", renderer)
        self.assertIn(
            "const closeZoom=2.25,focusZoom=1.8,"
            "fitObjectZoom=1.0,roomZoom=.96",
            renderer,
        )
        self.assertIn(
            "wheelDelta=Math.max(-240,Math.min(240,deviceDelta))", renderer
        )
        self.assertIn("old*Math.exp(-wheelDelta*.008)", renderer)
        self.assertIn("data-zoom-readout", renderer)
        self.assertIn("data-close-view", renderer)
        self.assertIn("data-fit-objects", renderer)
        self.assertIn("function roomExtent(", renderer)
        self.assertIn("function objectExtent(", renderer)
        self.assertIn(
            "zoom:closeZoom,minZoom:.03,maxZoom:1e18,zoomStep:2.0,"
            'panX:0,panY:0,target:null,focusRadius:null,frameMode:"objects"',
            renderer,
        )
        self.assertIn('v.frameMode="focus";v.zoom=focusZoom', renderer)
        self.assertIn(
            "zoom:1.25,minZoom:.05,maxZoom:1e9,zoomStep:2.2",
            renderer,
        )
        self.assertIn("Math.min(v.maxZoom,room.radius*.42/boxRadius)", renderer)
        self.assertIn("const cameraDistance=frameRadius*3.0", renderer)
        self.assertIn("perspective=focal*fitScale*view.zoom/depth", renderer)
        self.assertIn('data-fit-objects>Fit all objects</button>', renderer)
        self.assertIn('kind==="orbit"?"Fit room":"Fit map"', renderer)
        self.assertIn("v.panX=e.offsetX-canvas.clientWidth/2", renderer)
        self.assertIn("panning=e.button===2||e.shiftKey", renderer)
        self.assertIn('data-focus title="Focus the selected box"', renderer)
        self.assertIn("v.target=item.box.center.slice(0,2)", renderer)
        self.assertIn(
            "paint=draw2d(canvas,world,visibility,v,images)", renderer
        )
        self.assertIn("canvas.ondblclick", renderer)
        self.assertIn("v.panX+=dx;v.panY+=dy", renderer)
        self.assertIn("viewer.requestFullscreen?.()", renderer)
        self.assertIn("double-click a box to focus", renderer)
        self.assertIn('"Wall ≤10 cm"', renderer)
        self.assertIn("observed_wall_p95_error_m", renderer)
        self.assertIn("pending /", renderer)
        self.assertIn('"Candidates"', renderer)
        self.assertIn('candidate:"#376f9f"', renderer)
        self.assertIn("published_to_registry", renderer)

    def test_report_reconstructs_withheld_candidate_bbox(self) -> None:
        box = _box_from_debug_candidate(
            {
                "id": "raw-1",
                "cls": "window",
                "published_to_registry": False,
                "confirmation_unique_frames": 1,
                "confirmation_min_unique_frames": 2,
                "confirmation_mean_confidence": 0.42,
                "bbox_corners": [
                    [0.0, 0.0, 0.0],
                    [2.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.5],
                    [2.0, 1.0, 0.5],
                    [0.0, 1.0, 0.5],
                    [2.0, 0.0, 0.5],
                    [2.0, 1.0, 0.0],
                ],
            }
        )
        self.assertIsNotNone(box)
        self.assertEqual(box["id"], "candidate:raw-1")
        self.assertEqual(box["status"], "candidate")
        self.assertEqual(box["center"], [1.0, 0.5, 0.25])
        self.assertEqual(box["size"], [2.0, 1.0, 0.5])
        self.assertEqual(box["yaw"], 0.0)
        self.assertEqual(box["confirmation_unique_frames"], 1)

    def test_report_marks_partial_exploration_as_incomplete_acquisition(
        self,
    ) -> None:
        renderer = (
            ROOT / "testing/render_webots_scene_report.py"
        ).read_text(encoding="utf-8")
        self.assertIn("Incomplete acquisition:", renderer)
        self.assertIn("only the explored area, not the full world map", renderer)
        self.assertIn("world.coverage??0", renderer)

    def test_benchmark_preserves_explore_navigation_and_mapping_logs(
        self,
    ) -> None:
        benchmark = (
            ROOT / "testing/run_webots_scene_benchmark.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("capture_component_logs()", benchmark)
        self.assertIn("for component in explore nav2 mapping scene", benchmark)
        self.assertIn('capture_component_logs "$world_dir"', benchmark)
        self.assertIn("evaluate_webots_occupancy.py", benchmark)
        self.assertIn("ROBONIX_SCENE_MIN_VISIBLE_RATIO", benchmark)
        capture_index = benchmark.index('capture_component_logs "$world_dir"')
        self.assertLess(
            capture_index,
            benchmark.index("stop_one || true", capture_index),
        )
        self.assertIn(
            'DEFAULT_MANIFEST="$WEBOTS_DIR/robonix_manifest.mapping-nav-eval.yaml"',
            benchmark,
        )
        self.assertIn(
            'DEFAULT_MANIFEST="$WEBOTS_DIR/robonix_manifest.scene-eval.yaml"',
            benchmark,
        )

    def test_costmap_snapshot_becomes_transparent_review_layer(self) -> None:
        cells = bytes([0, 1, 99, 100, 255, 50])
        layer = _costmap_layer_payload(
            {
                "frame_id": "map",
                "source_frame_id": "map",
                "width": 3,
                "height": 2,
                "resolution": 0.05,
                "origin_x": -1.0,
                "origin_y": 2.0,
                "origin_yaw": 0.0,
                "encoding": "ros-occupancy-int8-zlib-base64",
                "data": base64.b64encode(zlib.compress(cells)).decode("ascii"),
            }
        )
        self.assertEqual(layer["width"], 3)
        self.assertEqual(layer["height"], 2)
        self.assertTrue(layer["png_b64"])

    def test_saved_map_payload_preserves_metric_origin(self) -> None:
        with tempfile.TemporaryDirectory() as raw_directory:
            directory = Path(raw_directory)
            image = directory / "map.pgm"
            image.write_bytes(b"P5\n2 1\n255\n\x00\xfe")
            metadata = directory / "map.yaml"
            metadata.write_text(
                "image: map.pgm\n"
                "resolution: 0.05\n"
                "origin: [-1.25, 2.5, 0.1]\n",
                encoding="utf-8",
            )
            layer = _saved_map_payload(metadata)
        self.assertEqual(layer["width"], 2)
        self.assertEqual(layer["height"], 1)
        self.assertEqual(layer["origin_x"], -1.25)
        self.assertEqual(layer["origin_y"], 2.5)
        self.assertEqual(layer["origin_yaw"], 0.1)

    def test_invalid_report_run_exposes_coverage_without_fake_metrics(self) -> None:
        with tempfile.TemporaryDirectory() as raw_directory:
            run_dir = Path(raw_directory)
            (run_dir / "visibility.json").write_text(
                json.dumps(
                    {
                        "visible_truth_ids": ["Chair:one"],
                        "truth_count": 20,
                        "path_length_m": 3.6,
                        "odom_discontinuity_count": 0,
                        "aborted_reason": "",
                    }
                ),
                encoding="utf-8",
            )
            payload = _invalid_world_payload("kitchen", run_dir)
        self.assertEqual(payload["benchmark_status"], "invalid")
        self.assertEqual(payload["coverage"], 0.05)
        self.assertEqual(payload["path_length_m"], 3.6)
        self.assertEqual(payload["metrics"], {})
        self.assertFalse(payload["renderable"])
        self.assertIn("below the 25.0% validity gate", payload["status_message"])

    def test_invalid_report_explains_smooth_odometry_scale_drift(self) -> None:
        agreement = odometry_path_agreement(
            {
                "ground_truth": {
                    "sample_count": 100,
                    "path_length_m": 10.40,
                },
                "wheel": {
                    "sample_count": 100,
                    "path_length_m": 12.65,
                },
                "fused": {
                    "sample_count": 100,
                    "path_length_m": 12.64,
                },
            }
        )
        with tempfile.TemporaryDirectory() as raw_directory:
            run_dir = Path(raw_directory)
            (run_dir / "visibility.json").write_text(
                json.dumps(
                    {
                        "visible_truth_ids": ["Chair:one"],
                        "truth_count": 1,
                        "path_length_m": 12.64,
                        "odom_discontinuity_count": 0,
                        "odometry_path_agreement": agreement,
                        "aborted_reason": "",
                    }
                ),
                encoding="utf-8",
            )
            payload = _invalid_world_payload("office", run_dir)
        self.assertEqual(payload["benchmark_status"], "invalid")
        self.assertFalse(payload["odometry_path_agreement"]["valid"])
        self.assertIn(
            "path differs from ground_truth",
            payload["status_message"],
        )

    def test_exported_truth_yaw_and_live_map_alignment_reach_report(self) -> None:
        truth = SemanticObjectGroundTruth(
            identity="Fixture:one",
            label="table",
            center_m=(1.0, 2.0, 3.0),
            association_radius_m=0.5,
            node_type="Fixture",
            name="one",
            size_m=(0.8, 0.4, 0.7),
            yaw_rad=0.25,
            evaluate_yaw=True,
        )
        with tempfile.TemporaryDirectory() as raw_directory:
            run_dir = Path(raw_directory)
            files = {
                "truth.json": _truth_payload("fixture", [truth]),
                "visibility.json": {
                    "visible_truth_ids": [truth.identity],
                    "truth_alignment": {
                        "target_frame": "map",
                        "translation_m": [10.0, -4.0, 0.5],
                        "yaw_rad": math.pi / 2.0,
                    },
                },
                "state.json": {
                    "objects": [],
                    "occupancy": {},
                    "robot": {},
                },
                "objects3d.json": {"objects": []},
                "evaluation.json": {
                    "tp": 0,
                    "fp": 0,
                    "fn": 1,
                    "precision": 0.0,
                    "recall": 0.0,
                    "f1": 0.0,
                    "label_accuracy": 0.0,
                    "per_target": [
                        {
                            "identity": truth.identity,
                            "matched": False,
                            "object_id": None,
                        }
                    ],
                    "duplicates": [],
                    "ghosts": [],
                },
            }
            for name, payload in files.items():
                (run_dir / name).write_text(
                    json.dumps(payload),
                    encoding="utf-8",
                )
            world = _world_payload("fixture", run_dir)

        exported = files["truth.json"]["truths"][0]
        self.assertEqual(exported["yaw_rad"], 0.25)
        rendered = world["truth"][0]
        self.assertAlmostEqual(rendered["center"][0], 8.0)
        self.assertAlmostEqual(rendered["center"][1], -3.0)
        self.assertAlmostEqual(rendered["center"][2], 3.5)
        self.assertAlmostEqual(rendered["yaw"], 0.25 + math.pi / 2.0)

    def test_equal_nearest_truth_distance_is_deterministic(self) -> None:
        truths = [
            SemanticObjectGroundTruth(
                identity=identity,
                label="chair",
                center_m=(0.0, 0.0, 0.5),
                association_radius_m=0.1,
                node_type="Chair",
                name=identity,
                size_m=(0.5, 0.5, 1.0),
            )
            for identity in ("b", "a")
        ]
        result = evaluate_semantic_inventory(
            [
                {
                    "id": "far",
                    "cls": "chair",
                    "pose": {"x": 2.0, "y": 0.0, "z": 0.5},
                    "missing": False,
                }
            ],
            truths,
            visible_truth_ids={"a", "b"},
        )
        self.assertEqual(result["ghosts"][0]["nearest_truth"], "a")

    def test_live_map_alignment_does_not_assume_zero_slam_origin(self) -> None:
        truths, _ = load_semantic_inventory(
            BENCHMARK,
            world_id="office",
            repository_root=ROOT,
        )
        source = truths[0]
        aligned = transform_semantic_inventory_planar(
            [source],
            translation_m=(1.5, -2.0, 0.25),
            yaw_rad=3.141592653589793 / 2.0,
        )[0]
        self.assertAlmostEqual(
            aligned.center_m[0],
            1.5 - source.center_m[1],
        )
        self.assertAlmostEqual(
            aligned.center_m[1],
            -2.0 + source.center_m[0],
        )
        self.assertAlmostEqual(
            aligned.center_m[2],
            0.25 + source.center_m[2],
        )
        self.assertEqual(aligned.identity, source.identity)

    def test_registered_rgbd_uses_authoritative_webots_optical_tf(self) -> None:
        resource = (
            ROOT
            / "examples/webots/sim/ros_ws/src/eaios_webots/resource"
            / "tiago_webots.urdf"
        )
        root = ET.parse(resource).getroot()
        frame_by_device = {
            device.attrib["reference"]: device.findtext("ros/frameName")
            for device in root.findall("./webots/device")
        }
        self.assertEqual(frame_by_device["Astra rgb"], "Astra rgb")
        self.assertEqual(frame_by_device["Astra depth"], "Astra rgb")

        camera_start = (
            ROOT / "examples/webots/primitives/tiago_camera/scripts/start.sh"
        ).read_text(encoding="utf-8")
        camera_driver = (
            ROOT / "examples/webots/primitives/tiago_camera/camera_driver/driver.py"
        ).read_text(encoding="utf-8")
        camera_manifest = (
            ROOT / "examples/webots/primitives/tiago_camera/package_manifest.yaml"
        ).read_text(encoding="utf-8")
        soma = (ROOT / "examples/webots/soma.yaml").read_text(encoding="utf-8")
        sweep = (ROOT / "testing/run_webots_scene_sweep.py").read_text(encoding="utf-8")
        self.assertNotIn("static_transform_publisher", camera_start)
        for source in (camera_driver, camera_manifest, soma, sweep):
            self.assertNotIn("robonix/primitive/camera/extrinsics", source)
            self.assertNotIn("/tiago/camera/extrinsics", source)

        benchmark = (
            ROOT / "testing/run_webots_scene_benchmark.sh"
        ).read_text(encoding="utf-8")
        trial = (
            ROOT / "testing/run_scene_review_trial.sh"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'docker cp "$REPO_ROOT/testing/webots_scene_motion.py"',
            benchmark,
        )
        self.assertIn('MOTION_MODE="${ROBONIX_SCENE_BENCHMARK_MOTION_MODE:-reactive}"', benchmark)
        self.assertIn("reactive | panorama | explore", benchmark)
        self.assertIn(
            'CONFIG_GATE_MODE="${ROBONIX_SCENE_BENCHMARK_CONFIG_GATE_MODE:-candidate}"',
            benchmark,
        )
        self.assertIn(
            'ALIGNMENT_TIMEOUT_S="${ROBONIX_SCENE_ALIGNMENT_TIMEOUT_S:-90}"',
            benchmark,
        )
        self.assertIn(
            "--alignment-timeout-s $ALIGNMENT_TIMEOUT_S",
            benchmark,
        )
        self.assertIn(
            '"alignment_timeout_s=${ROBONIX_SCENE_ALIGNMENT_TIMEOUT_S:-90}"',
            trial,
        )
        self.assertIn(
            'ROBONIX_SOURCE_PATH="$("$RBNX_BIN" path root)"',
            trial,
        )
        self.assertIn('"robonix_source_path=$ROBONIX_SOURCE_PATH"', trial)
        self.assertIn('"rbnx_on_path=$RBNX_ON_PATH"', trial)
        self.assertIn('dependency-evidence.txt', trial)
        self.assertIn('mapping_git_head=', trial)
        self.assertIn("candidate | baseline", benchmark)
        self.assertIn(
            '"$CONFIG_GATE_MODE" <<\'PY\'',
            benchmark,
        )
        self.assertIn(
            'if gate_mode == "baseline":',
            benchmark,
        )
        self.assertIn(
            'required = {"objects", "robot", "map_binding"}',
            benchmark,
        )
        self.assertIn('sweep_command="$sweep_command --panorama"', benchmark)
        self.assertIn('motion_mode = "panorama"', sweep)
        self.assertIn("command.angular.z = args.angular_speed_rps", sweep)
        self.assertIn('"$sweep_command --passive"', benchmark)
        self.assertLess(
            benchmark.index('"$sweep_command --passive"'),
            benchmark.index('"$EXPLORE_HELPER" start'),
        )
        self.assertIn('"$EXPLORE_HELPER" cancel', benchmark)
        self.assertIn("ROBONIX_SCENE_LABEL_JUDGMENTS_FILE", benchmark)
        self.assertIn(
            'compared_sources=("localized", "wheel", "fused")',
            benchmark,
        )
        self.assertIn("required_sources=()", benchmark)
        self.assertIn("ROBONIX_SCENE_MAX_LOCALIZED_P95_ERROR_M", benchmark)
        self.assertIn("localized translation P95 error", benchmark)
        self.assertIn('>\"$world_dir/evaluation.semantic.json\"', benchmark)
        self.assertIn("evaluation_path(world)", benchmark)

    def test_all_five_worlds_resolve_inventory_from_wbt(self) -> None:
        counts = {}
        window_counts = {}
        for world_id in (
            "office",
            "break_room",
            "kitchen",
            "apartment",
            "complete_apartment",
        ):
            truths, _ = load_semantic_inventory(
                BENCHMARK,
                world_id=world_id,
                repository_root=ROOT,
            )
            counts[world_id] = len(truths)
            window_counts[world_id] = sum(
                truth.label == "window" for truth in truths
            )
            self.assertEqual(
                len({truth.identity for truth in truths}),
                len(truths),
            )
            self.assertTrue(all(truth.association_radius_m > 0 for truth in truths))
            self.assertEqual(
                {_semantic_size_bucket(truth) for truth in truths},
                {
                    "small_cube_edge_lt_0_30m",
                    "medium_cube_edge_0_30_to_0_75m",
                    "large_cube_edge_ge_0_75m",
                },
                world_id,
            )
        self.assertGreaterEqual(counts["office"], 35)
        self.assertGreaterEqual(counts["kitchen"], 40)
        self.assertGreaterEqual(counts["apartment"], 30)
        self.assertGreaterEqual(counts["complete_apartment"], 100)
        self.assertEqual(window_counts["office"], 8)
        self.assertEqual(window_counts["break_room"], 8)
        self.assertEqual(window_counts["kitchen"], 2)
        self.assertEqual(window_counts["apartment"], 2)
        self.assertEqual(window_counts["complete_apartment"], 8)

    def test_door_truth_uses_leaf_not_frame_envelope(self) -> None:
        truths, _ = load_semantic_inventory(
            BENCHMARK,
            world_id="office",
            repository_root=ROOT,
        )
        door = next(truth for truth in truths if truth.node_type == "Door")
        # office.wbt: size 0.06 1 2.5, frameSize 0.05 0.05 0.01.
        # The actual R2025a leaf is 0.05 x 0.95 x 1.995 m and sits 0.105 m
        # along the Door node's local +X axis from the frame origin.
        self.assertEqual(door.size_m, (0.05, 0.95, 1.995))
        robot_relative_yaw = door.yaw_rad
        node_center_without_leaf_offset = (-10.444127977340377, 2.3073323898487184)
        self.assertAlmostEqual(
            door.center_m[0] - node_center_without_leaf_offset[0],
            math.cos(robot_relative_yaw) * 0.105,
            places=6,
        )
        self.assertAlmostEqual(
            door.center_m[1] - node_center_without_leaf_offset[1],
            math.sin(robot_relative_yaw) * 0.105,
            places=6,
        )

        kitchen_truths, _ = load_semantic_inventory(
            BENCHMARK,
            world_id="kitchen",
            repository_root=ROOT,
        )
        kitchen_door = next(
            truth for truth in kitchen_truths if truth.node_type == "Door"
        )
        # kitchen.wbt omits size, so this must use Door.proto's
        # 0.2 x 1.0 x 2.4 m frame default, not an axis-swapped approximation.
        self.assertEqual(kitchen_door.size_m, (0.05, 0.95, 1.975))

    def test_cabinet_truth_uses_proto_grid_extent_and_back_bottom_origin(
        self,
    ) -> None:
        truths, _ = load_semantic_inventory(
            BENCHMARK,
            world_id="kitchen",
            repository_root=ROOT,
        )
        cabinet = next(
            truth
            for truth in truths
            if truth.identity == "Cabinet:cabinet(1)"
        )
        # kitchen.wbt: depth .7, outer .02, columns [.4], rows
        # [.2, .2, .2, .2, .2, .34].  Cabinet.proto adds the outer frame
        # twice and places the local centre at (depth/2, 0, height/2).
        self.assertAlmostEqual(cabinet.size_m[0], 0.7, places=9)
        self.assertAlmostEqual(cabinet.size_m[1], 0.44, places=9)
        self.assertAlmostEqual(cabinet.size_m[2], 1.38, places=9)
        self.assertAlmostEqual(cabinet.center_m[2], 0.83 + 0.69, places=6)

    def test_window_truth_uses_glass_opening_not_wall_envelope(self) -> None:
        truths, _ = load_semantic_inventory(
            BENCHMARK,
            world_id="kitchen",
            repository_root=ROOT,
        )
        first = next(
            truth
            for truth in truths
            if truth.identity == "Window:window(1)"
        )
        second = next(
            truth
            for truth in truths
            if truth.identity == "Window:Window#2"
        )
        # Both nodes omit size/frame/thickness and therefore use Window.proto
        # defaults.  The WBT only changes bottomWallHeight and windowHeight.
        self.assertEqual(first.size_m, (0.05, 0.76, 0.9))
        self.assertEqual(second.size_m, (0.05, 0.76, 0.9))
        self.assertAlmostEqual(first.center_m[2], 1.4, places=6)
        self.assertAlmostEqual(second.center_m[2], 1.7, places=6)

    def test_webots_window_rerank_uses_measured_pair_and_margin(self) -> None:
        import yaml

        expected = {
            "examples/webots/robonix_manifest.yaml": (
                [["window", "picture frame"]],
                0.0,
            ),
            "examples/webots/robonix_manifest.scene-eval.yaml": (
                [
                    {
                        "labels": ["window", "picture frame"],
                        "min_margin": 0.002,
                    },
                    {
                        "labels": ["cereal box", "book", "box", "plate"],
                        "min_margin": 0.05,
                    },
                    {
                        "labels": ["monitor", "television"],
                        "min_margin": 0.0,
                    },
                ],
                0.05,
            ),
            "examples/webots/robonix_manifest.mapping-nav-eval.yaml": (
                [
                    {
                        "labels": ["window", "picture frame"],
                        "min_margin": 0.002,
                    },
                    {
                        "labels": ["cereal box", "book", "box", "plate"],
                        "min_margin": 0.05,
                    },
                    {
                        "labels": ["monitor", "television"],
                        "min_margin": 0.0,
                    },
                ],
                0.05,
            ),
        }
        for relative, (groups, min_margin) in expected.items():
            manifest = yaml.safe_load((ROOT / relative).read_text())
            scene = manifest["system"]["scene"]["config"]["perception"]
            rerank = scene["label"]["clip_rerank"]
            self.assertEqual(rerank["groups"], groups)
            self.assertEqual(rerank["min_score"], 0.20)
            self.assertEqual(rerank["min_margin"], min_margin)
            if relative != "examples/webots/robonix_manifest.yaml":
                self.assertEqual(
                    rerank["routes"]["cup"],
                    {
                        "labels": ["cup", "can"],
                        "min_margin": 0.02,
                    },
                )
                self.assertEqual(
                    rerank["prompts"]["can"],
                    [
                        "a cylindrical metal food or drink can",
                        "a sealed aluminum can on a table",
                    ],
                )
            self.assertNotIn(
                "confusable_class_groups",
                scene["association"],
            )
            self.assertNotIn(
                "allow_cross_class_merge",
                scene["association"],
            )

    def test_visibility_scope_prevents_unseen_targets_becoming_false_negatives(
        self,
    ) -> None:
        truths, _ = load_semantic_inventory(
            BENCHMARK,
            world_id="office",
            repository_root=ROOT,
        )
        first = truths[0]
        obj = {
            "id": "scene.object.test_001",
            "cls": first.label,
            "pose": {
                "x": first.center_m[0],
                "y": first.center_m[1],
                "z": first.center_m[2],
            },
            "missing": False,
        }
        result = evaluate_semantic_inventory(
            [obj],
            truths,
            visible_truth_ids={first.identity},
        )
        self.assertEqual(result["tp"], 1)
        self.assertEqual(result["fn"], 0)
        self.assertEqual(result["fp"], 0)
        self.assertEqual(result["recall"], 1.0)

    def test_metrics_separate_coverage_from_recall_and_size_strata(self) -> None:
        small = SemanticObjectGroundTruth(
            identity="small",
            label="cup",
            center_m=(0.0, 0.0, 0.1),
            association_radius_m=0.3,
            node_type="Cup",
            name="small cup",
            size_m=(0.08, 0.08, 0.12),
        )
        large = SemanticObjectGroundTruth(
            identity="large",
            label="table",
            center_m=(2.0, 0.0, 0.4),
            association_radius_m=0.5,
            node_type="Table",
            name="large table",
            size_m=(1.0, 0.8, 0.7),
        )
        medium = SemanticObjectGroundTruth(
            identity="medium",
            label="chair",
            center_m=(4.0, 0.0, 0.5),
            association_radius_m=0.5,
            node_type="Chair",
            name="medium chair",
            size_m=(0.50, 0.50, 0.80),
        )
        result = evaluate_semantic_inventory(
            [
                {
                    "id": "scene.object.cup_001",
                    "cls": "cup",
                    "pose": {"x": 0.0, "y": 0.0, "z": 0.1},
                    "missing": False,
                }
            ],
            [small, medium, large],
            visible_truth_ids={"small"},
        )

        self.assertEqual(result["coverage"], 1.0 / 3.0)
        self.assertEqual(result["recall_at_covered"], 1.0)
        self.assertEqual(result["fn"], 0)
        self.assertEqual(
            result["per_size_bucket"]["small_cube_edge_lt_0_30m"],
            {
                "equivalent_cube_edge_rule": "<0.30m",
                "total_truth_count": 1,
                "covered_truth_count": 1,
                "coverage": 1.0,
                "matched_count": 1,
                "correct_label_count": 1,
                "recall_at_covered": 1.0,
                "label_accuracy_among_matched": 1.0,
            },
        )
        self.assertEqual(
            result["per_size_bucket"][
                "medium_cube_edge_0_30_to_0_75m"
            ]["coverage"],
            0.0,
        )
        self.assertEqual(
            result["per_size_bucket"][
                "large_cube_edge_ge_0_75m"
            ]["coverage"],
            0.0,
        )

    def test_real_prediction_outside_visibility_scope_is_ignored_not_ghost(
        self,
    ) -> None:
        truths, _ = load_semantic_inventory(
            BENCHMARK,
            world_id="office",
            repository_root=ROOT,
        )
        visible = truths[0]
        ignored = next(
            truth
            for truth in truths[1:]
            if (
                (
                    (truth.center_m[0] - visible.center_m[0]) ** 2
                    + (truth.center_m[1] - visible.center_m[1]) ** 2
                )
                ** 0.5
                > truth.association_radius_m + visible.association_radius_m
            )
        )

        def prediction(object_id, truth):
            return {
                "id": object_id,
                "cls": truth.label,
                "pose": {
                    "x": truth.center_m[0],
                    "y": truth.center_m[1],
                    "z": truth.center_m[2],
                },
                "missing": False,
            }

        result = evaluate_semantic_inventory(
            [
                prediction("visible", visible),
                prediction("ignored-real", ignored),
            ],
            truths,
            visible_truth_ids={visible.identity},
        )
        self.assertEqual(result["tp"], 1)
        self.assertEqual(result["fp"], 0)
        self.assertEqual(result["fn"], 0)
        self.assertEqual(result["evaluated_prediction_count"], 1)
        self.assertEqual(result["ignored_real_prediction_count"], 1)
        self.assertEqual(result["ghost_fp_count"], 0)
        self.assertEqual(result["precision"], 1.0)

    def test_xy_association_keeps_bad_height_as_geometry_error(self) -> None:
        truths, _ = load_semantic_inventory(
            BENCHMARK,
            world_id="office",
            repository_root=ROOT,
        )
        truth = truths[0]
        obj = {
            "id": "bad-height",
            "cls": truth.label,
            "pose": {
                "x": truth.center_m[0],
                "y": truth.center_m[1],
                "z": truth.center_m[2] + 2.0,
            },
            "missing": False,
        }
        result = evaluate_semantic_inventory(
            [obj],
            [truth],
            visible_truth_ids={truth.identity},
        )
        self.assertEqual(result["tp"], 1)
        self.assertEqual(result["ghost_fp_count"], 0)
        self.assertEqual(result["per_target"][0]["center_xy_error_m"], 0.0)
        self.assertAlmostEqual(
            result["per_target"][0]["center_error_m"],
            2.0,
        )

    def test_small_colocated_child_does_not_count_as_parent_detection(self) -> None:
        truths, _ = load_semantic_inventory(
            BENCHMARK,
            world_id="office",
            repository_root=ROOT,
        )
        truth = next(item for item in truths if item.label == "table")
        obj = {
            "id": "small-child",
            "cls": "laptop",
            "pose": {
                "x": truth.center_m[0],
                "y": truth.center_m[1],
                "z": truth.center_m[2],
            },
            "bbox": {
                "size_x": 0.1,
                "size_y": 0.1,
                "size_z": 0.1,
            },
            "missing": False,
        }
        result = evaluate_semantic_inventory(
            [obj],
            [truth],
            visible_truth_ids={truth.identity},
            association_min_volume_ratio=0.03,
            association_prefer_semantic_labels=True,
        )
        self.assertEqual(result["tp"], 0)
        self.assertEqual(result["fn"], 1)
        self.assertEqual(result["ghost_fp_count"], 1)

    def test_reports_miss_wrong_label_duplicate_and_ghost_separately(self) -> None:
        truths, _ = load_semantic_inventory(
            BENCHMARK,
            world_id="office",
            repository_root=ROOT,
        )
        first, second = truths[:2]

        def prediction(object_id, label, center):
            return {
                "id": object_id,
                "cls": label,
                "pose": {"x": center[0], "y": center[1], "z": center[2]},
                "missing": False,
            }

        objects = [
            prediction("wrong-label", "definitely_wrong", first.center_m),
            prediction(
                "duplicate",
                first.label,
                (
                    first.center_m[0] + 0.01,
                    first.center_m[1],
                    first.center_m[2],
                ),
            ),
            prediction("ghost", "chair", (100.0, 100.0, 1.0)),
        ]
        result = evaluate_semantic_inventory(
            objects,
            truths,
            visible_truth_ids={first.identity, second.identity},
        )
        self.assertEqual(result["tp"], 1)
        self.assertEqual(result["fn"], 1)
        self.assertEqual(result["fp"], 2)
        self.assertEqual(result["label_accuracy"], 0.0)
        self.assertEqual(result["duplicate_fp_count"], 1)
        self.assertEqual(result["ghost_fp_count"], 1)

    def test_correct_window_is_not_stolen_by_closer_wrong_class(self) -> None:
        truth = SemanticObjectGroundTruth(
            identity="Window:office",
            label="window",
            center_m=(0.0, 0.0, 1.2),
            association_radius_m=0.9,
            node_type="Window",
            name="office",
            size_m=(0.15, 1.0, 2.4),
        )

        def prediction(object_id, label, x):
            return {
                "id": object_id,
                "cls": label,
                "pose": {"x": x, "y": 0.0, "z": 1.2},
                "missing": False,
            }

        result = evaluate_semantic_inventory(
            [
                prediction("closer-wrong", "lamp", 0.19),
                prediction("correct-window", "window", 0.48),
            ],
            [truth],
            visible_truth_ids={truth.identity},
            association_prefer_semantic_labels=True,
        )
        self.assertEqual(result["tp"], 1)
        self.assertEqual(result["per_target"][0]["object_id"], "correct-window")
        self.assertTrue(result["per_target"][0]["label_correct"])
        self.assertEqual(result["duplicates"][0]["object_id"], "closer-wrong")

    def test_only_wrong_class_still_counts_as_detected_classification_error(
        self,
    ) -> None:
        truth = SemanticObjectGroundTruth(
            identity="Window:office",
            label="window",
            center_m=(0.0, 0.0, 1.2),
            association_radius_m=0.9,
            node_type="Window",
            name="office",
            size_m=(0.15, 1.0, 2.4),
        )
        result = evaluate_semantic_inventory(
            [
                {
                    "id": "wrong-only",
                    "cls": "picture_frame",
                    "pose": {"x": 0.2, "y": 0.0, "z": 1.2},
                    "missing": False,
                }
            ],
            [truth],
            visible_truth_ids={truth.identity},
            association_prefer_semantic_labels=True,
        )
        self.assertEqual(result["tp"], 1)
        self.assertEqual(result["fn"], 0)
        self.assertFalse(result["per_target"][0]["label_correct"])
        self.assertEqual(result["label_accuracy"], 0.0)

    def test_semantics_never_make_out_of_radius_candidate_admissible(self) -> None:
        truth = SemanticObjectGroundTruth(
            identity="Window:office",
            label="window",
            center_m=(0.0, 0.0, 1.2),
            association_radius_m=0.9,
            node_type="Window",
            name="office",
            size_m=(0.15, 1.0, 2.4),
        )
        result = evaluate_semantic_inventory(
            [
                {
                    "id": "far-correct",
                    "cls": "window",
                    "pose": {"x": 1.0, "y": 0.0, "z": 1.2},
                    "missing": False,
                },
                {
                    "id": "near-wrong",
                    "cls": "lamp",
                    "pose": {"x": 0.2, "y": 0.0, "z": 1.2},
                    "missing": False,
                },
            ],
            [truth],
            visible_truth_ids={truth.identity},
            association_prefer_semantic_labels=True,
        )
        self.assertEqual(result["per_target"][0]["object_id"], "near-wrong")
        self.assertFalse(result["per_target"][0]["label_correct"])
        self.assertEqual(result["ghosts"][0]["object_id"], "far-correct")

    def test_configured_semantic_equivalence_is_not_string_equality(self) -> None:
        truth = SemanticObjectGroundTruth(
            identity="Sofa:lounge",
            label="couch",
            center_m=(0.0, 0.0, 0.5),
            association_radius_m=1.0,
            node_type="Sofa",
            name="lounge",
            size_m=(2.0, 0.9, 0.85),
        )
        result = evaluate_semantic_inventory(
            [
                {
                    "id": "sofa-observation",
                    "cls": "sofa",
                    "pose": {"x": 0.1, "y": 0.0, "z": 0.5},
                    "missing": False,
                }
            ],
            [truth],
            visible_truth_ids={truth.identity},
            association_prefer_semantic_labels=True,
            semantic_equivalence_groups=[["sofa", "couch"]],
        )
        self.assertTrue(result["per_target"][0]["label_correct"])
        self.assertEqual(result["label_accuracy"], 1.0)

    def test_external_gpt_judgment_controls_label_equivalence(self) -> None:
        truth = SemanticObjectGroundTruth(
            identity="Fixture:reviewed",
            label="expected category",
            center_m=(0.0, 0.0, 0.5),
            association_radius_m=1.0,
            node_type="Fixture",
            name="reviewed",
            size_m=(0.5, 0.5, 0.5),
        )
        objects = [
            {
                "id": "reviewed-observation",
                "cls": "observed category",
                "pose": {"x": 0.1, "y": 0.0, "z": 0.5},
                "missing": False,
            }
        ]
        raw = evaluate_semantic_inventory(
            objects,
            [truth],
            visible_truth_ids={truth.identity},
        )
        reviewed = evaluate_semantic_inventory(
            objects,
            [truth],
            visible_truth_ids={truth.identity},
            semantic_label_judgments={
                ("expected category", "observed category"): True,
            },
        )
        self.assertFalse(raw["per_target"][0]["label_correct"])
        self.assertTrue(reviewed["per_target"][0]["label_correct"])

    def test_label_judgment_file_requires_auditable_reason(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "judgments.json"
            path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "judge": {"provider": "codex"},
                        "judgments": [
                            {
                                "expected_label": "plant",
                                "observed_label": "potted_plant",
                                "equivalent": True,
                                "reason": "same practical category",
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            payload, values, details = _load_label_judgments(path)
        self.assertEqual(payload["judge"]["provider"], "codex")
        self.assertTrue(values[("plant", "potted_plant")])
        self.assertEqual(
            details[("plant", "potted_plant")]["reason"],
            "same practical category",
        )


if __name__ == "__main__":
    unittest.main()
