# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import math
import unittest
from pathlib import Path

from scene_quality_ground_truth import (
    bbox_iou_3d,
    evaluate_ground_truth,
    load_ground_truth,
    nearest_object,
    point_inlier_fraction,
)

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "testing/fixtures/webots_office_scene_quality.json"


class SceneQualityGroundTruthTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.truth, cls.config = load_ground_truth(FIXTURE, repository_root=ROOT)

    def _object(
        self,
        *,
        object_id: str = "scene.object.potted_plant_001",
        label: str = "potted_plant",
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
    ) -> dict:
        truth = self.truth
        return {
            "id": object_id,
            "cls": label,
            "pose": {
                "x": truth.center_m[0] + dx,
                "y": truth.center_m[1] + dy,
                "z": truth.center_m[2] + dz,
                "yaw": truth.yaw_rad,
            },
            "bbox": {
                "size_x": truth.size_m[0],
                "size_y": truth.size_m[1],
                "size_z": truth.size_m[2],
                "yaw": truth.yaw_rad,
            },
            "navigation_grade": True,
            "missing": False,
        }

    def test_world_file_resolves_expected_initial_map_center(self) -> None:
        self.assertEqual(self.truth.label, "potted_plant")
        self.assertAlmostEqual(self.truth.center_m[0], 1.54636, places=4)
        self.assertAlmostEqual(self.truth.center_m[1], 0.16051, places=4)
        self.assertAlmostEqual(self.truth.center_m[2], 0.65, places=6)

    def test_nearest_object_associates_before_checking_label(self) -> None:
        wrong_label = self._object(label="chair", dx=0.05)
        far_correct_label = self._object(
            object_id="scene.object.potted_plant_002",
            dx=1.0,
        )
        self.assertIs(
            nearest_object([far_correct_label, wrong_label], self.truth),
            wrong_label,
        )

    def test_oriented_bbox_iou_and_point_inliers(self) -> None:
        obj = self._object()
        self.assertAlmostEqual(bbox_iou_3d(obj, self.truth), 1.0, places=9)
        obj["pose"]["x"] += 2.0
        self.assertEqual(bbox_iou_3d(obj, self.truth), 0.0)

        center = self.truth.center_m
        fraction = point_inlier_fraction(
            [
                center,
                (center[0] + 0.05, center[1], center[2] + 0.2),
                (center[0] + 2.0, center[1], center[2]),
            ],
            self.truth,
            margin_m=0.0,
        )
        self.assertAlmostEqual(fraction or 0.0, 2.0 / 3.0)

    def test_evaluation_reports_accuracy_stability_and_cloud_quality(self) -> None:
        samples = [
            self._object(dx=0.01 * math.sin(index), dy=0.01 * math.cos(index))
            for index in range(8)
        ]
        center = self.truth.center_m
        points = [
            [
                center[0] + (index % 3 - 1) * 0.03,
                center[1] + (index % 5 - 2) * 0.02,
                center[2] + (index % 7 - 3) * 0.05,
            ]
            for index in range(50)
        ]
        point_samples = [
            {
                "id": "cg-fixture",
                "cls": "potted_plant",
                "center": list(center),
                "points": points,
                "n_points": 50,
            }
            for _ in range(8)
        ]
        result = evaluate_ground_truth(
            samples,
            point_samples,
            self.truth,
            expected_samples=8,
            thresholds=self.config["thresholds"],
        )
        self.assertTrue(result["ok"], result["failures"])
        self.assertEqual(result["stable_id_count"], 1)
        self.assertEqual(result["label_accuracy"], 1.0)
        self.assertGreater(result["median_point_inlier_fraction"], 0.99)

    def test_evaluation_rejects_contract_success_with_bad_world_state(self) -> None:
        samples = [
            self._object(
                object_id=f"scene.object.bad_{index % 2}",
                label="chair",
                dx=0.3 + index * 0.01,
                dz=0.6,
            )
            for index in range(8)
        ]
        result = evaluate_ground_truth(
            samples,
            [],
            self.truth,
            expected_samples=8,
            thresholds=self.config["thresholds"],
        )
        self.assertFalse(result["ok"])
        self.assertEqual(result["label_accuracy"], 0.0)
        self.assertEqual(result["stable_id_count"], 2)
        self.assertIn(
            "median_point_inlier_fraction below 0.65",
            result["failures"],
        )


if __name__ == "__main__":
    unittest.main()
