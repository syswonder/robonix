# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import math
import unittest
from pathlib import Path

from scene_quality_ground_truth import (
    ObjectGroundTruth,
    bbox_iou_3d,
    evaluate_ground_truth,
    evaluate_scene_ground_truth,
    load_ground_truth,
    load_ground_truths,
    match_objects_one_to_one,
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


class MultiObjectSceneGroundTruthTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.truths, cls.config = load_ground_truths(FIXTURE, repository_root=ROOT)

    @staticmethod
    def _object(
        truth,
        *,
        object_id: str,
        label: str | None = None,
        dx: float = 0.0,
        size_delta_m: float = 0.0,
        yaw_delta_rad: float = 0.0,
    ) -> dict:
        return {
            "id": object_id,
            "cls": label if label is not None else truth.label,
            "pose": {
                "x": truth.center_m[0] + dx,
                "y": truth.center_m[1],
                "z": truth.center_m[2],
                "yaw": truth.yaw_rad + yaw_delta_rad,
            },
            "bbox": {
                "size_x": truth.size_m[0] + size_delta_m,
                "size_y": truth.size_m[1],
                "size_z": truth.size_m[2],
                "yaw": truth.yaw_rad + yaw_delta_rad,
            },
            "missing": False,
        }

    def test_fixture_resolves_six_representative_nodes_from_wbt(self) -> None:
        self.assertEqual(
            {truth.label for truth in self.truths},
            {"table", "monitor", "keyboard", "chair", "cabinet", "potted_plant"},
        )
        self.assertEqual(len(self.truths), 6)
        target_specs = self.config["targets"]
        self.assertTrue(all("translation" not in spec for spec in target_specs))
        self.assertTrue(all("rotation" not in spec for spec in target_specs))
        table = next(truth for truth in self.truths if truth.label == "table")
        self.assertEqual(table.size_m, (1.0, 1.6, 0.74))
        monitor = next(truth for truth in self.truths if truth.label == "monitor")
        keyboard = next(truth for truth in self.truths if truth.label == "keyboard")
        cabinet = next(truth for truth in self.truths if truth.label == "cabinet")
        plant = next(
            truth for truth in self.truths if truth.label == "potted_plant"
        )
        self.assertEqual(
            monitor.size_m,
            (0.13594834, 0.46901633, 0.34921083),
        )
        self.assertEqual(keyboard.size_m, (0.13432679, 0.41001573, 0.01003296))
        self.assertEqual(cabinet.size_m, (0.5, 1.04, 0.39))
        self.assertEqual(plant.size_m, (0.6, 0.6, 1.341))
        # Keyboard's PROTO origin is already its bounding-box centre, unlike
        # the floor-standing default.  Its truth Z must therefore remain at
        # the WBT node translation instead of shifting upward by half-height.
        self.assertAlmostEqual(keyboard.center_m[2], 0.74, places=6)

    def test_matching_is_position_first_and_one_to_one(self) -> None:
        first, second = self.truths[:2]
        objects = [
            self._object(
                first,
                object_id="wrong-at-first",
                label=second.label,
            ),
            self._object(
                second,
                object_id="wrong-at-second",
                label=first.label,
            ),
        ]
        matches = match_objects_one_to_one(objects, (first, second))
        self.assertEqual(len(matches), 2)
        self.assertEqual(matches[0].obj["id"], "wrong-at-first")
        self.assertEqual(matches[1].obj["id"], "wrong-at-second")

        only_one = match_objects_one_to_one(objects[:1], (first, first))
        self.assertEqual(len(only_one), 1)
        self.assertEqual(len({match.object_index for match in only_one}), 1)

        flexible = ObjectGroundTruth(
            "first", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0), 0.0, 0.5
        )
        constrained = ObjectGroundTruth(
            "second", (1.0, 0.0, 0.0), (1.0, 1.0, 1.0), 0.0, 1.0
        )
        ambiguous = self._object(
            flexible,
            object_id="ambiguous",
            dx=0.1,
        )
        exclusive = self._object(
            flexible,
            object_id="exclusive-first",
            dx=-0.2,
        )
        maximum = match_objects_one_to_one(
            [ambiguous, exclusive],
            (flexible, constrained),
        )
        self.assertEqual(len(maximum), 2)
        self.assertEqual(maximum[0].obj["id"], "exclusive-first")
        self.assertEqual(maximum[1].obj["id"], "ambiguous")

    def test_evaluator_reports_duplicates_labels_and_geometry_errors(self) -> None:
        objects = [
            self._object(
                truth,
                object_id=f"object-{index}",
                label="wrong-label" if index == 0 else None,
                dx=0.05,
                size_delta_m=0.1,
                yaw_delta_rad=0.2,
            )
            for index, truth in enumerate(self.truths)
        ]
        objects.append(
            self._object(
                self.truths[0],
                object_id="duplicate-table",
                dx=0.08,
            )
        )
        result = evaluate_scene_ground_truth(
            objects,
            self.truths,
            thresholds=dict(
                self.config["scene_thresholds"],
                min_label_accuracy=0.8,
                max_duplicate_count=1,
            ),
        )
        self.assertTrue(result["ok"], result["failures"])
        self.assertEqual(result["matched_object_count"], 6)
        self.assertEqual(result["duplicate_count"], 1)
        self.assertEqual(result["ghost_count"], 0)
        self.assertAlmostEqual(result["label_accuracy"], 5.0 / 6.0)
        self.assertAlmostEqual(result["median_center_error_m"], 0.05)
        self.assertAlmostEqual(result["median_size_error_m"], 0.1)
        self.assertAlmostEqual(result["median_yaw_error_rad"], 0.2)
        plant_result = next(
            target
            for target in result["per_target"]
            if target["expected_label"] == "potted_plant"
        )
        self.assertFalse(plant_result["yaw_evaluated"])
        self.assertIsNone(plant_result["yaw_error_rad"])

        strict = self.config["scene_thresholds"]
        rejected = evaluate_scene_ground_truth(
            objects,
            self.truths,
            thresholds=strict,
        )
        self.assertFalse(rejected["ok"])
        self.assertIn("duplicate_count above 0", rejected["failures"])

    def test_equivalent_bbox_axis_encoding_has_zero_size_and_yaw_error(self) -> None:
        truth = next(item for item in self.truths if item.label == "keyboard")
        obj = self._object(truth, object_id="axis-swapped")
        obj["bbox"]["size_x"], obj["bbox"]["size_y"] = (
            obj["bbox"]["size_y"],
            obj["bbox"]["size_x"],
        )
        obj["bbox"]["yaw"] += math.pi * 0.5
        result = evaluate_scene_ground_truth([obj], [truth])
        self.assertAlmostEqual(result["median_size_error_m"], 0.0)
        self.assertAlmostEqual(result["median_yaw_error_rad"], 0.0)


if __name__ == "__main__":
    unittest.main()
