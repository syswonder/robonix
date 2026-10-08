#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Evaluate one Scene registry snapshot against a checked-in Webots WBT.

The optional visibility file contains ``{"visible_truth_ids": [...]}`` from a
camera/depth sweep.  Without it, all semantic assets in the world are scored;
that mode is useful for inventory inspection but is deliberately labeled
``truth_scope=all`` so it cannot be confused with a fair live recall result.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import urllib.request
from pathlib import Path
from typing import Any

from scene_quality_ground_truth import (
    evaluate_semantic_inventory,
    evaluate_scene_ground_truth,
    load_semantic_inventory,
    match_objects_one_to_one,
    point_inlier_fraction,
    transform_semantic_inventory_planar,
)


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def _load_label_judgments(
    path: Path,
) -> tuple[
    dict[str, Any],
    dict[tuple[str, str], bool],
    dict[tuple[str, str], dict[str, Any]],
]:
    payload = _read_json(path)
    if payload.get("schema_version") != 1:
        raise ValueError("label judgments require schema_version=1")
    judge = payload.get("judge")
    if not isinstance(judge, dict) or not str(judge.get("provider") or ""):
        raise ValueError("label judgments require judge.provider")
    rows = payload.get("judgments")
    if not isinstance(rows, list):
        raise ValueError("label judgments require a judgments list")
    values: dict[tuple[str, str], bool] = {}
    details: dict[tuple[str, str], dict[str, Any]] = {}
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError(f"label judgment {index} must be an object")
        expected = str(row.get("expected_label") or "").strip().lower()
        observed = str(row.get("observed_label") or "").strip().lower()
        reason = str(row.get("reason") or "").strip()
        equivalent = row.get("equivalent")
        if not expected or not observed:
            raise ValueError(
                f"label judgment {index} requires expected_label and "
                "observed_label"
            )
        if not isinstance(equivalent, bool):
            raise ValueError(
                f"label judgment {index} equivalent must be boolean"
            )
        if not reason:
            raise ValueError(f"label judgment {index} requires reason")
        key = (expected, observed)
        if key in values:
            raise ValueError(
                "duplicate label judgment for "
                f"{expected!r} -> {observed!r}"
            )
        values[key] = equivalent
        details[key] = {
            "equivalent": equivalent,
            "reason": reason,
        }
    return payload, values, details


def _state(args: argparse.Namespace) -> dict[str, Any]:
    if args.state_file is not None:
        return _read_json(args.state_file)
    with urllib.request.urlopen(args.scene_url, timeout=5) as response:
        value = json.load(response)
    if not isinstance(value, dict):
        raise ValueError("Scene state endpoint returned a non-object")
    return value


def main() -> int:
    repository_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--benchmark",
        type=Path,
        default=(
            repository_root / "testing" / "fixtures" / "webots_scene_benchmark.json"
        ),
    )
    parser.add_argument("--world-id", required=True)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--state-file", type=Path)
    source.add_argument(
        "--scene-url",
        default="http://127.0.0.1:50107/api/state",
    )
    parser.add_argument(
        "--objects3d-file",
        type=Path,
        help="optional /api/objects3d snapshot for point-cloud scoring",
    )
    parser.add_argument(
        "--visibility-file",
        type=Path,
        help="JSON containing visible_truth_ids from an RGB-D sweep",
    )
    parser.add_argument(
        "--label-judgments-file",
        type=Path,
        help=(
            "external semantic-equivalence judgments produced by a recorded "
            "GPT/Codex review; raw exact/configured accuracy is retained"
        ),
    )
    parser.add_argument(
        "--require-gates",
        action="store_true",
        help="return non-zero unless the benchmark score gates pass",
    )
    args = parser.parse_args()

    truths, benchmark = load_semantic_inventory(
        args.benchmark,
        world_id=args.world_id,
        repository_root=repository_root,
    )
    visible_truth_ids: set[str] | None = None
    if args.visibility_file is not None:
        visibility = _read_json(args.visibility_file)
        raw_ids = visibility.get("visible_truth_ids")
        if not isinstance(raw_ids, list):
            raise ValueError("visibility file requires visible_truth_ids list")
        alignment = visibility.get("truth_alignment")
        if not isinstance(alignment, dict):
            raise ValueError(
                "visibility file requires live truth_alignment; fixed map "
                "origins are not valid for a fresh SLAM session"
            )
        if alignment.get("target_frame") != "map":
            raise ValueError("truth_alignment target_frame must be map")
        truths = transform_semantic_inventory_planar(
            truths,
            translation_m=alignment.get("translation_m") or (),
            yaw_rad=alignment.get("yaw_rad"),
        )
        visible_truth_ids = {str(value) for value in raw_ids}
        unknown = visible_truth_ids - {truth.identity for truth in truths}
        if unknown:
            raise ValueError(
                "visibility file references unknown truth ids: "
                + ", ".join(sorted(unknown))
            )

    state = _state(args)
    objects = list(state.get("objects") or ())
    association = benchmark.get("association") or {}
    semantic_equivalence_groups = association.get(
        "semantic_equivalence_groups", ()
    )
    judgment_payload: dict[str, Any] | None = None
    judgment_values: dict[tuple[str, str], bool] = {}
    judgment_details: dict[tuple[str, str], dict[str, Any]] = {}
    if args.label_judgments_file is not None:
        (
            judgment_payload,
            judgment_values,
            judgment_details,
        ) = _load_label_judgments(args.label_judgments_file)

    raw_result = evaluate_semantic_inventory(
        objects,
        truths,
        visible_truth_ids=visible_truth_ids,
        association_min_volume_ratio=float(association.get("min_volume_ratio", 0.0)),
        association_prefer_semantic_labels=bool(
            association.get("prefer_semantic_labels", False)
        ),
        semantic_equivalence_groups=semantic_equivalence_groups,
    )
    result = (
        evaluate_semantic_inventory(
            objects,
            truths,
            visible_truth_ids=visible_truth_ids,
            association_min_volume_ratio=float(
                association.get("min_volume_ratio", 0.0)
            ),
            association_prefer_semantic_labels=bool(
                association.get("prefer_semantic_labels", False)
            ),
            semantic_equivalence_groups=semantic_equivalence_groups,
            semantic_label_judgments=judgment_values,
        )
        if judgment_payload is not None
        else raw_result
    )
    applied_judgment_count = 0
    for entry in result.get("per_target") or ():
        if not entry.get("matched"):
            continue
        key = (
            str(entry.get("expected_label") or "").strip().lower(),
            str(entry.get("observed_label") or "").strip().lower(),
        )
        detail = judgment_details.get(key)
        if detail is None:
            continue
        entry["gpt_label_judgment"] = detail
        applied_judgment_count += 1
    result["label_evaluation"] = {
        "method": (
            "external_semantic_judgments"
            if judgment_payload is not None
            else "exact_and_configured_equivalence"
        ),
        "raw_label_accuracy": raw_result["label_accuracy"],
        "adjusted_label_accuracy": result["label_accuracy"],
        "applied_judgment_count": applied_judgment_count,
        "judge": (
            judgment_payload.get("judge")
            if judgment_payload is not None
            else None
        ),
        "policy": (
            judgment_payload.get("policy")
            if judgment_payload is not None
            else None
        ),
        "judgments_file": (
            str(args.label_judgments_file)
            if args.label_judgments_file is not None
            else None
        ),
    }
    scoped_truths = (
        [
            truth
            for truth in truths
            if visible_truth_ids is None or truth.identity in visible_truth_ids
        ]
    )
    result["geometry"] = evaluate_scene_ground_truth(
        objects,
        scoped_truths,
    )
    navigation_objects = [
        obj for obj in objects if bool(obj.get("navigation_grade"))
    ]
    result["navigation_grade"] = evaluate_semantic_inventory(
        navigation_objects,
        truths,
        visible_truth_ids=visible_truth_ids,
        association_min_volume_ratio=float(
            association.get("min_volume_ratio", 0.0)
        ),
        association_prefer_semantic_labels=bool(
            association.get("prefer_semantic_labels", False)
        ),
        semantic_equivalence_groups=semantic_equivalence_groups,
        semantic_label_judgments=judgment_values,
    )
    if args.objects3d_file is not None:
        point_payload = _read_json(args.objects3d_file)
        point_objects = list(point_payload.get("objects") or ())
        point_matches = match_objects_one_to_one(
            point_objects,
            scoped_truths,
            center_key="center",
        )
        per_target_points = []
        inlier_fractions = []
        point_counts = []
        for match in point_matches:
            fraction = point_inlier_fraction(
                match.obj.get("points") or (),
                match.truth,
                margin_m=0.08,
            )
            count = int(
                match.obj.get("n_points")
                or len(match.obj.get("points") or ())
            )
            if fraction is not None:
                inlier_fractions.append(fraction)
            point_counts.append(count)
            per_target_points.append(
                {
                    "identity": match.truth.identity,
                    "observed_label": str(match.obj.get("cls") or ""),
                    "center_error_m": match.center_error_m,
                    "sampled_point_count": len(match.obj.get("points") or ()),
                    "full_point_count": count,
                    "point_inlier_fraction": fraction,
                }
            )
        result["point_cloud"] = {
            "matched_object_count": len(point_matches),
            "median_point_inlier_fraction": (
                float(statistics.median(inlier_fractions))
                if inlier_fractions
                else None
            ),
            "median_full_point_count": (
                float(statistics.median(point_counts))
                if point_counts
                else None
            ),
            "per_target": per_target_points,
        }
    gates = benchmark.get("score_gates") or {}
    evaluated_predictions = max(1, int(result["evaluated_prediction_count"]))
    duplicate_rate = float(result["duplicate_fp_count"]) / evaluated_predictions
    ghost_rate = float(result["ghost_fp_count"]) / evaluated_predictions
    failures: list[str] = []

    def at_least(metric: str, gate: str) -> None:
        if float(result[metric]) < float(gates[gate]):
            failures.append(f"{metric} below {gates[gate]}")

    at_least("precision", "min_detection_precision")
    at_least("recall", "min_detection_recall")
    at_least("f1", "min_detection_f1")
    at_least("label_accuracy", "min_label_accuracy")
    if duplicate_rate > float(gates["max_duplicate_rate"]):
        failures.append(f"duplicate_rate above {gates['max_duplicate_rate']}")
    if ghost_rate > float(gates["max_ghost_rate"]):
        failures.append(f"ghost_rate above {gates['max_ghost_rate']}")

    result.update(
        {
            "world_id": args.world_id,
            "world_semantic_inventory_count": len(truths),
            "duplicate_rate": duplicate_rate,
            "ghost_rate": ghost_rate,
            "gate_failures": failures,
            "ok": not failures,
        }
    )
    print(json.dumps(result, separators=(",", ":"), sort_keys=True))
    return int(args.require_gates and bool(failures))


if __name__ == "__main__":
    sys.exit(main())
