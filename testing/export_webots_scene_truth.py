#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Export WBT-resolved semantic truth for the in-container RGB-D sweep."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from scene_quality_ground_truth import load_semantic_inventory


def _truth_payload(world_id: str, truths) -> dict:
    return {
        "world_id": world_id,
        "truths": [
            {
                "identity": truth.identity,
                "label": truth.label,
                "center_m": list(truth.center_m),
                "size_m": list(truth.size_m),
                "yaw_rad": truth.yaw_rad,
                "evaluate_yaw": truth.evaluate_yaw,
            }
            for truth in truths
        ],
    }


def main() -> int:
    repository_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--world-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--benchmark",
        type=Path,
        default=(
            repository_root
            / "testing"
            / "fixtures"
            / "webots_scene_benchmark.json"
        ),
    )
    args = parser.parse_args()
    truths, _ = load_semantic_inventory(
        args.benchmark,
        world_id=args.world_id,
        repository_root=repository_root,
    )
    payload = _truth_payload(args.world_id, truths)
    args.output.write_text(
        json.dumps(payload, separators=(",", ":"), sort_keys=True),
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
