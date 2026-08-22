#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Estimate a WBT-derived acquisition budget for Scene benchmarks.

The benchmark used to assign a short duration to each world by name. That
made an incomplete Explore timeout look like a completed acquisition. The
budget below instead follows the exported semantic inventory: spatial extent
approximates transit work and inventory size approximates observation work.
It remains a hard upper bound; Explore may still report completion earlier.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def estimate_exploration_budget_s(
    truth_payload: dict[str, Any],
    *,
    max_speed_m_s: float,
    minimum_s: float = 180.0,
    maximum_s: float = 600.0,
    rounding_s: float = 15.0,
) -> int:
    """Return a bounded acquisition budget derived from WBT truth geometry."""

    if not math.isfinite(max_speed_m_s) or max_speed_m_s <= 0.0:
        raise ValueError("max_speed_m_s must be finite and positive")
    if (
        not math.isfinite(minimum_s)
        or not math.isfinite(maximum_s)
        or minimum_s <= 0.0
        or maximum_s < minimum_s
    ):
        raise ValueError(
            "minimum_s and maximum_s must be finite, positive, and ordered"
        )
    if not math.isfinite(rounding_s) or rounding_s <= 0.0:
        raise ValueError("rounding_s must be finite and positive")

    truths = list(truth_payload.get("truths") or ())
    points = []
    for truth in truths:
        center = truth.get("center_m") if isinstance(truth, dict) else None
        if (
            isinstance(center, (list, tuple))
            and len(center) >= 2
            and all(
                isinstance(value, (int, float)) and math.isfinite(float(value))
                for value in center[:2]
            )
        ):
            points.append((float(center[0]), float(center[1])))

    if points:
        xs, ys = zip(*points)
        diagonal_m = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    else:
        diagonal_m = 0.0

    # Allow more than one direct traversal of the world extent, plus camera
    # turns and multi-view observation time proportional to semantic density.
    transit_s = 1.5 * diagonal_m / max_speed_m_s
    observation_s = 0.75 * len(truths)
    raw_s = min(maximum_s, max(minimum_s, transit_s + observation_s))
    return int(math.ceil(raw_s / rounding_s) * rounding_s)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--truth-file", type=Path, required=True)
    parser.add_argument("--max-speed-m-s", type=float, required=True)
    parser.add_argument("--minimum-s", type=float, default=180.0)
    parser.add_argument("--maximum-s", type=float, default=600.0)
    args = parser.parse_args()
    payload = json.loads(args.truth_file.read_text(encoding="utf-8"))
    print(
        estimate_exploration_budget_s(
            payload,
            max_speed_m_s=args.max_speed_m_s,
            minimum_s=args.minimum_s,
            maximum_s=args.maximum_s,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
