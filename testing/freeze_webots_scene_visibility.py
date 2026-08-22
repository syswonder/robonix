#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Freeze a common visibility denominator without reusing another run's pose.

Each output keeps its trial-local ``truth_alignment`` and replaces only
``visible_truth_ids`` with the intersection observed by every supplied trial.
That prevents both route/coverage drift and the subtler error of scoring one
trial with another trial's SLAM alignment.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Iterable


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def freeze_common_visibility(
    paths: Iterable[Path],
    *,
    output_name: str = "visibility.common.json",
) -> dict[str, Any]:
    inputs = [Path(path) for path in paths]
    if len(inputs) < 2:
        raise ValueError("common visibility requires at least two trials")
    if Path(output_name).name != output_name:
        raise ValueError("output_name must be a file name, not a path")

    documents: list[dict[str, Any]] = []
    visible_sets: list[set[str]] = []
    source_records: list[dict[str, Any]] = []
    for path in inputs:
        document = json.loads(path.read_text(encoding="utf-8"))
        raw_ids = document.get("visible_truth_ids")
        alignment = document.get("truth_alignment")
        if not isinstance(raw_ids, list) or not all(
            isinstance(value, str) and value for value in raw_ids
        ):
            raise ValueError(f"{path} has invalid visible_truth_ids")
        if not isinstance(alignment, dict):
            raise ValueError(f"{path} has no trial-local truth_alignment")
        ids = set(raw_ids)
        documents.append(document)
        visible_sets.append(ids)
        source_records.append(
            {
                "path": str(path),
                "sha256": _sha256(path),
                "visible_count": len(ids),
            }
        )

    common = set.intersection(*visible_sets)
    if not common:
        raise ValueError("visibility intersection is empty")
    common_ids = sorted(common)
    reference = {
        "method": "intersection",
        "source_count": len(inputs),
        "common_visible_count": len(common_ids),
        "source_sha256": [item["sha256"] for item in source_records],
    }
    outputs: list[str] = []
    for path, document in zip(inputs, documents, strict=True):
        frozen = dict(document)
        frozen["visible_truth_ids"] = common_ids
        frozen["visibility_reference"] = reference
        output = path.with_name(output_name)
        temporary = output.with_suffix(output.suffix + ".tmp")
        temporary.write_text(
            json.dumps(frozen, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, output)
        outputs.append(str(output))

    return {
        "method": "intersection_with_trial_local_alignment",
        "common_visible_count": len(common_ids),
        "visible_truth_ids": common_ids,
        "sources": source_records,
        "outputs": outputs,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("visibility_files", nargs="+")
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--output-name", default="visibility.common.json")
    args = parser.parse_args()
    summary = freeze_common_visibility(
        (Path(value) for value in args.visibility_files),
        output_name=args.output_name,
    )
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.summary.with_suffix(args.summary.suffix + ".tmp")
    temporary.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, args.summary)
    print(args.summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
