#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Verify live Scene labels converge and remain stable on static Webots objects."""

from __future__ import annotations

import json
import os
import sys
import time
import urllib.request
from typing import Any


def _state(url: str) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=3) as response:
        return json.load(response)


def _visible_metric_objects(state: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        obj
        for obj in state.get("objects") or []
        if not obj.get("missing")
        and str(obj.get("cls") or "").strip().lower() != "robot"
        and int(obj.get("label_evidence_count") or 0) > 0
    ]


def main() -> int:
    port = os.environ.get("SCENE_WEB_PORT", "50107")
    url = f"http://127.0.0.1:{port}/api/state"
    deadline = time.monotonic() + 45.0
    state: dict[str, Any] = {}
    stable: list[dict[str, Any]] = []
    while time.monotonic() < deadline:
        try:
            state = _state(url)
            stable = [
                obj
                for obj in _visible_metric_objects(state)
                if not obj.get("label_provisional")
                and obj.get("navigation_grade") is True
            ]
            if stable:
                break
        except Exception:
            pass
        time.sleep(1.0)

    initial_labels = {
        str(obj["id"]): str(obj.get("cls") or "")
        for obj in stable
        if obj.get("id")
    }
    label_switches: list[dict[str, str]] = []
    samples = 0
    for _ in range(8):
        if not initial_labels:
            break
        time.sleep(1.0)
        try:
            latest = _state(url)
        except Exception:
            continue
        samples += 1
        current = {
            str(obj["id"]): str(obj.get("cls") or "")
            for obj in _visible_metric_objects(latest)
            if obj.get("id")
        }
        for object_id, initial_label in initial_labels.items():
            current_label = current.get(object_id)
            if current_label is not None and current_label != initial_label:
                label_switches.append(
                    {
                        "id": object_id,
                        "from": initial_label,
                        "to": current_label,
                    }
                )

    quality = state.get("perception_quality") or {}
    result = {
        "ok": bool(
            initial_labels
            and samples >= 6
            and quality.get("allow_cross_class_merge") is False
            and not label_switches
        ),
        "stable_object_count": len(initial_labels),
        "samples": samples,
        "label_switch_count": len(label_switches),
        "allow_cross_class_merge": quality.get("allow_cross_class_merge"),
        "labels": initial_labels,
        "label_switches": label_switches,
    }
    print(json.dumps(result, separators=(",", ":")))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
