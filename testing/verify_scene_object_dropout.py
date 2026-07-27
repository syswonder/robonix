#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Live Webots acceptance check for Scene object dropout and re-identification.

The Webots ROS 2 supervisor imports a chair into the robot's initial camera
view, Scene observes it, the supervisor removes it, and this verifier checks:

* repeated healthy RGB-D misses hide the object promptly;
* respawning at the same pose reuses the stable Scene object id; and
* the final removal is hard-pruned after the configured TTL.

This script is intentionally stdlib-only. It runs through Executor's
``run_command`` capability in the integration scenario so the same current PR
checkout that boots Webots also supplies the verifier.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request
from typing import Any, Callable


FIXTURE_NAME = "scene_dropout_fixture"
FIXTURE_NODE = (
    'OfficeChair { translation 0.0 -4.0 0 '
    'rotation 0 0 1 0 name "scene_dropout_fixture" }'
)


def _run_ros(container: str, command: str, request: str) -> str:
    """Run one ROS 2 CLI operation in the simulator container.

    The request is passed as positional ``$1`` rather than interpolated into
    the shell program, so the VRML/YAML quoting is preserved exactly.
    """
    proc = subprocess.run(
        [
            "docker",
            "exec",
            container,
            "bash",
            "-lc",
            (
                "set -e; source /opt/ros/humble/setup.bash; "
                f"{command} \"$1\""
            ),
            "scene-dropout-verifier",
            request,
        ],
        capture_output=True,
        text=True,
        timeout=20,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f"ROS command failed ({proc.returncode}): "
            f"{proc.stdout.strip()} {proc.stderr.strip()}"
        )
    return f"{proc.stdout}\n{proc.stderr}"


def _spawn(container: str) -> None:
    request = json.dumps({"data": FIXTURE_NODE})
    output = _run_ros(
        container,
        (
            "ros2 service call /spawn_node_from_string "
            "webots_ros2_msgs/srv/SpawnNodeFromString"
        ),
        request,
    )
    if "success=true" not in output.lower() and "success: true" not in output.lower():
        raise RuntimeError(f"Webots supervisor rejected fixture: {output.strip()}")


def _remove(container: str) -> None:
    request = json.dumps({"data": FIXTURE_NAME})
    _run_ros(
        container,
        "ros2 topic pub --once /remove_node std_msgs/msg/String",
        request,
    )


def _state(url: str) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=3) as response:
        return json.load(response)


def _objects(url: str) -> list[dict[str, Any]]:
    return list(_state(url).get("objects") or [])


def _wait(
    description: str,
    timeout_s: float,
    predicate: Callable[[list[dict[str, Any]]], Any],
    scene_url: str,
) -> tuple[Any, float]:
    start = time.monotonic()
    last: list[dict[str, Any]] = []
    while time.monotonic() - start < timeout_s:
        try:
            last = _objects(scene_url)
            value = predicate(last)
            if value:
                return value, time.monotonic() - start
        except Exception:
            pass
        time.sleep(0.5)
    compact = [
        {
            "id": obj.get("id"),
            "cls": obj.get("cls"),
            "missing": obj.get("missing"),
            "observation_count": obj.get("observation_count"),
        }
        for obj in last
        if str(obj.get("cls") or "").lower() != "robot"
    ]
    raise TimeoutError(f"{description} timed out after {timeout_s}s; objects={compact}")


def _visible_chair_not_in(
    baseline_ids: set[str],
) -> Callable[[list[dict[str, Any]]], str | None]:
    def predicate(objects: list[dict[str, Any]]) -> str | None:
        for obj in objects:
            oid = str(obj.get("id") or "")
            label = str(obj.get("cls") or "").lower()
            if (
                oid
                and oid not in baseline_ids
                and "chair" in label
                and not bool(obj.get("missing"))
            ):
                return oid
        return None

    return predicate


def _object_missing(oid: str) -> Callable[[list[dict[str, Any]]], bool]:
    return lambda objects: any(
        str(obj.get("id") or "") == oid and bool(obj.get("missing"))
        for obj in objects
    )


def _object_visible(oid: str) -> Callable[[list[dict[str, Any]]], bool]:
    return lambda objects: any(
        str(obj.get("id") or "") == oid and not bool(obj.get("missing"))
        for obj in objects
    )


def _object_absent(oid: str) -> Callable[[list[dict[str, Any]]], bool]:
    return lambda objects: all(str(obj.get("id") or "") != oid for obj in objects)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene-url",
        default=f"http://127.0.0.1:{os.environ.get('SCENE_WEB_PORT', '50107')}/api/state",
    )
    parser.add_argument(
        "--container",
        default=os.environ.get("ROBONIX_SIM_CONTAINER", "robonix_tiago_sim"),
    )
    args = parser.parse_args()

    spawned = False
    result: dict[str, Any] = {"ok": False}
    try:
        baseline_ids = {
            str(obj.get("id") or "")
            for obj in _objects(args.scene_url)
            if str(obj.get("id") or "")
        }
        # A previous interrupted verifier may have left the fixture imported.
        # remove_node is idempotent from this script's perspective.
        with __import__("contextlib").suppress(Exception):
            _remove(args.container)

        _spawn(args.container)
        spawned = True
        oid, appear_latency = _wait(
            "fixture appearance",
            20.0,
            _visible_chair_not_in(baseline_ids),
            args.scene_url,
        )

        _remove(args.container)
        spawned = False
        _, missing_latency = _wait(
            "visibility-verified missing transition",
            10.0,
            _object_missing(oid),
            args.scene_url,
        )

        _spawn(args.container)
        spawned = True
        _, reappear_latency = _wait(
            "stable-id reappearance",
            20.0,
            _object_visible(oid),
            args.scene_url,
        )

        _remove(args.container)
        spawned = False
        _, final_missing_latency = _wait(
            "second missing transition",
            10.0,
            _object_missing(oid),
            args.scene_url,
        )
        _, delete_latency = _wait(
            "TTL hard deletion",
            36.0,
            _object_absent(oid),
            args.scene_url,
        )

        result = {
            "ok": True,
            "object_id": oid,
            "appearance_latency_s": round(appear_latency, 3),
            "missing_latency_s": round(missing_latency, 3),
            "reappeared_same_id": True,
            "reappearance_latency_s": round(reappear_latency, 3),
            "final_missing_latency_s": round(final_missing_latency, 3),
            "delete_latency_s": round(delete_latency, 3),
        }
        print(json.dumps(result, separators=(",", ":")))
        return 0
    except Exception as exc:
        result["error"] = str(exc)
        print(json.dumps(result, separators=(",", ":")))
        return 1
    finally:
        if spawned:
            with __import__("contextlib").suppress(Exception):
                _remove(args.container)


if __name__ == "__main__":
    sys.exit(main())
