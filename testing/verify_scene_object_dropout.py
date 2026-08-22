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
from pathlib import Path
from typing import Any, Callable


FIXTURE_NAME = "scene_dropout_fixture_chair"
FIXTURE_PATH = (
    Path(__file__).resolve().parent
    / "fixtures"
    / "webots_scene_dropout_chair.wrl"
)


_SPAWN_SCRIPT = r"""
import json
import rclpy
import sys
from rclpy.node import Node
from webots_ros2_msgs.srv import SpawnNodeFromString

rclpy.init()
node = Node("scene_dropout_spawn")
try:
    expected_type = "webots_ros2_msgs/srv/SpawnNodeFromString"
    deadline = __import__("time").monotonic() + 20.0
    service_name = ""
    services = []
    while __import__("time").monotonic() < deadline:
        services = node.get_service_names_and_types()
        matches = [
            name for name, types in services
            if name.endswith("/spawn_node_from_string")
            and expected_type in types
        ]
        if len(matches) == 1:
            service_name = matches[0]
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    if not service_name:
        raise RuntimeError(
            "SpawnNodeFromString service unavailable; graph="
            + json.dumps(services, sort_keys=True)
        )
    client = node.create_client(SpawnNodeFromString, service_name)
    if not client.wait_for_service(timeout_sec=2.0):
        raise RuntimeError(service_name + " disappeared before request")
    request = SpawnNodeFromString.Request()
    request.data = sys.argv[1]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=12.0)
    if not future.done():
        raise RuntimeError("/spawn_node_from_string response timed out")
    response = future.result()
    if response is None:
        raise RuntimeError("/spawn_node_from_string returned no response")
    print(json.dumps({"success": bool(response.success)}))
finally:
    node.destroy_node()
    rclpy.shutdown()
"""

_REMOVE_SCRIPT = r"""
import json
import rclpy
import sys
import time
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node("scene_dropout_remove")
try:
    expected_type = "std_msgs/msg/String"
    deadline = time.monotonic() + 8.0
    topic_name = ""
    topics = []
    while time.monotonic() < deadline:
        topics = node.get_topic_names_and_types()
        matches = [
            name for name, types in topics
            if name.endswith("/remove_node") and expected_type in types
        ]
        if len(matches) == 1:
            topic_name = matches[0]
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    if not topic_name:
        raise RuntimeError(
            "remove_node topic unavailable; graph="
            + json.dumps(topics, sort_keys=True)
        )
    publisher = node.create_publisher(String, topic_name, 10)
    deadline = time.monotonic() + 8.0
    while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() < 1:
        raise RuntimeError(topic_name + " has no subscriber")
    message = String()
    message.data = sys.argv[1]
    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.5)
    print(json.dumps({"published": True}))
finally:
    node.destroy_node()
    rclpy.shutdown()
"""


def _run_ros(container: str, script: str, argument: str) -> str:
    """Run one bounded, daemon-free rclpy operation in the simulator.

    ``ros2 service call`` can leave its CLI process waiting after the request
    on rmw_zenoh_cpp. A direct client makes service discovery and response
    completion independently bounded, and passes the VRML payload as argv
    rather than interpolating it into shell syntax.
    """
    proc = subprocess.run(
        [
            "docker",
            "exec",
            "-i",
            container,
            "bash",
            "-lc",
            (
                "set -e; source /opt/ros/humble/setup.bash; "
                "export ROS2CLI_DISABLE_DAEMON=1; "
                'exec python3 - "$1"'
            ),
            "scene-dropout-verifier",
            argument,
        ],
        input=script,
        capture_output=True,
        text=True,
        timeout=25,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f"ROS command failed ({proc.returncode}): "
            f"{proc.stdout.strip()} {proc.stderr.strip()}"
        )
    return f"{proc.stdout}\n{proc.stderr}"


def _spawn(container: str) -> None:
    fixture_node = FIXTURE_PATH.read_text(encoding="utf-8")
    output = _run_ros(container, _SPAWN_SCRIPT, fixture_node)
    if '"success": true' not in output.lower():
        raise RuntimeError(f"Webots supervisor rejected fixture: {output.strip()}")


def _remove(container: str) -> None:
    _run_ros(container, _REMOVE_SCRIPT, FIXTURE_NAME)


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


def _visible_object_not_in(
    baseline_ids: set[str],
) -> Callable[[list[dict[str, Any]]], tuple[str, str] | None]:
    def predicate(
        objects: list[dict[str, Any]],
    ) -> tuple[str, str] | None:
        for obj in objects:
            oid = str(obj.get("id") or "")
            label = str(obj.get("cls") or "").lower()
            if (
                oid
                and oid not in baseline_ids
                and label
                and label != "robot"
                and not bool(obj.get("missing"))
            ):
                return oid, label
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
        appeared, appear_latency = _wait(
            "fixture appearance",
            20.0,
            _visible_object_not_in(baseline_ids),
            args.scene_url,
        )
        oid, observed_label = appeared

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
            "observed_label": observed_label,
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
