# SPDX-License-Identifier: MulanPSL-2.0
"""Unit tests for the map identity binding (scene_service.map_binding).

The precedence rule is pure Python and always runs. The latched-read
probe test exercises the graceful-degradation path: on a machine without
rclpy / the generated `map` interface package it must return None (not
raise); on a full ROS container it still returns None because nothing
publishes the probe topic within the short timeout.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scene_service.map_binding import (  # noqa: E402
    MapBinding,
    choose_map_binding,
    read_latched_lifecycle,
)


def test_broadcast_wins_over_config_and_env():
    b = choose_map_binding(
        {"map_id": "lab_a", "mode": "localization", "generation": 4},
        "cfg_map", "env_map",
    )
    assert b == MapBinding(
        map_id="lab_a", source="lifecycle", mode="localization", generation=4
    )
    print("  [PASS] test_broadcast_wins_over_config_and_env")


def test_empty_broadcast_map_id_falls_through():
    # mapping running ephemeral broadcasts map_id="" — that is "no named
    # identity", static levers must win.
    b = choose_map_binding({"map_id": "", "mode": "mapping", "generation": 1},
                           "cfg_map", "env_map")
    assert (b.map_id, b.source) == ("cfg_map", "config")
    print("  [PASS] test_empty_broadcast_map_id_falls_through")


def test_config_beats_env_then_env_then_default():
    assert choose_map_binding(None, "cfg_map", "env_map").source == "config"
    b = choose_map_binding(None, None, "env_map")
    assert (b.map_id, b.source) == ("env_map", "env")
    b = choose_map_binding(None, None, None)
    assert (b.map_id, b.source) == ("default", "default")
    # static sources carry no generation — the runtime watcher treats
    # None as "match any generation" until P3 tightens this.
    assert b.generation is None
    print("  [PASS] test_config_beats_env_then_env_then_default")


def test_probe_degrades_to_none_without_publisher():
    # No rclpy / no `map` interface package → None with a warning;
    # with ROS available → None after the (short) timeout since nothing
    # publishes this probe-only topic. Either way: no exception.
    assert read_latched_lifecycle("/robonix/test/map_binding_probe",
                                  timeout_s=0.3) is None
    print("  [PASS] test_probe_degrades_to_none_without_publisher")


if __name__ == "__main__":
    test_broadcast_wins_over_config_and_env()
    test_empty_broadcast_map_id_falls_through()
    test_config_beats_env_then_env_then_default()
    test_probe_degrades_to_none_without_publisher()
    print("test_map_binding: all tests passed")
