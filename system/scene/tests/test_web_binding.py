# SPDX-License-Identifier: MulanPSL-2.0
"""Scene web UI binding policy tests."""

import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scene_service.web_binding import resolve_web_host


def test_explicit_config_host_wins_over_environment():
    assert resolve_web_host(
        {"web_host": "127.0.0.1"}, {"SCENE_WEB_HOST": "0.0.0.0"}
    ) == "127.0.0.1"


def test_environment_host_is_supported_for_launcher_compatibility():
    assert resolve_web_host({}, {"SCENE_WEB_HOST": "127.0.0.1"}) == "127.0.0.1"


def test_existing_deployments_keep_legacy_default():
    assert resolve_web_host({}, {}) == "0.0.0.0"


@pytest.mark.parametrize(
    "bad",
    [
        " ",
        "0",
        "00",
        "0x0",
        "0.0",
        "0.0.0",
        "000.000.000.000",
        "0000000000",
        "http://127.0.0.1",
        "127.0.0.1/ui",
        "bad host",
    ],
)
def test_invalid_bind_hosts_fail_closed(bad):
    with pytest.raises(ValueError):
        resolve_web_host({"web_host": bad}, {})


@pytest.mark.parametrize("bad", [None, 0, False])
def test_non_string_manifest_hosts_fail_closed(bad):
    with pytest.raises(ValueError, match="must be a string"):
        resolve_web_host({"web_host": bad}, {"SCENE_WEB_HOST": "127.0.0.1"})


def test_explicit_blank_values_do_not_fall_back_to_all_interfaces():
    with pytest.raises(ValueError, match="must not be blank"):
        resolve_web_host({"web_host": ""}, {"SCENE_WEB_HOST": "127.0.0.1"})
    with pytest.raises(ValueError, match="must not be blank"):
        resolve_web_host({}, {"SCENE_WEB_HOST": ""})


def test_launchers_preserve_blank_values_for_runtime_rejection():
    root = Path(__file__).resolve().parents[1]
    docker_launcher = (root / "scripts" / "start.sh").read_text(encoding="utf-8")
    native_launcher = (root / "scripts" / "start_native.sh").read_text(
        encoding="utf-8"
    )
    expected = "SCENE_WEB_HOST=\"${SCENE_WEB_HOST-0.0.0.0}\""
    assert expected in docker_launcher
    assert expected in native_launcher
