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
    assert resolve_web_host({}, {"SCENE_WEB_HOST": "0.0.0.0"}) == "0.0.0.0"


def test_the_ui_is_local_unless_someone_opens_it():
    # The UI has no authentication and its annotation endpoints write map
    # data, so the default is loopback. Exposing it is a decision; a
    # deployment that wants it sets web_host or SCENE_WEB_HOST.
    assert resolve_web_host({}, {}) == "127.0.0.1"


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
    # `${VAR-default}` and not `${VAR:-default}`: an operator who exports an
    # empty SCENE_WEB_HOST has made a mistake, and it should reach
    # resolve_web_host to be rejected rather than be replaced silently.
    root = Path(__file__).resolve().parents[1]
    for name in ("start.sh", "start_native.sh"):
        launcher = (root / "scripts" / name).read_text(encoding="utf-8")
        assert 'SCENE_WEB_HOST="${SCENE_WEB_HOST-' in launcher, name
        assert 'SCENE_WEB_HOST="${SCENE_WEB_HOST:-' not in launcher, name


def test_launchers_default_the_ui_to_loopback():
    # The library default is loopback; a launcher that passed 0.0.0.0 would
    # override it and put the unauthenticated UI back on every interface.
    root = Path(__file__).resolve().parents[1]
    for name in ("start.sh", "start_native.sh"):
        launcher = (root / "scripts" / name).read_text(encoding="utf-8")
        assert 'SCENE_WEB_HOST="${SCENE_WEB_HOST-127.0.0.1}"' in launcher, name
