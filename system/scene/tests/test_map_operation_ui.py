# SPDX-License-Identifier: MulanPSL-2.0
"""Structural checks for the blocking Save/Load operation dialog."""

import os
import shutil
import subprocess
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def _user_html():
    try:
        from scene_service.web import _USER_HTML
    except ImportError as exc:
        pytest.skip(f"web deps unavailable: {exc}")
    return _USER_HTML


def test_map_operations_have_one_blocking_dialog_and_retry_path():
    html = _user_html()
    assert 'id="map-operation-modal"' in html
    assert "function beginMapOperation" in html
    assert "function finishMapOperation" in html
    assert "if (mapBusy) ev.preventDefault()" in html
    assert "Wait for a fresh occupancy grid" in html
    assert "Restore rooms and Scene objects" in html
    assert "retry: () => loadSelectedMap(id)" in html


def test_embedded_user_script_is_valid_javascript():
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    html = _user_html()
    script = html.rsplit("<script>", 1)[1].split("</script>", 1)[0]
    with tempfile.NamedTemporaryFile("w", suffix=".js") as handle:
        handle.write(script)
        handle.flush()
        subprocess.run([node, "--check", handle.name], check=True)


def test_robot_marker_is_high_contrast_and_directional():
    html = _user_html()
    assert "const robotMarkerNose = 24" in html
    assert "ctx.rotate(-yaw)" in html
    assert "ctx.strokeStyle = '#ffffff'" in html
    assert "ctx.fillStyle = '#ff5a1f'" in html
