# SPDX-License-Identifier: MulanPSL-2.0
"""Structural checks for the map operation pages."""

import re
import os
import shutil
import subprocess
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def _regions_html():
    try:
        from scene_service.web import _REGIONS_HTML
    except ImportError as exc:
        pytest.skip(f"web deps unavailable: {exc}")
    return _REGIONS_HTML


def test_map_operations_run_one_at_a_time_and_report_failure():
    """Save, load and delete live on the maps page."""
    try:
        from scene_service.web import _MAPS_BODY
    except ImportError as exc:
        pytest.skip(f"web deps unavailable: {exc}")
    assert "/api/maps/${op}" in _MAPS_BODY
    assert "call('save', id" in _MAPS_BODY
    assert "if (busy) return null;" in _MAPS_BODY
    assert "d.detail || t('maps.failed')" in _MAPS_BODY


def test_embedded_user_script_is_valid_javascript():
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    html = _regions_html()
    script = html.rsplit("<script>", 1)[1].split("</script>", 1)[0]
    with tempfile.NamedTemporaryFile("w", suffix=".js") as handle:
        handle.write(script)
        handle.flush()
        subprocess.run([node, "--check", handle.name], check=True)


def test_robot_marker_is_high_contrast_and_directional():
    html = _regions_html()
    # Directional: the canvas turns with the robot, and the marker reaches
    # well past its own body so the heading reads at a glance. Asserted as a
    # relation between the two constants rather than as their values, which
    # is what the property actually is -- the nose was once a single length
    # called robotMarkerNose and is now a cone plus an arrow.
    assert "ctx.rotate(-yaw)" in html
    body = re.search(r"const R = (\d+)", html)
    reach = re.search(r"const TIP = (\d+)", html)
    assert body and reach, "robot marker has no body radius / reach constants"
    assert int(reach.group(1)) > 2 * int(body.group(1)), (
        f"reach {reach.group(1)} is not clearly longer than body {body.group(1)}"
    )
    # High contrast: a dark halo under a light body, so it survives both the
    # white free space and the dark unknown region of the occupancy underlay.
    assert "rgba(8, 11, 17, 0.55)" in html
    assert "#7aa7ff" in html
