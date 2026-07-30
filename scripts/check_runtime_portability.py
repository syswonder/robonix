#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Reject known deployment-specific fallbacks in generic runtime sources."""
from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOTS = ("system", "services", "tools", "pylib", "capabilities")
SOURCE_SUFFIXES = {".py", ".rs", ".sh", ".toml", ".yaml", ".yml", ".json", ".proto"}
RUNTIME_MARKDOWN = {"system/pilot/rtdl_protocol.md"}
EXCLUDED_PARTS = {
    "tests",
    "test",
    "testing",
    "examples",
    "fixtures",
    "target",
    "generated",
}

RULES: tuple[tuple[str, re.Pattern[str], tuple[str, ...] | None], ...] = (
    (
        "developer home path",
        re.compile(r"/(?:home|Users)/[A-Za-z0-9_.-]+/"),
        None,
    ),
    (
        "legacy developer model directory",
        re.compile(r"EmbodyMemory"),
        None,
    ),
    (
        "fixed Scene robot geometry",
        re.compile(
            r"Tiago-sized|_GOAL_NEAR_ROBOT_RADIUS_M|cam_offset_z|"
            r"circumscribed_radius_m\)\s*\|\|\s*0\.3"
        ),
        ("system/scene/scene_service",),
    ),
    (
        "guessed Scene coordinate frame",
        re.compile(
            r'lookup_xy_yaw\("base_link",\s*"map"\)|'
            r'frame_id\s*=\s*["\']map["\']|'
            r'origin["\']?\s*:\s*["\']world["\']|'
            r'lambda:\s*["\']map["\']'
        ),
        ("system/scene/scene_service",),
    ),
    (
        "generic Pilot prompt names one robot provider",
        re.compile(r"tiago_camera\.camera_snapshot"),
        ("system/pilot/src/planner.rs", "system/pilot/rtdl_protocol.md"),
    ),
    (
        "generic chassis contract names one robot driver setting",
        re.compile(r"TIAGO_CHASSIS_CMD_DURATION_SEC"),
        ("capabilities/lib/chassis/",),
    ),
    (
        "public Python API example names one deployment",
        re.compile(r"mid360_lidar|/scanner/cloud"),
        ("pylib/robonix-api/robonix_api/__init__.py",),
    ),
)


def _candidate_files() -> list[Path]:
    """Return generic source files while excluding scoped fixtures/adapters."""
    files: list[Path] = []
    for source_root in SOURCE_ROOTS:
        for path in (ROOT / source_root).rglob("*"):
            relative = path.relative_to(ROOT)
            if (
                path.is_file()
                and (
                    path.suffix in SOURCE_SUFFIXES
                    or relative.as_posix() in RUNTIME_MARKDOWN
                )
                and not EXCLUDED_PARTS.intersection(relative.parts)
            ):
                files.append(path)
    return sorted(files)


def main() -> int:
    """Print actionable file/line violations and return nonzero on matches."""
    violations: list[str] = []
    for path in _candidate_files():
        relative = path.relative_to(ROOT).as_posix()
        text = path.read_text(encoding="utf-8", errors="replace")
        for line_number, line in enumerate(text.splitlines(), 1):
            for label, pattern, prefixes in RULES:
                if prefixes is not None and not any(
                    relative.startswith(prefix) for prefix in prefixes
                ):
                    continue
                if pattern.search(line):
                    violations.append(
                        f"{relative}:{line_number}: {label}: {line.strip()}"
                    )
    if violations:
        print("runtime portability audit failed:", file=sys.stderr)
        print("\n".join(violations), file=sys.stderr)
        return 1
    print("runtime portability audit passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
