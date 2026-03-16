#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Generate pyrightconfig.json extraPaths from rust/examples rbnx-build dirs."""

import json
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
EXAMPLES = WORKSPACE_ROOT / "rust" / "examples"
PYTHON_VER = "3.10"

# Standard paths per example (relative to example dir)
COMMON_PATHS = [
    f"rbnx-build/ws/install/robonix_interfaces/lib/python{PYTHON_VER}/site-packages",
    "rbnx-build/ws/src/generated/robonix_interfaces",
    f"rbnx-build/ws/install/robonix_interfaces_ros2/local/lib/python{PYTHON_VER}/dist-packages",
    f"rbnx-build/ws/install/robonix_msgs/local/lib/python{PYTHON_VER}/dist-packages",
]

# Extra paths for skill_demo (package-local interfaces)
SKILL_DEMO_EXTRAS = [
    f"rbnx-build/ws/install/skill_demo_interfaces/lib/python{PYTHON_VER}/site-packages",
    f"rbnx-build/ws/install/skill_demo_interfaces_ros2/local/lib/python{PYTHON_VER}/dist-packages",
    f"rbnx-build/ws/install/skill_demo_msgs/local/lib/python{PYTHON_VER}/dist-packages",
]


def main() -> None:
    paths: list[str] = []
    for pkg_dir in sorted(EXAMPLES.iterdir()):
        if not pkg_dir.is_dir():
            continue
        rbnx = pkg_dir / "rbnx-build" / "ws" / "install"
        if not rbnx.exists():
            continue
        rel = pkg_dir.relative_to(WORKSPACE_ROOT)
        for p in COMMON_PATHS:
            full = pkg_dir / p
            if full.exists() or (pkg_dir / "rbnx-build").exists():
                paths.append(str(rel / p))
        if pkg_dir.name == "skill_demo":
            for p in SKILL_DEMO_EXTRAS:
                paths.append(str(rel / p))

    out = WORKSPACE_ROOT / "pyrightconfig.json"
    cfg = {"extraPaths": sorted(set(paths))}
    out.write_text(json.dumps(cfg, indent=2) + "\n")
    print(f"Wrote {out} with {len(cfg['extraPaths'])} paths")


if __name__ == "__main__":
    main()
