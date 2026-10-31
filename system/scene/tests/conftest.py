# SPDX-License-Identifier: MulanPSL-2.0
"""Put the Scene package root and the shared resolver on sys.path.

pytest prepends the directory that holds a test file, not the package root, so
`from scene_service import ...` only resolves when pytest is run from
system/scene. The older tests each carry their own sys.path.insert; the ones
that do not import nothing when CI runs pytest from the repository root, which
is how test_goal_planner.py could sit in the tree unexercised. Doing it once
here means a new test does not have to remember.

The focused Scene CI job installs the test dependencies but not the uv
workspace, so make the in-repo `robonix-api` package importable as well.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE_ROOT = os.path.dirname(HERE)
REPO_ROOT = os.path.dirname(os.path.dirname(SCENE_ROOT))

sys.path.insert(0, SCENE_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "pylib", "robonix-api"))
