# SPDX-License-Identifier: MulanPSL-2.0
"""Put the Scene package root on sys.path for every test in this directory.

pytest prepends the directory that holds a test file, not the package root, so
`from scene_service import ...` only resolves when pytest is run from
system/scene. The older tests each carry their own sys.path.insert; the ones
that do not import nothing when CI runs pytest from the repository root, which
is how test_goal_planner.py could sit in the tree unexercised. Doing it once
here means a new test does not have to remember.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
