# SPDX-License-Identifier: MulanPSL-2.0
"""Write the bench's deployment: Robonix's own system components and one
mock package per provider of the recorded Webots deployment.

    python3 world/make_deploy.py --out .deploy
    cd .deploy && rbnx boot

Pilot's VLM is the bench's recording proxy (proxy.py); the robot body is
the Webots TIAGo's soma.yaml.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
CATALOG = json.loads((HERE.parent / "fixtures/catalog.json").read_text())

# Odd last digits keep clear of the round ports other services take.
PORTS = {"atlas": 52051, "executor": 52061, "pilot": 52071, "soma": 52091, "proxy": 52442}

BUILD = """#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
VENV="__VENV__"
RBNX_CODEGEN_PYTHON="$VENV/bin/python" PATH="$VENV/bin:$PATH" rbnx codegen -p "$PKG" --mcp
"""

START = """#!/usr/bin/env bash
set -euo pipefail
PKG="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:__WORLD__:${PYTHONPATH:-}"
export BENCH_WORLD_DIR="__STATE__"
exec "__VENV__/bin/python" "__WORLD__/mock_provider.py" --provider __ID__
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", help="default .deploy, or .deploy-N for a slot")
    ap.add_argument("--slot", type=int, default=0, help="runs side by side need their own ports: slot N adds N*100")
    ap.add_argument("--venv", default=str(HERE.parent / ".venv"))
    a = ap.parse_args()
    out = Path(a.out or HERE.parent / (f".deploy-{a.slot}" if a.slot else ".deploy")).resolve()
    ports = {k: v + 100 * a.slot for k, v in PORTS.items()}
    venv = Path(a.venv).resolve()
    state = out / "world"
    state.mkdir(parents=True, exist_ok=True)

    # The TIAGo body, with its URDF found from here.
    soma = (REPO / "examples/webots/soma.yaml").read_text()
    urdf = REPO / "examples/webots/sim/ros_ws/src/eaios_webots/resource/tiago_webots.urdf"
    (out / "soma.yaml").write_text(re.sub(r"(?m)^(\s*path:\s*)\./sim/.*\.urdf$", rf"\g<1>{urdf}", soma, count=1))

    sections: dict[str, list] = {"primitive": [], "service": [], "skill": []}
    for pid, spec in CATALOG.items():
        kind = spec["namespace"].split("/")[1]
        section = kind if kind in sections else "service"
        pkg = out / "packages" / pid
        (pkg / "scripts").mkdir(parents=True, exist_ok=True)
        (pkg / "package_manifest.yaml").write_text(yaml.safe_dump({
            "manifestVersion": 1,
            "package": {"name": f"robonix.{section}.bench_{pid}", "version": "0.0.1",
                        "description": f"Model bench stand-in for {pid}.", "license": "MulanPSL-2.0"},
            "build": "bash scripts/build.sh",
            "start": "bash scripts/start.sh",
            "capabilities": [],
        }, sort_keys=False, allow_unicode=True))
        for name, text in (("build.sh", BUILD), ("start.sh", START)):
            script = text.replace("__VENV__", str(venv)).replace("__WORLD__", str(HERE)).replace("__STATE__", str(state)).replace("__ID__", pid)
            (pkg / "scripts" / name).write_text(script)
            (pkg / "scripts" / name).chmod(0o755)
        sections[section].append({"name": pid, "path": f"./packages/{pid}", "config": {}})

    manifest = {
        "manifestVersion": 1,
        "name": "robonix-model-bench",
        "system": {
            "atlas": {"listen": f"127.0.0.1:{ports['atlas']}", "log": "info"},
            "soma": {"robot_yaml": str(out / "soma.yaml"), "listen": f"127.0.0.1:{ports['soma']}"},
            "executor": {"listen": f"127.0.0.1:{ports['executor']}"},
            "pilot": {
                "listen": f"127.0.0.1:{ports['pilot']}",
                "vlm": {"upstream": f"http://127.0.0.1:{ports['proxy']}/v1", "api_key": "bench", "model": "bench"},
            },
        },
        **sections,
    }
    (out / "robonix_manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True))
    if not (venv / "bin/python").exists():
        print(f"create the world's venv first: uv venv {venv} && uv pip install -p {venv} -e {REPO}/pylib/robonix-api pyyaml")
    print(out)


if __name__ == "__main__":
    main()
