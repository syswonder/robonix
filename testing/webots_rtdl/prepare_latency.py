# SPDX-License-Identifier: MulanPSL-2.0
"""Prepare disposable copies of real Webots sensor handlers for latency timing."""

import ast
import os
import shlex
import subprocess
from pathlib import Path

import yaml

root = Path(__file__).resolve().parents[2]
out = Path(os.environ["RTDL_TEST_OUT"]).resolve()
out.mkdir(exist_ok=True)
if (out / "manifest.yaml").exists():
    raise SystemExit(
        "Use a fresh RTDL_TEST_OUT; existing setup must not be overwritten"
    )
shared = root / "examples/webots/rbnx-boot/logs/soma-runtime/latency-handlers.jsonl"
shared.touch(exist_ok=True)
manifest = {
    "manifestVersion": 1,
    "name": "webots-latency",
    "system": {
        "atlas": {"listen": "127.0.0.1:50051"},
        "executor": {"listen": "127.0.0.1:50061"},
        "soma": {"robot_yaml": str(root / "examples/webots/soma.yaml")},
    },
    "primitive": [],
}
for kind in ["lidar", "camera"]:
    pkg = root / f"examples/webots/primitives/tiago_{kind}"
    source = (pkg / f"{kind}_driver/driver.py").read_text()
    tree = ast.parse(source)
    fn = next(
        n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "snapshot"
    )
    pos = fn.body[1].lineno - 1
    lines = source.splitlines(True)
    inject = f'''    import time as _latency_time
    _latency_ns = _latency_time.monotonic_ns()
    with open("/robonix_runtime/soma/latency-handlers.jsonl", "a") as _latency_log:
        _latency_log.write('{{"ns":' + str(_latency_ns) + ',"handler":"{kind}"}}\\n')
'''
    lines.insert(pos, inject)
    driver = out / f"{kind}-instrumented.py"
    driver.write_text("".join(lines))
    subprocess.run(
        ["docker", "cp", str(driver), f"robonix_tiago_sim:/tmp/latency-{kind}.py"],
        check=True,
        capture_output=True,
    )
    start = (
        (pkg / "scripts/start.sh")
        .read_text()
        .replace('$(dirname "$0")', "${RTDL_DRIVER_SCRIPT_DIR}")
        .replace(f"python3 -m {kind}_driver.driver", f"python3 /tmp/latency-{kind}.py")
    )
    start = (
        "export RTDL_DRIVER_SCRIPT_DIR="
        + shlex.quote(str(pkg / "scripts"))
        + "\n"
        + start
    )
    script = out / f"start-{kind}.sh"
    script.write_text(start)
    wrapper = out / f"tiago_{kind}"
    wrapper.mkdir(exist_ok=True)
    pm = yaml.safe_load((pkg / "package_manifest.yaml").read_text())
    pm["start"] = "bash " + shlex.quote(str(script))
    pm["build"] = "bash " + shlex.quote(str(pkg / "scripts/build.sh"))
    (wrapper / "package_manifest.yaml").write_text(yaml.safe_dump(pm))
    for name in ["config.spec", "CAPABILITY.md", "rbnx-build"]:
        p = wrapper / name
        if not p.exists():
            p.symlink_to(pkg / name)
    cfg = {"sentinel_timeout_s": 150}
    if kind == "camera":
        cfg.update(width=640, height=480, fx=554.0, fy=554.0, cx=320.0, cy=240.0)
    manifest["primitive"].append(
        {"name": f"tiago_{kind}", "path": str(wrapper), "config": cfg}
    )
(out / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
proto = out / "proto"
proto.mkdir(exist_ok=True)
src = max(
    (root / "target/debug/build").glob("robonix-pilot-*/out/robonix_contracts.proto"),
    key=lambda p: p.stat().st_mtime,
).parent
subprocess.run(
    [
        "python3",
        "-m",
        "grpc_tools.protoc",
        "-I",
        str(src),
        f"--python_out={proto}",
        f"--grpc_python_out={proto}",
        *map(str, src.glob("*.proto")),
    ],
    check=True,
)
print("Instrumented copies and minimal live deployment prepared")
