#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Drive the robot to hand-picked goals through the deployment's nav2 service.

Same MCP surface the explore skill uses (`navigate` + `status`), resolved
through atlas via `rbnx inspect`, so the motion in a demo is real navigation —
costmap, planner, controller — without explore's frontier picking, which puts
goals at wall boundaries and leaves nav2 spinning against them.

    nav_goal.py --rbnx rbnx  X,Y,YAW  [X,Y,YAW ...]
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import subprocess
import sys
import time
from pathlib import Path


def _nav_endpoint(rbnx: Path) -> str:
    snap = json.loads(subprocess.run([str(rbnx), "inspect"], check=True, capture_output=True, text=True).stdout)
    provider = (snap.get("providers") or {}).get("nav2") or {}
    for ep in provider.get("endpoints") or ():
        if str(ep.get("contract_id") or "") == "robonix/service/navigation/navigate":
            return str(ep.get("endpoint") or "")
    raise RuntimeError("nav2 navigate endpoint not registered")


def _payload(result) -> dict:
    for item in getattr(result, "content", ()):
        text = getattr(item, "text", "")
        if text:
            try:
                return json.loads(text)
            except json.JSONDecodeError:
                return {"raw": text}
    return {}


async def _drive(endpoint: str, goals: list[tuple[float, float, float]], timeout_s: float) -> int:
    from fastmcp import Client  # type: ignore[import-not-found]
    async with Client(endpoint) as c:
        for x, y, yaw in goals:
            goal = {"header": {"frame_id": "map", "stamp": {"sec": 0, "nanosec": 0}},
                    "pose": {"position": {"x": x, "y": y, "z": 0.0},
                             "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}}}
            resp = _payload(await c.call_tool("navigate", {"goal": goal}))
            print(f"goal ({x:.2f},{y:.2f},{yaw:.2f}) -> {resp}", flush=True)
            if not resp.get("accepted"):
                return 1
            run_id = resp.get("run_id", "")
            deadline = time.time() + timeout_s
            state = ""
            while time.time() < deadline:
                st = _payload(await c.call_tool("status", {"run_id": run_id}))
                state = str(st.get("state", "")).upper()
                if state in ("SUCCEEDED", "FAILED", "CANCELED", "TIMEOUT"):
                    break
                await asyncio.sleep(1.0)
            print(f"  -> {state or 'no terminal state'}", flush=True)
            if state != "SUCCEEDED":
                return 2
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rbnx", type=Path, required=True)
    ap.add_argument("--timeout-s", type=float, default=90.0)
    ap.add_argument("goals", nargs="+", help="X,Y,YAW in the map frame")
    a = ap.parse_args()
    goals = [tuple(float(v) for v in g.split(",")) for g in a.goals]
    return asyncio.run(_drive(_nav_endpoint(a.rbnx), goals, a.timeout_s))


if __name__ == "__main__":
    sys.exit(main())
