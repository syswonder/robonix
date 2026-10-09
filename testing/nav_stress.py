#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Explore, then send the robot to random reachable points, and count collisions.

Runs where a Robonix provider runs (the nav2 container in the Webots example):
it needs rclpy for /map and the simulator's collision topic, robonix_api for
Atlas, grpc for the skill's Driver, and fastmcp for the tools. Every call goes
through Robonix as a consumer would make it: explore is activated over its
Driver the way the executor does, and both explore and navigate are called as
MCP tools resolved through Atlas.

The judge is the simulator: /webots/collision reports any contact other than
wheels on the floor. Exit status 1 when there was one, 2 when the run itself
could not be carried out.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import random
import sys
import threading
import time
from collections import deque

CONSUMER = "nav-stress"
TERMINAL = {"SUCCEEDED", "FAILED", "CANCELED", "TIMEOUT"}


def log(msg: str) -> None:
    print(f"[nav-stress {time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ── Robonix ──────────────────────────────────────────────────────────────────
def endpoint(contract_id: str, transport: str, provider_id: str = "") -> tuple[str, str]:
    from robonix_api.atlas import ATLAS

    caps = ATLAS.find_capability(contract_id=contract_id, transport=transport, provider_id=provider_id)
    if not caps:
        raise RuntimeError(f"no provider for {contract_id} [{transport}]")
    cap = caps[0]
    ch = ATLAS.connect_capability(
        consumer_id=CONSUMER, provider_id=cap.provider_id, contract_id=contract_id, transport=transport
    )
    url = ch.endpoint
    ch.close()
    return cap.provider_id, url


def activate(provider_id: str) -> None:
    """Driver(CMD_ACTIVATE), as the executor sends it before a skill's first call."""
    import grpc
    from robonix_api.atlas import ATLAS
    from robonix_api.lifecycle import contract_id_to_pascal

    caps = [c for c in ATLAS.find_capability(provider_id=provider_id, transport="grpc") if c.contract_id.endswith("/driver")]
    if not caps:
        raise RuntimeError(f"{provider_id} has no */driver capability")
    contract = caps[0].contract_id
    _, url = endpoint(contract, "grpc", provider_id)
    call = grpc.insecure_channel(url.removeprefix("http://")).unary_unary(
        f"/robonix.contracts.{contract_id_to_pascal(contract)}/Driver",
        request_serializer=lambda b: b,
        response_deserializer=lambda b: b,
    )
    reply = call(b"\x08\x01", timeout=30)  # command = CMD_ACTIVATE
    # Field 1 (ok) is a varint; an ok=false reply leaves it out entirely.
    if not reply.startswith(b"\x08\x01") and b"active" not in reply:
        raise RuntimeError(f"Driver(CMD_ACTIVATE) on {provider_id} refused: {reply!r}")


class Tools:
    def __init__(self, url: str):
        from fastmcp import Client

        self._client = Client(url)

    def __call__(self, tool: str, **args) -> dict:
        async def go():
            async with self._client as c:
                result = await c.call_tool(tool, args)
                text = result.content[0].text if result.content else "{}"
                try:
                    return json.loads(text)
                except json.JSONDecodeError:
                    return {"raw": text}

        return asyncio.run(go())


# ── ROS ──────────────────────────────────────────────────────────────────────
class World:
    """The map, and the simulator's collision report."""

    def __init__(self):
        import rclpy
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String, UInt32

        rclpy.init()
        self.node = rclpy.create_node("nav_stress")
        self.map = None
        self.costmap = None
        self.collisions: list[dict] = []
        self.count = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        # What navigation itself treats as free: the map plus every obstacle
        # layer, the depth camera's included.
        self.node.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: setattr(self, "costmap", m), latched)
        self.node.create_subscription(String, "/webots/collision", lambda m: self.collisions.append(json.loads(m.data)), 50)
        self.node.create_subscription(UInt32, "/webots/collision_count", lambda m: setattr(self, "count", m.data), latched)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def _on_map(self, msg):
        self.map = msg


def candidates(grid, clearance_m: float) -> list[tuple[float, float]]:
    """Free cells at least `clearance_m` from any occupied or unknown cell.

    On the global costmap, any cost at all (inflation included) counts as
    occupied, so the clearance is on top of the robot's own radius."""
    w, h, res = grid.info.width, grid.info.height, grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    data = grid.data
    dist = [-1] * (w * h)
    queue = deque()
    for i, v in enumerate(data):
        if v != 0:  # occupied (>0) or unknown (-1)
            dist[i] = 0
            queue.append(i)
    while queue:
        i = queue.popleft()
        x, y = i % w, i // w
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < w and 0 <= ny < h and dist[ny * w + nx] < 0:
                dist[ny * w + nx] = dist[i] + 1
                queue.append(ny * w + nx)
    need = math.ceil(clearance_m / res)
    return [
        (ox + (i % w + 0.5) * res, oy + (i // w + 0.5) * res)
        for i, d in enumerate(dist)
        if d >= need and data[i] == 0
    ]


def goal_pose(x: float, y: float, yaw: float) -> dict:
    return {
        "header": {"frame_id": "map", "stamp": {"sec": 0, "nanosec": 0}},
        "pose": {
            "position": {"x": x, "y": y, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)},
        },
    }


def wait_terminal(status, run_id: str, timeout_s: float) -> dict:
    deadline = time.time() + timeout_s
    last = {}
    while time.time() < deadline:
        last = status(run_id=run_id) or {}
        if str(last.get("state", "")).upper() in TERMINAL:
            return last
        time.sleep(1.0)
    return {**last, "state": "TIMEOUT_LOCAL"}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--goals", type=int, default=10)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--explore-s", type=float, default=240.0, help="explore budget; 0 skips exploring")
    ap.add_argument("--clearance", type=float, default=0.2, help="metres beyond the costmap's own inflation")
    ap.add_argument("--min-hop", type=float, default=1.5, help="minimum distance between consecutive goals")
    ap.add_argument("--goal-timeout", type=float, default=150.0)
    ap.add_argument("--out", default="", help="write the summary JSON here too")
    a = ap.parse_args()

    world = World()
    try:
        _, nav_url = endpoint("robonix/service/navigation/navigate", "mcp")
        nav = Tools(nav_url)
        explore_id, explore_url = endpoint("robonix/skill/explore/explore", "mcp")
        explore = Tools(explore_url)
    except Exception as e:  # noqa: BLE001
        log(f"cannot resolve the tools: {e}")
        return 2

    if a.explore_s > 0:
        activate(explore_id)
        r = explore("explore", area_hint="", timeout_s=a.explore_s, max_speed_m_s=0.0)
        log(f"explore: {r}")
        if not r.get("accepted"):
            return 2
        end = wait_terminal(lambda **k: explore("status", **k), r.get("run_id", ""), a.explore_s + 60)
        log(f"explore finished: {end.get('state')} area={end.get('area_m2')} frontiers={end.get('frontiers_left')}")

    for _ in range(60):
        if world.costmap is not None or world.map is not None:
            break
        time.sleep(1.0)
    grid = world.costmap or world.map
    if grid is None:
        log("no /map or /global_costmap/costmap")
        return 2
    pool = candidates(grid, a.clearance)
    log(f"{len(pool)} candidate cells with {a.clearance} m clearance on {'the global costmap' if world.costmap else '/map'}")
    if not pool:
        return 2

    rng = random.Random(a.seed)
    results = []
    last = None
    for n in range(a.goals):
        far = [p for p in pool if last is None or math.dist(p, last) >= a.min_hop]
        x, y = rng.choice(far or pool)
        yaw = rng.uniform(-math.pi, math.pi)
        before = len(world.collisions)
        t0 = time.time()
        r = nav("navigate", goal=goal_pose(x, y, yaw))
        if not r.get("accepted"):
            end = {"state": "REJECTED", "detail": r.get("detail", "")}
        else:
            end = wait_terminal(lambda **k: nav("status", **k), r.get("run_id", ""), a.goal_timeout)
            if end.get("state") == "TIMEOUT_LOCAL":
                nav("cancel", run_id=r.get("run_id", ""))
        hits = world.collisions[before:]
        row = {
            "goal": [round(x, 2), round(y, 2), round(yaw, 2)],
            "state": end.get("state"),
            "seconds": round(time.time() - t0, 1),
            "detail": str(end.get("detail", ""))[:200],
            "collisions": hits,
        }
        results.append(row)
        last = (x, y)
        log(f"goal {n + 1}/{a.goals} ({x:.2f}, {y:.2f}): {row['state']} in {row['seconds']} s, collisions {len(hits)}")

    summary = {
        "goals": len(results),
        "succeeded": sum(r["state"] == "SUCCEEDED" for r in results),
        "collisions": len(world.collisions),
        "collision_count_topic": world.count,
        "results": results,
    }
    text = json.dumps(summary, indent=2, ensure_ascii=False)
    print(text)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            f.write(text)
    return 1 if summary["collisions"] else 0


if __name__ == "__main__":
    sys.exit(main())
