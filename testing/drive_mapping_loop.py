#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Close a 1.5 m mapping square through Navigation, without blind velocity commands."""
from __future__ import annotations

import atexit
import math
import signal
import threading
import time
from types import SimpleNamespace

from nav_stress import TERMINAL, Tools, World, activate, candidates, endpoint, goal_pose, wait_terminal


def mapping_square(grid, side_m: float = 1.5) -> list[tuple[float, float, float]]:
    """Keep the perimeter clear of inscribed, lethal and unknown costmap cells."""
    info = grid.info
    res = float(info.resolution)
    if not math.isfinite(res) or res <= 0 or not 0 < info.width * info.height <= 1_000_000:
        raise ValueError("invalid mapping costmap dimensions or resolution")
    if len(grid.data) != info.width * info.height:
        raise ValueError("mapping costmap data length differs from its dimensions")
    if not math.isfinite(side_m) or side_m < 1.5:
        raise ValueError("mapping square sides must be at least 1.5 m")
    ox, oy = info.origin.position.x, info.origin.position.y
    traversable = SimpleNamespace(info=info, data=[0 if 0 <= cost < 99 else 100 for cost in grid.data])
    # The existing four-neighbor distance needs sqrt(2) scaling to bound Euclidean clearance.
    cells = {(round((x - ox) / res - .5), round((y - oy) / res - .5))
             for x, y in candidates(traversable, clearance_m=math.sqrt(2) * .2)}
    steps = math.ceil(side_m / res)
    for x, y in sorted(cells):
        corners = [(x, y), (x + steps, y), (x + steps, y + steps), (x, y + steps)]
        if not all(point in cells for point in corners):
            continue
        if not all((x + d, y) in cells and (x + d, y + steps) in cells
                   and (x, y + d) in cells and (x + steps, y + d) in cells
                   for d in range(steps + 1)):
            continue
        return [(ox + (gx + .5) * res, oy + (gy + .5) * res, yaw)
                for (gx, gy), yaw in zip(corners + corners[:1],
                                        (0., math.pi / 2, math.pi, -math.pi / 2, 0.))]
    raise RuntimeError("no cleared 1.5 m mapping square in Navigation's costmap")


def read_collision_count(world) -> int:
    """Query the publisher's current latched count after Navigation completes."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import UInt32

    received = threading.Event()
    value = []
    def receive(message):
        value[:] = [message.data]
        received.set()
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    subscription = world.node.create_subscription(UInt32, "/webots/collision_count", receive, qos)
    try:
        if not received.wait(5.):
            raise RuntimeError("fresh simulator collision count did not arrive")
        return value[0]
    finally:
        world.node.destroy_subscription(subscription)


def drive_square(world, nav, waypoints) -> None:
    if world.count is None:
        raise RuntimeError("simulator collision counter is unavailable")
    before = world.count
    events_before = len(world.collisions)
    for index, (x, y, yaw) in enumerate(waypoints):
        result = nav("navigate", goal=goal_pose(x, y, yaw))
        if not result.get("accepted") or not result.get("run_id"):
            raise RuntimeError(f"mapping waypoint {index} rejected: {result}")
        run_id = result["run_id"]
        completed = False
        try:
            end = wait_terminal(lambda **args: nav("status", **args), run_id, 150.)
            completed = end.get("state") == "SUCCEEDED"
            if not completed:
                raise RuntimeError(f"mapping waypoint {index} failed: {end}")
        finally:
            if not completed:
                nav("cancel", run_id=run_id)
        if read_collision_count(world) != before or len(world.collisions) != events_before:
            raise RuntimeError(f"simulator reported a collision during mapping waypoint {index}")
        print(f"mapping waypoint {index}/4 reached ({x:.3f}, {y:.3f}) without collision", flush=True)


def main() -> None:
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    world = World()
    try:
        deadline = time.monotonic() + 60.
        while world.costmap is None or world.count is None:
            if time.monotonic() >= deadline:
                raise RuntimeError("Navigation costmap or collision counter did not become ready")
            time.sleep(.2)
        try:
            waypoints = mapping_square(world.costmap)
        except RuntimeError:
            # A partial scan can expose too little known space to close the square.
            before = read_collision_count(world)
            events_before = len(world.collisions)
            provider, url = endpoint("robonix/skill/explore/explore", "mcp")
            activate(provider)
            explore = Tools(url)
            result = explore("explore", area_hint="", timeout_s=120., max_speed_m_s=0.)
            if not result.get("accepted") or not result.get("run_id"):
                raise RuntimeError(f"mapping preparation rejected: {result}")
            completed = False
            try:
                end = wait_terminal(lambda **args: explore("status", **args), result["run_id"], 180.)
                completed = end.get("state") in TERMINAL
                if not completed:
                    raise RuntimeError(f"mapping preparation did not terminate: {end}")
                if end.get("state") not in ("SUCCEEDED", "TIMEOUT"):
                    raise RuntimeError(f"mapping preparation failed: {end}")
                print(f"mapping preparation ended: {end.get('state')} {str(end.get('detail', ''))[:200]}", flush=True)
            finally:
                if not completed:
                    explore("cancel", run_id=result["run_id"])
            if read_collision_count(world) != before or len(world.collisions) != events_before:
                raise RuntimeError("simulator reported a collision during mapping preparation")
            waypoints = mapping_square(world.costmap)
        _, url = endpoint("robonix/service/navigation/navigate", "mcp")
        drive_square(world, Tools(url), waypoints)
    finally:
        atexit.unregister(world.close)
        world.close()


if __name__ == "__main__":
    main()
