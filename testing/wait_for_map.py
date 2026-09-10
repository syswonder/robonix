#!/usr/bin/env python3
"""Wait until /map carries enough known cells to plan across.

A Webots boot starts a fresh live SLAM session by design — see the comment at
the top of examples/webots/robonix_manifest.yaml. RTAB-Map therefore opens an
empty database and fills it from the sensor stream as the simulation runs, and
for the first stretch after boot the occupancy grid is published but entirely
unknown.

Waiting for a publisher does not capture that: the publisher exists from the
moment the node starts. Waiting for a message does not either, because an
all-unknown grid is a perfectly good message. What a scenario that needs a
global plan actually depends on is free space, so that is what this waits for.

Prints the grid's size and known-cell count and exits 0 when the threshold is
met, or exits 1 on timeout having printed the best it saw.
"""

import argparse
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/map")
    ap.add_argument("--timeout", type=float, default=180.0)
    # A tenth of a percent of a large grid is still hundreds of cells. The
    # threshold is deliberately low: this gate is here to catch "the map has
    # not started yet", not to judge coverage.
    ap.add_argument("--min-known", type=int, default=200)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("robonix_ci_wait_for_map")

    # The map is latched: a subscriber that joins late must still receive the
    # last grid rather than waiting for the next publish.
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )

    best = {"known": -1, "width": 0, "height": 0}

    def on_map(msg: OccupancyGrid) -> None:
        known = sum(1 for cell in msg.data if cell >= 0)
        if known > best["known"]:
            best.update(
                known=known, width=msg.info.width, height=msg.info.height
            )

    node.create_subscription(OccupancyGrid, args.topic, on_map, qos)

    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            if best["known"] >= args.min_known:
                print(
                    f"{args.topic}: {best['width']}x{best['height']} grid, "
                    f"{best['known']} known cells"
                )
                return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if best["known"] < 0:
        print(f"{args.topic}: no message received in {args.timeout:.0f}s", file=sys.stderr)
    else:
        print(
            f"{args.topic}: only {best['known']} known cells after "
            f"{args.timeout:.0f}s (needed {args.min_known})",
            file=sys.stderr,
        )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
