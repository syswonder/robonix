#!/usr/bin/env python3
"""piper_bridge.py — stdin/stdout JSON protocol wrapper for PiperCollector.

Spawned by mock Soma as a long-running subprocess.  Reads JSON commands from
stdin and writes JSON responses to stdout (one line per response).

Usage: piper_bridge.py <can_port>

Protocol:
  ← {"cmd":"collect"}
  → {"body_type":"arm","model":"piper","state":<int>,"message":"",
     "components":[{"name":"joint_1","kind":"joint",...}, ...]}
"""

from __future__ import annotations

import json
import sys

from piper_body import PiperCollector


def main() -> None:
    if len(sys.argv) < 2:
        print("usage: piper_bridge.py <can_port>", file=sys.stderr, flush=True)
        sys.exit(1)

    can_port = sys.argv[1]
    collector = PiperCollector(can_port)
    print(f"[piper_bridge] connected to Piper via {can_port}", file=sys.stderr, flush=True)

    for line in sys.stdin:
        try:
            cmd = json.loads(line.strip())
        except json.JSONDecodeError:
            continue
        if cmd.get("cmd") == "collect":
            result = collector.collect()
            print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
